#include "gla3a.h"
#include <lv2/core/lv2.h>
#include <lv2/log/logger.h>
#include <lv2/log/log.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// --- Costanti e Definizioni ---
#define M_PI_F 3.14159265358979323846f

// --- Calibrazione del Compressore ---
#define PEAK_REDUCTION_MIN_DB -60.0f // Soglia min per Peak Reduction (più compressione)
#define PEAK_REDUCTION_MAX_DB -10.0f // Soglia max per Peak Reduction (meno compressione)
#define GAIN_MAX_DB 12.0f            // Guadagno massimo applicabile in dB
#define KNEE_WIDTH_DB 10.0f          // Larghezza della soft-knee in dB

// Tempi di smoothing per il detector e il gain (questi cambieranno con la ratio mode)
#define DETECTOR_ATTACK_FACTOR_BASE 0.005f // Base per tempo attacco (veloce)
#define DETECTOR_RELEASE_FACTOR_BASE 0.2f  // Base per tempo rilascio (lento)

// --- J-FET Distortion ---
#define JF_K_FACTOR 2.0f // Fattore di "durezza" della distorsione J-FET (regola il carattere)
#define JF_DRY_WET_MIX 0.3f // Mix tra segnale pulito e distorto dal J-FET (0.0 a 1.0)
#define JF_SATURATION_THRESHOLD 0.5f // Soglia (lineare) oltre la quale la distorsione J-FET è più evidente

// --- Soft-Clipping Finale (Limiter) ---
#define FINAL_SOFT_CLIP_THRESHOLD_DB -1.0f // Inizia il soft-clip finale a -1 dBFS
#define FINAL_SOFT_CLIP_AMOUNT 0.5f        // Quanto è "soft" il clip finale (0.0 a 1.0, 1.0 è hard clip)

// --- RMS Meter Smoothing ---
#define RMS_METER_SMOOTH_MS 50.0f // Tempo in ms per la costante di tempo RMS del meter

// --- OVERSEMPLING/UPSAMPLING ---
#define UPSAMPLE_FACTOR 4 // Fattore di oversampling (2x, 4x, 8x, etc.)
// Useremo 3 filtri biquad in cascata per l'upsampling e il downsampling,
// per ottenere un filtro passa-basso di 6° ordine (36 dB/ottava).
#define NUM_BIQUADS_FOR_OS_FILTER 3 
#define OS_FILTER_Q 0.707f // Q di Butterworth per risposta piatta (o calibra per più risonanza)

// --- FILTRI BIQUAD PER SIDECHAIN (6° ORDINE = 3 BIQUAD IN CASCATA) ---
#define NUM_BIQUADS_FOR_6TH_ORDER 3 // Ogni biquad è 2° ordine (12 dB/ottava)


// --- Funzioni di Utilità Generali ---

static float to_db(float linear_val) {
    if (linear_val <= 0.00000000001f) return -90.0f;
    return 20.0f * log10f(linear_val);
}

static float db_to_linear(float db_val) {
    return powf(10.0f, db_val / 20.0f);
}

// Funzione per applicare il soft-clipping finale
static float apply_final_soft_clip(float sample, float threshold_linear, float amount) {
    float sign = (sample >= 0) ? 1.0f : -1.0f;
    float abs_sample = fabsf(sample);

    if (abs_sample <= threshold_linear) {
        return sample;
    } else {
        float normalized_over_threshold = (abs_sample - threshold_linear) / (1.0f - threshold_linear);
        float clipped_val = threshold_linear + (1.0f - threshold_linear) * (1.0f - expf(-amount * normalized_over_threshold));
        return sign * fminf(clipped_val, 1.0f);
    }
}

// Funzione per la distorsione J-FET (approssimazione sigmoide)
static float apply_jfet_distortion(float sample, float k_factor, float threshold_jfet, float dry_wet_mix) {
    float distorted_sample;
    float abs_sample = fabsf(sample);

    if (abs_sample <= threshold_jfet) {
        distorted_sample = sample; // Nessuna distorsione significativa sotto la soglia
    } else {
        // Normalizza il sample oltre la soglia
        float x_norm = (abs_sample - threshold_jfet) / (1.0f - threshold_jfet);
        // Applica la funzione di shaping basata su arctan o tanh (più comune per J-FET)
        // Usiamo una curva sigmoide più generale che include k_factor per variabilità
        float shaped_x = x_norm / (1.0f + k_factor * x_norm);
        distorted_sample = threshold_jfet + (1.0f - threshold_jfet) * shaped_x;

        // Ri-applica il segno originale
        distorted_sample = (sample >= 0) ? distorted_sample : -distorted_sample;
    }
    
    // Mix dry/wet
    return sample * (1.0f - dry_wet_mix) + distorted_sample * dry_wet_mix;
}


// Funzione per calcolare l'RMS per il meter di output
static float calculate_rms_level(const float* buffer, uint32_t n_samples, float current_rms, float alpha) {
    if (n_samples == 0) return current_rms;
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < n_samples; ++i) {
        sum_sq += buffer[i] * buffer[i];
    }
    float block_rms_linear = sqrtf(sum_sq / n_samples);
    return (current_rms * (1.0f - alpha)) + (block_rms_linear * alpha);
}


// --- Strutture e Funzioni per Filtri Biquad ---

typedef struct {
    float a0, a1, a2, b0, b1, b2; // Coefficienti
    float z1, z2;                 // Stati precedenti
} BiquadFilter;

static void biquad_init(BiquadFilter* f) {
    f->a0 = f->a1 = f->a2 = f->b0 = f->b1 = f->b2 = 0.0f;
    f->z1 = f->z2 = 0.0f;
}

static float biquad_process(BiquadFilter* f, float in) {
    float out = in * f->b0 + f->z1;
    f->z1 = in * f->b1 + f->z2 - f->a1 * out;
    f->z2 = in * f->b2 - f->a2 * out;
    return out;
}

// Calcola i coefficienti per un filtro biquad (Low Pass o High Pass)
// freq_hz: frequenza di taglio
// q_val: fattore di qualità (risonanza)
// type: 0 per Low Pass, 1 per High Pass
static void calculate_biquad_coeffs(BiquadFilter* f, double samplerate, float freq_hz, float q_val, int type) {
    if (freq_hz <= 0.0f) freq_hz = 1.0f; // Evita divisione per zero o log(0)
    if (q_val <= 0.0f) q_val = 0.1f;    // Evita divisione per zero o Q troppo basso

    float omega = 2.0f * M_PI_F * freq_hz / samplerate;
    float sin_omega = sinf(omega);
    float cos_omega = cosf(omega);
    float alpha = sin_omega / (2.0f * q_val); // Q del filtro

    float b0, b1, b2, a0, a1, a2;

    if (type == 0) { // Low Pass Filter
        b0 = (1.0f - cos_omega) / 2.0f;
        b1 = 1.0f - cos_omega;
        b2 = (1.0f - cos_omega) / 2.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_omega;
        a2 = 1.0f - alpha;
    } else { // High Pass Filter
        b0 = (1.0f + cos_omega) / 2.0f;
        b1 = -(1.0f + cos_omega);
        b2 = (1.0f + cos_omega) / 2.0f;
        a0 = 1.0f + alpha;
        a1 = -2.0f * cos_omega;
        a2 = 1.0f - alpha;
    }

    // Normalizza i coefficienti per a0
    f->b0 = b0 / a0;
    f->b1 = b1 / a0;
    f->b2 = b2 / a0;
    f->a1 = a1 / a0;
    f->a2 = a2 / a0;
    f->a0 = 1.0f; // Questo non viene usato nel process, è solo per chiarezza
}


// Struct del plugin
typedef struct {
    // Puntatori ai parametri di controllo
    float* peak_reduction_ptr;
    float* gain_ptr;
    float* meter_ptr;
    float* bypass_ptr;
    float* ms_mode_active_ptr;
    float* ratio_mode_ptr;

    float* sc_lp_on_ptr;
    float* sc_lp_freq_ptr;
    float* sc_lp_q_ptr;
    float* sc_hp_on_ptr;
    float* sc_hp_freq_ptr;
    float* sc_hp_q_ptr;

    // Puntatori per i meter (output del plugin, input per la GUI)
    float* output_rms_ptr;
    float* gain_reduction_meter_ptr;

    // Puntatori ai buffer audio
    const float* audio_in_l_ptr;
    const float* audio_in_r_ptr;
    float* audio_out_l_ptr;
    float* audio_out_r_ptr;

    // Variabili di stato del plugin
    double samplerate;
    double oversampled_samplerate; // Nuovo
    LV2_Log_Log* log;
    LV2_Log_Logger logger;

    // Variabili di stato per l'algoritmo di compressione
    float detector_envelope_M; // Envelope del detector per Mid/Left
    float detector_envelope_S; // Envelope del detector per Side/Right

    float current_gain_M;      // Guadagno attuale per Mid/Left (lineare)
    float current_gain_S;      // Guadagno attuale per Side/Right (lineare)

    // Filtri Sidechain (6° Ordine)
    BiquadFilter sc_lp_filters_M[NUM_BIQUADS_FOR_6TH_ORDER];
    BiquadFilter sc_hp_filters_M[NUM_BIQUADS_FOR_6TH_ORDER];
    BiquadFilter sc_lp_filters_S[NUM_BIQUADS_FOR_6TH_ORDER];
    BiquadFilter sc_hp_filters_S[NUM_BIQUADS_FOR_6TH_ORDER];

    // Parametri di smoothing (alpha) pre-calcolati (variano con la ratio mode)
    float detector_attack_alpha;
    float detector_release_alpha;
    float gain_smooth_alpha; // Smoothing molto veloce per il guadagno applicato
    float rms_meter_alpha;   // Smoothing per il meter RMS di output

    // Cache per i parametri dei filtri (per evitare ricalcoli inutili)
    float last_sc_lp_freq;
    float last_sc_lp_q;
    float last_sc_hp_freq;
    float last_sc_hp_q;

    // Buffer per oversampling (per blocco di input completo)
    float* oversample_buffer_M; // Nuovo
    float* oversample_buffer_S; // Nuovo
    uint32_t oversample_buffer_size; // size = sample_count * UPSAMPLE_FACTOR // Nuovo

    // Filtri per upsampling/downsampling (Biquad di 6° Ordine) // Nuovo
    BiquadFilter upsample_lp_filters_M[NUM_BIQUADS_FOR_OS_FILTER];
    BiquadFilter upsample_lp_filters_S[NUM_BIQUADS_FOR_OS_FILTER];
    BiquadFilter downsample_lp_filters_M[NUM_BIQUADS_FOR_OS_FILTER];
    BiquadFilter downsample_lp_filters_S[NUM_BIQUADS_FOR_OS_FILTER];
    
    // Meter display
    float current_output_rms_level;
    float current_gain_reduction_display;

} Gla3a;


// Funzione di istanziazione del plugin
static LV2_Handle
instantiate(const LV2_Descriptor* descriptor,
            double                    samplerate,
            const char* bundle_path,
            const LV2_Feature* const* features) {
    Gla3a* self = (Gla3a*)calloc(1, sizeof(Gla3a));
    if (!self) return NULL;

    self->samplerate = samplerate;
    self->oversampled_samplerate = samplerate * UPSAMPLE_FACTOR; // Nuovo

    for (int i = 0; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_LOG__log)) {
            self->log = (LV2_Log_Log*)features[i]->data;
        }
    }
    lv2_log_logger_init(&self->logger, NULL, self->log);

    // Inizializzazione variabili di stato
    self->detector_envelope_M = 0.0f;
    self->detector_envelope_S = 0.0f;
    self->current_gain_M = 1.0f;
    self->current_gain_S = 1.0f;

    // Inizializzazione filtri biquad sidechain
    for(int i = 0; i < NUM_BIQUADS_FOR_6TH_ORDER; ++i) {
        biquad_init(&self->sc_lp_filters_M[i]);
        biquad_init(&self->sc_hp_filters_M[i]);
        biquad_init(&self->sc_lp_filters_S[i]);
        biquad_init(&self->sc_hp_filters_S[i]);
    }

    // Inizializzazione filtri biquad per oversampling/downsampling // Nuovo
    for(int i = 0; i < NUM_BIQUADS_FOR_OS_FILTER; ++i) {
        biquad_init(&self->upsample_lp_filters_M[i]);
        biquad_init(&self->upsample_lp_filters_S[i]);
        biquad_init(&self->downsample_lp_filters_M[i]);
        biquad_init(&self->downsample_lp_filters_S[i]);
    }

    self->rms_meter_alpha = 1.0f - expf(-1.0f / (self->samplerate * (RMS_METER_SMOOTH_MS / 1000.0f)));

    // Inizializza cache per i parametri dei filtri con valori "impossibili" per forzare il primo calcolo
    self->last_sc_lp_freq = -1.0f;
    self->last_sc_lp_q = -1.0f;
    self->last_sc_hp_freq = -1.0f;
    self->last_sc_hp_q = -1.0f;

    // Alloca buffer per oversampling // Nuovo
    self->oversample_buffer_size = 1024 * UPSAMPLE_FACTOR; // Max block size * OS_FACTOR
    self->oversample_buffer_M = (float*)calloc(self->oversample_buffer_size, sizeof(float));
    self->oversample_buffer_S = (float*)calloc(self->oversample_buffer_size, sizeof(float));

    if (!self->oversample_buffer_M || !self->oversample_buffer_S) {
        free(self->oversample_buffer_M);
        free(self->oversample_buffer_S);
        free(self);
        return NULL;
    }
    
    return (LV2_Handle)self;
}

// Funzione per connettere le porte
static void
connect_port(LV2_Handle instance, uint32_t port, void* data_location) {
    Gla3a* self = (Gla3a*)instance;

    switch ((GLA3A_PortIndex)port) {
        case GLA3A_PEAK_REDUCTION:     self->peak_reduction_ptr = (float*)data_location; break;
        case GLA3A_GAIN:               self->gain_ptr = (float*)data_location; break;
        case GLA3A_METER:              self->meter_ptr = (float*)data_location; break;
        case GLA3A_BYPASS:             self->bypass_ptr = (float*)data_location; break;
        case GLA3A_MS_MODE_ACTIVE:     self->ms_mode_active_ptr = (float*)data_location; break;
        case GLA3A_RATIO_MODE:         self->ratio_mode_ptr = (float*)data_location; break;
        case GLA3A_SC_LP_ON:           self->sc_lp_on_ptr = (float*)data_location; break;
        case GLA3A_SC_LP_FREQ:         self->sc_lp_freq_ptr = (float*)data_location; break;
        case GLA3A_SC_LP_Q:            self->sc_lp_q_ptr = (float*)data_location; break;
        case GLA3A_SC_HP_ON:           self->sc_hp_on_ptr = (float*)data_location; break;
        case GLA3A_SC_HP_FREQ:         self->sc_hp_freq_ptr = (float*)data_location; break;
        case GLA3A_SC_HP_Q:            self->sc_hp_q_ptr = (float*)data_location; break;
        case GLA3A_OUTPUT_RMS:         self->output_rms_ptr = (float*)data_location; break;
        case GLA3A_GAIN_REDUCTION_METER: self->gain_reduction_meter_ptr = (float*)data_location; break;
        case GLA3A_AUDIO_IN_L:         self->audio_in_l_ptr = (const float*)data_location; break;
        case GLA3A_AUDIO_IN_R:         self->audio_in_r_ptr = (const float*)data_location; break;
        case GLA3A_AUDIO_OUT_L:        self->audio_out_l_ptr = (float*)data_location; break;
        case GLA3A_AUDIO_OUT_R:        self->audio_out_r_ptr = (float*)data_location; break;
    }
}

// Funzione di attivazione (resettare lo stato del plugin)
static void
activate(LV2_Handle instance) {
    Gla3a* self = (Gla3a*)instance;
    self->detector_envelope_M = 0.0f;
    self->detector_envelope_S = 0.0f;
    self->current_gain_M = 1.0f;
    self->current_gain_S = 1.0f;
    self->current_output_rms_level = db_to_linear(-60.0f);
    self->current_gain_reduction_display = 0.0f;

    // Reinitalizza stati interni dei filtri biquad sidechain
    for(int i = 0; i < NUM_BIQUADS_FOR_6TH_ORDER; ++i) {
        biquad_init(&self->sc_lp_filters_M[i]);
        biquad_init(&self->sc_hp_filters_M[i]);
        biquad_init(&self->sc_lp_filters_S[i]);
        biquad_init(&self->sc_hp_filters_S[i]);
    }

    // Reinitalizza stati interni dei filtri di oversampling/downsampling // Nuovo
    for(int i = 0; i < NUM_BIQUADS_FOR_OS_FILTER; ++i) {
        biquad_init(&self->upsample_lp_filters_M[i]);
        biquad_init(&self->upsample_lp_filters_S[i]);
        biquad_init(&self->downsample_lp_filters_M[i]);
        biquad_init(&self->downsample_lp_filters_S[i]);
    }

    // Forza il ricalcolo dei coefficienti alla prossima run
    self->last_sc_lp_freq = -1.0f;
    self->last_sc_lp_q = -1.0f;
    self->last_sc_hp_freq = -1.0f;
    self->last_sc_hp_q = -1.0f;
}

// Funzione di elaborazione audio (run)
static void
run(LV2_Handle instance, uint32_t sample_count) {
    Gla3a* self = (Gla3a*)instance;

    const float* in_l = self->audio_in_l_ptr;
    const float* in_r = self->audio_in_r_ptr;
    float* out_l = self->audio_out_l_ptr;
    float* out_r = self->audio_out_r_ptr;

    const float bypass = *self->bypass_ptr;
    const float ms_mode_active = *self->ms_mode_active_ptr;
    const float ratio_mode = *self->ratio_mode_ptr;

    // Sidechain filter controls
    const float sc_lp_on = *self->sc_lp_on_ptr;
    const float sc_lp_freq = *self->sc_lp_freq_ptr;
    const float sc_lp_q = *self->sc_lp_q_ptr;
    const float sc_hp_on = *self->sc_hp_on_ptr;
    const float sc_hp_freq = *self->sc_hp_freq_ptr;
    const float sc_hp_q = *self->sc_hp_q_ptr;

    // --- Calcolo Parametri di Controllo del Compressore ---
    const float current_threshold_db = PEAK_REDUCTION_MIN_DB + (*self->peak_reduction_ptr * (PEAK_REDUCTION_MAX_DB - PEAK_REDUCTION_MIN_DB));
    const float current_threshold_linear = db_to_linear(current_threshold_db);

    const float make_up_gain_db = *self->gain_ptr * GAIN_MAX_DB;
    const float make_up_gain_linear = db_to_linear(make_up_gain_db);

    const float final_soft_clip_threshold_linear = db_to_linear(FINAL_SOFT_CLIP_THRESHOLD_DB);

    // --- Parametri di Attacco/Rilascio e Ratio in base alla modalità ---
    float current_ratio;
    float current_detector_attack_ms;
    float current_detector_release_ms;

    switch ((GLA3A_RatioMode)ratio_mode) {
        case GLA3A_RATIO_3_TO_1:
            current_ratio = 3.0f;
            current_detector_attack_ms = 10.0f; // Veloce
            current_detector_release_ms = 200.0f; // Lento
            break;
        case GLA3A_RATIO_6_TO_1:
            current_ratio = 6.0f;
            current_detector_attack_ms = 5.0f; // Più veloce
            current_detector_release_ms = 100.0f; // Medio
            break;
        case GLA3A_RATIO_9_TO_1:
            current_ratio = 9.0f;
            current_detector_attack_ms = 3.0f; // Ancora più veloce
            current_detector_release_ms = 50.0f; // Più veloce
            break;
        case GLA3A_RATIO_LIMIT: // Comportamento da Limiter
            current_ratio = 20.0f; // Ratio molto alta, quasi infinita
            current_detector_attack_ms = 1.0f; // Molto veloce
            current_detector_release_ms = 20.0f; // Veloce
            break;
    }

    // Ricomputa gli alpha in base ai nuovi tempi
    self->detector_attack_alpha = 1.0f - expf(-1.0f / (self->samplerate * (current_detector_attack_ms / 1000.0f)));
    self->detector_release_alpha = 1.0f - expf(-1.0f / (self->samplerate * (current_detector_release_ms / 1000.0f)));
    self->gain_smooth_alpha = 1.0f - expf(-1.0f / (self->samplerate * 0.001f)); // Molto veloce


    // --- Aggiornamento Coefficienti Filtri Sidechain (solo se i parametri sono cambiati) ---
    bool lp_coeffs_changed = false;
    if (fabsf(sc_lp_freq - self->last_sc_lp_freq) > 1e-6 || fabsf(sc_lp_q - self->last_sc_lp_q) > 1e-6) {
        lp_coeffs_changed = true;
        self->last_sc_lp_freq = sc_lp_freq;
        self->last_sc_lp_q = sc_lp_q;
    }

    bool hp_coeffs_changed = false;
    if (fabsf(sc_hp_freq - self->last_sc_hp_freq) > 1e-6 || fabsf(sc_hp_q - self->last_sc_hp_q) > 1e-6) {
        hp_coeffs_changed = true;
        self->last_sc_hp_freq = sc_hp_freq;
        self->last_sc_hp_q = sc_hp_q;
    }

    if (lp_coeffs_changed) {
        for(int i = 0; i < NUM_BIQUADS_FOR_6TH_ORDER; ++i) {
            calculate_biquad_coeffs(&self->sc_lp_filters_M[i], self->samplerate, sc_lp_freq, sc_lp_q, 0); // Type 0 = LP
            calculate_biquad_coeffs(&self->sc_lp_filters_S[i], self->samplerate, sc_lp_freq, sc_lp_q, 0);
        }
    }
    if (hp_coeffs_changed) {
        for(int i = 0; i < NUM_BIQUADS_FOR_6TH_ORDER; ++i) {
            calculate_biquad_coeffs(&self->sc_hp_filters_M[i], self->samplerate, sc_hp_freq, sc_hp_q, 1); // Type 1 = HP
            calculate_biquad_coeffs(&self->sc_hp_filters_S[i], self->samplerate, sc_hp_freq, sc_hp_q, 1);
        }
    }

    // --- Aggiornamento Coefficienti Filtri Oversampling (sempre un Low Pass) --- // Nuovo
    // Frequenza di taglio è Nyquist del sample rate originale, divisa per il fattore di oversampling
    float os_cutoff_freq = (self->samplerate / 2.0f) / UPSAMPLE_FACTOR;
    for(int i = 0; i < NUM_BIQUADS_FOR_OS_FILTER; ++i) {
        calculate_biquad_coeffs(&self->upsample_lp_filters_M[i], self->oversampled_samplerate, os_cutoff_freq, OS_FILTER_Q, 0); // Type 0 = LP
        calculate_biquad_coeffs(&self->upsample_lp_filters_S[i], self->oversampled_samplerate, os_cutoff_freq, OS_FILTER_Q, 0);
        calculate_biquad_coeffs(&self->downsample_lp_filters_M[i], self->oversampled_samplerate, os_cutoff_freq, OS_FILTER_Q, 0);
        calculate_biquad_coeffs(&self->downsample_lp_filters_S[i], self->oversampled_samplerate, os_cutoff_freq, OS_FILTER_Q, 0);
    }


    // --- Logica True Bypass ---
    if (bypass > 0.5f) {
        if (in_l != out_l) { memcpy(out_l, in_l, sizeof(float) * sample_count); }
        if (in_r != out_r) { memcpy(out_r, in_r, sizeof(float) * sample_count); }

        // Aggiorna l'RMS dell'output con il segnale di input in bypass
        float temp_rms_buffer_M[sample_count];
        float temp_rms_buffer_S[sample_count];
        if (ms_mode_active > 0.5f) {
            for(uint32_t j=0; j<sample_count; ++j) {
                temp_rms_buffer_M[j] = (in_l[j] + in_r[j]) * 0.5f;
                temp_rms_buffer_S[j] = (in_l[j] - in_r[j]) * 0.5f;
            }
        } else {
            memcpy(temp_rms_buffer_M, in_l, sizeof(float) * sample_count);
            memcpy(temp_rms_buffer_S, in_r, sizeof(float) * sample_count);
        }
        self->current_output_rms_level = calculate_rms_level(temp_rms_buffer_M, sample_count, self->current_output_rms_level, self->rms_meter_alpha);
        *self->output_rms_ptr = to_db(self->current_output_rms_level);
        *self->gain_reduction_meter_ptr = 0.0f; // Nessuna gain reduction in bypass
        return;
    }

    // Assicurati che il buffer di oversampling sia abbastanza grande // Nuovo
    if (sample_count * UPSAMPLE_FACTOR > self->oversample_buffer_size) {
        lv2_log_logger_error(&self->logger, "Oversample buffer too small! Reallocating...");
        // Reallocate if needed, though this should ideally be avoided in run()
        free(self->oversample_buffer_M);
        free(self->oversample_buffer_S);
        self->oversample_buffer_size = sample_count * UPSAMPLE_FACTOR;
        self->oversample_buffer_M = (float*)calloc(self->oversample_buffer_size, sizeof(float));
        self->oversample_buffer_S = (float*)calloc(self->oversample_buffer_size, sizeof(float));
        if (!self->oversample_buffer_M || !self->oversample_buffer_S) {
            lv2_log_logger_error(&self->logger, "Failed to reallocate oversample buffers!");
            // Fallback to bypass or handle error
            if (in_l != out_l) { memcpy(out_l, in_l, sizeof(float) * sample_count); }
            if (in_r != out_r) { memcpy(out_r, in_r, sizeof(float) * sample_count); }
            return;
        }
    }


    // --- Oversampling per il Blocco Corrente --- // Nuovo
    // Interleave sample e applica filtro LP di interpolazione (6° ordine)
    for (uint32_t i = 0; i < sample_count; ++i) {
        float input_l = in_l[i];
        float input_r = in_r[i];

        // Converti a M/S o resta L/R per il processing interno
        float M_original_input, S_original_input;
        if (ms_mode_active > 0.5f) {
            M_original_input = (input_l + input_r) * 0.5f;
            S_original_input = (input_l - input_r) * 0.5f;
        } else {
            M_original_input = input_l;
            S_original_input = input_r;
        }

        for (int j = 0; j < UPSAMPLE_FACTOR; ++j) {
            // Per una semplice interpolazione, ripetiamo il campione e lo filtriamo.
            // Tecniche più avanzate userebbero un FIR sinc interpolation.
            float interpolated_M = M_original_input; 
            float interpolated_S = S_original_input;

            // Apply interpolation filter (LPF)
            for(int k = 0; k < NUM_BIQUADS_FOR_OS_FILTER; ++k) {
                interpolated_M = biquad_process(&self->upsample_lp_filters_M[k], interpolated_M);
                interpolated_S = biquad_process(&self->upsample_lp_filters_S[k], interpolated_S);
            }
            
            self->oversample_buffer_M[i * UPSAMPLE_FACTOR + j] = interpolated_M;
            self->oversample_buffer_S[i * UPSAMPLE_FACTOR + j] = interpolated_S;
        }
    }

    // --- Loop di elaborazione audio sample per sample a Frequenza Campionamento Maggiore --- // Nuovo
    for (uint32_t os_idx = 0; os_idx < sample_count * UPSAMPLE_FACTOR; ++os_idx) {
        float M_audio_os = self->oversample_buffer_M[os_idx];
        float S_audio_os = self->oversample_buffer_S[os_idx];

        // --- Saturazione J-FET (applicata ad alta frequenza campionamento) ---
        M_audio_os = apply_jfet_distortion(M_audio_os, JF_K_FACTOR, JF_SATURATION_THRESHOLD, JF_DRY_WET_MIX);
        S_audio_os = apply_jfet_distortion(S_audio_os, JF_K_FACTOR, JF_SATURATION_THRESHOLD, JF_DRY_WET_MIX);

        self->oversample_buffer_M[os_idx] = M_audio_os;
        self->oversample_buffer_S[os_idx] = S_audio_os;
    }

    // --- Loop di elaborazione audio sample per sample a Frequenza Campionamento Originale ---
    for (uint32_t i = 0; i < sample_count; ++i) {
        // Prendiamo il campione oversamplato dal buffer che ha subito la distorsione
        // e lo passiamo attraverso il filtro di decimazione.
        float M_audio_pre_comp = self->oversample_buffer_M[i * UPSAMPLE_FACTOR]; 
        float S_audio_pre_comp = self->oversample_buffer_S[i * UPSAMPLE_FACTOR];

        // Applica filtro LP di decimazione (6° ordine) per anti-aliasing // Nuovo
        for(int k = 0; k < NUM_BIQUADS_FOR_OS_FILTER; ++k) {
            M_audio_pre_comp = biquad_process(&self->downsample_lp_filters_M[k], M_audio_pre_comp);
            S_audio_pre_comp = biquad_process(&self->downsample_lp_filters_S[k], S_audio_pre_comp);
        }

        // Il resto della logica del compressore opera su sample_count originale
        float M_sidechain_in = fabsf(M_audio_pre_comp); // Detector su ampiezza del segnale filtrato
        float S_sidechain_in = fabsf(S_audio_pre_comp);

        // --- FILTRAGGIO SIDECHAIN (6° Ordine) ---
        if (sc_lp_on > 0.5f) {
            for(int k = 0; k < NUM_BIQUADS_FOR_6TH_ORDER; ++k) {
                M_sidechain_in = biquad_process(&self->sc_lp_filters_M[k], M_sidechain_in);
                S_sidechain_in = biquad_process(&self->sc_lp_filters_S[k], S_sidechain_in);
            }
        }
        if (sc_hp_on > 0.5f) {
            for(int k = 0; k < NUM_BIQUADS_FOR_6TH_ORDER; ++k) {
                M_sidechain_in = biquad_process(&self->sc_hp_filters_M[k], M_sidechain_in);
                S_sidechain_in = biquad_process(&self->sc_hp_filters_S[k], S_sidechain_in);
            }
        }

        // --- COMPRESSIONE con Soft-Knee e Ratio Variabile ---
        // Canale M/Left
        if (M_sidechain_in > self->detector_envelope_M) { // Attacco
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_attack_alpha)) + (M_sidechain_in * self->detector_attack_alpha);
        } else { // Rilascio
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_release_alpha)) + (M_sidechain_in * self->detector_release_alpha);
        }

        float target_gr_db_M = 0.0f;
        float detector_env_db_M = to_db(self->detector_envelope_M);

        if (detector_env_db_M > (current_threshold_db + KNEE_WIDTH_DB)) {
            float over_threshold_db = detector_env_db_M - (current_threshold_db + KNEE_WIDTH_DB);
            target_gr_db_M = over_threshold_db * (1.0f - (1.0f / current_ratio));
        } else if (detector_env_db_M > current_threshold_db) {
            float normalized_pos_in_knee = (detector_env_db_M - current_threshold_db) / KNEE_WIDTH_DB;
            float effective_ratio_in_knee = 1.0f + (current_ratio - 1.0f) * normalized_pos_in_knee;
            target_gr_db_M = (detector_env_db_M - current_threshold_db) * (1.0f - (1.0f / effective_ratio_in_knee));
        }
        target_gr_db_M = fmaxf(0.0f, target_gr_db_M);

        float target_total_gain_M = db_to_linear(-target_gr_db_M) * make_up_gain_linear;
        self->current_gain_M = (self->current_gain_M * (1.0f - self->gain_smooth_alpha)) + (target_total_gain_M * self->gain_smooth_alpha);
        
        float processed_M = M_audio_pre_comp * self->current_gain_M;

        // Canale S/Right (duplicazione logica di compressione)
        if (S_sidechain_in > self->detector_envelope_S) { // Attacco
            self->detector_envelope_S = (self->detector_envelope_S * (1.0f - self->detector_attack_alpha)) + (S_sidechain_in * self->detector_attack_alpha);
        } else { // Rilascio
            self->detector_envelope_S = (self->detector_envelope_S * (1.0f - self->detector_release_alpha)) + (S_sidechain_in * self->detector_release_alpha);
        }

        float target_gr_db_S = 0.0f;
        float detector_env_db_S = to_db(self->detector_envelope_S);

        if (detector_env_db_S > (current_threshold_db + KNEE_WIDTH_DB)) {
            float over_threshold_db = detector_env_db_S - (current_threshold_db + KNEE_WIDTH_DB);
            target_gr_db_S = over_threshold_db * (1.0f - (1.0f / current_ratio));
        } else if (detector_env_db_S > current_threshold_db) {
            float normalized_pos_in_knee = (detector_env_db_S - current_threshold_db) / KNEE_WIDTH_DB;
            float effective_ratio_in_knee = 1.0f + (current_ratio - 1.0f) * normalized_pos_in_knee;
            target_gr_db_S = (detector_env_db_S - current_threshold_db) * (1.0f - (1.0f / effective_ratio_in_knee));
        }
        target_gr_db_S = fmaxf(0.0f, target_gr_db_S);

        float target_total_gain_S = db_to_linear(-target_gr_db_S) * make_up_gain_linear;
        self->current_gain_S = (self->current_gain_S * (1.0f - self->gain_smooth_alpha)) + (target_total_gain_S * self->gain_smooth_alpha);
        
        float processed_S = S_audio_pre_comp * self->current_gain_S;


        // --- Decodifica M/S in L/R (a valle della compressione/distorsione) ---
        float output_l, output_r;
        if (ms_mode_active > 0.5f) {
            output_l = processed_M + processed_S;
            output_r = processed_M - processed_S;
        } else {
            output_l = processed_M;
            output_r = processed_S;
        }

        // --- Soft-Clipping Finale (Limiter di Sicurezza in Output) ---
        output_l = apply_final_soft_clip(output_l, final_soft_clip_threshold_linear, FINAL_SOFT_CLIP_AMOUNT);
        output_r = apply_final_soft_clip(output_r, final_soft_clip_threshold_linear, FINAL_SOFT_CLIP_AMOUNT);

        // Scrivi i sample elaborati nei buffer di output
        out_l[i] = output_l;
        out_r[i] = output_r;
    }

    // --- Aggiornamento dei valori dei meter per l'intero blocco ---
    self->current_output_rms_level = calculate_rms_level(out_l, sample_count, self->current_output_rms_level, self->rms_meter_alpha);
    *self->output_rms_ptr = to_db(self->current_output_rms_level);

    // Calcolo della Gain Reduction Media per il meter di GR
    float actual_gr_db_M = to_db(make_up_gain_linear) - to_db(self->current_gain_M);
    float actual_gr_db_S = to_db(make_up_gain_linear) - to_db(self->current_gain_S);

    self->current_gain_reduction_display = fmaxf(0.0f, fmaxf(actual_gr_db_M, actual_gr_db_S));
    *self->gain_reduction_meter_ptr = self->current_gain_reduction_display;
}

// Funzione di pulizia
static void
cleanup(LV2_Handle instance) {
    Gla3a* self = (Gla3a*)instance;
    free(self->oversample_buffer_M); // Nuovo
    free(self->oversample_buffer_S); // Nuovo
    free(instance);
}

// Descrittore del plugin
static const LV2_Descriptor descriptor = {
    GLA3A_URI,
    instantiate,
    connect_port,
    activate,
    run,
    NULL, // deactivate
    cleanup,
    NULL // extension_data
};

// Punto di ingresso LV2
LV2_SYMBOL_EXPORT
const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    if (index == 0) {
        return &descriptor;
    }
    return NULL;
}
