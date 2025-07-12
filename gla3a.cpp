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
static float apply_jfet_distortion(float sample, float k_factor, float threshold_jfet) {
    float normalized_sample = sample / threshold_jfet;
    float distorted_sample = normalized_sample / (1.0f + fabsf(normalized_sample * k_factor));
    return distorted_sample * threshold_jfet;
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

    float* sc_lp_on_ptr;   // Nuovo
    float* sc_lp_freq_ptr; // Nuovo
    float* sc_lp_q_ptr;    // Nuovo
    float* sc_hp_on_ptr;   // Nuovo
    float* sc_hp_freq_ptr; // Nuovo
    float* sc_hp_q_ptr;    // Nuovo

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

    // Inizializzazione filtri biquad
    for(int i = 0; i < NUM_BIQUADS_FOR_6TH_ORDER; ++i) {
        biquad_init(&self->sc_lp_filters_M[i]);
        biquad_init(&self->sc_hp_filters_M[i]);
        biquad_init(&self->sc_lp_filters_S[i]);
        biquad_init(&self->sc_hp_filters_S[i]);
    }

    self->rms_meter_alpha = 1.0f - expf(-1.0f / (self->samplerate * (RMS_METER_SMOOTH_MS / 1000.0f)));

    // Inizializza cache per i parametri dei filtri con valori "impossibili" per forzare il primo calcolo
    self->last_sc_lp_freq = -1.0f;
    self->last_sc_lp_q = -1.0f;
    self->last_sc_hp_freq = -1.0f;
    self->last_sc_hp_q = -1.0f;

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

    // Reinitalizza stati interni dei filtri biquad
    for(int i = 0; i < NUM_BIQUADS_FOR_6TH_ORDER; ++i) {
        biquad_init(&self->sc_lp_filters_M[i]);
        biquad_init(&self->sc_hp_filters_M[i]);
        biquad_init(&self->sc_lp_filters_S[i]);
        biquad_init(&self->sc_hp_filters_S[i]);
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

    // --- Loop di elaborazione audio sample per sample ---
    for (uint32_t i = 0; i < sample_count; ++i) {
        float input_l = in_l[i];
        float input_r = in_r[i];

        float M_audio = 0.0f, S_audio = 0.0f; // Segnali audio per l'elaborazione (Mid/Left e Side/Right)
        float M_sidechain_in = 0.0f, S_sidechain_in = 0.0f; // Segnali di input per la sidechain

        if (ms_mode_active > 0.5f) {
            M_audio = (input_l + input_r) * 0.5f; // Normalizza subito i segnali M/S
            S_audio = (input_l - input_r) * 0.5f;

            M_sidechain_in = (input_l + input_r) * 0.5f;
            S_sidechain_in = (input_l - input_r) * 0.5f;
        } else {
            M_audio = input_l;
            S_audio = input_r;

            M_sidechain_in = input_l;
            S_sidechain_in = input_r;
        }

        // --- FILTRAGGIO SIDECHAIN (6° Ordine) ---
        float M_sidechain_filtered = fabsf(M_sidechain_in); // Detector su ampiezza
        float S_sidechain_filtered = fabsf(S_sidechain_in);

        if (sc_lp_on > 0.5f) {
            for(int k = 0; k < NUM_BIQUADS_FOR_6TH_ORDER; ++k) {
                M_sidechain_filtered = biquad_process(&self->sc_lp_filters_M[k], M_sidechain_filtered);
                S_sidechain_filtered = biquad_process(&self->sc_lp_filters_S[k], S_sidechain_filtered);
            }
        }
        if (sc_hp_on > 0.5f) {
            for(int k = 0; k < NUM_BIQUADS_FOR_6TH_ORDER; ++k) {
                M_sidechain_filtered = biquad_process(&self->sc_hp_filters_M[k], M_sidechain_filtered);
                S_sidechain_filtered = biquad_process(&self->sc_hp_filters_S[k], S_sidechain_filtered);
            }
        }

        // --- COMPRESSIONE con Soft-Knee e Ratio Variabile ---
        // Canale M/Left
        if (M_sidechain_filtered > self->detector_envelope_M) { // Attacco
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_attack_alpha)) + (M_sidechain_filtered * self->detector_attack_alpha);
        } else { // Rilascio
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_release_alpha)) + (M_sidechain_filtered * self->detector_release_alpha);
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
        
        float processed_M = M_audio * self->current_gain_M;


 
