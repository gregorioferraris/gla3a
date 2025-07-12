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

// --- Sidechain Filter (Shelf) ---
#define SIDECHAIN_HF_FREQ_MIN 200.0f  // Frequenza minima per il filtro HF (Hz)
#define SIDECHAIN_HF_FREQ_MAX 10000.0f // Frequenza massima per il filtro HF (Hz)
#define SIDECHAIN_HF_GAIN_DB 6.0f     // Guadagno massimo del filtro shelf in dB

// --- J-FET Distortion ---
#define JF_K_FACTOR 2.0f // Fattore di "durezza" della distorsione J-FET (regola il carattere)
#define JF_DRY_WET_MIX 0.3f // Mix tra segnale pulito e distorto dal J-FET (0.0 a 1.0)
#define JF_SATURATION_THRESHOLD 0.5f // Soglia (lineare) oltre la quale la distorsione J-FET è più evidente

// --- Soft-Clipping Finale (Limiter) ---
#define FINAL_SOFT_CLIP_THRESHOLD_DB -1.0f // Inizia il soft-clip finale a -1 dBFS
#define FINAL_SOFT_CLIP_AMOUNT 0.5f        // Quanto è "soft" il clip finale (0.0 a 1.0, 1.0 è hard clip)


// --- Funzioni di Utilità ---

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
// 'k' controlla la pendenza/durezza della curva
static float apply_jfet_distortion(float sample, float k_factor, float threshold_jfet) {
    // Normalizza il sample in base a una soglia per controllare dove inizia la "curva"
    float normalized_sample = sample / threshold_jfet;
    
    // Funzione di shaping tipo arctan o tanh per modellare il J-FET
    // Questa è una semplice funzione sigmoide, puoi sperimentare altre come atan(k*x)
    float distorted_sample = normalized_sample / (1.0f + fabsf(normalized_sample * k_factor));
    
    // Ri-scala al range originale
    return distorted_sample * threshold_jfet;
}


// Struttura per un filtro IIR di primo ordine (per sidechain)
typedef struct {
    float a0, b1; // Coefficienti
    float z1;     // Stato precedente
} OnePoleFilter;

static void one_pole_filter_init(OnePoleFilter* f) {
    f->a0 = 0.0f;
    f->b1 = 0.0f;
    f->z1 = 0.0f;
}

static float one_pole_filter_process(OnePoleFilter* f, float in) {
    float out = in * f->a0 + f->z1;
    f->z1 = in * f->b1;
    return out;
}

// Calcola i coefficienti per un filtro shelf di primo ordine (per sidechain)
// freq: frequenza di taglio in Hz
// gain: guadagno dello shelf in dB
static void calculate_shelf_coeffs(OnePoleFilter* f, double samplerate, float freq, float gain_db) {
    float gain_linear = db_to_linear(gain_db);
    float omega = 2.0f * M_PI_F * freq / samplerate;
    float alpha = sinf(omega) / (2.0f * cosf(omega)); // Semplificazione per shelf

    if (gain_db >= 0) { // High Shelf (boost)
        f->b1 = (1.0f - alpha * sqrtf(gain_linear)) / (1.0f + alpha * sqrtf(gain_linear));
        f->a0 = (1.0f + f->b1) / 2.0f;
    } else { // High Shelf (cut)
        gain_linear = 1.0f / gain_linear; // Inverti il gain per il cut
        f->b1 = (1.0f - alpha * sqrtf(gain_linear)) / (1.0f + alpha * sqrtf(gain_linear));
        f->a0 = (1.0f + f->b1) / 2.0f; // E' approssimativo, per un cut potresti voler un passa alto semplice
        // Alternativa per passa-alto semplice:
        // float x = expf(-2.0f * M_PI_F * freq / samplerate);
        // f->a0 = (1.0f + x) * 0.5f;
        // f->b1 = -(1.0f + x) * 0.5f; // Per passa-alto
    }
}


// Struct del plugin
typedef struct {
    // Puntatori ai parametri di controllo
    float* peak_reduction_ptr;
    float* gain_ptr;
    float* meter_ptr;
    float* bypass_ptr;
    float* ms_mode_active_ptr;
    float* sidechain_hf_freq_ptr; // Nuovo
    float* ratio_mode_ptr;        // Nuovo

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

    // Filtri Sidechain
    OnePoleFilter sc_filter_M; // Filtro sidechain per canale Mid/Left
    OnePoleFilter sc_filter_S; // Filtro sidechain per canale Side/Right

    // Parametri di smoothing (alpha) pre-calcolati (variano con la ratio mode)
    float detector_attack_alpha;
    float detector_release_alpha;
    float gain_smooth_alpha; // Smoothing molto veloce per il guadagno applicato
    float rms_meter_alpha;   // Smoothing per il meter RMS di output

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

    one_pole_filter_init(&self->sc_filter_M);
    one_pole_filter_init(&self->sc_filter_S);

    self->rms_meter_alpha = 1.0f - expf(-1.0f / (self->samplerate * (50.0f / 1000.0f))); // 50ms per meter RMS

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
        case GLA3A_SIDECHAIN_HF_FREQ:  self->sidechain_hf_freq_ptr = (float*)data_location; break;
        case GLA3A_RATIO_MODE:         self->ratio_mode_ptr = (float*)data_location; break;
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

    // Reinitalizza i filtri sidechain
    one_pole_filter_init(&self->sc_filter_M);
    one_pole_filter_init(&self->sc_filter_S);
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

    // --- Calcolo Parametri di Controllo ---
    const float current_threshold_db = PEAK_REDUCTION_MIN_DB + (*self->peak_reduction_ptr * (PEAK_REDUCTION_MAX_DB - PEAK_REDUCTION_MIN_DB));
    const float current_threshold_linear = db_to_linear(current_threshold_db);

    const float make_up_gain_db = *self->gain_ptr * GAIN_MAX_DB;
    const float make_up_gain_linear = db_to_linear(make_up_gain_db);

    const float final_soft_clip_threshold_linear = db_to_linear(FINAL_SOFT_CLIP_THRESHOLD_DB);

    // --- Parametri di Attacco/Rilascio e Ratio in base alla modalità ---
    float current_ratio;
    float current_detector_attack_ms = DETECTOR_ATTACK_FACTOR_BASE;
    float current_detector_release_ms = DETECTOR_RELEASE_FACTOR_BASE;

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

    // Parametri Sidechain Filter
    float sidechain_freq_hz = SIDECHAIN_HF_FREQ_MIN + (*self->sidechain_hf_freq_ptr * (SIDECHAIN_HF_FREQ_MAX - SIDECHAIN_HF_FREQ_MIN));
    calculate_shelf_coeffs(&self->sc_filter_M, self->samplerate, sidechain_freq_hz, SIDECHAIN_HF_GAIN_DB);
    self->sc_filter_S.a0 = self->sc_filter_M.a0; // Copia per simmetria
    self->sc_filter_S.b1 = self->sc_filter_M.b1;


    // --- Logica True Bypass ---
    if (bypass > 0.5f) {
        if (in_l != out_l) { memcpy(out_l, in_l, sizeof(float) * sample_count); }
        if (in_r != out_r) { memcpy(out_r, in_r, sizeof(float) * sample_count); }

        self->current_output_rms_level = calculate_rms_level(in_l, sample_count, self->current_output_rms_level, self->rms_meter_alpha);
        *self->output_rms_ptr = to_db(self->current_output_rms_level);
        *self->gain_reduction_meter_ptr = 0.0f; // Nessuna gain reduction in bypass
        return;
    }

    // --- Loop di elaborazione audio sample per sample ---
    for (uint32_t i = 0; i < sample_count; ++i) {
        float input_l = in_l[i];
        float input_r = in_r[i];

        float M_audio = 0.0f, S_audio = 0.0f; // Segnali audio per l'elaborazione (Mid/Left e Side/Right)
        float M_sidechain = 0.0f, S_sidechain = 0.0f; // Segnali per la sidechain

        if (ms_mode_active > 0.5f) {
            // Codifica L/R in M/S per l'audio principale
            M_audio = (input_l + input_r) * 0.5f; // Normalizza subito i segnali M/S
            S_audio = (input_l - input_r) * 0.5f;

            // Codifica L/R in M/S per la sidechain (se la sidechain usa M/S)
            M_sidechain = (input_l + input_r) * 0.5f;
            S_sidechain = (input_l - input_r) * 0.5f;
        } else {
            // Se non M/S, Left e Right sono i canali indipendenti
            M_audio = input_l;
            S_audio = input_r;

            M_sidechain = input_l;
            S_sidechain = input_r;
        }

        // --- FILTRAGGIO SIDECHAIN (J-FET) ---
        // Applica il filtro shelving ai segnali della sidechain
        M_sidechain = one_pole_filter_process(&self->sc_filter_M, fabsf(M_sidechain)); // Usiamo l'assoluto per il detector
        S_sidechain = one_pole_filter_process(&self->sc_filter_S, fabsf(S_sidechain));

        // --- COMPRESSIONE con Soft-Knee e Ratio Variabile ---
        // Canale M/Left
        // 1. Aggiorna l'envelope del detector (basato sul segnale sidechain filtrato)
        if (M_sidechain > self->detector_envelope_M) { // Attacco
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_attack_alpha)) + (M_sidechain * self->detector_attack_alpha);
        } else { // Rilascio
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_release_alpha)) + (M_sidechain * self->detector_release_alpha);
        }

        // 2. Calcola la gain reduction desiderata con soft-knee
        float target_gr_db_M = 0.0f; // Gain Reduction in dB
        float detector_env_db_M = to_db(self->detector_envelope_M);

        if (detector_env_db_M > (current_threshold_db + KNEE_WIDTH_DB)) {
            // Sopra la knee completa: compressione con rapporto fisso
            float over_threshold_db = detector_env_db_M - (current_threshold_db + KNEE_WIDTH_DB);
            target_gr_db_M = over_threshold_db * (1.0f - (1.0f / current_ratio));
        } else if (detector_env_db_M > current_threshold_db) {
            // Dentro la knee: soft-knee compression
            float normalized_pos_in_knee = (detector_env_db_M - current_threshold_db) / KNEE_WIDTH_DB;
            // Interpola il rapporto da 1:1 a 'current_ratio'
            float effective_ratio_in_knee = 1.0f + (current_ratio - 1.0f) * normalized_pos_in_knee;
            target_gr_db_M = (detector_env_db_M - current_threshold_db) * (1.0f - (1.0f / effective_ratio_in_knee));
        }
        
        // Clampa la gain reduction per evitare gain negativo (boost) o eccessiva riduzione
        target_gr_db_M = fmaxf(0.0f, target_gr_db_M); // La GR deve essere >= 0dB (attenuazione)

        // Converti la GR in dB a un fattore lineare e applica make-up gain
        float target_total_gain_M = db_to_linear(-target_gr_db_M) * make_up_gain_linear;
        
        // Smooth (rampa) il guadagno applicato per evitare clicks/pops
        self->current_gain_M = (self->current_gain_M * (1.0f - self->gain_smooth_alpha)) + (target_total_gain_M * self->gain_smooth_alpha);
        
        float processed_M = M_audio * self->current_gain_M;


        // Canale S/Right (Logica identica al canale M/Left)
        if (S_sidechain > self->detector_envelope_S) {
            self->detector_envelope_S = (self->detector_envelope_S * (1.0f - self->detector_attack_alpha)) + (S_sidechain * self->detector_attack_alpha);
        } else {
            self->detector_envelope_S = (self->detector_envelope_S * (1.0f - self->detector_release_alpha)) + (S_sidechain * self->detector_release_alpha);
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

        float processed_S = S_audio * self->current_gain_S;


        // --- DISTORSIONE J-FET (NON LINEARITÀ E ARMONICHE) ---
        // Applica la distorsione J-FET dopo la compressione e prima della decodifica/clipping finale
        // Questo simula la colorazione del circuito.
        processed_M = apply_jfet_distortion(processed_M, JF_K_FACTOR, JF_SATURATION_THRESHOLD);
        processed_S = apply_jfet_distortion(processed_S, JF_K_FACTOR, JF_SATURATION_THRESHOLD);

        // Mixare con il segnale "dry" per controllare l'intensità della distorsione
        processed_M = processed_M * JF_DRY_WET_MIX + M_audio * (1.0f - JF_DRY_WET_MIX);
        processed_S = processed_S * JF_DRY_WET_MIX + S_audio * (1.0f - JF_DRY_WET_MIX);


        // --- Decodifica M/S in L/R (a valle della compressione/distorsione) ---
        float output_l, output_r;
        if (ms_mode_active > 0.5f) {
            output_l = processed_M + processed_S; // Già normalizzati 0.5 all'inizio
            output_r = processed_M - processed_S;
        } else {
            output_l = processed_M;
            output_r = processed_S;
        }

        // --- Soft-Clipping Finale (Limiter di Sicurezza) ---
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
    // Qui prendiamo il valore in dB della GR effettiva
    float actual_gr_db_M = to_db(make_up_gain_linear) - to_db(self->current_gain_M);
    float actual_gr_db_S = to_db(make_up_gain_linear) - to_db(self->current_gain_S);

    // Il meter mostra la GR del canale che sta riducendo di 
