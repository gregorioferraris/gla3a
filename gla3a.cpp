#include "gla3a.h" // Assicurati di includere il tuo file .h
#include <lv2/core/lv2.h>
#include <lv2/log/logger.h>
#include <lv2/log/log.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h> // Per memcpy

// Costanti utili
#define M_PI_F 3.14159265358979323846f

// --- Parametri di calibrazione dell'algoritmo ---
// Tempi del detector (simil-ottico, da calibrare)
#define DETECTOR_ATTACK_MS 10.0f  // Tempo di attacco del detector (ms)
#define DETECTOR_RELEASE_MS 50.0f // Tempo di rilascio del detector (ms)

// Caratteristiche della Knee
#define KNEE_WIDTH_DB 10.0f     // Larghezza della soft-knee in dB
#define THRESHOLD_MIN_DB -60.0f // Soglia minima quando Peak Reduction è 0
#define THRESHOLD_MAX_DB -10.0f // Soglia massima quando Peak Reduction è 1 (meno compressione)

// Make-up Gain
#define MAKEUP_GAIN_MAX_DB 12.0f // Guadagno massimo applicabile in dB

// Soft-Clipping
#define SOFT_CLIP_THRESHOLD_DB -2.0f // Inizia il soft-clip a -2 dBFS
#define SOFT_CLIP_AMOUNT 0.5f        // Quanto è "soft" il clip (da 0.0 a 1.0, 1.0 è hard clip)

// RMS Meter Smoothing
#define RMS_SMOOTH_MS 50.0f // Tempo in ms per la costante di tempo RMS del meter

// Funzione di utilità per il calcolo RMS
static float calculate_rms_level(const float* buffer, uint32_t n_samples, float current_rms, float alpha) {
    if (n_samples == 0) return current_rms;
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < n_samples; ++i) {
        sum_sq += buffer[i] * buffer[i];
    }
    float block_rms_linear = sqrtf(sum_sq / n_samples);
    return (current_rms * (1.0f - alpha)) + (block_rms_linear * alpha);
}

// Funzione di utilità per convertire da lineare a dB
static float to_db(float linear_val) {
    if (linear_val <= 0.00000000001f) return -90.0f; // Evita log(0) e valori molto bassi
    return 20.0f * log10f(linear_val);
}

// Funzione di utilità per convertire da dB a lineare
static float db_to_linear(float db_val) {
    return powf(10.0f, db_val / 20.0f);
}

// Funzione per applicare il soft-clipping
// Fonte di ispirazione: algorithms like the ArcTan soft-clipper, or similar cubic shapers
static float apply_soft_clip(float sample, float threshold_linear, float amount) {
    if (threshold_linear <= 0.000000001f) return sample; // Avoid division by zero or invalid threshold

    float sign = (sample >= 0) ? 1.0f : -1.0f;
    float abs_sample = fabsf(sample);

    if (abs_sample <= threshold_linear) {
        return sample; // Nessun clipping sotto soglia
    } else {
        // Applica una curva smoother sopra la soglia
        float normalized_over_threshold = (abs_sample - threshold_linear) / (1.0f - threshold_linear);
        // Usiamo una funzione smoothstep (o simile) per una transizione graduale
        // Questa è una versione semplificata che si avvicina a tanh(x)
        float clipped_val = threshold_linear + (1.0f - threshold_linear) * (1.0f - expf(-amount * normalized_over_threshold));
        return sign * fminf(clipped_val, 1.0f); // Clampa a 1.0 per evitare overshoot
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
    float detector_envelope_M; // Envelope del detector per il canale Mid/Left
    float detector_envelope_S; // Envelope del detector per il canale Side/Right

    float current_gain_M; // Guadagno attuale applicato (include make-up e GR) per Mid/Left (lineare)
    float current_gain_S; // Guadagno attuale applicato (include make-up e GR) per Side/Right (lineare)

    // Variabili di stato per i meter
    float current_output_rms_level;      // Stato persistente per il calcolo RMS dell'output (lineare)
    float current_gain_reduction_display; // Stato persistente per il meter di gain reduction (visualizzato in dB)

    // Parametri di smoothing (alpha) pre-calcolati al samplerate
    float rms_alpha;
    float detector_attack_alpha;
    float detector_release_alpha;
    float gain_smooth_alpha; // Per lo smoothing del guadagno applicato (per prevenire artefatti)


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

    // Inizializzazione del logger
    for (int i = 0; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_LOG__log)) {
            self->log = (LV2_Log_Log*)features[i]->data;
        }
    }
    lv2_log_logger_init(&self->logger, NULL, self->log);

    // Inizializzazione delle variabili di stato del compressore
    self->detector_envelope_M = 0.0f;
    self->detector_envelope_S = 0.0f;
    self->current_gain_M = 1.0f; // Guadagno iniziale 0dB
    self->current_gain_S = 1.0f; // Guadagno iniziale 0dB

    // Inizializzazione dello stato dei meter
    self->current_output_rms_level = db_to_linear(-60.0f); // Inizializza a un livello basso in lineare
    self->current_gain_reduction_display = 0.0f; // Nessuna gain reduction all'inizio (in dB)

    // Calcolo dei fattori di smoothing (alpha) basati sul sample rate
    self->rms_alpha = 1.0f - expf(-1.0f / (self->samplerate * (RMS_SMOOTH_MS / 1000.0f)));
    self->detector_attack_alpha = 1.0f - expf(-1.0f / (self->samplerate * (DETECTOR_ATTACK_MS / 1000.0f)));
    self->detector_release_alpha = 1.0f - expf(-1.0f / (self->samplerate * (DETECTOR_RELEASE_MS / 1000.0f)));
    self->gain_smooth_alpha = 1.0f - expf(-1.0f / (self->samplerate * 0.001f)); // Smoothing molto veloce per il gain applicato

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
    // Resetta lo stato interno del compressore e dei meter
    self->detector_envelope_M = 0.0f;
    self->detector_envelope_S = 0.0f;
    self->current_gain_M = 1.0f;
    self->current_gain_S = 1.0f;
    self->current_output_rms_level = db_to_linear(-60.0f);
    self->current_gain_reduction_display = 0.0f;
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

    // Parametri di controllo convertiti in range utile
    // Peak Reduction: controlla la soglia di compressione.
    // Il valore 0.0 -> THRESHOLD_MIN_DB (più compressione)
    // Il valore 1.0 -> THRESHOLD_MAX_DB (meno compressione)
    const float current_threshold_db = THRESHOLD_MIN_DB + (*self->peak_reduction_ptr * (THRESHOLD_MAX_DB - THRESHOLD_MIN_DB));
    const float current_threshold_linear = db_to_linear(current_threshold_db);

    // Gain: make-up gain
    const float make_up_gain_db = *self->gain_ptr * MAKEUP_GAIN_MAX_DB;
    const float make_up_gain_linear = db_to_linear(make_up_gain_db);

    // Soft-clip threshold
    const float soft_clip_threshold_linear = db_to_linear(SOFT_CLIP_THRESHOLD_DB);

    // --- Logica True Bypass ---
    if (bypass > 0.5f) {
        if (in_l != out_l) { memcpy(out_l, in_l, sizeof(float) * sample_count); }
        if (in_r != out_r) { memcpy(out_r, in_r, sizeof(float) * sample_count); }

        self->current_output_rms_level = calculate_rms_level(in_l, sample_count, self->current_output_rms_level, self->rms_alpha);
        *self->output_rms_ptr = to_db(self->current_output_rms_level);
        *self->gain_reduction_meter_ptr = 0.0f; // Nessuna gain reduction in bypass
        return;
    }

    // --- Loop di elaborazione audio sample per sample ---
    for (uint32_t i = 0; i < sample_count; ++i) {
        float input_l = in_l[i];
        float input_r = in_r[i];

        float M_signal = 0.0f, S_signal = 0.0f; // Segnali per l'elaborazione (Mid/Left e Side/Right)

        if (ms_mode_active > 0.5f) {
            // --- Codifica L/R in M/S (a monte della compressione) ---
            M_signal = (input_l + input_r);
            S_signal = (input_l - input_r);
        } else {
            // Se la modalità M/S non è attiva, processa i canali Left e Right separatamente.
            M_signal = input_l; // Tratta Left come "Mid" per la pipeline di elaborazione
            S_signal = input_r; // Tratta Right come "Side" per la pipeline di elaborazione
        }

        // --- APPLICA LA LOGICA DI COMPRESSIONE GLA3A (Soft-Knee) ---
        // Ogni canale (M_signal, S_signal) è processato in modo indipendente.

        // Canale M/Left
        // 1. Aggiorna l'envelope del detector (usando valori assoluti)
        float abs_M_signal = fabsf(M_signal);
        if (abs_M_signal > self->detector_envelope_M) { // Attacco
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_attack_alpha)) + (abs_M_signal * self->detector_attack_alpha);
        } else { // Rilascio
            self->detector_envelope_M = (self->detector_envelope_M * (1.0f - self->detector_release_alpha)) + (abs_M_signal * self->detector_release_alpha);
        }

        // 2. Calcola la gain reduction desiderata con soft-knee
        float gain_reduction_factor_M;
        float input_db_M = to_db(self->detector_envelope_M);

        if (input_db_M < current_threshold_db) {
            // Sotto la knee: nessuna compressione (gain reduction 0dB / factor 1.0)
            gain_reduction_factor_M = 1.0f;
        } else if (input_db_M > (current_threshold_db + KNEE_WIDTH_DB)) {
            // Sopra la knee completa: compressione con rapporto fisso (es. ~3:1 per LA-3A)
            // Un rapporto fisso di 3:1 in dB significa che ogni 3dB in più sull'input, 1dB in più sull'output.
            // O, in termini di gain reduction, G = 1/R. GR = 1 - (1/R)
            // Se G = Vin/Vout, R = delta_Vin/delta_Vout. Per la GR, è 1 / (Gain dopo compressione)
            // L'LA-3A è un compressore di guadagno variabile, quindi il rapporto non è fisso.
            // Qui emuliamo un rapporto che si intensifica sopra la knee.
            float diff_db = input_db_M - (current_threshold_db + KNEE_WIDTH_DB);
            // Questo è un'approssimazione del rapporto non lineare.
            // Un 3:1 fisso sarebbe db_out = db_knee_end + diff_db / 3.0f;
            // gain_reduction_db = db_in - db_out;
            // gain_reduction_db = diff_db - (diff_db / 3.0f) = diff_db * (2.0f/3.0f);
            float target_gr_db = diff_db * (2.0f / 3.0f); // ~3:1 ratio for simplicity
            gain_reduction_factor_M = db_to_linear(-target_gr_db);
        } else {
            // Dentro la knee: soft-knee compression
            float normalized_pos_in_knee = (input_db_M - current_threshold_db) / KNEE_WIDTH_DB;
            // Una curva smoother per la knee (es. quadratica o sigmoidale)
            // Qui usiamo una interpolazione lineare del rapporto. Rapporto va da 1:1 a ~3:1
            float ratio_interp = 1.0f + (2.0f * normalized_pos_in_knee); // Da 1 a 3
            float target_gr_db = (input_db_M - current_threshold_db) * (1.0f - (1.0f / ratio_interp));
            gain_reduction_factor_M = db_to_linear(-target_gr_db);
        }

        // 3. Smooth (rampa) il guadagno applicato per evitare clicks/pops
        float target_gain_M = gain_reduction_factor_M * make_up_gain_linear;
        self->current_gain_M = (self->current_gain_M * (1.0f - self->gain_smooth_alpha)) + (target_gain_M * self->gain_smooth_alpha);

        float processed_M = M_signal * self->current_gain_M;


        // Canale S/Right (Logica identica al canale M/Left)
        float abs_S_signal = fabsf(S_signal);
        if (abs_S_signal > self->detector_envelope_S) {
            self->detector_envelope_S = (self->detector_envelope_S * (1.0f - self->detector_attack_alpha)) + (abs_S_signal * self->detector_attack_alpha);
        } else {
            self->detector_envelope_S = (self->detector_envelope_S * (1.0f - self->detector_release_alpha)) + (abs_S_signal * self->detector_release_alpha);
        }

        float gain_reduction_factor_S;
        float input_db_S = to_db(self->detector_envelope_S);

        if (input_db_S < current_threshold_db) {
            gain_reduction_factor_S = 1.0f;
        } else if (input_db_S > (current_threshold_db + KNEE_WIDTH_DB)) {
            float diff_db = input_db_S - (current_threshold_db + KNEE_WIDTH_DB);
            float target_gr_db = diff_db * (2.0f / 3.0f);
            gain_reduction_factor_S = db_to_linear(-target_gr_db);
        } else {
            float normalized_pos_in_knee = (input_db_S - current_threshold_db) / KNEE_WIDTH_DB;
            float ratio_interp = 1.0f + (2.0f * normalized_pos_in_knee);
            float target_gr_db = (input_db_S - current_threshold_db) * (1.0f - (1.0f / ratio_interp));
            gain_reduction_factor_S = db_to_linear(-target_gr_db);
        }

        float target_gain_S = gain_reduction_factor_S * make_up_gain_linear;
        self->current_gain_S = (self->current_gain_S * (1.0f - self->gain_smooth_alpha)) + (target_gain_S * self->gain_smooth_alpha);

        float processed_S = S_signal * self->current_gain_S;

        // --- Decodifica M/S in L/R (a valle della compressione) ---
        float output_l, output_r;
        if (ms_mode_active > 0.5f) {
            output_l = (processed_M + processed_S) * 0.5f;
            output_r = (processed_M - processed_S) * 0.5f;
        } else {
            output_l = processed_M;
            output_r = processed_S;
        }

        // --- Applica Soft-Clipping all'output (a valle della decodifica L/R) ---
        output_l = apply_soft_clip(output_l, soft_clip_threshold_linear, SOFT_CLIP_AMOUNT);
        output_r = apply_soft_clip(output_r, soft_clip_threshold_linear, SOFT_CLIP_AMOUNT);

        // Scrivi i sample elaborati nei buffer di output
        out_l[i] = output_l;
        out_r[i] = output_r;
    }

    // --- Aggiornamento dei valori dei meter per l'intero blocco ---
    self->current_output_rms_level = calculate_rms_level(out_l, sample_count, self->current_output_rms_level, self->rms_alpha);
    *self->output_rms_ptr = to_db(self->current_output_rms_level);

    // Calcolo della Gain Reduction Media per il meter di GR
    // La GR è 1.0 / guadagno applicato (quindi 1.0 / self->current_gain_M)
    // Se self->current_gain_M include già il make-up gain, dobbiamo isolare solo la GR.
    // Oppure, più semplice, misuriamo la differenza in dB tra input ed output.
    // Per semplicità qui, prendiamo la media delle gain reduction derivate dai fattori di guadagno attuali.
    // current_gain_M / make_up_gain_linear = fattore di riduzione dovuto alla compressione
    float gr_factor_M_only = self->current_gain_M / make_up_gain_linear;
    float gr_factor_S_only = self->current_gain_S / make_up_gain_linear;

    // Prendiamo il valore più grande di riduzione di guadagno applicato (più "negativo" in dB)
    float avg_gr_factor = fminf(gr_factor_M_only, gr_factor_S_only); // O una media fminf(fminf(gr_M, gr_S), 1.0f)

    // Se non c'è compressione, avg_gr_factor sarà circa 1.0, quindi GR sarà 0dB.
    *self->gain_reduction_meter_ptr = to_db(avg_gr_factor);
}

// Funzione di pulizia
static void
cleanup(LV2_Handle instance) {
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
