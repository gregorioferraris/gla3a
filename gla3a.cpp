#include "gla3a.h" // Assicurati di includere il tuo file .h
#include <lv2/core/lv2.h>
#include <lv2/log/logger.h>
#include <lv2/log/log.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h> // Per memcpy

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

    // TODO: Aggiungi qui le variabili di stato specifiche per il tuo algoritmo di compressione GLA3A
    // Ogni canale (L/R o M/S) avrà probabilmente bisogno del proprio set di variabili di stato
    // Ad esempio, per un compressore ottico come il LA-3A, avrai bisogno di variabili
    // per il detector di envelope, la gain reduction applicata, ecc. per ogni canale.
    // Esempio:
    // float gr_detector_state_M;
    // float gr_detector_state_S;
    // float current_gain_M;
    // float current_gain_S;


    float current_output_rms;
    float current_gain_reduction;

    // Parametri per il calcolo RMS
    float rms_alpha;

} Gla3a;

// Funzione di utilità per il calcolo RMS
static float calculate_rms_level(const float* buffer, uint32_t n_samples, float current_rms, float alpha) {
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < n_samples; ++i) {
        sum_sq += buffer[i] * buffer[i];
    }
    float block_rms = sqrtf(sum_sq / n_samples);
    return (current_rms * (1.0f - alpha)) + (block_rms * alpha);
}

// Funzione di utilità per convertire da lineare a dB
static float to_db(float linear_val) {
    if (linear_val <= 0.000000001f) return -90.0f;
    return 20.0f * log10f(linear_val);
}

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

    // Inizializzazione dello stato dei meter e del filtro RMS
    self->current_output_rms = -60.0f;
    self->current_gain_reduction = 0.0f;
    self->rms_alpha = 1.0f - expf(-1.0f / (self->samplerate * 0.05f));

    // TODO: Inizializza qui anche le variabili di stato per la compressione (es. detector state)
    // self->gr_detector_state_M = 0.0f;
    // self->gr_detector_state_S = 0.0f;
    // self->current_gain_M = 1.0f;
    // self->current_gain_S = 1.0f;

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
    self->current_output_rms = -60.0f;
    self->current_gain_reduction = 0.0f;
    // TODO: Resetta anche le variabili di stato per la compressione qui
    // self->gr_detector_state_M = 0.0f;
    // self->gr_detector_state_S = 0.0f;
    // self->current_gain_M = 1.0f;
    // self->current_gain_S = 1.0f;
}

// Funzione di elaborazione audio (run)
static void
run(LV2_Handle instance, uint32_t sample_count) {
    Gla3a* self = (Gla3a*)instance;

    // Puntatori ai buffer audio
    const float* in_l = self->audio_in_l_ptr;
    const float* in_r = self->audio_in_r_ptr;
    float* out_l = self->audio_out_l_ptr;
    float* out_r = self->audio_out_r_ptr;

    // Valori dei parametri di controllo
    const float bypass = *self->bypass_ptr;
    const float ms_mode_active = *self->ms_mode_active_ptr;

    // --- Logica True Bypass ---
    if (bypass > 0.5f) {
        if (in_l != out_l) { memcpy(out_l, in_l, sizeof(float) * sample_count); }
        if (in_r != out_r) { memcpy(out_r, in_r, sizeof(float) * sample_count); }

        *self->output_rms_ptr = to_db(calculate_rms_level(in_l, sample_count, self->current_output_rms, self->rms_alpha));
        *self->gain_reduction_meter_ptr = 0.0f; // Nessuna gain reduction in bypass
        return;
    }

    // --- Loop di elaborazione audio sample per sample ---
    for (uint32_t i = 0; i < sample_count; ++i) {
        float input_l = in_l[i];
        float input_r = in_r[i];

        float M = 0.0f, S = 0.0f; // Segnali Mid e Side per l'elaborazione

        if (ms_mode_active > 0.5f) {
            // --- Codifica L/R in M/S (a monte della compressione) ---
            // M = (Left + Right)
            // S = (Left - Right)
            // Nota: Spesso per M/S si applica una normalizzazione iniziale, es. M*=0.5, S*=0.5
            // per mantenere i livelli simili, e poi non si normalizza in decodifica.
            // Qui usiamo la forma standard che normalizza in decodifica.
            M = (input_l + input_r);
            S = (input_l - input_r);
        } else {
            // Se la modalità M/S non è attiva, processa i canali Left e Right separatamente.
            // Per comodità, li trattiamo come "M" e "S" individuali per l'algoritmo di compressione,
            // che poi verranno scritti direttamente in output_l e output_r.
            M = input_l;
            S = input_r;
        }

        // --- APPLICA LA TUA LOGICA DI COMPRESSIONE GLA3A QUI ---
        // Ora `M` e `S` contengono i segnali da comprimere.
        // Se `ms_mode_active` è attivo:
        //    M è il segnale Mid, S è il segnale Side. Applica la compressione a questi due.
        // Se `ms_mode_active` NON è attivo:
        //    M è il segnale Left, S è il segnale Right. Applica la compressione a questi due.

        float processed_M; // Output del tuo compressore per il canale M/Left
        float processed_S; // Output del tuo compressore per il canale S/Right

        // Implementa qui la logica del compressore GLA3A
        // Esempio (sostituisci con il tuo vero algoritmo):
        // float peak_reduction_val = *self->peak_reduction_ptr;
        // float gain_val = *self->gain_ptr;
        //
        // // Logica di Gain Reduction per M (ad esempio, basata su un detector di envelope)
        // float gr_M = calculate_gain_reduction(M, peak_reduction_val, self->gr_detector_state_M, self->samplerate);
        // processed_M = M * (1.0f - gr_M) * gain_val; // Esempio semplificato
        //
        // // Logica di Gain Reduction per S
        // float gr_S = calculate_gain_reduction(S, peak_reduction_val, self->gr_detector_state_S, self->samplerate);
        // processed_S = S * (1.0f - gr_S) * gain_val; // Esempio semplificato
        //
        // self->current_gain_reduction = (gr_M + gr_S) * 0.5f; // Media per il meter (o scegli il massimo)


        // <<< Inserisci la tua logica GLA3A qui per calcolare processed_M e processed_S >>>
        // Per ora, i valori passano attraverso senza compressione:
        processed_M = M;
        processed_S = S;
        self->current_gain_reduction = 0.0f; // Placeholder, aggiorna questo con il tuo calcolo reale

        // --- Decodifica M/S in L/R (a valle della compressione) ---
        float output_l, output_r;
        if (ms_mode_active > 0.5f) {
            // Left  = (Mid_processed + Side_processed) / 2
            // Right = (Mid_processed - Side_processed) / 2
            output_l = (processed_M + processed_S) * 0.5f;
            output_r = (processed_M - processed_S) * 0.5f;
        } else {
            // Se non M/S, processed_M e processed_S erano già i canali L e R
            output_l = processed_M;
            output_r = processed_S;
        }

        out_l[i] = output_l;
        out_r[i] = output_r;
    }

    // --- Aggiornamento dei valori dei meter ---
    *self->output_rms_ptr = to_db(calculate_rms_level(out_l, sample_count, self->current_output_rms, self->rms_alpha));
    // self->current_gain_reduction deve essere aggiornato dalla tua logica di compressione
    *self->gain_reduction_meter_ptr = self->current_gain_reduction;
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
