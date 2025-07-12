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

    float* bypass_ptr;         // NUOVO: Puntatore per il controllo Bypass
    float* ms_mode_active_ptr; // NUOVO: Puntatore per il controllo M/S Mode

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
    // es: envelope detector state, gain computer state, filter state, etc.
    float current_output_rms;      // Stato persistente per il calcolo RMS
    float current_gain_reduction;  // Stato persistente per il meter di gain reduction

    // Parametri per il calcolo RMS (già usati in GUA76)
    float rms_alpha;

} Gla3a;

// Funzione di utilità per il calcolo RMS (copiata da GUA76)
static float calculate_rms_level(const float* buffer, uint32_t n_samples, float current_rms, float alpha) {
    float sum_sq = 0.0f;
    for (uint32_t i = 0; i < n_samples; ++i) {
        sum_sq += buffer[i] * buffer[i];
    }
    float block_rms = sqrtf(sum_sq / n_samples);
    return (current_rms * (1.0f - alpha)) + (block_rms * alpha);
}

// Funzione di utilità per convertire da lineare a dB (copiata da GUA76)
static float to_db(float linear_val) {
    if (linear_val <= 0.000000001f) return -90.0f; // Evita log(0) e valori molto bassi
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

    // Inizializzazione del logger (come in GUA76)
    for (int i = 0; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_LOG__log)) {
            self->log = (LV2_Log_Log*)features[i]->data;
        }
    }
    lv2_log_logger_init(&self->logger, NULL, self->log);

    // Inizializzazione dello stato dei meter e del filtro RMS
    self->current_output_rms = -60.0f; // Valore iniziale basso per RMS
    self->current_gain_reduction = 0.0f; // Nessuna gain reduction all'inizio
    self->rms_alpha = 1.0f - expf(-1.0f / (self->samplerate * 0.05f)); // Costante di tempo di 50ms per RMS

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
        case GLA3A_BYPASS:             self->bypass_ptr = (float*)data_location; break;         // NUOVO
        case GLA3A_MS_MODE_ACTIVE:     self->ms_mode_active_ptr = (float*)data_location; break; // NUOVO
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
    const float bypass = *self->bypass_ptr;                 // NUOVO
    const float ms_mode_active = *self->ms_mode_active_ptr; // NUOVO

    // --- Logica True Bypass ---
    // Se il bypass è attivo (valore > 0.5f, poiché è un toggle 0 o 1)
    if (bypass > 0.5f) {
        // Copia direttamente l'input all'output senza elaborazione
        if (in_l != out_l) { memcpy(out_l, in_l, sizeof(float) * sample_count); }
        if (in_r != out_r) { memcpy(out_r, in_r, sizeof(float) * sample_count); }

        // Aggiorna i meter anche in bypass (mostrando l'input RMS e GR a 0)
        *self->output_rms_ptr = to_db(calculate_rms_level(in_l, sample_count, self->current_output_rms, self->rms_alpha));
        *self->gain_reduction_meter_ptr = 0.0f; // Nessuna gain reduction in bypass
        return; // Esci dalla funzione, nessuna ulteriore elaborazione
    }

    // --- Loop di elaborazione audio sample per sample ---
    for (uint32_t i = 0; i < sample_count; ++i) {
        float mono_in_l = in_l[i];
        float mono_in_r = in_r[i];

        float M = 0.0f, S = 0.0f; // Segnali Mid e Side

        if (ms_mode_active > 0.5f) {
            // --- Codifica L/R in M/S ---
            // M = (Left + Right)
            // S = (Left - Right)
            M = (mono_in_l + mono_in_r);
            S = (mono_in_l - mono_in_r);
            // Nota: La normalizzazione (es. M *= 0.5f; S *= 0.5f;) qui dipende da come il tuo algoritmo GLA3A
            // è calibrato per i livelli. Per ora, la lasciamo non normalizzata per la massima compatibilità
            // con l'algoritmo esistente, e la normalizzazione avverrà in decodifica.
        } else {
            // Se la modalità M/S non è attiva, processa i canali Left e Right separatamente.
            // Per comodità di riutilizzo del codice di compressione, li trattiamo come M e S
            // che verranno poi decodificati direttamente come L e R.
            M = mono_in_l;
            S = mono_in_r;
        }

        // TODO: QUI VA LA TUA LOGICA DI COMPRESSIONE SPECIFICA DEL GLA3A
        // Applica i controlli (peak_reduction, gain, meter) ai segnali M e S.
        // Se `ms_mode_active` è attivo, dovrai decidere se applicare la compressione
        // a M, a S, o a entrambi in modo indipendente.
        // Se `ms_mode_active` non è attivo, `M` e `S` sono essenzialmente i tuoi canali L e R,
        // quindi la tua logica di compressione stereo standard andrà qui.

        float processed_M = M; // Placeholder: Sostituisci con la logica GLA3A per il canale Mid/Left
        float processed_S = S; // Placeholder: Sostituisci con la logica GLA3A per il canale Side/Right

        // TODO: Calcola la gain reduction per questo sample.
        // Questo valore influenzerà il self->current_gain_reduction che aggiorna il meter.
        // self->current_gain_reduction = ...; // Calcolato dalla tua logica GLA3A

        float out_sample_l, out_sample_r;
        if (ms_mode_active > 0.5f) {
            // --- Decodifica M/S in L/R ---
            // Left  = (Mid + Side) / 2
            // Right = (Mid - Side) / 2
            out_sample_l = (processed_M + processed_S) * 0.5f;
            out_sample_r = (processed_M - processed_S) * 0.5f;
        } else {
            // Se non M/S, M e S erano già i canali L e R elaborati.
            out_sample_l = processed_M;
            out_sample_r = processed_S;
        }

        out_l[i] = out_sample_l;
        out_r[i] = out_sample_r;
    }

    // --- Aggiornamento dei valori dei meter ---
    // Questi valori vengono inviati alla GUI
    *self->output_rms_ptr = to_db(calculate_rms_level(out_l, sample_count, self->current_output_rms, self->rms_alpha));
    *self->gain_reduction_meter_ptr = self->current_gain_reduction; // Assicurati che questo sia aggiornato dalla tua logica GLA3A
}

// Funzione di pulizia
static void
cleanup(LV2_Handle instance) {
    free(instance);
}

// Descrittore del plugin
static const LV2_Descriptor descriptor = {
    GLA3A_URI,       // URI del plugin
    instantiate,
    connect_port,
    activate,
    run,
    NULL,            // deactivate (opzionale)
    cleanup,
    NULL             // extension_data
};

// Punto di ingresso LV2
LV2_SYMBOL_EXPORT
const LV2_Descriptor* lv2_descriptor(uint32_t index) {
    if (index == 0) {
        return &descriptor;
    }
    return NULL;
}
