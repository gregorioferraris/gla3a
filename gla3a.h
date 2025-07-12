#ifndef GLA3A_H
#define GLA3A_H

#include <lv2/core/lv2.h>

// Definizione dell'URI del plugin.
#define GLA3A_URI "http://moddevices.com/plugins/mod-devel/gla3a"

// Definizione dell'URI della GUI.
#define GLA3A_GUI_URI "http://moddevices.com/plugins/mod-devel/gla3a_ui"

// Enum degli indici delle porte del plugin.
typedef enum {
    GLA3A_PEAK_REDUCTION = 0,
    GLA3A_GAIN = 1,
    GLA3A_METER = 2,
    GLA3A_BYPASS = 3,
    GLA3A_MS_MODE_ACTIVE = 4,
    GLA3A_RATIO_MODE = 5,
    GLA3A_SC_LP_ON = 6,          // NUOVO: On/Off LowPass Sidechain
    GLA3A_SC_LP_FREQ = 7,        // NUOVO: Frequenza LowPass Sidechain
    GLA3A_SC_LP_Q = 8,           // NUOVO: Q/Risonanza LowPass Sidechain
    GLA3A_SC_HP_ON = 9,          // NUOVO: On/Off HighPass Sidechain
    GLA3A_SC_HP_FREQ = 10,       // NUOVO: Frequenza HighPass Sidechain
    GLA3A_SC_HP_Q = 11,          // NUOVO: Q/Risonanza HighPass Sidechain
    GLA3A_OUTPUT_RMS = 12,
    GLA3A_GAIN_REDUCTION_METER = 13,
    GLA3A_AUDIO_IN_L = 14,
    GLA3A_AUDIO_IN_R = 15,
    GLA3A_AUDIO_OUT_L = 16,
    GLA3A_AUDIO_OUT_R = 17
} GLA3A_PortIndex;

// Enum per le modalità di ratio (per chiarezza nel codice C++)
typedef enum {
    GLA3A_RATIO_3_TO_1 = 0,
    GLA3A_RATIO_6_TO_1 = 1,
    GLA3A_RATIO_9_TO_1 = 2,
    GLA3A_RATIO_LIMIT  = 3
} GLA3A_RatioMode;

#endif // GLA3A_H
