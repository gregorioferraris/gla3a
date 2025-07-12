\    #ifndef GLA3A_H
#define GLA3A_H

#include <lv2/core/lv2.h> // Richiesto per tipi LV2 come LV2_URID

// Definizione dell'URI del plugin.
// DEVE corrispondere a quanto dichiarato in gla3a.lv2/manifest.ttl e gla3a.lv2/gla3a.ttl
#define GLA3A_URI "http://moddevices.com/plugins/mod-devel/gla3a"

// Definizione dell'URI della GUI.
// DEVE corrispondere a quanto dichiarato in gla3a.lv2/manifest.ttl e in gla3a_gui.cpp
#define GLA3A_GUI_URI "http://moddevices.com/plugins/mod-devel/gla3a_ui"

// Enum degli indici delle porte del plugin.
// Questi indici DEVONO corrispondere esattamente agli indici definiti in gla3a.lv2/gla3a.ttl
typedef enum {
    GLA3A_PEAK_REDUCTION = 0,
    GLA3A_GAIN = 1,
    GLA3A_METER = 2,
    GLA3A_BYPASS = 3,               // NUOVO: Controllo True Bypass
    GLA3A_MS_MODE_ACTIVE = 4,       // NUOVO: Controllo per attivare/disattivare la modalità Mid/Side
    GLA3A_OUTPUT_RMS = 5,           // Porta di output per il meter RMS
    GLA3A_GAIN_REDUCTION_METER = 6, // Porta di output per il meter di Gain Reduction
    GLA3A_AUDIO_IN_L = 7,           // Ingresso audio canale Left
    GLA3A_AUDIO_IN_R = 8,           // Ingresso audio canale Right
    GLA3A_AUDIO_OUT_L = 9,          // Uscita audio canale Left
    GLA3A_AUDIO_OUT_R = 10          // Uscita audio canale Right
} GLA3A_PortIndex;

// Puoi aggiungere qui altre definizioni o forward declarations se necessario.

#endif // GLA3A_H
