#include "lv2/ui/ui.h"
#include "lv2/atom/atom.h"
#include "lv2/atom/util.h"
#include "lv2/urid/urid.h"
#include "lv2/midi/midi.h" // Se gestisci MIDI nella GUI
#include "lv2/log/log.h"
#include "lv2/log/logger.h"

#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Slider.H>
#include <FL/Fl_Value_Slider.H> // Per slider con valore numerico
#include <FL/Fl_Group.H> // Per raggruppare elementi

#include <iostream> // Per debug con std::cerr
#include <vector>

// Definizione dell'URI della GUI (DEVE CORRISPONDERE all'URI della GUI nel .ttl)
#define GLA3A_GUI_URI "http://moddevices.com/plugins/mod-devel/gla3a_ui"

// Definizione dell'URI del plugin (DEVE CORRISPONDERE ALL'URI DEL PLUGIN NEL .ttl E .cpp)
#define GLA3A_PLUGIN_URI "http://moddevices.com/plugins/mod-devel/gla3a"

// Colori in scala di grigi
#define GREY_BACKGROUND FL_DARK_GREY - 1 // Grigio scuro per lo sfondo
#define GREY_MID       FL_DARK_GREY + 1 // Grigio medio per i pannelli
#define GREY_LIGHT     FL_DARK_GREY + 3 // Grigio chiaro per i controlli attivi
#define GREY_TEXT      FL_WHITE        // Testo bianco su sfondo scuro

// Struttura dei dati della GUI
typedef struct {
    LV2_URID_Map* map;
    LV2_Log_Logger* logger;
    LV2_Atom_Forge  forge;
    LV2_URID        midi_MidiEvent; // Se gestisci MIDI
    LV2_URID        atom_float;     // Per atom::Float

    // Finestra FLTK
    Fl_Window* window;

    // Controlli FLTK (Slider e Box per etichette)
    Fl_Value_Slider* input_gain_slider;
    Fl_Value_Slider* threshold_slider;
    Fl_Value_Slider* ratio_slider;
    Fl_Value_Slider* attack_slider;
    Fl_Value_Slider* release_slider;

    // NUOVI Controlli J-FET
    Fl_Group* jfet_group; // Gruppo per i controlli J-FET
    Fl_Value_Slider* jfet_drive_slider;
    Fl_Value_Slider* jfet_vp_slider;
    Fl_Value_Slider* jfet_idss_slider;
    Fl_Value_Slider* jfet_output_slider;


    // Puntatori alle porte del plugin (per inviare valori dalla GUI)
    float* input_gain_port;
    float* threshold_port;
    float* ratio_port;
    float* attack_port;
    float* release_port;
    float* jfet_drive_port;
    float* jfet_vp_port;
    float* jfet_idss_port;
    float* jfet_output_port;


    // Funzione per inviare i messaggi host (per aggiornare il plugin)
    LV2UI_Write_Function write_function;
    LV2UI_Controller     controller;

    uint32_t     input_gain_urid;
    uint32_t     threshold_urid;
    uint32_t     ratio_urid;
    uint32_t     attack_urid;
    uint32_t     release_urid;
    uint32_t     jfet_drive_urid;
    uint32_t     jfet_vp_urid;
    uint32_t     jfet_idss_urid;
    uint32_t     jfet_output_urid;

} GLA3AUi;


// Callback per gli slider: Invia il valore al plugin
static void slider_cb(Fl_Widget* w, void* data) {
    GLA3AUi* ui = (GLA3AUi*)data;
    Fl_Value_Slider* slider = (Fl_Value_Slider*)w;

    uint32_t port_urid = 0;
    float* port_value_ptr = NULL;

    // Determina quale slider è stato mosso e quale URID e puntatore usare
    if (slider == ui->input_gain_slider) {
        port_urid = ui->input_gain_urid;
        port_value_ptr = ui->input_gain_port;
    } else if (slider == ui->threshold_slider) {
        port_urid = ui->threshold_urid;
        port_value_ptr = ui->threshold_port;
    } else if (slider == ui->ratio_slider) {
        port_urid = ui->ratio_urid;
        port_value_ptr = ui->ratio_port;
    } else if (slider == ui->attack_slider) {
        port_urid = ui->attack_urid;
        port_value_ptr = ui->attack_port;
    } else if (slider == ui->release_slider) {
        port_urid = ui->release_urid;
        port_value_ptr = ui->release_port;
    } else if (slider == ui->jfet_drive_slider) {
        port_urid = ui->jfet_drive_urid;
        port_value_ptr = ui->jfet_drive_port;
    } else if (slider == ui->jfet_vp_slider) {
        port_urid = ui->jfet_vp_urid;
        port_value_ptr = ui->jfet_vp_port;
    } else if (slider == ui->jfet_idss_slider) {
        port_urid = ui->jfet_idss_urid;
        port_value_ptr = ui->jfet_idss_port;
    } else if (slider == ui->jfet_output_slider) {
        port_urid = ui->jfet_output_urid;
        port_value_ptr = ui->jfet_output_port;
    }

    // Invia il valore al plugin via LV2 atom message
    if (port_urid != 0 && port_value_ptr != NULL) {
        // Aggiorna il valore locale (utile per la logica interna della GUI)
        *port_value_ptr = slider->value();

        // Forge un messaggio atomico per inviare il valore
        uint8_t buffer[1024]; // Buffer per il messaggio atomico
        lv2_atom_forge_set_buffer(&ui->forge, buffer, sizeof(buffer));

        LV2_Atom_Forge_Frame frame;
        lv2_atom_forge_init(&ui->forge);
        lv2_atom_forge_object(&ui->forge, &frame, 0, ui->map->map(ui->map->handle, LV2_UI__atomTransfer));
        lv2_atom_forge_key(&ui->forge, port_urid);
        lv2_atom_forge_float(&ui->forge, *port_value_ptr);
        lv2_atom_forge_pop(&ui->forge, &frame);

        ui->write_function(ui->controller, 0, lv2_atom_forge_size(&ui->forge), ui->map->map(ui->map->handle, LV2_ATOM__Chunk), buffer);
    }
}


// Funzione per istanziare la GUI
static LV2UI_Handle instantiate(const LV2UI_Descriptor* descriptor,
                                const char* plugin_uri,
                                const char* bundle_path,
                                LV2UI_Write_Function      write_function,
                                LV2UI_Controller          controller,
                                LV2UI_Port_Subscribe_Function subscribe_function,
                                const LV2_Feature* const* features) {
    // Verifica che sia la GUI per il nostro plugin
    if (strcmp(plugin_uri, GLA3A_PLUGIN_URI) != 0) {
        std::cerr << "GLA3A GUI: Plugin URI mismatch." << std::endl;
        return NULL;
    }

    GLA3AUi* ui = (GLA3AUi*)calloc(1, sizeof(GLA3AUi));
    if (!ui) return NULL;

    ui->write_function = write_function;
    ui->controller     = controller;

    // Ottieni i feature necessari
    for (int i = 0; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_URID__map)) {
            ui->map = (LV2_URID_Map*)features[i]->data;
        } else if (!strcmp(features[i]->URI, LV2_LOG__log)) {
            ui->logger = (LV2_Log_Logger*)features[i]->data;
        }
    }

    if (!ui->map) {
        std::cerr << "GLA3A GUI: Host does not provide URID map." << std::endl;
        free(ui);
        return NULL;
    }

    // Inizializza l'atom forge e gli URID
    lv2_atom_forge_init(&ui->forge);
    ui->midi_MidiEvent = ui->map->map(ui->map->handle, LV2_MIDI__MidiEvent);
    ui->atom_float = ui->map->map(ui->map->handle, LV2_ATOM__Float);

    // Mappa gli URID delle porte di controllo
    ui->input_gain_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#input_gain");
    ui->threshold_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#threshold");
    ui->ratio_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#ratio");
    ui->attack_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#attack");
    ui->release_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#release");
    // Nuovi URID per i parametri J-FET
    ui->jfet_drive_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#jfet_drive");
    ui->jfet_vp_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#jfet_vp");
    ui->jfet_idss_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#jfet_idss");
    ui->jfet_output_urid = ui->map->map(ui->map->handle, "http://moddevices.com/plugins/mod-devel/gla3a#jfet_output");


    // --- Creazione della Finestra e dei Controlli FLTK ---
    Fl::scheme("gtk+"); // Schema per un look moderno
    ui->window = new Fl_Window(400, 300, "GLA3A Compressor"); // Larghezza, Altezza, Titolo
    ui->window->color(GREY_BACKGROUND); // Sfondo della finestra

    // Gruppo per i controlli del Compressore
    Fl_Group* comp_group = new Fl_Group(10, 10, 180, 280, "Compressor");
    comp_group->box(FL_ENGRAVED_BOX);
    comp_group->color(GREY_MID);
    comp_group->labelcolor(GREY_TEXT);
    comp_group->align(FL_ALIGN_TOP | FL_ALIGN_INSIDE);
    comp_group->begin();

    // Input Gain
    ui->input_gain_slider = new Fl_Value_Slider(30, 40, 30, 200, "Input Gain");
    ui->input_gain_slider->type(FL_VERTICAL_SLIDER);
    ui->input_gain_slider->range(0.0, 1.0);
    ui->input_gain_slider->value(0.5); // Valore di default
    ui->input_gain_slider->callback(slider_cb, ui);
    ui->input_gain_slider->color(GREY_LIGHT);
    ui->input_gain_slider->textcolor(GREY_TEXT);

    // Threshold
    ui->threshold_slider = new Fl_Value_Slider(70, 40, 30, 200, "Threshold");
    ui->threshold_slider->type(FL_VERTICAL_SLIDER);
    ui->threshold_slider->range(0.0, 1.0);
    ui->threshold_slider->value(0.5);
    ui->threshold_slider->callback(slider_cb, ui);
    ui->threshold_slider->color(GREY_LIGHT);
    ui->threshold_slider->textcolor(GREY_TEXT);

    // Ratio
    ui->ratio_slider = new Fl_Value_Slider(110, 40, 30, 200, "Ratio");
    ui->ratio_slider->type(FL_VERTICAL_SLIDER);
    ui->ratio_slider->range(1.0, 10.0);
    ui->ratio_slider->value(2.0);
    ui->ratio_slider->callback(slider_cb, ui);
    ui->ratio_slider->color(GREY_LIGHT);
    ui->ratio_slider->textcolor(GREY_TEXT);

    // Attack
    ui->attack_slider = new Fl_Value_Slider(150, 40, 30, 200, "Attack (ms)");
    ui->attack_slider->type(FL_VERTICAL_SLIDER);
    ui->attack_slider->range(1.0, 500.0);
    ui->attack_slider->value(10.0);
    ui->attack_slider->callback(slider_cb, ui);
    ui->attack_slider->color(GREY_LIGHT);
    ui->attack_slider->textcolor(GREY_TEXT);

    // Release
    ui->release_slider = new Fl_Value_Slider(190, 40, 30, 200, "Release (ms)"); // Adatta la posizione
    ui->release_slider->type(FL_VERTICAL_SLIDER);
    ui->release_slider->range(10.0, 2000.0);
    ui->release_slider->value(200.0);
    ui->release_slider->callback(slider_cb, ui);
    ui->release_slider->color(GREY_LIGHT);
    ui->release_slider->textcolor(GREY_TEXT);

    comp_group->end(); // Fine gruppo Compressore

    // --- Gruppo per i Controlli J-FET (nuovo) ---
    ui->jfet_group = new Fl_Group(200, 10, 190, 280, "J-FET Stage");
    ui->jfet_group->box(FL_ENGRAVED_BOX);
    ui->jfet_group->color(GREY_MID);
    ui->jfet_group->labelcolor(GREY_TEXT);
    ui->jfet_group->align(FL_ALIGN_TOP | FL_ALIGN_INSIDE);
    ui->jfet_group->begin();

    // J-FET Drive
    ui->jfet_drive_slider = new Fl_Value_Slider(220, 40, 30, 200, "J-FET Drive");
    ui->jfet_drive_slider->type(FL_VERTICAL_SLIDER);
    ui->jfet_drive_slider->range(0.1, 10.0);
    ui->jfet_drive_slider->value(1.0);
    ui->jfet_drive_slider->callback(slider_cb, ui);
    ui->jfet_drive_slider->color(GREY_LIGHT);
    ui->jfet_drive_slider->textcolor(GREY_TEXT);

    // J-FET Vp
    ui->jfet_vp_slider = new Fl_Value_Slider(260, 40, 30, 200, "Vp (V)");
    ui->jfet_vp_slider->type(FL_VERTICAL_SLIDER);
    ui->jfet_vp_slider->range(-5.0, -0.5);
    ui->jfet_vp_slider->value(-2.0);
    ui->jfet_vp_slider->callback(slider_cb, ui);
    ui->jfet_vp_slider->color(GREY_LIGHT);
    ui->jfet_vp_slider->textcolor(GREY_TEXT);

    // J-FET Idss
    ui->jfet_idss_slider = new Fl_Value_Slider(300, 40, 30, 200, "Idss (A)");
    ui->jfet_idss_slider->type(FL_VERTICAL_SLIDER);
    ui->jfet_idss_slider->range(0.0001, 0.05);
    ui->jfet_idss_slider->value(0.005);
    ui->jfet_idss_slider->callback(slider_cb, ui);
    ui->jfet_idss_slider->color(GREY_LIGHT);
    ui->jfet_idss_slider->textcolor(GREY_TEXT);
    ui->jfet_idss_slider->format("%.4f"); // Formatta per mostrare decimali

    // J-FET Output
    ui->jfet_output_slider = new Fl_Value_Slider(340, 40, 30, 200, "J-FET Output");
    ui->jfet_output_slider->type(FL_VERTICAL_SLIDER);
    ui->jfet_output_slider->range(0.0, 2.0);
    ui->jfet_output_slider->value(1.0);
    ui->jfet_output_slider->callback(slider_cb, ui);
    ui->jfet_output_slider->color(GREY_LIGHT);
    ui->jfet_output_slider->textcolor(GREY_TEXT);

    ui->jfet_group->end(); // Fine gruppo J-FET

    ui->window->end(); // Fine della finestra FLTK

    // Mostra la finestra FLTK
    ui->window->show();

    // Ritorna il puntatore all'interfaccia utente
    return ui;
}

// Funzione per ripulire la GUI
static void cleanup(LV2UI_Handle handle) {
    GLA3AUi* ui = (GLA3AUi*)handle;
    delete ui->window; // Elimina la finestra (e tutti i suoi widget)
    free(ui);
}

// Funzione per ridimensionare la GUI (se il host lo supporta)
static int resize(LV2UI_Handle handle, int width, int height) {
    GLA3AUi* ui = (GLA3AUi*)handle;
    ui->window->resize(0, 0, width, height);
    return 0; // 0 significa successo
}


// Funzione per la ricezione di messaggi dal plugin (per aggiornare la GUI)
static void port_event(LV2UI_Handle handle,
                       uint32_t     port_index,
                       uint32_t     buffer_size,
                       uint32_t     format,
                       const void* buffer) {
    GLA3AUi* ui = (GLA3AUi*)handle;
    // std::cerr << "Port Event received for port " << port_index << std::endl; // Debug

    // Questo è il modo in cui il plugin invia i valori aggiornati alla GUI
    // Ipotizziamo che il plugin invii valori float sulle porte di controllo
    if (format == ui->atom_float) {
        float value = *(const float*)buffer;
        
        switch (port_index) {
            case 0: ui->input_gain_slider->value(value); break;
            case 1: ui->threshold_slider->value(value); break;
            case 2: ui->ratio_slider->value(value); break;
            case 3: ui->attack_slider->value(value); break;
            case 4: ui->release_slider->value(value); break;
            case 5: ui->jfet_drive_slider->value(value); break;
            case 6: ui->jfet_vp_slider->value(value); break;
            case 7: ui->jfet_idss_slider->value(value); break;
            case 8: ui->jfet_output_slider->value(value); break;
            // Aggiungi altri case per le nuove porte J-FET
            default: break;
        }
    }
}

// Descrittore della GUI
static const LV2UI_Descriptor descriptor = {
    GLA3A_GUI_URI,      // URI della GUI (deve essere unico)
    instantiate,        // Funzione di istanziazione
    cleanup,            // Funzione di pulizia
    port_event,         // Funzione per la ricezione di eventi dalle porte
    NULL,               // idle: non usiamo una funzione idle per aggiornamenti costanti
    resize              // Funzione per il ridimensionamento
};

// Funzione di ingresso (entry point) della GUI
LV2UI_Descriptor const* lv2ui_descriptor(uint32_t index) {
    switch (index) {
        case 0: return &descriptor;
        default: return NULL;
    }
}
