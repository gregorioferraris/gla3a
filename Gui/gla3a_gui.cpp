#include <lv2/ui/ui.h>
#include <lv2/atom/atom.h>
#include <lv2/atom/util.h>
#include <lv2/urid/urid.h>
#include <lv2/core/lv2.h>
#include <lv2/log/log.h>
#include <lv2/log/logger.h>

// --- DIPENDENZE IMGUI ---
#include "imgui.h"
#include "backends/imgui_impl_opengl3.h"
#include <cmath>     // Per funzioni matematiche
#include <chrono>    // Per la gestione del tempo
#include <vector>    // Per std::vector
#include <string>    // Per std::string
#include <iostream>  // Per cout/cerr (debug)

// --- DIPENDENZE STB_IMAGE (per caricare immagini) ---
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h" // Assicurati che questo header sia nel tuo INCLUDE path

// --- DIPENDENZE PER IL CONTESTO OPENGL (Linux/X11) ---
#include <GL/gl.h>
#include <GL/glx.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>   // Per XGetWindowAttributes
#include <X11/keysym.h>  // Per la gestione della tastiera

// --- URID e LOGGING (definizione esterna, come nel tuo codice LV2) ---
LV2_LOG_Logger logger;
LV2_URID_Map urid_map;
LV2_URID_Unmap urid_unmap;

// =========================================================================
// URI dei Parametri (Devono corrispondere esattamente al tuo plugin audio)
// =========================================================================
// URI per i parametri del tuo compressore Gla3a (modificati per LA-3A)
static LV2_URID peakReduction_URID; // Gain Reduction (Peak Reduction)
static LV2_URID gain_URID;          // Gain (Output Gain)
static LV2_URID hfComp_URID;        // High Frequency Compression (tipico LA-3A)
static LV2_URID bypass_URID;
static LV2_URID ratioMode_URID;     // Per 1:1 o Compress/Limit (se applicabile, o rimuovere se fisso)
static LV2_URID inputPad10dB_URID;  // Per l'Input Pad (se presente)
static LV2_URID oversamplingOn_URID;
static LV2_URID sidechainMode_URID; // External Sidechain
static LV2_URID scLpOn_URID;
static LV2_URID scLpFq_URID;
static LV2_URID scLpQ_URID;
static LV2_URID scHpOn_URID;
static LV2_URID scHpFq_URID;
static LV2_URID scHpQ_URID;

// URI per i valori dei meter (che il plugin audio invierà alla GUI)
static LV2_URID peakGR_URID;       // Gain Reduction Meter
static LV2_URID peakInL_URID;      // Input Peak Left
static LV2_URID peakInR_URID;      // Input Peak Right
static LV2_URID peakOutL_URID;     // Output Peak Left
static LV2_URID peakOutR_URID;     // Output Peak Right


// =========================================================================
// Funzioni Helper per ImGui
// =========================================================================

static double get_time_in_seconds() {
    static auto start_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = current_time - start_time;
    return elapsed.count();
}

GLuint LoadTextureFromFile(const char* filename, int* out_width, int* out_height)
{
    int image_width = 0;
    int image_height = 0;
    unsigned char* image_data = stbi_load(filename, &image_width, &image_height, NULL, 4);
    if (image_data == NULL) {
        lv2_log_error(&logger, "Error: Could not load texture from file: %s\n", filename);
        return 0;
    }

    GLuint image_texture;
    glGenTextures(1, &image_texture);
    glBindTexture(GL_TEXTURE_2D, image_texture);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data);

    stbi_image_free(image_data);

    *out_width = image_width;
    *out_height = image_height;

    return image_texture;
}

bool KnobRotaryImage(const char* label, float* p_value, float v_min, float v_max,
                     GLuint texture_id, int frame_width, int frame_height, int total_frames,
                     ImVec2 knob_size_pixels, const char* format = "%.2f")
{
    ImGuiWindow* window = ImGui::GetCurrentWindow();
    if (window->SkipItems)
        return false;

    ImGuiContext& g = *GImGui;
    const ImGuiStyle& style = g.Style;
    const ImGuiID id = window->GetID(label);

    ImVec2 pos = window->DC.CursorPos;
    ImRect bb(pos, ImVec2(pos.x + knob_size_pixels.x, pos.y + knob_size_pixels.y + ImGui::GetTextLineHeightWithSpacing()));

    ImGui::ItemSize(bb); // Riserva spazio per il knob e il label
    if (!ImGui::ItemAdd(bb, id))
        return false;

    const bool hovered = ImGui::ItemHoverable(bb, id);
    bool value_changed = false;
    bool held = ImGui::IsItemActive();

    if (hovered && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        if (!held) ImGui::SetActiveID(id, window);
    }

    if (held) {
        float delta_y = ImGui::GetIO().MouseDelta.y;
        float speed = (v_max - v_min) / (knob_size_pixels.y * 2.0f);
        *p_value -= delta_y * speed;
        *p_value = ImClamp(*p_value, v_min, v_max);
        value_changed = true;

        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            ImGui::ClearActiveID();
        }
    }

    if (hovered && ImGui::GetIO().MouseWheel != 0.0f) {
        float wheel_speed = (v_max - v_min) / 50.0f;
        *p_value += ImGui::GetIO().MouseWheel * wheel_speed;
        *p_value = ImClamp(*p_value, v_min, v_max);
        value_changed = true;
    }

    float normalized_value = (*p_value - v_min) / (v_max - v_min);
    int frame_index = static_cast<int>(normalized_value * (total_frames - 1));
    frame_index = ImClamp(frame_index, 0, total_frames - 1);

    ImVec2 uv0 = ImVec2(0.0f, (float)frame_index / total_frames);
    ImVec2 uv1 = ImVec2(1.0f, (float)(frame_index + 1) / total_frames);

    ImGui::GetWindowDrawList()->AddImage((ImTextureID)(intptr_t)texture_id,
                                         bb.Min, ImVec2(bb.Min.x + knob_size_pixels.x, bb.Min.y + knob_size_pixels.y),
                                         uv0, uv1);

    ImGui::SetCursorScreenPos(ImVec2(bb.Min.x, bb.Min.y + knob_size_pixels.y + style.ItemInnerSpacing.y));
    ImGui::Text(label);

    char value_buf[64];
    ImFormatString(value_buf, IM_ARRAYSIZE(value_buf), format, *p_value);
    ImGui::SetCursorScreenPos(ImVec2(bb.Min.x, bb.Min.y + knob_size_pixels.y + style.ItemInnerSpacing.y + ImGui::GetTextLineHeight()));
    ImGui::Text(value_buf);

    return value_changed;
}


// =========================================================================
// Struttura dello Stato della UI
// =========================================================================
typedef struct {
    LV2_UI_Write_Function write_function;
    LV2_UI_Controller controller;
    LV2_Log_Logger* logger;
    LV2_URID_Map* map;
    LV2_URID_Unmap* unmap;

    Display* display;
    Window window;
    GLXContext glx_context;

    bool imgui_initialized;

    // --- Valori dei Parametri (sincronizzati con il plugin audio) ---
    float peakReduction_val;
    float gain_val;
    float hfComp_val; // High Frequency Compression
    bool bypass_val;
    bool ratioMode_val; // false=Comp, true=Limit (se il tuo LA-3A ha questo toggle)
    bool inputPad10dB_val;
    bool oversamplingOn_val;
    bool sidechainMode_val; // External Sidechain
    bool scLpOn_val;
    float scLpFq_val;
    float scLpQ_val;
    bool scHpOn_val;
    float scHpFq_val;
    float scHpQ_val;

    // --- Valori dei Meter (ricevuti dal plugin, normalizzati 0.0-1.0) ---
    float peakGR_val;
    float peakInL_val;
    float peakInR_val;
    float peakOutL_val;
    float peakOutR_val;

    bool showOutputMeter; // Toggle per mostrare input/output sul meter principale

    // --- Texture IDs per i Knob ---
    GLuint knobTextureID_peakReduction;
    GLuint knobTextureID_gain;
    GLuint knobTextureID_hfComp; // Knob per HF Comp
    GLuint knobTextureID_scLpFq;
    GLuint knobTextureID_scLpQ;
    GLuint knobTextureID_scHpFq;
    GLuint knobTextureID_scHpQ;

    // --- Dimensione e numero di frame per i Knob ---
    int knobFrameWidth;
    int knobFrameHeight;
    int knobTotalFrames;

    // --- Texture IDs per i Pulsanti Toggle ---
    GLuint toggleSwitchTextureID_on;
    GLuint toggleSwitchTextureID_off;
    int toggleSwitchWidth;
    int toggleSwitchHeight;


} Gla3aUI;


// =========================================================================
// Callbacks LV2 (instantiate, cleanup, port_event, ui_idle)
// =========================================================================

static LV2_UI_Handle instantiate(const LV2_UI_Descriptor* descriptor,
                                 const char* plugin_uri,
                                 const char* bundle_path,
                                 LV2_UI_Write_Function    write_function,
                                 LV2_UI_Controller        controller,
                                 LV2_UI_Widget_Handle* widget,
                                 const LV2_Feature* const* features) {
    Gla3aUI* ui = (Gla3aUI*)calloc(1, sizeof(Gla3aUI));
    ui->write_function = write_function;
    ui->controller = controller;
    ui->imgui_initialized = false;
    ui->showOutputMeter = true;

    // Inizializza i valori predefiniti dei parametri (devono corrispondere a quelli del plugin)
    ui->peakReduction_val = -20.0f;
    ui->gain_val = 0.0f;
    ui->hfComp_val = 0.0f; // Valore iniziale per HF Comp
    ui->bypass_val = false;
    ui->ratioMode_val = false; // Compressione (se hai questo toggle nel tuo LA-3A)
    ui->inputPad10dB_val = false;
    ui->oversamplingOn_val = true;
    ui->sidechainMode_val = false;
    ui->scLpOn_val = false;
    ui->scLpFq_val = 2000.0f;
    ui->scLpQ_val = 0.707f;
    ui->scHpOn_val = false;
    ui->scHpFq_val = 100.0f;
    ui->scHpQ_val = 0.707f;

    // Inizializza i valori dei meter (0.0 = -Inf dB, usa -60dB o simile come base)
    ui->peakGR_val = 0.0f;
    ui->peakInL_val = -60.0f;
    ui->peakInR_val = -60.0f;
    ui->peakOutL_val = -60.0f;
    ui->peakOutR_val = -60.0f;

    // Estrai le feature necessarie
    for (int i = 0; features[i]; ++i) {
        if (!strcmp(features[i]->URI, LV2_URID__map)) {
            ui->map = (LV2_URID_Map*)features[i]->data;
            urid_map = *ui->map;
        } else if (!strcmp(features[i]->URI, LV2_URID__unmap)) {
            ui->unmap = (LV2_URID_Unmap*)features[i]->data;
            urid_unmap = *ui->unmap;
        } else if (!strcmp(features[i]->URI, LV2_LOG__log)) {
            ui->logger = (LV2_Log_Logger*)features[i]->data;
            lv2_log_logger_set_log_level(ui->logger, LV2_LOG_Trace);
            logger = *ui->logger;
        } else if (!strcmp(features[i]->URI, LV2_UI__parent)) {
            ui->window = (Window)(uintptr_t)features[i]->data;
        } else if (!strcmp(features[i]->URI, LV2_UI__X11Display)) {
            ui->display = (Display*)features[i]->data;
        }
    }

    if (!ui->map || !ui->display || !ui->window) {
        lv2_log_error(&logger, "Gla3a UI: Missing required features (URID map, X11 Display, Parent window).\n");
        free(ui);
        return NULL;
    }

    // Mappa gli URI dei parametri (DEVONO CORRISPONDERE agli URI nel tuo plugin TTL e codice audio)
    // Sostituisci "http://your-plugin.com/gla3a#" con l'URI effettivo del tuo plugin
    peakReduction_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#peakReduction");
    gain_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#gain");
    hfComp_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#hfComp"); // Nuovo parametro HF Comp
    bypass_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#bypass");
    ratioMode_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#ratioMode");
    inputPad10dB_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#inputPad10dB");
    oversamplingOn_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#oversamplingOn");
    sidechainMode_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#sidechainMode");
    scLpOn_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#scLpOn");
    scLpFq_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#scLpFq");
    scLpQ_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#scLpQ");
    scHpOn_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#scHpOn");
    scHpFq_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#scHpFq");
    scHpQ_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#scHpQ");

    peakGR_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#peakGR");
    peakInL_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#peakInL");
    peakInR_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#peakInR");
    peakOutL_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#peakOutL");
    peakOutR_URID = ui->map->map(ui->map->handle, "http://your-plugin.com/gla3a#peakOutR");


    // --- Inizializzazione OpenGL ---
    XWindowAttributes wa;
    XGetWindowAttributes(ui->display, ui->window, &wa);

    GLint att[] = { GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None };
    XVisualInfo* vi = glXChooseVisual(ui->display, 0, att);
    if (!vi) {
        lv2_log_error(&logger, "Gla3a UI: No appropriate visual found for OpenGL.\n");
        free(ui);
        return NULL;
    }

    ui->glx_context = glXCreateContext(ui->display, vi, NULL, GL_TRUE);
    if (!ui->glx_context) {
        lv2_log_error(&logger, "Gla3a UI: Failed to create GLX context.\n");
        XFree(vi);
        free(ui);
        return NULL;
    }
    XFree(vi);

    glXMakeCurrent(ui->display, ui->window, ui->glx_context);

    // --- Caricamento delle texture per i Knob e Toggle Switches ---
    std::string bundle_str(bundle_path);
    std::string assets_path = bundle_str + "/gui/assets/"; // Assumi che hai una cartella 'assets' dentro 'gui'

    // Carica il primo knob (dimensione del frame viene impostata qui)
    ui->knobTextureID_peakReduction = LoadTextureFromFile((assets_path + "knob_pr_la3a.png").c_str(), &ui->knobFrameWidth, &ui->knobFrameHeight);
    if (ui->knobTextureID_peakReduction != 0) {
        ui->knobTotalFrames = ui->knobFrameHeight / ui->knobFrameWidth;
        if (ui->knobTotalFrames == 0) {
             lv2_log_error(&logger, "Gla3a UI: Knob texture 'knob_pr_la3a.png' has invalid dimensions (height not multiple of width for square frames).\n");
        } else {
             lv2_log_info(&logger, "Gla3a UI: Loaded knob_pr_la3a.png (%dx%d) with %d frames.\n", ui->knobFrameWidth, ui->knobFrameHeight, ui->knobTotalFrames);
        }
    } else {
         lv2_log_error(&logger, "Gla3a UI: Failed to load knob_pr_la3a.png\n");
    }

    // Carica gli altri knob, usando la stessa dimensione dei frame (assunzione)
    ui->knobTextureID_gain       = LoadTextureFromFile((assets_path + "knob_gain_la3a.png").c_str(), &ui->knobFrameWidth, &ui->knobFrameHeight);
    ui->knobTextureID_hfComp     = LoadTextureFromFile((assets_path + "knob_hfcomp_la3a.png").c_str(), &ui->knobFrameWidth, &ui->knobFrameHeight);
    ui->knobTextureID_scLpFq     = LoadTextureFromFile((assets_path + "knob_sc_fq_la3a.png").c_str(), &ui->knobFrameWidth, &ui->knobFrameHeight);
    ui->knobTextureID_scLpQ      = LoadTextureFromFile((assets_path + "knob_sc_q_la3a.png").c_str(), &ui->knobFrameWidth, &ui->knobFrameHeight);
    ui->knobTextureID_scHpFq     = LoadTextureFromFile((assets_path + "knob_sc_fq_la3a.png").c_str(), &ui->knobFrameWidth, &ui->knobFrameHeight);
    ui->knobTextureID_scHpQ      = LoadTextureFromFile((assets_path + "knob_sc_q_la3a.png").c_str(), &ui->knobFrameWidth, &ui->knobFrameHeight);


    // Carica le texture per i toggle switches (es. immagini separate per ON/OFF)
    ui->toggleSwitchTextureID_on = LoadTextureFromFile((assets_path + "toggle_on_la3a.png").c_str(), &ui->toggleSwitchWidth, &ui->toggleSwitchHeight);
    ui->toggleSwitchTextureID_off = LoadTextureFromFile((assets_path + "toggle_off_la3a.png").c_str(), &ui->toggleSwitchWidth, &ui->toggleSwitchHeight);
    if (ui->toggleSwitchTextureID_on == 0 || ui->toggleSwitchTextureID_off == 0) {
        lv2_log_error(&logger, "Gla3a UI: Failed to load toggle switch textures (toggle_on_la3a.png or toggle_off_la3a.png).\n");
    } else {
        lv2_log_info(&logger, "Gla3a UI: Loaded toggle switches (%dx%d).\n", ui->toggleSwitchWidth, ui->toggleSwitchHeight);
    }


    // --- Inizializzazione ImGui ---
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    io.BackendFlags |= ImGuiBackendFlags_HasMouseCursors;
    io.BackendFlags |= ImGuiBackendFlags_HasSetMousePos;

    io.SetPlatformImeDataFn = NULL;
    io.GetClipboardTextFn = NULL;
    io.SetClipboardTextFn = NULL;


    // --- Imposta lo stile di ImGui per un look LA-3A Verde Scuro ---
    ImGui::StyleColorsDark(); // Base per partire, poi personalizziamo

    ImGuiStyle& style = ImGui::GetStyle();
    style.FrameRounding = 4.0f;
    style.GrabRounding = 4.0f;
    style.ChildRounding = 4.0f;
    style.PopupRounding = 4.0f;
    style.WindowRounding = 4.0f;
    style.ScrollbarRounding = 9.0f;
    style.TabRounding = 4.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.WindowMenuButtonPosition = ImGuiDir_None;

    // Colori per un look LA-3A Verde Scuro / Grigio-Verde
    ImVec4 base_color    = ImVec4(0.25f, 0.30f, 0.25f, 1.00f);     // Verde scuro militare
    ImVec4 darker_color  = ImVec4(0.20f, 0.25f, 0.20f, 1.00f);     // Ancora più scuro
    ImVec4 lighter_color = ImVec4(0.30f, 0.35f, 0.30f, 1.00f);     // Leggermente più chiaro
    ImVec4 text_color    = ImVec4(0.90f, 0.90f, 0.90f, 1.00f);     // Testo quasi bianco
    // Nota: i colori per i meter sono gestiti con PushStyleColor/PopStyleColor per precisione.

    // Aggiorna i colori dello stile:
    style.Colors[ImGuiCol_WindowBg] = base_color;
    style.Colors[ImGuiCol_FrameBg] = lighter_color;
    style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(lighter_color.x + 0.05f, lighter_color.y + 0.05f, lighter_color.z + 0.05f, 1.00f);
    style.Colors[ImGuiCol_FrameBgActive] = darker_color;
    style.Colors[ImGuiCol_TitleBgActive] = darker_color;
    style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(darker_color.x, darker_color.y, darker_color.z, 0.53f);

    style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f); // Manopole e slider: grigio medio
    style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);
    style.Colors[ImGuiCol_Button] = darker_color;
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(darker_color.x + 0.05f, darker_color.y + 0.05f, darker_color.z + 0.05f, 1.00f);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(darker_color.x + 0.10f, darker_color.y + 0.10f, darker_color.z + 0.10f, 1.00f);

    style.Colors[ImGuiCol_CheckMark] = ImVec4(0.00f, 0.60f, 0.00f, 1.00f); // Verde per checkbox
    style.Colors[ImGuiCol_Text] = text_color; // Testo chiaro su sfondo scuro
    style.Colors[ImGuiCol_Border] = ImVec4(0.10f, 0.10f, 0.10f, 0.70f); // Bordi scuri

    style.Colors[ImGuiCol_Tab] = darker_color;
    style.Colors[ImGuiCol_TabHovered] = base_color;
    style.Colors[ImGuiCol_TabActive] = base_color;
    style.Colors[ImGuiCol_TabUnfocused] = darker_color;
    style.Colors[ImGuiCol_TabUnfocusedActive] = lighter_color;


    ImGui_ImplOpenGL3_Init("#version 130");

    ui->imgui_initialized = true;

    *widget = (LV2_UI_Widget_Handle)ui->window;
    return (LV2_UI_Handle)ui;
}

static void cleanup(LV2_UI_Handle ui_handle) {
    Gla3aUI* ui = (Gla3aUI*)ui_handle;

    if (ui->imgui_initialized) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui::DestroyContext();
    }

    // Libera le texture OpenGL
    if (ui->knobTextureID_peakReduction) glDeleteTextures(1, &ui->knobTextureID_peakReduction);
    if (ui->knobTextureID_gain) glDeleteTextures(1, &ui->knobTextureID_gain);
    if (ui->knobTextureID_hfComp) glDeleteTextures(1, &ui->knobTextureID_hfComp);
    if (ui->knobTextureID_scLpFq) glDeleteTextures(1, &ui->knobTextureID_scLpFq);
    if (ui->knobTextureID_scLpQ) glDeleteTextures(1, &ui->knobTextureID_scLpQ);
    if (ui->knobTextureID_scHpFq) glDeleteTextures(1, &ui->knobTextureID_scHpFq);
    if (ui->knobTextureID_scHpQ) glDeleteTextures(1, &ui->knobTextureID_scHpQ);
    if (ui->toggleSwitchTextureID_on) glDeleteTextures(1, &ui->toggleSwitchTextureID_on);
    if (ui->toggleSwitchTextureID_off) glDeleteTextures(1, &ui->toggleSwitchTextureID_off);


    if (ui->glx_context) {
        glXMakeCurrent(ui->display, None, NULL);
        glXDestroyContext(ui->display, ui->glx_context);
    }
    free(ui);
}

static int handle_xevent(Gla3aUI* ui, XEvent* event) {
    ImGuiIO& io = ImGui::GetIO();

    switch (event->type) {
        case MotionNotify:
            io.AddMousePosEvent((float)event->xmotion.x, (float)event->xmotion.y);
            break;
        case ButtonPress:
        case ButtonRelease:
            if (event->xbutton.button >= 1 && event->xbutton.button <= 5) {
                if (event->xbutton.button == 1) io.AddMouseButtonEvent(0, event->xbutton.type == ButtonPress);
                if (event->xbutton.button == 2) io.AddMouseButtonEvent(2, event->xbutton.type == ButtonPress);
                if (event->xbutton.button == 3) io.AddMouseButtonEvent(1, event->xbutton.type == ButtonPress);
                if (event->xbutton.button == 4) io.MouseWheel += 1.0f;
                if (event->xbutton.button == 5) io.MouseWheel -= 1.0f;
            }
            break;
        case KeyPress:
        case KeyRelease: {
            bool down = (event->xkey.type == KeyPress);
            KeySym key_sym = XLookupKeysym(&event->xkey, 0);

            ImGuiKey imgui_key = ImGuiKey_None;
            switch (key_sym) {
                case XK_Tab: imgui_key = ImGuiKey_Tab; break;
                case XK_Left: imgui_key = ImGuiKey_LeftArrow; break;
                case XK_Right: imgui_key = ImGuiKey_RightArrow; break;
                case XK_Up: imgui_key = ImGuiKey_UpArrow; break;
                case XK_Down: imgui_key = ImGuiKey_DownArrow; break;
                case XK_Delete: imgui_key = ImGuiKey_Delete; break;
                case XK_BackSpace: imgui_key = ImGuiKey_Backspace; break;
                case XK_Return: imgui_key = ImGuiKey_Enter; break;
                case XK_Escape: imgui_key = ImGuiKey_Escape; break;
                case XK_a: imgui_key = ImGuiKey_A; break;
                case XK_c: imgui_key = ImGuiKey_C; break;
                case XK_v: imgui_key = ImGuiKey_V; break;
                case XK_x: imgui_key = ImGuiKey_X; break;
                case XK_y: imgui_key = ImGuiKey_Y; break;
                case XK_z: imgui_key = ImGuiKey_Z; break;
                case XK_space: imgui_key = ImGuiKey_Space; break;
                default: break;
            }
            if (imgui_key != ImGuiKey_None) {
                io.AddKeyEvent(imgui_key, down);
            }

            io.AddKeyEvent(ImGuiMod_Ctrl, event->xkey.state & ControlMask);
            io.AddKeyEvent(ImGuiMod_Shift, event->xkey.state & ShiftMask);
            io.AddKeyEvent(ImGuiMod_Alt, event->xkey.state & Mod1Mask);
            io.AddKeyEvent(ImGuiMod_Super, event->xkey.state & Mod4Mask);
            break;
        }
        case ConfigureNotify:
            io.DisplaySize = ImVec2((float)event->xconfigure.width, (float)event->xconfigure.height);
            break;
        default:
            return 0;
    }
    return 1;
}

static void draw_ui(Gla3aUI* ui) {
    if (!ui->imgui_initialized) return;

    glXMakeCurrent(ui->display, ui->window, ui->glx_context);

    ImGuiIO& io = ImGui::GetIO();

    static double g_Time = 0.0;
    double current_time = get_time_in_seconds();
    io.DeltaTime = (float)(current_time - g_Time);
    g_Time = current_time;

    XEvent event;
    while (XPending(ui->display)) {
        XNextEvent(ui->display, &event);
        if (event.xany.window == ui->window) {
            handle_xevent(ui, &event);
        }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();

    XWindowAttributes wa;
    XGetWindowAttributes(ui->display, ui->window, &wa);
    float window_width = (float)wa.width;
    float window_height = (float)wa.height;

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(window_width, window_height));
    ImGui::Begin("Gla3a Compressor", NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoBackground);

    // =====================================================================
    // Layout della UI con Tabs
    // =====================================================================
    if (ImGui::BeginTabBar("MyTabs"))
    {
        // --- Tab Principale ---
        if (ImGui::BeginTabItem("Main"))
        {
            ImGui::Columns(2, "MainLayout", false);
            ImGui::SetColumnWidth(0, window_width * 0.6f);
            ImGui::Text("Main Controls");
            ImGui::Separator();
            ImGui::Dummy(ImVec2(0, 10));

            ImVec2 knob_img_size = ImVec2((float)ui->knobFrameWidth, (float)ui->knobFrameWidth);

            // Knob per Peak Reduction
            ImGui::PushID("PeakReduction");
            if (KnobRotaryImage("Peak Reduction", &ui->peakReduction_val, -60.0f, -10.0f,
                                ui->knobTextureID_peakReduction, ui->knobFrameWidth, ui->knobFrameWidth,
                                ui->knobTotalFrames, knob_img_size, "%.1f dB")) {
                ui->write_function(ui->controller, peakReduction_URID, sizeof(float), 0, &ui->peakReduction_val);
            }
            ImGui::PopID();
            ImGui::SameLine(0, 20);

            // Knob per Gain Out
            ImGui::PushID("Gain");
            if (KnobRotaryImage("Gain Out", &ui->gain_val, 0.0f, 12.0f,
                                ui->knobTextureID_gain, ui->knobFrameWidth, ui->knobFrameWidth,
                                ui->knobTotalFrames, knob_img_size, "%.1f dB")) {
                ui->write_function(ui->controller, gain_URID, sizeof(float), 0, &ui->gain_val);
            }
            ImGui::PopID();

            ImGui::Dummy(ImVec2(0, 20));

            // Toggle Button (Levette) per Input Pad
            ImGui::Text("Input Pad -10dB");
            ImGui::SameLine();
            ImGui::PushID("InputPad");
            ImTextureID toggle_tex_id_input_pad = ui->inputPad10dB_val ? (ImTextureID)(intptr_t)ui->toggleSwitchTextureID_on : (ImTextureID)(intptr_t)ui->toggleSwitchTextureID_off;
            if (ImGui::ImageButton("##InputPadBtn", toggle_tex_id_input_pad, ImVec2((float)ui->toggleSwitchWidth, (float)ui->toggleSwitchHeight))) {
                ui->inputPad10dB_val = !ui->inputPad10dB_val;
                float val = ui->inputPad10dB_val ? 1.0f : 0.0f;
                ui->write_function(ui->controller, inputPad10dB_URID, sizeof(float), 0, &val);
            }
            ImGui::PopID();

            // Toggle Button per Ratio Mode (Comp/Limit, se presente nell'LA-3A)
            ImGui::Text("Ratio Mode");
            ImGui::SameLine();
            ImGui::PushID("RatioMode");
            ImTextureID toggle_tex_id_ratio_mode = ui->ratioMode_val ? (ImTextureID)(intptr_t)ui->toggleSwitchTextureID_on : (ImTextureID)(intptr_t)ui->toggleSwitchTextureID_off;
            if (ImGui::ImageButton("##RatioModeBtn", toggle_tex_id_ratio_mode, ImVec2((float)ui->toggleSwitchWidth, (float)ui->toggleSwitchHeight))) {
                ui->ratioMode_val = !ui->ratioMode_val;
                float val = ui->ratioMode_val ? 1.0f : 0.0f;
                ui->write_function(ui->controller, ratioMode_URID, sizeof(float), 0, &val);
            }
            ImGui::SameLine(); ImGui::Text(ui->ratioMode_val ? "(Limit)" : "(Comp)");
            ImGui::PopID();

            ImGui::Dummy(ImVec2(0, 20));

            // Knob per High Frequency Compression (LA-3A tipico)
            ImGui::PushID("HFComp");
            if (KnobRotaryImage("HF Comp", &ui->hfComp_val, 0.0f, 1.0f, // Range tipico 0.0-1.0 o dB
                                ui->knobTextureID_hfComp, ui->knobFrameWidth, ui->knobFrameWidth,
                                ui->knobTotalFrames, knob_img_size, "%.2f")) {
                ui->write_function(ui->controller, hfComp_URID, sizeof(float), 0, &ui->hfComp_val);
            }
            ImGui::PopID();

            ImGui::Dummy(ImVec2(0, 20));

            // Pulsante normale per Bypass
            ImGui::PushID("Bypass");
            float bypass_button_width = 100;
            float current_cursor_x = ImGui::GetCursorPosX();
            float column_width_0 = ImGui::GetColumnWidth(0);
            ImGui::SetCursorPosX(current_cursor_x + (column_width_0 - bypass_button_width) / 2);
            if (ImGui::Button(ui->bypass_val ? "BYPASS ON" : "BYPASS OFF", ImVec2(bypass_button_width, 30))) {
                ui->bypass_val = !ui->bypass_val;
                float val = ui->bypass_val ? 1.0f : 0.0f;
                ui->write_function(ui->controller, bypass_URID, sizeof(float), 0, &val);
            }
            ImGui::PopID();


            ImGui::NextColumn(); // Passa alla seconda colonna (Meters)

            // Colonna 2: Meter e I/O
            ImGui::SetColumnWidth(1, window_width * 0.4f);
            ImGui::Text("Meters");
            ImGui::Separator();
            ImGui::Dummy(ImVec2(0, 10));

            // VU Meter di Gain Reduction
            ImGui::Text("Gain Reduction (dB)");
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.9f, 0.4f, 0.1f, 1.0f)); // Arancione brillante per GR
            float normalized_gr = ImClamp(ui->peakGR_val / -30.0f, 0.0f, 1.0f);
            ImGui::ProgressBar(normalized_gr, ImVec2(ImGui::GetColumnWidth(), 100), "");
            ImGui::PopStyleColor();
            ImGui::Dummy(ImVec2(0, 20));


            // Switch I/O per il meter principale
            ImGui::Text("Show Output Meter");
            ImGui::SameLine();
            if (ImGui::Checkbox("##ShowOutputMeter", &ui->showOutputMeter)) {
                // Non c'è un parametro LV2 per questo, è solo un toggle della UI
            }
            ImGui::SameLine(); ImGui::Text(ui->showOutputMeter ? "(Output)" : "(Input)");

            // VU Meter I/O principale (Input L/R o Output L/R)
            ImGui::Text("Input/Output Peak (dB)");
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.0f, 0.8f, 0.0f, 1.0f)); // Verde brillante
            float currentIOMeterValL = ui->showOutputMeter ? ui->peakOutL_val : ui->peakInL_val;
            float currentIOMeterValR = ui->showOutputMeter ? ui->peakOutR_val : ui->peakInR_val;
            float normalized_in_out_L = ImClamp((currentIOMeterValL + 60.0f) / 60.0f, 0.0f, 1.0f);
            float normalized_in_out_R = ImClamp((currentIOMeterValR + 60.0f) / 60.0f, 0.0f, 1.0f);

            ImGui::ProgressBar(normalized_in_out_L, ImVec2(ImGui::GetColumnWidth(), 50), "L");
            ImGui::ProgressBar(normalized_in_out_R, ImVec2(ImGui::GetColumnWidth(), 50), "R");
            ImGui::PopStyleColor();


            ImGui::Columns(1);
            ImGui::EndTabItem();
        }

        // --- Tab Sidechain ---
        if (ImGui::BeginTabItem("Sidechain"))
        {
            ImGui::Text("Sidechain Controls");
            ImGui::Separator();
            ImGui::Dummy(ImVec2(0, 10));

            ImVec2 knob_img_size_small = ImVec2((float)ui->knobFrameWidth * 0.7f, (float)ui->knobFrameWidth * 0.7f);

            // Oversampling
            if (ImGui::Checkbox("Oversampling On", &ui->oversamplingOn_val)) {
                float val = ui->oversamplingOn_val ? 1.0f : 0.0f;
                ui->write_function(ui->controller, oversamplingOn_URID, sizeof(float), 0, &val);
            }

            // Sidechain Mode (External Sidechain)
            if (ImGui::Checkbox("External Sidechain", &ui->sidechainMode_val)) {
                float val = ui->sidechainMode_val ? 1.0f : 0.0f;
                ui->write_function(ui->controller, sidechainMode_URID, sizeof(float), 0, &val);
            }

            ImGui::Dummy(ImVec2(0, 20));

            // Filtri Sidechain (HP/LP)
            ImGui::Columns(2, "SidechainFilters", false);

            ImGui::Text("HP Filter");
            if (ImGui::Checkbox("HP On", &ui->scHpOn_val)) {
                float val = ui->scHpOn_val ? 1.0f : 0.0f;
                ui->write_function(ui->controller, scHpOn_URID, sizeof(float), 0, &val);
            }
            ImGui::PushID("HpFreq");
            if (KnobRotaryImage("Freq", &ui->scHpFq_val, 20.0f, 20000.0f,
                                ui->knobTextureID_scHpFq, ui->knobFrameWidth, ui->knobFrameWidth,
                                ui->knobTotalFrames, knob_img_size_small, "%.0f Hz")) {
                ui->write_function(ui->controller, scHpFq_URID, sizeof(float), 0, &ui->scHpFq_val);
            }
            ImGui::PopID();
            ImGui::PushID("HpQ");
            if (KnobRotaryImage("Q", &ui->scHpQ_val, 0.1f, 10.0f,
                                ui->knobTextureID_scHpQ, ui->knobFrameWidth, ui->knobFrameWidth,
                                ui->knobTotalFrames, knob_img_size_small, "%.2f")) {
                ui->write_function(ui->controller, scHpQ_URID, sizeof(float), 0, &ui->scHpQ_val);
            }
            ImGui::PopID();

            ImGui::NextColumn();

            ImGui::Text("LP Filter");
            if (ImGui::Checkbox("LP On", &ui->scLpOn_val)) {
                float val = ui->scLpOn_val ? 1.0f : 0.0f;
                ui->write_function(ui->controller, scLpOn_URID, sizeof(float), 0, &val);
            }
            ImGui::PushID("LpFreq");
            if (KnobRotaryImage("Freq", &ui->scLpFq_val, 20.0f, 20000.0f,
                                ui->knobTextureID_scLpFq, ui->knobFrameWidth, ui->knobFrameWidth,
                                ui->knobTotalFrames, knob_img_size_small, "%.0f Hz")) {
                ui->write_function(ui->controller, scLpFq_URID, sizeof(float), 0, &ui->scLpFq_val);
            }
            ImGui::PopID();
            ImGui::PushID("LpQ");
            if (KnobRotaryImage("Q", &ui->scLpQ_val, 0.1f, 10.0f,
                                ui->knobTextureID_scLpQ, ui->knobFrameWidth, ui->knobFrameWidth,
                                ui->knobTotalFrames, knob_img_size_small, "%.2f")) {
                ui->write_function(ui->controller, scLpQ_URID, sizeof(float), 0, &ui->scLpQ_val);
            }
            ImGui::PopID();


            ImGui::Columns(1);
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();

    ImGui::Render();
    glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
    glClearColor(0.25f, 0.30f, 0.25f, 1.00f); // Sfondo esterno al widget ImGui (il colore base del LA-3A)
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        ImGui::UpdatePlatformWindows();
        ImGui::RenderPlatformWindowsDefault();
    }

    glXSwapBuffers(ui->display, ui->window);
}

static void port_event(LV2_UI_Handle handle, LV2_URID port_urid, uint32_t buffer_size, uint32_t format, const void* buffer) {
    Gla3aUI* ui = (Gla3aUI*)handle;

    if (format == 0) { // LV2_Atom_Port_Float
        if (port_urid == peakReduction_URID) {
            ui->peakReduction_val = *(const float*)buffer;
        } else if (port_urid == gain_URID) {
            ui->gain_val = *(const float*)buffer;
        } else if (port_urid == hfComp_URID) {
            ui->hfComp_val = *(const float*)buffer;
        } else if (port_urid == bypass_URID) {
            ui->bypass_val = (*(const float*)buffer != 0.0f);
        } else if (port_urid == ratioMode_URID) {
            ui->ratioMode_val = (*(const float*)buffer != 0.0f);
        } else if (port_urid == inputPad10dB_URID) {
            ui->inputPad10dB_val = (*(const float*)buffer != 0.0f);
        } else if (port_urid == oversamplingOn_URID) {
            ui->oversamplingOn_val = (*(const float*)buffer != 0.0f);
        } else if (port_urid == sidechainMode_URID) {
            ui->sidechainMode_val = (*(const float*)buffer != 0.0f);
        } else if (port_urid == scLpOn_URID) {
            ui->scLpOn_val = (*(const float*)buffer != 0.0f);
        } else if (port_urid == scLpFq_URID) {
            ui->scLpFq_val = *(const float*)buffer;
        } else if (port_urid == scLpQ_URID) {
            ui->scLpQ_val = *(const float*)buffer;
        } else if (port_urid == scHpOn_URID) {
            ui->scHpOn_val = (*(const float*)buffer != 0.0f);
        } else if (port_urid == scHpFq_URID) {
            ui->scHpFq_val = *(const float*)buffer;
        } else if (port_urid == scHpQ_URID) {
            ui->scHpQ_val = *(const float*)buffer;
        }
        else if (port_urid == peakGR_URID) {
            ui->peakGR_val = *(const float*)buffer;
        } else if (port_urid == peakInL_URID) {
            ui->peakInL_val = *(const float*)buffer;
        } else if (port_urid == peakInR_URID) {
            ui->peakInR_val = *(const float*)buffer;
        } else if (port_urid == peakOutL_URID) {
            ui->peakOutL_val = *(const float*)buffer;
        } else if (port_urid == peakOutR_URID) {
            ui->peakOutR_val = *(const float*)buffer;
        }
    }
    draw_ui(ui);
}


static int ui_extension_data(const char* uri) {
    if (!strcmp(uri, LV2_UI__idleInterface)) {
        return 0;
    }
    return 0;
}

static void ui_idle(LV2_UI_Handle handle) {
    Gla3aUI* ui = (Gla3aUI*)handle;
    draw_ui(ui);
}

static const LV2_UI_Idle_Interface idle_iface = { ui_idle };

static const void* ui_extension_data_interface(const char* uri) {
    if (!strcmp(uri, LV2_UI__idleInterface)) {
        return &idle_iface;
    }
    return NULL;
}


// =========================================================================
// Descrittori della UI
// =========================================================================
static const LV2_UI_Descriptor descriptors[] = {
    {
        "http://your-plugin.com/gla3a-ui", // URI della UI (DEVE ESSERE DIVERSO dal plugin audio)
        instantiate,
        cleanup,
        port_event,
        ui_extension_data_interface
    }
};

const LV2_UI_Descriptor* lv2_ui_descriptor(uint32_t index) {
    if (index == 0) return &descriptors[0];
    return NULL;
}
