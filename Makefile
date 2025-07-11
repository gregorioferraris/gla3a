# ===============================================================
# Configurazione Generale del Progetto
# ===============================================================

# Nome del plugin (senza estensione .lv2). Questo sarà anche il nome della directory del plugin.
PLUGIN_NAME = gla3a

# Directory di destinazione del plugin. Sarà PLUGIN_NAME.lv2
BUNDLE_DIR = $(PLUGIN_NAME).lv2

# Percorso completo dell'URI del plugin LV2.
# DEVE CORRISPONDERE ESATTAMENTE A QUELLO DEFINITO NEL TUO .cpp E .ttl
# http://moddevices.com/plugins/mod-devel/gla3a
URI = http://moddevices.com/plugins/mod-devel/$(PLUGIN_NAME)

# ===============================================================
# Configurazione Compilatore
# ===============================================================

# Compilatore C++ da usare
CXX = g++

# Flag del compilatore
# -g: Abilita il debugging
# -O2: Ottimizzazione di livello 2
# -Wall: Abilita tutti gli avvisi
# -fPIC: Compila codice indipendente dalla posizione (necessario per librerie condivise)
CXXFLAGS = -g -O2 -Wall -fPIC

# Flag del linker
# -shared: Crea una libreria condivisa (.so)
LDFLAGS = -shared

# ===============================================================
# Dipendenze LV2
# ===============================================================

# Ottieni i flag di compilazione e linking per LV2 tramite pkg-config
LV2_CFLAGS = $(shell pkg-config --cflags lv2)
LV2_LIBS = $(shell pkg-config --libs lv2)

# ===============================================================
# Dipendenze per la GUI (wxWidgets)
# ===============================================================

# Directory della GUI
GUI_DIR = gui

# Ottieni i flag di compilazione e linking per wxWidgets
# La versione di wxWidgets può variare (es. wxwidgets_gtk3-3.0, wxwidgets_gtk3-3.2)
# Sostituisci 'wxwidgets_gtk3-3.2' con la versione corretta installata sul tuo sistema Ubuntu.
WX_CFLAGS = $(shell pkg-config --cflags wxwidgets_gtk3-3.2)
WX_LIBS = $(shell pkg-config --libs wxwidgets_gtk3-3.2)

# ===============================================================
# File Sorgente e Obiettivi di Compilazione
# ===============================================================

# Sorgenti del core del plugin
SOURCES_PLUGIN = $(PLUGIN_NAME).cpp

# Sorgenti della GUI del plugin
SOURCES_GUI = $(GUI_DIR)/$(PLUGIN_NAME)_gui.cpp

# File oggetto del core del plugin
OBJECTS_PLUGIN = $(SOURCES_PLUGIN:.cpp=.o)

# File oggetto della GUI del plugin
OBJECTS_GUI = $(SOURCES_GUI:.cpp=.o)

# Nome del file .so del plugin (la libreria condivisa principale)
TARGET_PLUGIN_SO = $(PLUGIN_NAME).so

# Nome del file .so della GUI (la libreria condivisa della GUI)
TARGET_GUI_SO = $(GUI_DIR)/$(PLUGIN_NAME)_gui.so

# ===============================================================
# Regole di Compilazione
# ===============================================================

# Regola predefinita: compila tutto
all: $(BUNDLE_DIR)

# Creazione della directory del bundle LV2
$(BUNDLE_DIR): $(TARGET_PLUGIN_SO) $(TARGET_GUI_SO) $(PLUGIN_NAME).ttl
	@mkdir -p $(BUNDLE_DIR)
	@cp $(TARGET_PLUGIN_SO) $(BUNDLE_DIR)/$(TARGET_PLUGIN_SO)
	@cp $(PLUGIN_NAME).ttl $(BUNDLE_DIR)/$(PLUGIN_NAME).ttl
	@mkdir -p $(BUNDLE_DIR)/$(GUI_DIR)
	@cp $(TARGET_GUI_SO) $(BUNDLE_DIR)/$(GUI_DIR)/$(TARGET_GUI_SO)
	@echo "Plugin LV2 ($(BUNDLE_DIR)) compilato e pronto."

# Regola per la compilazione del core del plugin (.cpp a .o)
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(LV2_CFLAGS) -c $< -o $@

# Regola per la compilazione della GUI (.cpp a .o)
$(GUI_DIR)/%.o: $(GUI_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) $(LV2_CFLAGS) $(WX_CFLAGS) -c $< -o $@

# Regola per il linking del core del plugin (.o a .so)
$(TARGET_PLUGIN_SO): $(OBJECTS_PLUGIN)
	$(CXX) $(LDFLAGS) $(OBJECTS_PLUGIN) $(LV2_LIBS) -o $@

# Regola per il linking della GUI (.o a .so)
$(TARGET_GUI_SO): $(OBJECTS_GUI)
	$(CXX) $(LDFLAGS) $(OBJECTS_GUI) $(LV2_LIBS) $(WX_LIBS) -o $@

# ===============================================================
# Regole di Pulizia e Installazione
# ===============================================================

# Pulisce i file compilati e le directory temporanee
clean:
	@rm -rf *.o $(GUI_DIR)/*.o $(TARGET_PLUGIN_SO) $(TARGET_GUI_SO) $(BUNDLE_DIR)
	@echo "Pulizia completata."

# Regola di installazione (copia il plugin nella directory utente LV2)
install: all
	@mkdir -p ~/.lv2
	@cp -r $(BUNDLE_DIR) ~/.lv2/
	@echo "Plugin $(PLUGIN_NAME) installato in ~/.lv2/"

# Regola di disinstallazione
uninstall:
	@rm -rf ~/.lv2/$(BUNDLE_DIR)
	@echo "Plugin $(PLUGIN_NAME) disinstallato da ~/.lv2/"

.PHONY: all clean install uninstall
