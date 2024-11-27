#include <module.h>  // これを最初に
#include <utils/flog.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <config.h>   
#include <gui/widgets/stepped_slider.h>
#include <gui/smgui.h>
#include "../3rdparty/ExtIO_sddc/libsddc/libsddc.h"
#include "../3rdparty/ExtIO_sddc/Core/RadioHandler.h"
#include "../3rdparty/ExtIO_sddc/Core/r2iq.h"  
#include <thread>
#include <string.h>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "sddc_source",
    /* Description:     */ "SDDC source module for SDR++",
    /* Author:          */ "Jack Heinlein",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ConfigManager config;


const double sampleRates[] = {
    32000000,  // 32 MSPS
    16000000,  // 16 MSPS
    8000000,   // 8 MSPS
    4000000,   // 4 MSPS
    2000000    // 2 MSPS
};

const char* sampleRatesTxt[] = {
    "32 MSPS",
    "16 MSPS", 
    "8 MSPS",
    "4 MSPS",
    "2 MSPS"
};

class RadioHandlerWrapper {
public:
    RadioHandlerWrapper() {
        flog::info("Creating USB Handler...");
        fx3 = CreateUsbHandler();
        if (fx3 == nullptr) {
            flog::error("Failed to create USB handler");
            handler = nullptr;
            deviceFound = false;
            return;
        }

        flog::info("Creating Radio Handler...");
        try {
            handler = new RadioHandlerClass();
            
            RadioModel model = handler->getModel();
            switch(model) {
                case RadioModel::BBRF103:
                    flog::info("Found BBRF103 device");
                    deviceFound = true;
                    break;
                case RadioModel::HF103:
                    flog::info("Found HF103 device");
                    deviceFound = true;
                    break;
                case RadioModel::RX888:
                    flog::info("Found RX888 device");
                    deviceFound = true;
                    break;
                case RadioModel::RX888r2:
                    flog::info("Found RX888R2 device");
                    deviceFound = true;
                    break;
                case RadioModel::RX888r3:
                    flog::info("Found RX888R3 device");
                    deviceFound = true;
                    break;
                case RadioModel::RX999:
                    flog::info("Found RX999 device");
                    deviceFound = true;
                    break;
                default:
                    flog::error("No supported device found");
                    deviceFound = false;
                    if (handler) {
                        delete handler;
                        handler = nullptr;
                    }
                    if (fx3) {
                        delete fx3;
                        fx3 = nullptr;
                    }
                    return;
            }

            if (deviceFound) {
                flog::info("RadioHandlerWrapper initialization complete");
            }
        }
        catch (...) {
            flog::error("Exception during RadioHandler creation");
            if (handler) {
                delete handler;
                handler = nullptr;
            }
            if (fx3) {
                delete fx3;
                fx3 = nullptr;
            }
            deviceFound = false;
        }
    }

    ~RadioHandlerWrapper() {
        flog::info("Destroying RadioHandlerWrapper");
        if (handler) {
            flog::info("Deleting handler");
            delete handler;
        }
        if (fx3) {
            flog::info("Deleting fx3");
            delete fx3;
        }
    }

    bool Init(void (*callback)(void* context, const float* data, uint32_t len), void* ctx) {
        if (!deviceFound || !handler || !fx3) {
            flog::error("RadioHandler not initialized or device not found");
            return false;
        }
        bool result = handler->Init(fx3, callback, nullptr, ctx);
        flog::info("RadioHandler initialization {}", result ? "successful" : "failed");
        return result;
    }

    void Start(int srateIdx) {
        if (!deviceFound) {
            flog::error("Cannot start - no device found");
            return;
        }
        if (!handler) {
            flog::error("Cannot start - not initialized");
            return;
        }
        handler->Start(srateIdx);
        flog::info("RadioHandler started with sample rate index {}", srateIdx);
    }

    void Stop() {
        if (!handler || !deviceFound) return;
        handler->Stop();
    }

    void Close() {
        if (!handler || !deviceFound) return;
        handler->Close();
    }

    RadioHandlerClass* getHandler() {
        return handler;
    }

    operator bool() const {
        return deviceFound && handler != nullptr && fx3 != nullptr;
    }

private:
    fx3class* fx3 = nullptr;
    RadioHandlerClass* handler = nullptr;
    bool deviceFound = false;
};

class SDDCSourceModule : public ModuleManager::Instance {
public:
    SDDCSourceModule(std::string name);
    ~SDDCSourceModule();

    void postInit() {}
    void enable();
    void disable();
    bool isEnabled();
    void refresh();

private:
    void selectFirst();
    void selectByName(std::string name);
    void selectById(int id);
    void loadConfig(json& conf);

    static void menuSelected(void* ctx);
    static void menuDeselected(void* ctx);
    static void start(void* ctx);
    static void stop(void* ctx);
    static void tune(double freq, void* ctx);
    static void dataHandler(void* context, const float* data, uint32_t len);
    static void menuHandler(void* ctx);

    std::string name;
    RadioHandlerWrapper* radio = nullptr;
    bool enabled = true;
    dsp::stream<dsp::complex_t> stream;
    SourceManager::SourceHandler handler;
    bool running = false;
    double freq;

    std::string selectedDevName = "";
    int devId = 0;
    int srId = 0; 
    int devCount = 0;

    double sampleRate;
    int rfMode = HFMODE;  // Changed from HF_MODE to HFMODE

    // HF Mode gains
    float hfRFGain = 0.0f;    // -31.5 to 0 dB, 0.5dB steps
    float hfIFGain = 0.0f;    // LOW_MODE(0-18) or HIGH_MODE(19+)
    bool hfGainMode = false;   // false=LOW_MODE, true=HIGH_MODE

    // VHF Mode gains
    float vhfRFGain = 0.0f;   // 29 steps from rf_steps table
    float vhfIFGain = 0.0f;   // 16 steps from if_steps table
    bool gainAuto = true;

    bool dither = false;
    bool random = false;
    bool hfBias = false;
    bool vhfBias = false;
    float freqCorr = 0.0f;

    const float* rf_steps = nullptr;
    const float* if_steps = nullptr;
    int rf_steps_count = 0;
    int if_steps_count = 0;

    std::vector<std::string> devNames;
    std::string devListTxt;
    std::string sampleRateListTxt;
};

SDDCSourceModule::SDDCSourceModule(std::string name) {
    this->name = name;
    
    // Initialize parameters
    hfRFGain = 0.0f;
    hfIFGain = 0.0f;
    hfGainMode = false;
    vhfRFGain = 0.0f;
    vhfIFGain = 0.0f;
    gainAuto = true;

    dither = false;
    random = false;
    hfBias = false;
    vhfBias = false;
    freqCorr = 0.0f;

    sampleRate = sampleRates[0];

    handler.ctx = this;
    handler.selectHandler = menuSelected;
    handler.deselectHandler = menuDeselected;
    handler.menuHandler = menuHandler;
    handler.startHandler = start;
    handler.stopHandler = stop;
    handler.tuneHandler = tune;
    handler.stream = &stream;

    for (int i = 0; i < sizeof(sampleRates)/sizeof(sampleRates[0]); i++) {
        sampleRateListTxt += sampleRatesTxt[i];
        sampleRateListTxt += '\0';
    }

    refresh();

    config.acquire();
    if (!config.conf["device"].is_string()) {
        selectedDevName = "";
        config.conf["device"] = "";
    }
    else {
        selectedDevName = config.conf["device"];
    }
    config.release(true);
    selectByName(selectedDevName);

    sigpath::sourceManager.registerSource("SDDC", &handler);
}

SDDCSourceModule::~SDDCSourceModule() {
    stop(this);
    sigpath::sourceManager.unregisterSource("SDDC");
    if (radio) delete radio;
}

void SDDCSourceModule::enable() {
    enabled = true;
}

void SDDCSourceModule::disable() {
    enabled = false;
}

bool SDDCSourceModule::isEnabled() {
    return enabled;
}

void SDDCSourceModule::refresh() {
    devNames.clear();
    devListTxt = "";
    devCount = 0;

    // Try to create temporary handler to check device presence
    std::unique_ptr<RadioHandlerWrapper> tempRadio(new RadioHandlerWrapper());
    if (!tempRadio || !*tempRadio) {
        flog::warn("No SDDC devices found");
        return;
    }

    // Device found
    devCount = 1;
    devNames.push_back("RX888 MK2");
    devListTxt = "RX888 MK2\0";
}

void SDDCSourceModule::selectFirst() {
    if (devCount > 0) {
        selectById(0);
    }
}

void SDDCSourceModule::selectByName(std::string name) {
    for (int i = 0; i < devCount; i++) {
        if (name == devNames[i]) {
            selectById(i);
            return;
        }
    }
    selectFirst();
}

void SDDCSourceModule::selectById(int id) {
    if (devCount <= 0) return;
    selectedDevName = devNames[id];
    devId = id;

    if (radio) {
        delete radio;
    }
    radio = new RadioHandlerWrapper();
    if (!radio->Init(dataHandler, this)) {
        selectedDevName = "";
        delete radio;
        radio = nullptr;
        return;
    }

    // Get initial gain steps
    RadioHandlerClass* handler = radio->getHandler();
    handler->GetRFAttSteps(&rf_steps);
    rf_steps_count = handler->GetRFAttSteps(&rf_steps);
    handler->GetIFGainSteps(&if_steps);
    if_steps_count = handler->GetIFGainSteps(&if_steps);

    bool created = false;
    config.acquire();
    if (!config.conf["devices"].contains(selectedDevName)) {
        created = true;
        config.conf["devices"][selectedDevName]["sampleRate"] = 32000000;
        config.conf["devices"][selectedDevName]["rfMode"] = rfMode;
        config.conf["devices"][selectedDevName]["hfRFGain"] = hfRFGain;
        config.conf["devices"][selectedDevName]["hfIFGain"] = hfIFGain;
        config.conf["devices"][selectedDevName]["hfGainMode"] = hfGainMode;
        config.conf["devices"][selectedDevName]["vhfRFGain"] = vhfRFGain;
        config.conf["devices"][selectedDevName]["vhfIFGain"] = vhfIFGain;
        config.conf["devices"][selectedDevName]["gainAuto"] = gainAuto;
        config.conf["devices"][selectedDevName]["dither"] = dither;
        config.conf["devices"][selectedDevName]["random"] = random;
        config.conf["devices"][selectedDevName]["hfBias"] = hfBias;
        config.conf["devices"][selectedDevName]["vhfBias"] = vhfBias;
        config.conf["devices"][selectedDevName]["freqCorr"] = freqCorr;
    }

    loadConfig(config.conf["devices"][selectedDevName]);
    config.release(created);

    // Initial radio setup
    handler->UptDither(dither);
    handler->UptRand(random);
    handler->UpdBiasT_HF(hfBias);
    handler->UpdBiasT_VHF(vhfBias);

    if (rfMode == VHFMODE) {
        handler->UpdatemodeRF(VHFMODE);
        if (!gainAuto) {
            handler->UpdateattRF(vhfRFGain);
            handler->UpdateIFGain(vhfIFGain);
        }
    } else {
        handler->UpdatemodeRF(HFMODE);
        handler->UpdateattRF(hfRFGain);
        handler->UpdateIFGain(hfGainMode ? (hfIFGain + 0x80) : hfIFGain);
    }
}

void SDDCSourceModule::loadConfig(json& conf) {
    if (conf.contains("sampleRate")) {
        double selectedSr = conf["sampleRate"];
        for (int i = 0; i < sizeof(sampleRates)/sizeof(sampleRates[0]); i++) {
            if (sampleRates[i] == selectedSr) {
                srId = i;
                sampleRate = selectedSr;
                break;
            }
        }
    }
    if (conf.contains("rfMode")) rfMode = conf["rfMode"];
    if (conf.contains("hfRFGain")) hfRFGain = conf["hfRFGain"];
    if (conf.contains("hfIFGain")) hfIFGain = conf["hfIFGain"];
    if (conf.contains("hfGainMode")) hfGainMode = conf["hfGainMode"];
    if (conf.contains("vhfRFGain")) vhfRFGain = conf["vhfRFGain"];
    if (conf.contains("vhfIFGain")) vhfIFGain = conf["vhfIFGain"];
    if (conf.contains("gainAuto")) gainAuto = conf["gainAuto"];
    if (conf.contains("dither")) dither = conf["dither"];
    if (conf.contains("random")) random = conf["random"];
    if (conf.contains("hfBias")) hfBias = conf["hfBias"];
    if (conf.contains("vhfBias")) vhfBias = conf["vhfBias"];
    if (conf.contains("freqCorr")) freqCorr = conf["freqCorr"];
}

void SDDCSourceModule::menuSelected(void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
    core::setInputSampleRate(_this->sampleRate);
}

void SDDCSourceModule::menuDeselected(void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
}

void SDDCSourceModule::start(void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
    if (_this->running || _this->selectedDevName == "" || _this->radio == nullptr) return;

    _this->radio->Start(_this->srId);
    _this->running = true;
}

void SDDCSourceModule::stop(void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
    if (!_this->running || _this->radio == nullptr) return;

    _this->radio->Stop();
    _this->running = false;
}

void SDDCSourceModule::tune(double freq, void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
    if (_this->running && _this->radio) {
        RadioHandlerClass* handler = _this->radio->getHandler();
        rf_mode newMode = handler->PrepareLo(freq);
        if (newMode != handler->GetmodeRF()) {
            _this->rfMode = (newMode == VHFMODE) ? VHFMODE : HFMODE;
            handler->UpdatemodeRF(newMode);

            if (_this->rfMode == VHFMODE) {
                if (!_this->gainAuto) {
                    handler->UpdateattRF(_this->vhfRFGain);
                    handler->UpdateIFGain(_this->vhfIFGain);
                }
            } else {
                handler->UpdateattRF(_this->hfRFGain);
                handler->UpdateIFGain(_this->hfGainMode ? (_this->hfIFGain + 0x80) : _this->hfIFGain);
            }
        }
        handler->TuneLO(freq);
    }
    _this->freq = freq;
}

void SDDCSourceModule::dataHandler(void* context, const float* data, uint32_t len) {
    SDDCSourceModule* _this = (SDDCSourceModule*)context;

    int count = len;
    for (int i = 0; i < count; i++) {
        _this->stream.writeBuf[i].re = data[i*2];
        _this->stream.writeBuf[i].im = data[i*2 + 1];
    }

    if (!_this->stream.swap(count)) return;
}

void SDDCSourceModule::menuHandler(void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
    RadioHandlerClass* handler = _this->radio ? _this->radio->getHandler() : nullptr;

    if (_this->running) { SmGui::BeginDisabled(); }

    // Device selection
    SmGui::FillWidth();
    if (SmGui::Combo(CONCAT("##_sddc_dev_sel_", _this->name), &_this->devId, _this->devListTxt.c_str())) {
        _this->selectById(_this->devId);
        core::setInputSampleRate(_this->sampleRate);
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["device"] = _this->selectedDevName;
            config.release(true);
        }
    }

    // Sample rate selection
    SmGui::FillWidth();
    if (SmGui::Combo(CONCAT("##_sddc_sr_sel_", _this->name), &_this->srId, _this->sampleRateListTxt.c_str())) {
        _this->sampleRate = sampleRates[_this->srId];
        core::setInputSampleRate(_this->sampleRate);
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["devices"][_this->selectedDevName]["sampleRate"] = _this->sampleRate;
            config.release(true);
        }
    }

    // Refresh button
    SmGui::FillWidth();
    if (SmGui::Button(CONCAT("Refresh##_sddc_refr_", _this->name))) {
        _this->refresh();
        _this->selectByName(_this->selectedDevName);
        core::setInputSampleRate(_this->sampleRate);
    }

    if (_this->running) { SmGui::EndDisabled(); }

    char buf[64];

    // RF Mode indicator
    sprintf(buf, "Current Mode: %s", _this->rfMode == VHFMODE ? "VHF" : "HF");
    SmGui::Text(buf);

    // Gain Controls
    SmGui::BeginGroup();
    SmGui::Text("Gain Control");

    if (_this->rfMode == VHFMODE) {
        // VHF Mode Gain Controls
        if (SmGui::Checkbox(CONCAT("Auto Gain##_sddc_auto_gain_", _this->name), &_this->gainAuto)) {
            if (_this->running && handler) {
                if (_this->gainAuto) {
                    _this->vhfRFGain = 0.0f;
                    _this->vhfIFGain = 0.0f;
                    handler->UpdateattRF(_this->vhfRFGain);
                    handler->UpdateIFGain(_this->vhfIFGain);
                }
            }
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["gainAuto"] = _this->gainAuto;
                config.release(true);
            }
        }

        if (!_this->gainAuto) {
            // VHF RF Gain (29 steps)
            if (SmGui::SliderInt(CONCAT("RF Gain Step##_sddc_vhf_rf_gain_", _this->name),
                               (int*)&_this->vhfRFGain, 0, _this->rf_steps_count - 1)) {
                if (_this->running && handler) {
                    handler->UpdateattRF(_this->vhfRFGain);
                }
                if (_this->selectedDevName != "") {
                    config.acquire();
                    config.conf["devices"][_this->selectedDevName]["vhfRFGain"] = _this->vhfRFGain;
                    config.release(true);
                }
            }
            sprintf(buf, "RF Gain: %.1f dB", _this->rf_steps[(int)_this->vhfRFGain]);
            SmGui::Text(buf);

            // VHF IF Gain (16 steps)
            if (SmGui::SliderInt(CONCAT("IF Gain Step##_sddc_vhf_if_gain_", _this->name),
                               (int*)&_this->vhfIFGain, 0, _this->if_steps_count - 1)) {
                if (_this->running && handler) {
                    handler->UpdateIFGain(_this->vhfIFGain);
                }
                if (_this->selectedDevName != "") {
                    config.acquire();
                    config.conf["devices"][_this->selectedDevName]["vhfIFGain"] = _this->vhfIFGain;
                    config.release(true);
                }
            }
            sprintf(buf, "IF Gain: %.1f dB", _this->if_steps[(int)_this->vhfIFGain]);
            SmGui::Text(buf);
        }
    }
    else {
        // HF Mode Gain Controls
        // RF Attenuation (-31.5dB to 0dB, 0.5dB steps)
        if (SmGui::SliderFloatWithSteps(CONCAT("RF Attenuation##_sddc_hf_rf_att_", _this->name),
                                      &_this->hfRFGain, -31.5f, 0.0f, 0.5f,
                                      SmGui::FMT_STR_FLOAT_ONE_DECIMAL)) {
            if (_this->running && handler) {
                handler->UpdateattRF(abs(_this->hfRFGain * 2)); // Convert to steps
            }
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["hfRFGain"] = _this->hfRFGain;
                config.release(true);
            }
        }

        // IF Gain Mode selection
        if (SmGui::Checkbox(CONCAT("High Gain Mode##_sddc_hf_gain_mode_", _this->name), &_this->hfGainMode)) {
            if (_this->running && handler) {
                handler->UpdateIFGain(_this->hfGainMode ? (_this->hfIFGain + 0x80) : _this->hfIFGain);
            }
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["hfGainMode"] = _this->hfGainMode;
                config.release(true);
            }
        }

        // IF Gain Level
        int maxSteps = _this->hfGainMode ? 45 : 18; // Different ranges for HIGH/LOW modes
        if (SmGui::SliderInt(CONCAT("IF Gain Level##_sddc_hf_if_gain_", _this->name),
                           (int*)&_this->hfIFGain, 0, maxSteps)) {
            if (_this->running && handler) {
                handler->UpdateIFGain(_this->hfGainMode ? (_this->hfIFGain + 0x80) : _this->hfIFGain);
            }
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["hfIFGain"] = _this->hfIFGain;
                config.release(true);
            }
        }
    }
    SmGui::EndGroup();

    // ADC Settings
    SmGui::BeginGroup();
    SmGui::Text("ADC Settings");
    if (SmGui::Checkbox(CONCAT("Dither##_sddc_dither_", _this->name), &_this->dither)) {
        if (_this->running && handler) {
            handler->UptDither(_this->dither);
        }
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["devices"][_this->selectedDevName]["dither"] = _this->dither;
            config.release(true);
        }
    }

    if (SmGui::Checkbox(CONCAT("Randomizer##_sddc_random_", _this->name), &_this->random)) {
        if (_this->running && handler) {
            handler->UptRand(_this->random);
        }
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["devices"][_this->selectedDevName]["random"] = _this->random;
            config.release(true);
        }
    }
    SmGui::EndGroup();

    // Bias-T Controls
    SmGui::BeginGroup();
    SmGui::Text("Bias-T");
    if (SmGui::Checkbox(CONCAT("HF##_sddc_hf_bias_", _this->name), &_this->hfBias)) {
        if (_this->running && handler) {
            handler->UpdBiasT_HF(_this->hfBias);
        }
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["devices"][_this->selectedDevName]["hfBias"] = _this->hfBias;
            config.release(true);
        }
    }

    SmGui::SameLine();
    if (SmGui::Checkbox(CONCAT("VHF##_sddc_vhf_bias_", _this->name), &_this->vhfBias)) {
        if (_this->running && handler) {
            handler->UpdBiasT_VHF(_this->vhfBias);
        }
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["devices"][_this->selectedDevName]["vhfBias"] = _this->vhfBias;
            config.release(true);
        }
    }
    SmGui::EndGroup();

    // Frequency Correction
    if (!_this->running) {
        if (SmGui::SliderFloatWithSteps(CONCAT("PPM##_sddc_ppm_", _this->name), 
                                      &_this->freqCorr, 
                                      -100.0f, 
                                      100.0f,
                                      0.1f,
                                      SmGui::FMT_STR_FLOAT_ONE_DECIMAL)) {
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["freqCorr"] = _this->freqCorr;
                config.release(true);
            }
        }
    }
}

MOD_EXPORT void _INIT_() {
    json def = json({});
    def["devices"] = json({});
    def["device"] = "";
    config.setPath(core::args["root"].s() + "/sddc_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new SDDCSourceModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (SDDCSourceModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}