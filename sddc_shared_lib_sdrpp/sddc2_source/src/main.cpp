#include <module.h>
#include <utils/flog.h>
#include <signal_path/signal_path.h>
#include <core.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <config.h>   
#include <gui/widgets/stepped_slider.h>
#include <gui/smgui.h>
#include <libsddc.h>
#include <thread>
#include <string.h>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "sddc2_source",
    /* Description:     */ "SDDC2 source module for SDR++",
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
    static void dataHandler(uint32_t size, uint8_t* data, void* ctx);
    static void menuHandler(void* ctx);

    std::string name;
    sddc_t* sddc = nullptr;
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
    RFMode rfMode = HF_MODE;

    // HF Mode gains
    float hfRFGain = 0.0f;    // -31.5 to 0 dB
    float hfIFGain = 0.0f;    // LOW_MODE(0-18) or HIGH_MODE(19+)
    bool hfGainMode = false;   // false=LOW_MODE, true=HIGH_MODE

    // VHF Mode gains
    float vhfRFGain = 0.0f;   
    float vhfIFGain = 0.0f;   
    bool gainAuto = true;

    bool dither = false;
    bool random = false;
    bool hfBias = false;
    bool vhfBias = false;
    float freqCorr = 0.0f;

    std::vector<std::string> devNames;
    std::string devListTxt;
    std::string sampleRateListTxt;

    std::vector<float> convBuffer;
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

    sigpath::sourceManager.registerSource("SDDC2", &handler);
}

SDDCSourceModule::~SDDCSourceModule() {
    stop(this);
    sigpath::sourceManager.unregisterSource("SDDC2");
    if (sddc) sddc_close(sddc);
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

    // Get device count and info
    devCount = sddc_get_device_count();
    if (devCount <= 0) {
        flog::warn("No SDDC devices found");
        return;
    }

    struct sddc_device_info* device_list;
    int count = sddc_get_device_info(&device_list);
    if (count > 0) {
        devNames.push_back(device_list->product);
        devListTxt = std::string(device_list->product) + '\0';
        sddc_free_device_info(device_list);
    }
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

    if (sddc) {
        sddc_close(sddc);
    }

    sddc = sddc_open(id, "SDDC_FX3.img");  // Note: path to firmware needs to be configurable
    if (!sddc) {
        selectedDevName = "";
        return;
    }

    // Configure async transfer parameters
    sddc_set_async_params(sddc, 32768, 15, dataHandler, this);

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

    // Initial device setup
    sddc_set_adc_dither(sddc, dither);
    sddc_set_adc_random(sddc, random);
    sddc_set_hf_bias(sddc, hfBias);
    sddc_set_vhf_bias(sddc, vhfBias);

    if (rfMode == VHF_MODE) {
        sddc_set_rf_mode(sddc, VHF_MODE);
        if (!gainAuto) {
            sddc_set_tuner_rf_attenuation(sddc, vhfRFGain);
            sddc_set_tuner_if_attenuation(sddc, vhfIFGain);
        }
    } else {
        sddc_set_rf_mode(sddc, HF_MODE);
        sddc_set_hf_attenuation(sddc, hfRFGain);
        // Note: HF IF gain setting needs to be implemented in libsddc
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
    if (conf.contains("rfMode")) rfMode = (RFMode)(int)conf["rfMode"];
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
    if (_this->running || _this->selectedDevName == "" || !_this->sddc) return;

    // Set sample rate
    sddc_set_sample_rate(_this->sddc, _this->sampleRate);

    // Start streaming
    sddc_start_streaming(_this->sddc);
    _this->running = true;
}

void SDDCSourceModule::stop(void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
    if (!_this->running || !_this->sddc) return;

    sddc_stop_streaming(_this->sddc);
    _this->running = false;
}

void SDDCSourceModule::tune(double freq, void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;
    if (!_this->running || !_this->sddc) return;

    // Check if mode needs to be changed based on frequency
    RFMode newMode = (freq >= 64000000) ? VHF_MODE : HF_MODE;
    if (newMode != _this->rfMode) {
        _this->rfMode = newMode;
        sddc_set_rf_mode(_this->sddc, newMode);

        if (_this->rfMode == VHF_MODE) {
            if (!_this->gainAuto) {
                sddc_set_tuner_rf_attenuation(_this->sddc, _this->vhfRFGain);
                sddc_set_tuner_if_attenuation(_this->sddc, _this->vhfIFGain);
            }
        } else {
            sddc_set_hf_attenuation(_this->sddc, _this->hfRFGain);
            // Note: HF IF gain setting needs to be implemented in libsddc
        }
    }

    // Set frequency
    sddc_set_tuner_frequency(_this->sddc, freq);
    _this->freq = freq;
}

void SDDCSourceModule::dataHandler(uint32_t size, uint8_t* data, void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;

    // Ensure conversion buffer is large enough
    if (_this->convBuffer.size() < size) {
        _this->convBuffer.resize(size);
    }

    // Convert data to float samples
    for (uint32_t i = 0; i < size; i++) {
        _this->convBuffer[i] = (float)((int8_t*)data)[i] / 128.0f;
    }

    int count = size / 2;  // Because data contains I/Q pairs
    for (int i = 0; i < count; i++) {
        _this->stream.writeBuf[i].re = _this->convBuffer[i*2];
        _this->stream.writeBuf[i].im = _this->convBuffer[i*2 + 1];
    }

    if (!_this->stream.swap(count)) return;
}

void SDDCSourceModule::menuHandler(void* ctx) {
    SDDCSourceModule* _this = (SDDCSourceModule*)ctx;

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
    sprintf(buf, "Current Mode: %s", _this->rfMode == VHF_MODE ? "VHF" : "HF");
    SmGui::Text(buf);

    // Gain Controls
    SmGui::BeginGroup();
    SmGui::Text("Gain Control");

    if (_this->rfMode == VHF_MODE) {
        // VHF Mode Gain Controls
        if (SmGui::Checkbox(CONCAT("Auto Gain##_sddc_auto_gain_", _this->name), &_this->gainAuto)) {
            if (_this->running && _this->sddc) {
                if (_this->gainAuto) {
                    _this->vhfRFGain = 0.0f;
                    _this->vhfIFGain = 0.0f;
                    sddc_set_tuner_rf_attenuation(_this->sddc, _this->vhfRFGain);
                    sddc_set_tuner_if_attenuation(_this->sddc, _this->vhfIFGain);
                }
            }
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["gainAuto"] = _this->gainAuto;
                config.release(true);
            }
        }

        if (!_this->gainAuto) {
            // VHF RF Gain
            if (SmGui::SliderFloat(CONCAT("RF Gain##_sddc_vhf_rf_gain_", _this->name),
                               &_this->vhfRFGain, 0.0f, 30.0f)) {
                if (_this->running && _this->sddc) {
                    sddc_set_tuner_rf_attenuation(_this->sddc, _this->vhfRFGain);
                }
                if (_this->selectedDevName != "") {
                    config.acquire();
                    config.conf["devices"][_this->selectedDevName]["vhfRFGain"] = _this->vhfRFGain;
                    config.release(true);
                }
            }

            // VHF IF Gain
            if (SmGui::SliderFloat(CONCAT("IF Gain##_sddc_vhf_if_gain_", _this->name),
                               &_this->vhfIFGain, 0.0f, 15.0f)) {
                if (_this->running && _this->sddc) {
                    sddc_set_tuner_if_attenuation(_this->sddc, _this->vhfIFGain);
                }
                if (_this->selectedDevName != "") {
                    config.acquire();
                    config.conf["devices"][_this->selectedDevName]["vhfIFGain"] = _this->vhfIFGain;
                    config.release(true);
                }
            }
        }
    }
    else {
        // HF Mode Gain Controls
        if (SmGui::SliderFloatWithSteps(CONCAT("RF Attenuation##_sddc_hf_rf_att_", _this->name),
                                      &_this->hfRFGain, -31.5f, 0.0f, 0.5f,
                                      SmGui::FMT_STR_FLOAT_ONE_DECIMAL)) {
            if (_this->running && _this->sddc) {
                sddc_set_hf_attenuation(_this->sddc, _this->hfRFGain);
            }
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["hfRFGain"] = _this->hfRFGain;
                config.release(true);
            }
        }

        if (SmGui::Checkbox(CONCAT("High Gain Mode##_sddc_hf_gain_mode_", _this->name), &_this->hfGainMode)) {
            if (_this->selectedDevName != "") {
                config.acquire();
                config.conf["devices"][_this->selectedDevName]["hfGainMode"] = _this->hfGainMode;
                config.release(true);
            }
        }

        int maxSteps = _this->hfGainMode ? 45 : 18;
        if (SmGui::SliderInt(CONCAT("IF Gain Level##_sddc_hf_if_gain_", _this->name),
                           (int*)&_this->hfIFGain, 0, maxSteps)) {
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
        if (_this->running && _this->sddc) {
            sddc_set_adc_dither(_this->sddc, _this->dither);
        }
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["devices"][_this->selectedDevName]["dither"] = _this->dither;
            config.release(true);
        }
    }

    if (SmGui::Checkbox(CONCAT("Randomizer##_sddc_random_", _this->name), &_this->random)) {
        if (_this->running && _this->sddc) {
            sddc_set_adc_random(_this->sddc, _this->random);
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
    bool biasChanged = false;
    
    if (SmGui::Checkbox(CONCAT("HF##_sddc_hf_bias_", _this->name), &_this->hfBias)) {
        if (_this->running && _this->sddc) {
            sddc_set_hf_bias(_this->sddc, _this->hfBias);
        }
        if (_this->selectedDevName != "") {
            config.acquire();
            config.conf["devices"][_this->selectedDevName]["hfBias"] = _this->hfBias;
            config.release(true);
        }
    }

    SmGui::SameLine();
    if (SmGui::Checkbox(CONCAT("VHF##_sddc_vhf_bias_", _this->name), &_this->vhfBias)) {
        if (_this->running && _this->sddc) {
            sddc_set_vhf_bias(_this->sddc, _this->vhfBias);
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
    config.setPath(core::args["root"].s() + "/sddc2_config.json");
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