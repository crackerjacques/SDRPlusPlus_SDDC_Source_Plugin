#include <imgui.h>
#include <module.h>
#include <gui/gui.h>
#include <gui/file_dialogs.h>
#include <signal_path/signal_path.h>
#include <signal_path/sink.h>
#include <dsp/buffer/packer.h>
#include <dsp/convert/stereo_to_mono.h>
#include <utils/flog.h>
#include <RtAudio.h>
#include <config.h>
#include <core.h>

#ifdef USE_JUCE
    #define JUCE_GLOBAL_MODULE_SETTINGS_INCLUDED 1
    #if !defined(JUCE_STANDALONE_APPLICATION)
        #define JUCE_STANDALONE_APPLICATION 0
    #endif
    
    #include <juce_core/juce_core.h>
    #include <juce_events/juce_events.h>
    #include <juce_data_structures/juce_data_structures.h>
    
    #include <juce_audio_basics/juce_audio_basics.h>
    #include <juce_audio_devices/juce_audio_devices.h>
    #include <juce_audio_formats/juce_audio_formats.h>
    #include <juce_audio_processors/juce_audio_processors.h>
    
    #include <juce_gui_basics/juce_gui_basics.h>
    #include <juce_gui_extra/juce_gui_extra.h>
#endif

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO {
    /* Name:            */ "fxaudio_sink",
    /* Description:     */ "Audio sink module with FX chain for SDR++",
    /* Author:          */ "Jack Heinlein",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ConfigManager config;

class PluginSlot {
public:
    enum class PluginType {
        VST,
        VST3,
        LV2
    };

private:
    double currentSampleRate = 48000.0;
    int currentBlockSize = 512;
    PluginType currentType = PluginType::VST;
    
    #ifdef USE_JUCE
    std::unique_ptr<juce::AudioPluginInstance> processor;
    std::unique_ptr<juce::AudioProcessorEditor> editor;
    #endif
    bool bypassed = false;
    std::string currentPluginPath;

#ifdef USE_JUCE
    void showEditor() {
        if (!processor || !processor->hasEditor()) return;

        try {
            editor.reset(processor->createEditorIfNeeded());
            if (editor) {
                juce::ComponentPeer::StyleFlags style = 
                    static_cast<juce::ComponentPeer::StyleFlags>(
                        juce::ComponentPeer::StyleFlags::windowHasTitleBar |
                        juce::ComponentPeer::StyleFlags::windowIsResizable
                    );
                
                editor->addToDesktop(style);
                editor->setVisible(true);
                editor->setBounds(100, 100, editor->getWidth(), editor->getHeight());
                
                if (auto* peer = editor->getPeer()) {
                    peer->setBounds(editor->getBounds(), false);
                    peer->setAlwaysOnTop(true);
                }
                editor->toFront(true);
            }
        }
        catch (const std::exception& e) {
            flog::error("Error showing plugin editor: {}", e.what());
        }
    }

    void hideEditor() {
        if (editor) {
            try {
                if (auto* peer = editor->getPeer()) {
                    peer->setAlwaysOnTop(false);
                }
                editor->setVisible(false);
                editor->removeFromDesktop();
                editor.reset();
            }
            catch (const std::exception& e) {
                flog::error("Error hiding plugin editor: {}", e.what());
            }
        }
    }
    #endif

public:
    bool isEditorVisible = false;

    PluginSlot() = default;
    ~PluginSlot() {
        unloadPlugin();
    }

    void updateProcessingDetails(double sampleRate, int blockSize) {
    #ifdef USE_JUCE
        currentSampleRate = sampleRate;
        currentBlockSize = blockSize;
        if (processor) {
            processor->releaseResources();
            processor->prepareToPlay(sampleRate, blockSize);
        }
    #endif
    }

    PluginType getCurrentType() const { return currentType; }
    void setPluginType(PluginType type) { currentType = type; }

    std::string openFileDialog() {
        switch (currentType) {
            case PluginType::VST: {
                pfd::open_file dialog("Load VST Plugin", "",
                    { 
                        "VST Plugin", "*.so *.dll",
                        "All Files", "*"
                    });
                auto result = dialog.result();
                if (!result.empty()) {
                    return result[0];
                }
                break;
            }
            case PluginType::VST3:
            case PluginType::LV2: {
                const char* title = (currentType == PluginType::VST3) ? 
                    "Select VST3 Plugin Directory" : "Select LV2 Plugin Directory";
                const char* extension = (currentType == PluginType::VST3) ? 
                    ".vst3" : ".lv2";
                    
                pfd::select_folder dialog(title);
                auto result = dialog.result();
                if (!result.empty()) {
                    if (result.length() >= strlen(extension) &&
                        result.compare(result.length() - strlen(extension), strlen(extension), extension) == 0) {
                        return result;
                    }
                    flog::error("Selected folder must end with {}", extension);
                }
                break;
            }
        }
        return "";
    }

    bool loadPlugin(const std::string& path) {
        unloadPlugin();
        currentPluginPath = path;
        
        if (path.empty()) return true;

#ifdef USE_JUCE
        try {
            if (!juce::MessageManager::getInstance())
                juce::MessageManager::getInstance()->setCurrentThreadAsMessageThread();
            
            juce::AudioPluginFormatManager formatManager;
            formatManager.addDefaultFormats();
            
            juce::OwnedArray<juce::PluginDescription> typesFound;
            for (auto* format : formatManager.getFormats()) {
                format->findAllTypesForFile(typesFound, path);
            }

            if (typesFound.isEmpty()) {
                flog::error("No plugin found at path: {}", path);
                return false;
            }

            juce::String errorMessage;
            processor = formatManager.createPluginInstance(
                *typesFound.getFirst(),
                currentSampleRate,
                currentBlockSize,
                errorMessage
            );
                
            if (!processor) {
                flog::error("Failed to load plugin: {}", errorMessage.toStdString());
                return false;
            }

            processor->prepareToPlay(currentSampleRate, currentBlockSize);
            processor->setNonRealtime(false);
            return true;
        }
        catch (const std::exception& e) {
            flog::error("Error loading plugin: {}", e.what());
            return false;
        }
#else
        return false;
#endif
    }

void process(float* buffer, int numSamples) {
#ifdef USE_JUCE
        if (!processor || bypassed) return;

        const juce::ScopedLock sl(processor->getCallbackLock());
        
        if (!processor->isSuspended()) {
            juce::AudioBuffer<float> audioBuffer(2, numSamples);
            
            for (int i = 0; i < numSamples; i++) {
                audioBuffer.setSample(0, i, buffer[i * 2]);
                audioBuffer.setSample(1, i, buffer[i * 2 + 1]);
            }

            juce::MidiBuffer midiMessages;
            processor->processBlock(audioBuffer, midiMessages);

            for (int i = 0; i < numSamples; i++) {
                buffer[i * 2] = audioBuffer.getSample(0, i);
                buffer[i * 2 + 1] = audioBuffer.getSample(1, i);
            }
        }
#endif
    }

    void toggleEditor() {
#ifdef USE_JUCE
        if (isEditorVisible) {
            hideEditor();
        }
        else {
            showEditor();
        }
        isEditorVisible = !isEditorVisible;
#endif
    }

    void unloadPlugin() {
#ifdef USE_JUCE
        hideEditor();
        if (processor) {
            processor->releaseResources();
            processor = nullptr;
        }
#endif
        currentPluginPath.clear();
    }

    bool isLoaded() const {
#ifdef USE_JUCE
        return processor != nullptr;
#else
        return false;
#endif
    }

    bool isBypassed() const {
        return bypassed;
    }

    void setBypassed(bool value) {
        bypassed = value;
    }

    const char* getPluginName() const {
#ifdef USE_JUCE
        if (processor) {
            return processor->getName().toRawUTF8();
        }
#endif
        return "No Plugin";
    }
};
class FXAudioSink : public SinkManager::Sink {
private:
    SinkManager::Stream* _stream;
    dsp::convert::StereoToMono s2m;
    dsp::buffer::Packer<float> monoPacker;
    dsp::buffer::Packer<dsp::stereo_t> stereoPacker;

    std::string _streamName;

    int srId = 0;
    int devCount;
    int devId = 0;
    bool running = false;

    unsigned int defaultDevId = 0;

    std::vector<RtAudio::DeviceInfo> devList;
    std::vector<unsigned int> deviceIds;
    std::string txtDevList;

    std::vector<unsigned int> sampleRates;
    std::string sampleRatesTxt;
    unsigned int sampleRate = 48000;

    RtAudio audio;
    std::array<std::unique_ptr<PluginSlot>, 4> pluginSlots;

    bool doStart() {
        RtAudio::StreamParameters parameters;
        parameters.deviceId = deviceIds[devId];
        parameters.nChannels = 2;
        unsigned int bufferFrames = sampleRate / 60;
        RtAudio::StreamOptions opts;
        opts.flags = RTAUDIO_MINIMIZE_LATENCY;
        opts.streamName = _streamName;

        try {
            audio.openStream(&parameters, NULL, RTAUDIO_FLOAT32, 
                           sampleRate, &bufferFrames, 
                           &callback, this, &opts);
            stereoPacker.setSampleCount(bufferFrames);
            audio.startStream();
            stereoPacker.start();
        }
        catch (const std::exception& e) {
            flog::error("Could not open audio device: {}", e.what());
            return false;
        }

        flog::info("RtAudio stream opened");
        return true;
    }

    void doStop() {
        s2m.stop();
        monoPacker.stop();
        stereoPacker.stop();
        monoPacker.out.stopReader();
        stereoPacker.out.stopReader();
        audio.stopStream();
        audio.closeStream();
        monoPacker.out.clearReadStop();
        stereoPacker.out.clearReadStop();
    }

    static int callback(void* outputBuffer, void* inputBuffer, 
                       unsigned int nBufferFrames, double streamTime, 
                       RtAudioStreamStatus status, void* userData) {
        FXAudioSink* _this = (FXAudioSink*)userData;
        int count = _this->stereoPacker.out.read();
        if (count < 0) { return 0; }

        float* outBuf = (float*)outputBuffer;
        memcpy(outBuf, _this->stereoPacker.out.readBuf, nBufferFrames * sizeof(dsp::stereo_t));

        for (auto& slot : _this->pluginSlots) {
            if (slot && !slot->isBypassed()) {
                slot->process(outBuf, nBufferFrames);
            }
        }

        _this->stereoPacker.out.flush();
        return 0;
    }

#if RTAUDIO_VERSION_MAJOR >= 6
    static void errorCallback(RtAudioErrorType type, const std::string& errorText) {
        switch (type) {
        case RtAudioErrorType::RTAUDIO_NO_ERROR:
            return;
        case RtAudioErrorType::RTAUDIO_WARNING:
        case RtAudioErrorType::RTAUDIO_NO_DEVICES_FOUND:
        case RtAudioErrorType::RTAUDIO_DEVICE_DISCONNECT:
            flog::warn("FXAudioSink Warning: {} ({})", errorText, (int)type);
            break;
        default:
            throw std::runtime_error(errorText);
        }
    }
#endif

public:
    FXAudioSink(SinkManager::Stream* stream, std::string streamName) {
        _stream = stream;
        _streamName = streamName;
        s2m.init(_stream->sinkOut);
        monoPacker.init(&s2m.out, 512);
        stereoPacker.init(_stream->sinkOut, 512);

#if RTAUDIO_VERSION_MAJOR >= 6
        audio.setErrorCallback(&errorCallback);
#endif

        for (int i = 0; i < 4; i++) {
            pluginSlots[i] = std::make_unique<PluginSlot>();
        }

        bool created = false;
        std::string device = "";
        config.acquire();
        if (!config.conf.contains(_streamName)) {
            created = true;
            config.conf[_streamName]["device"] = "";
            config.conf[_streamName]["devices"] = json({});
            config.conf[_streamName]["plugins"] = json({});
        }
        device = config.conf[_streamName]["device"];

        if (config.conf[_streamName].contains("plugins")) {
            for (int i = 0; i < 4; i++) {
                std::string slotKey = "slot" + std::to_string(i);
                if (config.conf[_streamName]["plugins"].contains(slotKey)) {
                    std::string pluginPath = config.conf[_streamName]["plugins"][slotKey];
                    if (!pluginPath.empty()) {
                        pluginSlots[i]->loadPlugin(pluginPath);
                        if (config.conf[_streamName]["plugins"].contains(slotKey + "_bypassed")) {
                            pluginSlots[i]->setBypassed(config.conf[_streamName]["plugins"][slotKey + "_bypassed"]);
                        }
                        if (config.conf[_streamName]["plugins"].contains(slotKey + "_type")) {
                            int type = config.conf[_streamName]["plugins"][slotKey + "_type"];
                            pluginSlots[i]->setPluginType(static_cast<PluginSlot::PluginType>(type));
                        }
                    }
                }
            }
        }

config.release(created);

        RtAudio::DeviceInfo info;
#if RTAUDIO_VERSION_MAJOR >= 6
        for (int i : audio.getDeviceIds()) {
#else
        int count = audio.getDeviceCount();
        for (int i = 0; i < count; i++) {
#endif
            try {
                info = audio.getDeviceInfo(i);
#if !defined(RTAUDIO_VERSION_MAJOR) || RTAUDIO_VERSION_MAJOR < 6
                if (!info.probed) { continue; }
#endif
                if (info.outputChannels < 2) { continue; }
                if (info.isDefaultOutput) { defaultDevId = devList.size(); }
                devList.push_back(info);
                deviceIds.push_back(i);
                txtDevList += info.name;
                txtDevList += '\0';
            }
            catch (const std::exception& e) {
                flog::error("Error getting audio device ({}) info: {}", i, e.what());
            }
        }
        selectByName(device);
    }

    ~FXAudioSink() {
        stop();
    }

    void start() {
        if (running) { return; }
        running = doStart();
    }

    void stop() {
        if (!running) { return; }
        doStop();
        running = false;
    }

    void selectFirst() {
        selectById(defaultDevId);
    }

    void selectByName(std::string name) {
        for (int i = 0; i < devList.size(); i++) {
            if (devList[i].name == name) {
                selectById(i);
                return;
            }
        }
        selectFirst();
    }

    void selectById(int id) {
        devId = id;
        bool created = false;
        config.acquire();
        if (!config.conf[_streamName]["devices"].contains(devList[id].name)) {
            created = true;
            config.conf[_streamName]["devices"][devList[id].name] = devList[id].preferredSampleRate;
        }
        sampleRate = config.conf[_streamName]["devices"][devList[id].name];
        config.release(created);

        sampleRates = devList[id].sampleRates;
        sampleRatesTxt = "";
        char buf[256];
        bool found = false;
        unsigned int defaultId = 0;
        unsigned int defaultSr = devList[id].preferredSampleRate;
        for (int i = 0; i < sampleRates.size(); i++) {
            if (sampleRates[i] == sampleRate) {
                found = true;
                srId = i;
            }
            if (sampleRates[i] == defaultSr) {
                defaultId = i;
            }
            sprintf(buf, "%d", sampleRates[i]);
            sampleRatesTxt += buf;
            sampleRatesTxt += '\0';
        }
        if (!found) {
            sampleRate = defaultSr;
            srId = defaultId;
        }

        _stream->setSampleRate(sampleRate);

        int bufferSize = sampleRate / 60;
        for (auto& slot : pluginSlots) {
            slot->updateProcessingDetails(sampleRate, bufferSize);
        }

        if (running) { 
            doStop();
            doStart();
        }
    }

    void menuHandler() {
        float menuWidth = ImGui::GetContentRegionAvail().x;

        ImGui::SetNextItemWidth(menuWidth);
        if (ImGui::Combo(("##_fxaudio_sink_dev_" + _streamName).c_str(), &devId, txtDevList.c_str())) {
            selectById(devId);
            config.acquire();
            config.conf[_streamName]["device"] = devList[devId].name;
            config.release(true);
        }

        ImGui::SetNextItemWidth(menuWidth);
        if (ImGui::Combo(("##_fxaudio_sink_sr_" + _streamName).c_str(), &srId, sampleRatesTxt.c_str())) {
            sampleRate = sampleRates[srId];
            _stream->setSampleRate(sampleRate);
            if (running) {
                doStop();
                doStart();
            }
            config.acquire();
            config.conf[_streamName]["devices"][devList[devId].name] = sampleRate;
            config.release(true);
        }

        ImGui::Separator();
        ImGui::Text("Plugin Chain");
        
        for (int i = 0; i < 4; i++) {
            std::string id = std::to_string(i + 1);
            ImGui::Text("Slot %d: %s", i + 1, pluginSlots[i]->getPluginName());

            if (!pluginSlots[i]->isLoaded()) {
                const char* types[] = { "VST", "VST3", "LV2" };
                int currentType = static_cast<int>(pluginSlots[i]->getCurrentType());
                
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.3f);
                if (ImGui::Combo(CONCAT("##type_", id), &currentType, types, IM_ARRAYSIZE(types))) {
                    pluginSlots[i]->setPluginType(static_cast<PluginSlot::PluginType>(currentType));
                    config.acquire();
                    config.conf[_streamName]["plugins"]["slot" + std::to_string(i) + "_type"] = currentType;
                    config.release(true);
                }
                
                ImGui::SameLine();
                
                if (ImGui::Button(CONCAT("Load Plugin##slot_", id))) {
                    std::string path = pluginSlots[i]->openFileDialog();
                    if (!path.empty()) {
                        if (pluginSlots[i]->loadPlugin(path)) {
                            config.acquire();
                            config.conf[_streamName]["plugins"]["slot" + std::to_string(i)] = path;
                            config.release(true);
                        }
                    }
                }
            }
            else {
                if (ImGui::Button(CONCAT("Unload##slot_", id))) {
                    pluginSlots[i]->unloadPlugin();
                    config.acquire();
                    config.conf[_streamName]["plugins"]["slot" + std::to_string(i)] = "";
                    config.release(true);
                }

                ImGui::SameLine();

                if (ImGui::Button(CONCAT(pluginSlots[i]->isEditorVisible ? 
                                   "Hide##edit_" : "View##edit_", id))) {
                    pluginSlots[i]->toggleEditor();
                }

                ImGui::SameLine();

                bool bypassed = pluginSlots[i]->isBypassed();
                if (ImGui::Checkbox(CONCAT("Bypass##bypass_", id), &bypassed)) {
                    pluginSlots[i]->setBypassed(bypassed);
                    config.acquire();
                    config.conf[_streamName]["plugins"]["slot" + std::to_string(i) + "_bypassed"] = bypassed;
                    config.release(true);
                }
            }

            ImGui::Separator();
        }
    }
};

class FXAudioSinkModule : public ModuleManager::Instance {
public:
    FXAudioSinkModule(std::string name) {
        this->name = name;
        provider.create = create_sink;
        provider.ctx = this;

        sigpath::sinkManager.registerSinkProvider("FX Audio", provider);
    }

    ~FXAudioSinkModule() {
        sigpath::sinkManager.unregisterSinkProvider("FX Audio");
    }

    void postInit() {}

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

private:
    static SinkManager::Sink* create_sink(SinkManager::Stream* stream, std::string streamName, void* ctx) {
        return (SinkManager::Sink*)(new FXAudioSink(stream, streamName));
    }

    std::string name;
    bool enabled = true;
    SinkManager::SinkProvider provider;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    def["devices"] = json({});
    def["device"] = "";
    def["plugins"] = json({});
    config.setPath(core::args["root"].s() + "/fxaudio_sink_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new FXAudioSinkModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(ModuleManager::Instance* instance) {
    delete (FXAudioSinkModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
