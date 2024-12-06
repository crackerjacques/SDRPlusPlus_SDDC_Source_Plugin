#include <imgui.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <signal_path/sink.h>
#include <dsp/buffer/packer.h>
#include <dsp/convert/stereo_to_mono.h>
#include <utils/flog.h>
#include <rtaudio/RtAudio.h>
#include <rnnoise.h>
#include <config.h>
#include <core.h>
#include <dirent.h>
#include <deque>
#include <memory>
#include <algorithm>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "nraudio_sink",
    /* Description:     */ "Audio sink module with noise reduction for SDR++",
    /* Author:          */ "Jack Heinlein",
    /* Version:         */ 0, 1, 2,
    /* Max instances    */ 1
};

ConfigManager config;

class NRAudioSink : public SinkManager::Sink {
public:
    struct ModelInfo {
        std::string name;
        std::string path;
    };

    NRAudioSink(SinkManager::Stream* stream, std::string streamName) {
        _stream = stream;
        _streamName = streamName;
        s2m.init(_stream->sinkOut);
        monoPacker.init(&s2m.out, 512);
        stereoPacker.init(_stream->sinkOut, 512);

        rnnFrameSize = rnnoise_get_frame_size();
        flog::info("RNNoise frame size: {}", rnnFrameSize);
        processingBufferL.resize(rnnFrameSize);
        processingBufferR.resize(rnnFrameSize);

#if RTAUDIO_VERSION_MAJOR >= 6
        audio.setErrorCallback(&errorCallback);
#endif

        // Load models
        loadAvailableModels();

        // Load config
        config.acquire();
        if (!config.conf.contains(_streamName)) {
            config.conf[_streamName]["device"] = "";
            config.conf[_streamName]["devices"] = json({});
            config.conf[_streamName]["rnnoise_enabled"] = false;
            config.conf[_streamName]["mono_mode"] = true;
            config.conf[_streamName]["gain_alpha"] = gainAlpha;
            config.conf[_streamName]["noise_gate"] = noiseGate;
            config.conf[_streamName]["scale"] = scale;
            config.conf[_streamName]["min_gain"] = minGain;
            config.conf[_streamName]["max_gain"] = maxGain;
            config.conf[_streamName]["model_index"] = currentModelIndex;
            config.conf[_streamName]["vad_threshold"] = vadThreshold;
            config.conf[_streamName]["vad_grace_period_blocks"] = vadGracePeriodBlocks;
            config.conf[_streamName]["retroactive_vad_grace_blocks"] = retroactiveVADGraceBlocks;

            config.conf[_streamName]["defaults"]["gain_alpha"] = defaults.gainAlpha;
            config.conf[_streamName]["defaults"]["noise_gate"] = defaults.noiseGate;
            config.conf[_streamName]["defaults"]["scale"] = defaults.scale;
            config.conf[_streamName]["defaults"]["min_gain"] = defaults.minGain;
            config.conf[_streamName]["defaults"]["max_gain"] = defaults.maxGain;
            config.conf[_streamName]["defaults"]["vad_threshold"] = defaults.vadThreshold;
            config.conf[_streamName]["defaults"]["vad_grace_period_blocks"] = defaults.vadGracePeriodBlocks;
            config.conf[_streamName]["defaults"]["retroactive_vad_grace_blocks"] = defaults.retroactiveVADGraceBlocks;
        }

        device = config.conf[_streamName]["device"];
        rnNoiseEnabled = config.conf[_streamName]["rnnoise_enabled"];
        isMonoMode = config.conf[_streamName]["mono_mode"];
        gainAlpha = config.conf[_streamName]["gain_alpha"];
        noiseGate = config.conf[_streamName]["noise_gate"];
        scale = config.conf[_streamName]["scale"];
        minGain = config.conf[_streamName]["min_gain"];
        maxGain = config.conf[_streamName]["max_gain"];
        currentModelIndex = config.conf[_streamName]["model_index"];
        vadThreshold = config.conf[_streamName]["vad_threshold"];
        vadGracePeriodBlocks = config.conf[_streamName]["vad_grace_period_blocks"];
        retroactiveVADGraceBlocks = config.conf[_streamName]["retroactive_vad_grace_blocks"];

        if (config.conf[_streamName].contains("defaults")) {
            defaults.gainAlpha = config.conf[_streamName]["defaults"]["gain_alpha"];
            defaults.noiseGate = config.conf[_streamName]["defaults"]["noise_gate"];
            defaults.scale = config.conf[_streamName]["defaults"]["scale"];
            defaults.minGain = config.conf[_streamName]["defaults"]["min_gain"];
            defaults.maxGain = config.conf[_streamName]["defaults"]["max_gain"];
            defaults.vadThreshold = config.conf[_streamName]["defaults"]["vad_threshold"];
            defaults.vadGracePeriodBlocks = config.conf[_streamName]["defaults"]["vad_grace_period_blocks"];
            defaults.retroactiveVADGraceBlocks = config.conf[_streamName]["defaults"]["retroactive_vad_grace_blocks"];
        }
        config.release();

        // Initialize audio devices
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
                if (info.outputChannels == 0) { continue; }
                if (info.isDefaultOutput) { defaultDevId = devList.size(); }
                devList.push_back(info);
                deviceIds.push_back(i);
                txtDevList += info.name;
                txtDevList += '\0';
            }
            catch (const std::exception& e) {
                flog::error("NRAudioSink Error getting audio device ({}) info: {}", i, e.what());
            }
        }
        
        loadModel(currentModelIndex);
        selectByName(device);
    }

    ~NRAudioSink() {
        stop();
        if (rnNoiseStateL) rnnoise_destroy(rnNoiseStateL);
        if (rnNoiseStateR) rnnoise_destroy(rnNoiseStateR);
        if (loadedModel) rnnoise_model_free(loadedModel);
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

    void loadAvailableModels() {
        availableModels.push_back({"Default Model", ""});

        // Search model dir
        std::vector<std::string> searchPaths = {
            std::string(getenv("HOME")) + "/.config/sdrpp/rnnoise_models/",
            std::string(core::args["root"].s() + "/rnnoise_models/"),
            "/usr/share/sdrpp/rnnoise_models/",
            std::string(getenv("HOME")) + "/.local/share/sdrpp/rnnoise_models/"
        };

        for (const auto& basePath : searchPaths) {
            DIR* dir = opendir(basePath.c_str());
            if (!dir) continue;

            struct dirent* entry;
            while ((entry = readdir(dir)) != nullptr) {
                std::string filename = entry->d_name;
                if (filename.size() > 4 && 
                    filename.substr(filename.size() - 4) == ".bin") {
                    std::string modelName = filename.substr(0, filename.size() - 4);
                    availableModels.push_back({modelName, basePath + filename});
                }
            }
            closedir(dir);
        }
    }

    bool loadModel(int index) {
        if (index == 0) {
            if (loadedModel) {
                rnnoise_model_free(loadedModel);
                loadedModel = nullptr;
            }
            createDenoiseState();
            return true;
        }

        if (index < 0 || index >= availableModels.size()) {
            return false;
        }

        FILE* f = fopen(availableModels[index].path.c_str(), "rb");
        if (!f) {
            flog::error("Failed to open model file: {}", availableModels[index].path);
            return false;
        }

        if (loadedModel) {
            rnnoise_model_free(loadedModel);
            loadedModel = nullptr;
        }

        loadedModel = rnnoise_model_from_file(f);
        fclose(f);

        if (!loadedModel) {
            flog::error("Failed to load model from file");
            return false;
        }

        createDenoiseState();
        return true;
    }

    void menuHandler() {
        float menuWidth = ImGui::GetContentRegionAvail().x;

        // Audio device controls
        ImGui::SetNextItemWidth(menuWidth);
        if (ImGui::Combo(("##_nraudio_sink_dev_" + _streamName).c_str(), &devId, txtDevList.c_str())) {
            selectById(devId);
            config.acquire();
            config.conf[_streamName]["device"] = devList[devId].name;
            config.release(true);
        }

        ImGui::SetNextItemWidth(menuWidth);
        if (ImGui::Combo(("##_nraudio_sink_sr_" + _streamName).c_str(), &srId, sampleRatesTxt.c_str())) {
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

        // RNNoise controls
        if (ImGui::Checkbox(("Enable Noise Reduction##_nr_" + _streamName).c_str(), &rnNoiseEnabled)) {
            config.acquire();
            config.conf[_streamName]["rnnoise_enabled"] = rnNoiseEnabled;
            config.release(true);
        }

        if (rnNoiseEnabled) {
            if (ImGui::Checkbox(("Mono Mode##_mono_" + _streamName).c_str(), &isMonoMode)) {
                config.acquire();
                config.conf[_streamName]["mono_mode"] = isMonoMode;
                config.release(true);
                createDenoiseState();
            }

            std::string modelList;
            for (const auto& model : availableModels) {
                modelList += model.name;
                modelList += '\0';
            }

            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::Combo(("Model##_model_" + _streamName).c_str(), &currentModelIndex, modelList.c_str())) {
                if (loadModel(currentModelIndex)) {
                    config.acquire();
                    config.conf[_streamName]["model_index"] = currentModelIndex;
                    config.release(true);
                }
            }

            ImGui::Dummy(ImVec2(0.0f, 10.0f));

            auto addParameterControls = [&](const char* label, float& value, float& defaultValue, 
                                          float step, float min, float max, const char* tooltip,
                                          const char* format = "%.3f") {
                ImGui::PushID(label);
                std::string labelText = std::string(label);
                ImGui::SetNextItemWidth(menuWidth * 0.5f);
                bool changed = ImGui::SliderFloat(labelText.c_str(), &value, min, max, format);
                ImGui::SameLine();
                if (ImGui::Button("-")) {
                    value = std::max(value - step, min);
                    changed = true;
                }
                ImGui::SameLine();
                if (ImGui::Button("+")) {
                    value = std::min(value + step, max);
                    changed = true;
                }
                if (changed) {
                    config.acquire();
                    config.conf[_streamName][label] = value;
                    config.release(true);
                }
                if (ImGui::IsItemHovered()) {
                    float diff = value - defaultValue;
                    ImGui::SetTooltip("%s\nDefault: %.6f\nCurrent: %.6f\nDiff: %+.6f", 
                                    tooltip, defaultValue, value, diff);
                }
                ImGui::PopID();
            };

            addParameterControls("gain_alpha", gainAlpha, defaults.gainAlpha, 
                               gainAlphaStep, 0.001f, 0.999f,
                               "Smoothing factor for gain changes\nHigher = smoother but slower response");
            
            addParameterControls("noise_gate", noiseGate, defaults.noiseGate,
                               noiseGateStep, 0.0f, 1.0f,
                               "Threshold for noise gate\nHigher = more aggressive noise reduction",
                               "%.6f");

            addParameterControls("min_gain", minGain, defaults.minGain,
                               gainStep, 0.0f, maxGain,
                               "Minimum gain during silence");

            addParameterControls("max_gain", maxGain, defaults.maxGain,
                               gainStep, minGain, 1.1f,
                               "Maximum gain during speech");

            ImGui::Dummy(ImVec2(0.0f, 10.0f));
            ImGui::Text("Voice Activity Detection (VAD) Settings");

            // VAD Threshold
            ImGui::SetNextItemWidth(menuWidth * 0.5f);
            if (ImGui::SliderFloat("VAD Threshold", &vadThreshold, 0.0f, 1.0f, "%.3f")) {
                config.acquire();
                config.conf[_streamName]["vad_threshold"] = vadThreshold;
                config.release(true);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Voice activity detection threshold\n"
                                "Higher values = more aggressive voice detection\n"
                                "Default: %.3f", defaults.vadThreshold);
            }

            // VAD Grace Period
            ImGui::SetNextItemWidth(menuWidth * 0.5f); 
            if (ImGui::SliderInt("VAD Grace Period", (int*)&vadGracePeriodBlocks, 20, 100)) {
                config.acquire();
                config.conf[_streamName]["vad_grace_period_blocks"] = vadGracePeriodBlocks;
                config.release(true);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Number of blocks to keep unmuted after voice detection\n"
                                "Higher values = longer tail after speech\n"
                                "Default: %d", defaults.vadGracePeriodBlocks);
            }

            // Retroactive VAD Grace
            ImGui::SetNextItemWidth(menuWidth * 0.5f);
            if (ImGui::SliderInt("Retroactive VAD Grace", (int*)&retroactiveVADGraceBlocks, 0, 99)) {
                config.acquire();
                config.conf[_streamName]["retroactive_vad_grace_blocks"] = retroactiveVADGraceBlocks;
                config.release(true);
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Number of past blocks to unmute when voice is detected\n"
                                "Higher values = less cutting of speech beginnings but more latency\n"
                                "Default: %d", defaults.retroactiveVADGraceBlocks);
            }

            ImGui::Dummy(ImVec2(0.0f, 10.0f));

            if (ImGui::Button(("Save Current as Defaults##_save_" + _streamName).c_str())) {
                defaults = {gainAlpha, noiseGate, scale, minGain, maxGain, 
                          vadThreshold, vadGracePeriodBlocks, retroactiveVADGraceBlocks};
                config.acquire();
                config.conf[_streamName]["defaults"]["gain_alpha"] = defaults.gainAlpha;
                config.conf[_streamName]["defaults"]["noise_gate"] = defaults.noiseGate;
                config.conf[_streamName]["defaults"]["scale"] = defaults.scale;
                config.conf[_streamName]["defaults"]["min_gain"] = defaults.minGain;
                config.conf[_streamName]["defaults"]["max_gain"] = defaults.maxGain;
                config.conf[_streamName]["defaults"]["vad_threshold"] = defaults.vadThreshold;
                config.conf[_streamName]["defaults"]["vad_grace_period_blocks"] = defaults.vadGracePeriodBlocks;
                config.conf[_streamName]["defaults"]["retroactive_vad_grace_blocks"] = defaults.retroactiveVADGraceBlocks;
                config.release(true);
            }

            ImGui::SameLine();
            if (ImGui::Button(("Reset to Saved Defaults##_reset_" + _streamName).c_str())) {
                gainAlpha = defaults.gainAlpha;
                noiseGate = defaults.noiseGate;
                scale = defaults.scale;
                minGain = defaults.minGain;
                maxGain = defaults.maxGain;
                vadThreshold = defaults.vadThreshold;
                vadGracePeriodBlocks = defaults.vadGracePeriodBlocks;
                retroactiveVADGraceBlocks = defaults.retroactiveVADGraceBlocks;
                config.acquire();
                config.conf[_streamName]["gain_alpha"] = gainAlpha;
                config.conf[_streamName]["noise_gate"] = noiseGate;
                config.conf[_streamName]["scale"] = scale;
                config.conf[_streamName]["min_gain"] = minGain;
                config.conf[_streamName]["max_gain"] = maxGain;
                config.conf[_streamName]["vad_threshold"] = vadThreshold;
                config.conf[_streamName]["vad_grace_period_blocks"] = vadGracePeriodBlocks;
                config.conf[_streamName]["retroactive_vad_grace_blocks"] = retroactiveVADGraceBlocks;
                config.release(true);
            }
        }
    }

private:
    void createDenoiseState() {
        if (rnNoiseStateL) {
            rnnoise_destroy(rnNoiseStateL);
            rnNoiseStateL = nullptr;
        }
        if (rnNoiseStateR) {
            rnnoise_destroy(rnNoiseStateR);
            rnNoiseStateR = nullptr;
        }

        rnNoiseStateL = rnnoise_create(loadedModel);
        if (!rnNoiseStateL) {
            flog::error("Failed to create left channel RNNoise state");
            return;
        }

        if (!isMonoMode) {
            rnNoiseStateR = rnnoise_create(loadedModel);
            if (!rnNoiseStateR) {
                flog::error("Failed to create right channel RNNoise state");
                rnnoise_destroy(rnNoiseStateL);
                rnNoiseStateL = nullptr;
                return;
            }
        }

        // Initialize circular buffers for retroactive unmuting
        retroactiveBufferL.clear();
        retroactiveBufferR.clear();
        retroactiveVADProbBuffer.clear();

        flog::info("Successfully created RNNoise states with {} (mono mode: {})", 
            availableModels[currentModelIndex].name, isMonoMode);
    }

    struct AudioBlock {
        std::vector<float> data;
        bool wasVoiceActive;
    };

    void processAudio(float* buffer, int frameCount) {
        if (!rnNoiseEnabled || !rnNoiseStateL) return;

        static int debugCounter = 0;
        debugCounter++;
        bool shouldDebug = (debugCounter % 100 == 0);

        if (shouldDebug) {
            flog::info("=== Audio Processing Debug ===");
            flog::info("Frame count: {}, RNN frame size: {}", frameCount, rnnFrameSize);
            flog::info("Mono mode: {}, States: L={:x}, R={:x}", 
                isMonoMode, (uintptr_t)rnNoiseStateL, (uintptr_t)rnNoiseStateR);
        }

        const float invScale = 1.0f / scale;

        for (int offset = 0; offset < frameCount; offset += rnnFrameSize) {
            int remainingFrames = std::min(rnnFrameSize, frameCount - offset);

            if (isMonoMode) {
                // Process mono audio
                for (int i = 0; i < remainingFrames; i++) {
                    float left = buffer[(offset + i) * 2];
                    float right = buffer[(offset + i) * 2 + 1];
                    processingBufferL[i] = (left + right) * 0.5f * scale;
                }

                if (shouldDebug) {
                    float rms = 0.0f;
                    for (int i = 0; i < remainingFrames; i++) {
                        float val = processingBufferL[i] * invScale;
                        rms += val * val;
                    }
                    rms = std::sqrt(rms / remainingFrames);
                    flog::info("Pre-process: RMS={:.6f}, first={:.6f}", 
                        rms, processingBufferL[0] * invScale);
                }

                float vadProb = rnnoise_process_frame(rnNoiseStateL, 
                                                    processingBufferL.data(), 
                                                    processingBufferL.data());

                // Store VAD probability for retroactive processing
                retroactiveVADProbBuffer.push_back(vadProb);
                if (retroactiveVADProbBuffer.size() > retroactiveVADGraceBlocks + 1) {
                    retroactiveVADProbBuffer.pop_front();
                }

                // Store audio block for potential retroactive unmuting
                AudioBlock block;
                block.data.assign(processingBufferL.begin(), 
                                processingBufferL.begin() + remainingFrames);
                block.wasVoiceActive = (vadProb >= vadThreshold);
                retroactiveBufferL.push_back(block);

                if (retroactiveBufferL.size() > retroactiveVADGraceBlocks + 1) {
                    retroactiveBufferL.pop_front();
                }

                // Process current block
                static float smoothedGain = minGain;
                static uint32_t blocksAfterVAD = 0;

                bool isVoiceActive = vadProb >= vadThreshold;
                if (isVoiceActive) {
                    blocksAfterVAD = 0;
                    
                    // Retroactively unmute previous blocks
                    if (retroactiveVADGraceBlocks > 0) {
                        size_t retroBlocks = std::min(static_cast<size_t>(retroactiveVADGraceBlocks),
                                                    retroactiveBufferL.size() > 0 ? retroactiveBufferL.size() - 1 : 0);
                        for (size_t i = 0; i < retroBlocks; i++) {
                            if (!retroactiveBufferL[i].wasVoiceActive) {
                                // Process and update the block
                                AudioBlock& retroBlock = retroactiveBufferL[i];
                                for (float& sample : retroBlock.data) {
                                    float current = sample * invScale;
                                    float targetGain = (std::abs(current) > noiseGate ? maxGain : minGain);
                                    smoothedGain = std::max(minGain, std::min(maxGain, 
                                                gainAlpha * smoothedGain + 
                                                (1.0f - gainAlpha) * targetGain));
                                    sample = current * smoothedGain;
                                }
                            }
                        }
                    }
                } else {
                    blocksAfterVAD++;
                }

                bool shouldBeUnmuted = isVoiceActive || 
                                     blocksAfterVAD < vadGracePeriodBlocks;

                for (int i = 0; i < remainingFrames; i++) {
                    float current = processingBufferL[i] * invScale;
                    float targetGain;
                    
                    if (shouldBeUnmuted) {
                        targetGain = (std::abs(current) > noiseGate ? maxGain : minGain);
                    } else {
                        targetGain = minGain;
                    }
                    
                    smoothedGain = std::max(minGain, std::min(maxGain, 
                                gainAlpha * smoothedGain + (1.0f - gainAlpha) * targetGain));
                    processingBufferL[i] = current * smoothedGain;
                }

                if (shouldDebug) {
                    float rms = 0.0f;
                    for (int i = 0; i < remainingFrames; i++) {
                        float val = processingBufferL[i];
                        rms += val * val;
                    }
                    rms = std::sqrt(rms / remainingFrames);
                    flog::info("Post-process: RMS={:.6f}, first={:.6f}, VAD={:.3f}, Gain={:.3f}", 
                        rms, processingBufferL[0], vadProb, smoothedGain);
                }

                for (int i = 0; i < remainingFrames; i++) {
                    buffer[(offset + i) * 2] = processingBufferL[i];
                    buffer[(offset + i) * 2 + 1] = processingBufferL[i];
                }
            } else {
                // Process stereo audio
                for (int i = 0; i < remainingFrames; i++) {
                    processingBufferL[i] = buffer[(offset + i) * 2] * scale;
                    processingBufferR[i] = buffer[(offset + i) * 2 + 1] * scale;
                }

                if (shouldDebug) {
                    float rmsL = 0.0f, rmsR = 0.0f;
                    for (int i = 0; i < remainingFrames; i++) {
                        float valL = processingBufferL[i] * invScale;
                        float valR = processingBufferR[i] * invScale;
                        rmsL += valL * valL;
                        rmsR += valR * valR;
                    }
                    rmsL = std::sqrt(rmsL / remainingFrames);
                    rmsR = std::sqrt(rmsR / remainingFrames);
                    flog::info("Pre-process L/R: RMS={:.6f}/{:.6f}, first={:.6f}/{:.6f}", 
                        rmsL, rmsR, processingBufferL[0] * invScale, processingBufferR[0] * invScale);
                }

                float vadProbL = rnnoise_process_frame(rnNoiseStateL, 
                                                     processingBufferL.data(), 
                                                     processingBufferL.data());
                float vadProbR = rnnoise_process_frame(rnNoiseStateR, 
                                                     processingBufferR.data(), 
                                                     processingBufferR.data());
                
                float combinedVADProb = std::max(vadProbL, vadProbR);
                retroactiveVADProbBuffer.push_back(combinedVADProb);
                if (retroactiveVADProbBuffer.size() > retroactiveVADGraceBlocks + 1) {
                    retroactiveVADProbBuffer.pop_front();
                }

                // Store audio blocks for potential retroactive unmuting
                AudioBlock blockL, blockR;
                blockL.data.assign(processingBufferL.begin(), 
                                 processingBufferL.begin() + remainingFrames);
                blockR.data.assign(processingBufferR.begin(), 
                                 processingBufferR.begin() + remainingFrames);
                blockL.wasVoiceActive = blockR.wasVoiceActive = 
                    (combinedVADProb >= vadThreshold);
                
                retroactiveBufferL.push_back(blockL);
                retroactiveBufferR.push_back(blockR);

                if (retroactiveBufferL.size() > retroactiveVADGraceBlocks + 1) {
                    retroactiveBufferL.pop_front();
                    retroactiveBufferR.pop_front();
                }

                static float smoothedGainL = minGain;
                static float smoothedGainR = minGain;
                static uint32_t blocksAfterVAD = 0;

                bool isVoiceActive = combinedVADProb >= vadThreshold;
                if (isVoiceActive) {
                    blocksAfterVAD = 0;
                    
                    if (retroactiveVADGraceBlocks > 0) {
                        size_t retroBlocks = std::min(static_cast<size_t>(retroactiveVADGraceBlocks),
                                                    retroactiveBufferL.size() > 0 ? retroactiveBufferL.size() - 1 : 0);
                        for (size_t i = 0; i < retroBlocks; i++) {
                            if (!retroactiveBufferL[i].wasVoiceActive) {
                                AudioBlock& retroBlockL = retroactiveBufferL[i];
                                AudioBlock& retroBlockR = retroactiveBufferR[i];
                                
                                for (size_t j = 0; j < retroBlockL.data.size(); j++) {
                                    float currentL = retroBlockL.data[j] * invScale;
                                    float currentR = retroBlockR.data[j] * invScale;
                                    
                                    float targetGainL = (std::abs(currentL) > noiseGate ? maxGain : minGain);
                                    float targetGainR = (std::abs(currentR) > noiseGate ? maxGain : minGain);
                                    
                                    smoothedGainL = std::max(minGain, std::min(maxGain, 
                                                gainAlpha * smoothedGainL + 
                                                (1.0f - gainAlpha) * targetGainL));
                                    smoothedGainR = std::max(minGain, std::min(maxGain, 
                                                gainAlpha * smoothedGainR + 
                                                (1.0f - gainAlpha) * targetGainR));
                                    
                                    retroBlockL.data[j] = currentL * smoothedGainL;
                                    retroBlockR.data[j] = currentR * smoothedGainR;
                                }
                            }
                        }
                    }
                } else {
                    blocksAfterVAD++;
                }

                bool shouldBeUnmuted = isVoiceActive || 
                                     blocksAfterVAD < vadGracePeriodBlocks;

                for (int i = 0; i < remainingFrames; i++) {
                    float currentL = processingBufferL[i] * invScale;
                    float currentR = processingBufferR[i] * invScale;
                    
                    float targetGainL, targetGainR;
                    if (shouldBeUnmuted) {
                        targetGainL = (std::abs(currentL) > noiseGate ? maxGain : minGain);
                        targetGainR = (std::abs(currentR) > noiseGate ? maxGain : minGain);
                    } else {
                        targetGainL = targetGainR = minGain;
                    }
                    
                    smoothedGainL = std::max(minGain, std::min(maxGain, 
                                gainAlpha * smoothedGainL + 
                                (1.0f - gainAlpha) * targetGainL));
                    smoothedGainR = std::max(minGain, std::min(maxGain, 
                                gainAlpha * smoothedGainR + 
                                (1.0f - gainAlpha) * targetGainR));
                    
                    processingBufferL[i] = currentL * smoothedGainL;
                    processingBufferR[i] = currentR * smoothedGainR;
                }

                if (shouldDebug) {
                    float rmsL = 0.0f, rmsR = 0.0f;
                    for (int i = 0; i < remainingFrames; i++) {
                        float valL = processingBufferL[i];
                        float valR = processingBufferR[i];
                        rmsL += valL * valL;
                        rmsR += valR * valR;
                    }
                    rmsL = std::sqrt(rmsL / remainingFrames);
                    rmsR = std::sqrt(rmsR / remainingFrames);

                    flog::info("Post-process L/R: RMS={:.6f}/{:.6f}, first={:.6f}/{:.6f}, VAD={:.3f}/{:.3f}, Gain={:.3f}/{:.3f}", 
                        rmsL, rmsR, processingBufferL[0], processingBufferR[0], 
                        vadProbL, vadProbR, smoothedGainL, smoothedGainR);
                }

                for (int i = 0; i < remainingFrames; i++) {
                    buffer[(offset + i) * 2] = processingBufferL[i];
                    buffer[(offset + i) * 2 + 1] = processingBufferR[i];
                }
            }
        }

        if (shouldDebug) {
            flog::info("=== Processing Complete ===\n");
        }
    }

    bool doStart() {
        RtAudio::StreamParameters parameters;
        parameters.deviceId = deviceIds[devId];
        parameters.nChannels = 2;
        unsigned int bufferFrames = sampleRate / 60;
        RtAudio::StreamOptions opts;
        opts.flags = RTAUDIO_MINIMIZE_LATENCY;
        opts.streamName = _streamName;

        try {
            audio.openStream(&parameters, NULL, RTAUDIO_FLOAT32, sampleRate, &bufferFrames, &callback, this, &opts);
            stereoPacker.setSampleCount(bufferFrames);
            audio.startStream();
            stereoPacker.start();
        }
        catch (const std::exception& e) {
            flog::error("Could not open audio device: {0}", e.what());
            return false;
        }
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

        if (running) {
            doStop();
            doStart();
        }
    }

    static int callback(void* outputBuffer, void* inputBuffer, unsigned int nBufferFrames, 
                       double streamTime, RtAudioStreamStatus status, void* userData) {
        NRAudioSink* _this = (NRAudioSink*)userData;
        int count = _this->stereoPacker.out.read();
        if (count < 0) { return 0; }

        if (_this->rnNoiseEnabled) {
            _this->processAudio((float*)_this->stereoPacker.out.readBuf, nBufferFrames);
        }

        memcpy(outputBuffer, _this->stereoPacker.out.readBuf, nBufferFrames * sizeof(dsp::stereo_t));
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
            flog::warn("NRAudioSink Warning: {} ({})", errorText, (int)type);
            break;
        default:
            throw std::runtime_error(errorText);
        }
    }
#endif

    SinkManager::Stream* _stream;
    std::string _streamName;
    dsp::convert::StereoToMono s2m;
    dsp::buffer::Packer<float> monoPacker;
    dsp::buffer::Packer<dsp::stereo_t> stereoPacker;

    // RNNoise related
    DenoiseState* rnNoiseStateL = nullptr;
    DenoiseState* rnNoiseStateR = nullptr;
    RNNModel* loadedModel = nullptr;
    bool rnNoiseEnabled = false;
    bool isMonoMode = true;
    int rnnFrameSize = 0;
    std::vector<float> processingBufferL;
    std::vector<float> processingBufferR;
    std::vector<ModelInfo> availableModels;
    int currentModelIndex = 0;

    // VAD related
    std::deque<AudioBlock> retroactiveBufferL;
    std::deque<AudioBlock> retroactiveBufferR;
    std::deque<float> retroactiveVADProbBuffer;
    float vadThreshold = 0.5f;
    uint32_t vadGracePeriodBlocks = 20;
    uint32_t retroactiveVADGraceBlocks = 0;

    // Parameters
    float gainAlpha = 0.98f;
    float noiseGate = 0.001f;
    float scale = 32768.0f;
    float minGain = 0.1f;
    float maxGain = 1.0f;

    struct DefaultValues {
        float gainAlpha = 0.98f;
        float noiseGate = 0.001f;
        float scale = 32768.0f;
        float minGain = 0.1f;
        float maxGain = 1.0f;
        float vadThreshold = 0.5f;
        uint32_t vadGracePeriodBlocks = 20;
        uint32_t retroactiveVADGraceBlocks = 0;
    } defaults;

    float gainAlphaStep = 0.001f;
    float noiseGateStep = 0.00001f;
    float scaleStep = 1000.0f;
    float gainStep = 0.001f;
    float vadThresholdStep = 0.01f;
    uint32_t vadGracePeriodStep = 1;
    uint32_t retroactiveVADStep = 1;

    // Audio device related
    RtAudio audio;
    int srId = 0;
    int devId = 0;
    bool running = false;
    unsigned int defaultDevId = 0;
    std::vector<RtAudio::DeviceInfo> devList;
    std::vector<unsigned int> deviceIds;
    std::string txtDevList;
    std::vector<unsigned int> sampleRates;
    std::string sampleRatesTxt;
    unsigned int sampleRate = 48000;
    std::string device = "";
};

class NRAudioSinkModule : public ModuleManager::Instance {
public:
    NRAudioSinkModule(std::string name) {
        this->name = name;
        provider.create = create_sink;
        provider.ctx = this;
        sigpath::sinkManager.registerSinkProvider("NRAudio", provider);
    }

    ~NRAudioSinkModule() {
        sigpath::sinkManager.unregisterSinkProvider("NRAudio");
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
        return (SinkManager::Sink*)(new NRAudioSink(stream, streamName));
    }

    std::string name;
    bool enabled = true;
    SinkManager::SinkProvider provider;
};

MOD_EXPORT void _INIT_() {
    json def = json({
        {"device", ""},
        {"devices", json::object()},
        {"rnnoise_enabled", false},
        {"mono_mode", true},
        {"gain_alpha", 0.98f},
        {"noise_gate", 0.001f},
        {"scale", 32768.0f},
        {"min_gain", 0.1f},
        {"max_gain", 1.0f},
        {"model_index", 0},
        {"vad_threshold", 0.5f},
        {"vad_grace_period_blocks", 20},
        {"retroactive_vad_grace_blocks", 0},
        {"defaults", {
            {"gain_alpha", 0.98f},
            {"noise_gate", 0.001f},
            {"scale", 32768.0f},
            {"min_gain", 0.1f},
            {"max_gain", 1.0f},
            {"vad_threshold", 0.5f},
            {"vad_grace_period_blocks", 20},
            {"retroactive_vad_grace_blocks", 0}
        }}
    });
    
    config.setPath(core::args["root"].s() + "/nraudio_sink_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT void* _CREATE_INSTANCE_(std::string name) {
    NRAudioSinkModule* instance = new NRAudioSinkModule(name);
    return instance;
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (NRAudioSinkModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
