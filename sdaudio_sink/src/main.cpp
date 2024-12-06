#include <imgui.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <signal_path/sink.h>
#include <dsp/buffer/packer.h>
#include <dsp/convert/stereo_to_mono.h>
#include <utils/flog.h>
#include <rtaudio/RtAudio.h>
#include <config.h>
#include <core.h>
#include <memory>
#include "specbleach_denoiser.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

SDRPP_MOD_INFO{
    /* Name:            */ "sdaudio_sink",
    /* Description:     */ "Audio sink with SpecBleach noise reduction, filters and gate for SDR++",
    /* Author:          */ "Jack Heinlein",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};

ConfigManager config;

///----------filtres


class Filter {
public:
    virtual ~Filter() = default;
    virtual void process(float* buffer, int length) = 0;
    virtual void setSampleRate(float sampleRate) = 0;
    virtual void reset() = 0;
};

class ButterworthFilter : public Filter {
public:
    ButterworthFilter(float sampleRate, float cutoff, bool isHighPass) 
        : fs(sampleRate), fc(cutoff), isHP(isHighPass) {
        calculateCoefficients();
    }

    void setSampleRate(float sampleRate) override {
        fs = sampleRate;
        calculateCoefficients();
    }

    void setCutoff(float cutoff) {
        fc = cutoff;
        calculateCoefficients();
    }

    void reset() override {
        x1 = x2 = y1 = y2 = 0.0f;
    }

    void process(float* buffer, int length) override {
        for (int i = 0; i < length; i++) {
            float x = buffer[i];
            float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
            
            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = y;
            
            buffer[i] = y;
        }
    }

private:
    void calculateCoefficients() {
        float w0 = 2.0f * M_PI * fc / fs;
        float alpha = std::sin(w0) / (2.0f * 0.707f);
        float cosw0 = std::cos(w0);

        if (isHP) {
            b0 = (1.0f + cosw0) / 2.0f;
            b1 = -(1.0f + cosw0);
            b2 = (1.0f + cosw0) / 2.0f;
        } else {
            b0 = (1.0f - cosw0) / 2.0f;
            b1 = 1.0f - cosw0;
            b2 = (1.0f - cosw0) / 2.0f;
        }

        float a0 = 1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 = 1.0f - alpha;

        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;
    }

    float fs;
    float fc;
    bool isHP;
    float x1 = 0.0f;
    float x2 = 0.0f;
    float y1 = 0.0f;
    float y2 = 0.0f;
    float a1, a2;
    float b0, b1, b2;
};

class NotchFilter : public Filter {
public:
    NotchFilter(float sampleRate, float frequency, float q = 30.0f) 
        : fs(sampleRate), freq(frequency), Q(q) {
        calculateCoefficients();
    }

    void setSampleRate(float sampleRate) override {
        fs = sampleRate;
        calculateCoefficients();
    }

    void setFrequency(float frequency) {
        freq = frequency;
        calculateCoefficients();
    }

    void setQ(float q) {
        Q = q;
        calculateCoefficients();
    }

    void reset() override {
        x1 = x2 = y1 = y2 = 0.0f;
    }

    void process(float* buffer, int length) override {
        for (int i = 0; i < length; i++) {
            float x = buffer[i];
            float y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;
            
            x2 = x1;
            x1 = x;
            y2 = y1;
            y1 = y;
            
            buffer[i] = y;
        }
    }

private:
    void calculateCoefficients() {
        float w0 = 2.0f * M_PI * freq / fs;
        float alpha = std::sin(w0) / (2.0f * Q);
        float cosw0 = std::cos(w0);

        b0 = 1.0f;
        b1 = -2.0f * cosw0;
        b2 = 1.0f;

        float a0 = 1.0f + alpha;
        a1 = -2.0f * cosw0;
        a2 = 1.0f - alpha;

        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;
    }

    float fs;
    float freq;
    float Q;
    float x1 = 0.0f;
    float x2 = 0.0f;
    float y1 = 0.0f;
    float y2 = 0.0f;
    float a1, a2;
    float b0, b1, b2;
};


///----------

class SignalCrossfade {
public:
    SignalCrossfade(uint32_t sampleRate) {
        tau = 1.0f - std::exp(-128.0f * M_PI * 30.0f / (float)sampleRate);
        wetDry = 0.0f;
        wetDryTarget = 0.0f;
    }

    void run(uint32_t nSamples, const float* input, float* output, bool enable) {
        wetDryTarget = enable ? 1.0f : 0.0f;
        wetDry += tau * (wetDryTarget - wetDry);

        for (uint32_t i = 0; i < nSamples; i++) {
            output[i] = (1.0f - wetDry) * input[i] + output[i] * wetDry;
        }
    }

private:
    float tau;
    float wetDry;
    float wetDryTarget;
};

class SDAudioSink : public SinkManager::Sink {
public:
    SDAudioSink(SinkManager::Stream* stream, std::string streamName);
    virtual ~SDAudioSink() override;

    virtual void start() override;
    virtual void stop() override;
    virtual void menuHandler() override;

private:
    void updateSpecBleachParams();
    void startNoiseCapture();
    void releaseNoiseProfile();
    bool doStart();
    void doStop();
    void selectByName(std::string name);
    void selectById(int id);
    static void HelpMarker(const char* desc);
    static int audioCallback(void* outputBuffer, void* inputBuffer, 
                           unsigned int nBufferFrames, double streamTime, 
                           RtAudioStreamStatus status, void* userData);
    void calculateGateCoefficients();
    float processGate(float input, float& currentGain);
    float calculateRMS(const float* buffer, int nFrames);

    unsigned int defaultId = 0;

    // Base functionality
    SinkManager::Stream* _stream;
    std::string _streamName;
    dsp::convert::StereoToMono s2m;
    dsp::buffer::Packer<dsp::stereo_t> stereoPacker;

    // Processing buffers
    std::vector<float> processingBufferIn;
    std::vector<float> processingBufferOut;
    size_t frameBufferSize = 256;

    // Signal crossfade
    std::unique_ptr<SignalCrossfade> crossfade;

    // SpecBleach processing
    SpectralBleachHandle specbleach = nullptr;
    SpectralBleachParameters params;
    bool nrEnabled = false;
    bool isCapturing = false;
    float captureProgress = 0.0f;
    int captureSamples = 0;
    int totalCaptureSamples = 0;

    // SpecBleach parameters
    float reductionAmount = 5.0f;
    float smoothingFactor = 0.0f;
    float whiteningFactor = 0.0f;
    float noiseRescale = 2.0f;
    float postFilterThreshold = -10.0f;
    bool transientProtection = false;
    int noiseScalingType = 0;

    // Filter settings
    bool highpassEnabled = false;
    bool lowpassEnabled = false;
    bool notchEnabled = false;
    float highpassFreq = 20.0f;
    float lowpassFreq = 15000.0f;
    float notchFreq = 50.0f;
    float notchQ = 30.0f;

    std::unique_ptr<ButterworthFilter> highpassFilter;
    std::unique_ptr<ButterworthFilter> lowpassFilter;
    std::unique_ptr<NotchFilter> notchFilter;

    // Gate processing
    bool gateEnabled = false;
    float gateThreshold = -50.0f;
    float gateRatio = 10.0f;
    float gateAttack = 10.0f;
    float gateRelease = 100.0f;
    float gateAttackCoeff = 0.0f;
    float gateReleaseCoeff = 0.0f;
    float currentGainL = 1.0f;
    float currentGainR = 1.0f;
    const float minGateGain = 0.001f;

    // Audio device
    RtAudio audio;
    int srId = 0;
    int devId = 0;
    bool running = false;
    unsigned int defaultDevId = 0;
    std::vector<RtAudio::DeviceInfo> devList;
    std::vector<unsigned int> deviceIds;
    std::vector<unsigned int> sampleRates;
    unsigned int sampleRate = 48000;
    std::string device = "";
};

SDAudioSink::SDAudioSink(SinkManager::Stream* stream, std::string streamName) {
    _stream = stream;
    _streamName = streamName;
    s2m.init(_stream->sinkOut);
    stereoPacker.init(_stream->sinkOut, frameBufferSize);

    crossfade = std::make_unique<SignalCrossfade>(sampleRate);

    highpassFilter = std::make_unique<ButterworthFilter>(sampleRate, highpassFreq, true);
    lowpassFilter = std::make_unique<ButterworthFilter>(sampleRate, lowpassFreq, false);
    notchFilter = std::make_unique<NotchFilter>(sampleRate, notchFreq, notchQ);


    //specbleach
    config.acquire();
    if (!config.conf.contains(_streamName)) {
        config.conf[_streamName]["device"] = "";
        config.conf[_streamName]["devices"] = json({});
        config.conf[_streamName]["nr_enabled"] = false;
        config.conf[_streamName]["reduction_amount"] = reductionAmount;
        config.conf[_streamName]["smoothing_factor"] = smoothingFactor;
        config.conf[_streamName]["whitening_factor"] = whiteningFactor;
        config.conf[_streamName]["noise_rescale"] = noiseRescale;
        config.conf[_streamName]["post_filter_threshold"] = postFilterThreshold;
        config.conf[_streamName]["transient_protection"] = transientProtection;
        config.conf[_streamName]["noise_scaling_type"] = noiseScalingType;

        // Filter settings
        config.conf[_streamName]["highpass_enabled"] = false;
        config.conf[_streamName]["lowpass_enabled"] = false;
        config.conf[_streamName]["notch_enabled"] = false;
        config.conf[_streamName]["highpass_freq"] = highpassFreq;
        config.conf[_streamName]["lowpass_freq"] = lowpassFreq;
        config.conf[_streamName]["notch_freq"] = notchFreq;
        config.conf[_streamName]["notch_q"] = notchQ;

        // Gate settings
        config.conf[_streamName]["gate_enabled"] = false;
        config.conf[_streamName]["gate_threshold"] = -50.0f;
        config.conf[_streamName]["gate_ratio"] = 10.0f;
        config.conf[_streamName]["gate_attack"] = 10.0f;
        config.conf[_streamName]["gate_release"] = 100.0f;
    }

    // Load settings
    device = config.conf[_streamName]["device"];
    nrEnabled = config.conf[_streamName]["nr_enabled"];
    reductionAmount = config.conf[_streamName]["reduction_amount"];
    smoothingFactor = config.conf[_streamName]["smoothing_factor"];
    whiteningFactor = config.conf[_streamName]["whitening_factor"];
    noiseRescale = config.conf[_streamName]["noise_rescale"];
    postFilterThreshold = config.conf[_streamName]["post_filter_threshold"];
    transientProtection = config.conf[_streamName]["transient_protection"];
    noiseScalingType = config.conf[_streamName]["noise_scaling_type"];

    // Load filter settings
    highpassEnabled = config.conf[_streamName]["highpass_enabled"];
    lowpassEnabled = config.conf[_streamName]["lowpass_enabled"];
    notchEnabled = config.conf[_streamName]["notch_enabled"];
    highpassFreq = config.conf[_streamName]["highpass_freq"];
    lowpassFreq = config.conf[_streamName]["lowpass_freq"];
    notchFreq = config.conf[_streamName]["notch_freq"];
    notchQ = config.conf[_streamName]["notch_q"];

    // Load Gate settings
    gateEnabled = config.conf[_streamName]["gate_enabled"];
    gateThreshold = config.conf[_streamName]["gate_threshold"];
    gateRatio = config.conf[_streamName]["gate_ratio"];
    gateAttack = config.conf[_streamName]["gate_attack"];
    gateRelease = config.conf[_streamName]["gate_release"];
    
    config.release();

    // Audio devices
    RtAudio::DeviceInfo info;
    for (int i = 0; i < audio.getDeviceCount(); i++) {
        try {
            info = audio.getDeviceInfo(i);
            if (!info.probed) { continue; }
            if (info.outputChannels == 0) { continue; }
            if (info.isDefaultOutput) { defaultDevId = devList.size(); }
            devList.push_back(info);
            deviceIds.push_back(i);
        }
        catch (const std::exception& e) {
            flog::error("Error getting audio device info: {}", e.what());
        }
    }

    selectByName(device);

    // SpecBleach
    specbleach = specbleach_initialize(sampleRate, frameBufferSize);
    if (!specbleach) {
        flog::error("Failed to initialize SpecBleach");
        return;
    }

    // parameters
    params = {
        .learn_noise = 0,
        .residual_listen = false,
        .reduction_amount = reductionAmount,
        .smoothing_factor = smoothingFactor,
        .transient_protection = transientProtection,
        .whitening_factor = whiteningFactor,
        .noise_scaling_type = noiseScalingType,
        .noise_rescale = noiseRescale,
        .post_filter_threshold = postFilterThreshold
    };

    if (!specbleach_load_parameters(specbleach, params)) {
        flog::error("Failed to load initial parameters");
    }

    // Gate coefficients
    calculateGateCoefficients();

    // processing buffers
    processingBufferIn.resize(frameBufferSize);
    processingBufferOut.resize(frameBufferSize);
}

SDAudioSink::~SDAudioSink() {
    stop();
    if (specbleach) {
        specbleach_free(specbleach);
        specbleach = nullptr;
    }
    processingBufferIn.clear();
    processingBufferOut.clear();
    crossfade.reset();
}

void SDAudioSink::start() {
    if (running) return;
    running = doStart();
}

void SDAudioSink::stop() {
    if (!running) return;
    doStop();
    running = false;
}

void SDAudioSink::calculateGateCoefficients() {
    float attackSamples = (gateAttack * 0.001f) * sampleRate;
    float releaseSamples = (gateRelease * 0.001f) * sampleRate;
    
    gateAttackCoeff = std::exp(-1.0f / attackSamples);
    gateReleaseCoeff = std::exp(-1.0f / releaseSamples);
}

float SDAudioSink::processGate(float input, float& currentGain) {
    float dbInput = 20.0f * std::log10(std::abs(input) + 1e-10f);
    
    float targetGain = 1.0f;
    if (dbInput < gateThreshold) {
        float dbReduction = (dbInput - gateThreshold) * (1.0f - 1.0f/gateRatio);
        targetGain = std::pow(10.0f, dbReduction/20.0f);
    }
    
    if (targetGain < currentGain) {
        currentGain = gateAttackCoeff * currentGain + (1.0f - gateAttackCoeff) * targetGain;
    } else {
        currentGain = gateReleaseCoeff * currentGain + (1.0f - gateReleaseCoeff) * targetGain;
    }
    
    return input * std::max(currentGain, minGateGain);
}

float SDAudioSink::calculateRMS(const float* buffer, int nFrames) {
    float sum = 0.0f;
    for (int i = 0; i < nFrames; i++) {
        float sample = buffer[i];
        sum += sample * sample;
    }
    float rms = std::sqrt(sum / nFrames);
    float dbRMS = 20.0f * std::log10(rms + 1e-10f);
    return dbRMS;
}

void SDAudioSink::menuHandler() {
    float menuWidth = ImGui::GetContentRegionAvail().x;
    float halfWidth = menuWidth * 0.5f;

    if (ImGui::BeginCombo("Audio Device", devList[devId].name.c_str())) {
        for (int i = 0; i < devList.size(); i++) {
            if (ImGui::Selectable(devList[i].name.c_str(), i == devId)) {
                selectById(i);
                config.acquire();
                config.conf[_streamName]["device"] = devList[devId].name;
                config.release(true);
            }
        }
        ImGui::EndCombo();
    }

    if (ImGui::BeginCombo("Sample Rate", std::to_string(sampleRate).c_str())) {
        for (int i = 0; i < sampleRates.size(); i++) {
            if (ImGui::Selectable(std::to_string(sampleRates[i]).c_str(), i == srId)) {
                srId = i;
                sampleRate = sampleRates[i];
                config.acquire();
                config.conf[_streamName]["devices"][devList[devId].name] = sampleRate;
                config.release(true);
                _stream->setSampleRate(sampleRate);
                calculateGateCoefficients();
                if (running) {
                    doStop();
                    doStart();
                }
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Separator();

    if (ImGui::Checkbox("Enable Noise Reduction", &nrEnabled)) {
        config.acquire();
        config.conf[_streamName]["nr_enabled"] = nrEnabled;
        config.release(true);
    }

    if (nrEnabled) {
        // Capture
        ImGui::BeginGroup();
        if (!isCapturing) {
            if (ImGui::Button("Capture Noise Profile", ImVec2(halfWidth - 5, 0))) {
                startNoiseCapture();
            }
            ImGui::SameLine();
            if (ImGui::Button("Release Profile", ImVec2(halfWidth - 5, 0))) {
                releaseNoiseProfile();
            }
        } else {
            ImGui::ProgressBar(captureProgress, ImVec2(-1, 0), "Capturing Noise Profile...");
        }
        ImGui::EndGroup();

        ImGui::SetNextItemWidth(halfWidth);
        if (ImGui::SliderFloat("Reduction Amount (dB)", &reductionAmount, 0.0f, 10.0f, "%.1f")) {
            config.acquire();
            config.conf[_streamName]["reduction_amount"] = reductionAmount;
            config.release(true);
            updateSpecBleachParams();
        }

        if (ImGui::TreeNode("Advanced Settings")) {
            bool paramsChanged = false;

            ImGui::SetNextItemWidth(halfWidth);
            if (ImGui::SliderFloat("Smoothing Factor (%)", &smoothingFactor, 0.0f, 100.0f, "%.1f")) {
                paramsChanged = true;
                config.acquire();
                config.conf[_streamName]["smoothing_factor"] = smoothingFactor;
                config.release(true);
            }
            
            if (ImGui::Checkbox("Transient Protection", &transientProtection)) {
                paramsChanged = true;
                config.acquire();
                config.conf[_streamName]["transient_protection"] = transientProtection;
                config.release(true);
            }

            ImGui::SetNextItemWidth(halfWidth);
            if (ImGui::SliderFloat("Whitening Factor (%)", &whiteningFactor, 0.0f, 100.0f, "%.1f")) {
                paramsChanged = true;
                config.acquire();
                config.conf[_streamName]["whitening_factor"] = whiteningFactor;
                config.release(true);
            }

            if (paramsChanged) {
                updateSpecBleachParams();
            }

            ImGui::TreePop();
        }

        if (specbleach && specbleach_noise_profile_available(specbleach)) {
            ImGui::Text("Profile: Active");
            ImGui::Text("Blocks: %u", specbleach_get_noise_profile_blocks_averaged(specbleach));
        } else {
            ImGui::Text("Profile: None");
        }
    }

    ImGui::Separator();

    // Filter settings
    if (ImGui::TreeNode("Filters")) {
        // High-pass filter
        if (ImGui::Checkbox("Enable HPF", &highpassEnabled)) {
            config.acquire();
            config.conf[_streamName]["highpass_enabled"] = highpassEnabled;
            config.release(true);
        }
        if (highpassEnabled) {
            ImGui::SetNextItemWidth(halfWidth);
            if (ImGui::SliderFloat("HPF (Hz)", &highpassFreq, 10.0f, 1000.0f, "%.1f")) {
                highpassFilter->setCutoff(highpassFreq);
                config.acquire();
                config.conf[_streamName]["highpass_freq"] = highpassFreq;
                config.release(true);
            }
        }

        // Low-pass filter
        if (ImGui::Checkbox("Enable LPF", &lowpassEnabled)) {
            config.acquire();
            config.conf[_streamName]["lowpass_enabled"] = lowpassEnabled;
            config.release(true);
        }
        if (lowpassEnabled) {
            ImGui::SetNextItemWidth(halfWidth);
            if (ImGui::SliderFloat("LPF (Hz)", &lowpassFreq, 1000.0f, 20000.0f, "%.1f")) {
                lowpassFilter->setCutoff(lowpassFreq);
                config.acquire();
                config.conf[_streamName]["lowpass_freq"] = lowpassFreq;
                config.release(true);
            }
        }

        // Notch filter
        if (ImGui::Checkbox("Enable Notch", &notchEnabled)) {
            config.acquire();
            config.conf[_streamName]["notch_enabled"] = notchEnabled;
            config.release(true);
        }
        if (notchEnabled) {
            bool notchChanged = false;
            ImGui::SetNextItemWidth(halfWidth);
            if (ImGui::SliderFloat("Notch Freq (Hz)", &notchFreq, 45.0f, 1000.0f, "%.1f")) {
                notchChanged = true;
                config.acquire();
                config.conf[_streamName]["notch_freq"] = notchFreq;
                config.release(true);
            }

            ImGui::SetNextItemWidth(halfWidth);
            if (ImGui::SliderFloat("Notch Q", &notchQ, 10.0f, 100.0f, "%.1f")) {
                notchChanged = true;
                config.acquire();
                config.conf[_streamName]["notch_q"] = notchQ;
                config.release(true);
            }

            if (notchChanged) {
                notchFilter->setFrequency(notchFreq);
                notchFilter->setQ(notchQ);
            }
        }

        ImGui::TreePop();
    }

    ImGui::Separator();

    // Gate settings
    if (ImGui::Checkbox("Enable Gate", &gateEnabled)) {
        config.acquire();
        config.conf[_streamName]["gate_enabled"] = gateEnabled;
        config.release(true);
    }

    if (gateEnabled) {
        bool gateParamsChanged = false;

        ImGui::SetNextItemWidth(halfWidth);
        if (ImGui::SliderFloat("Gate Threshold (dB)", &gateThreshold, -100.0f, 0.0f, "%.1f")) {
            gateParamsChanged = true;
            config.acquire();
            config.conf[_streamName]["gate_threshold"] = gateThreshold;
            config.release(true);
        }

        ImGui::SetNextItemWidth(halfWidth);
        if (ImGui::SliderFloat("Gate Ratio", &gateRatio, 1.0f, 20.0f, "%.1f")) {
            gateParamsChanged = true;
            config.acquire();
            config.conf[_streamName]["gate_ratio"] = gateRatio;
            config.release(true);
        }

        ImGui::SetNextItemWidth(halfWidth);
        if (ImGui::SliderFloat("Attack Time (ms)", &gateAttack, 0.1f, 100.0f, "%.1f")) {
            gateParamsChanged = true;
            config.acquire();
            config.conf[_streamName]["gate_attack"] = gateAttack;
            config.release(true);
        }

        ImGui::SetNextItemWidth(halfWidth);
        if (ImGui::SliderFloat("Release Time (ms)", &gateRelease, 1.0f, 1000.0f, "%.1f")) {
            gateParamsChanged = true;
            config.acquire();
            config.conf[_streamName]["gate_release"] = gateRelease;
            config.release(true);
        }

        if (gateParamsChanged) {
            calculateGateCoefficients();
        }
    }
}

int SDAudioSink::audioCallback(void* outputBuffer, void* inputBuffer, 
                              unsigned int nBufferFrames, double streamTime, 
                              RtAudioStreamStatus status, void* userData) {
    SDAudioSink* _this = (SDAudioSink*)userData;
    if (!_this || !_this->specbleach) return 0;

    int count = _this->stereoPacker.out.read();
    if (count < 0) return 0;

    float* inBuf = (float*)_this->stereoPacker.out.readBuf;
    float* outBuf = (float*)outputBuffer;
    if (!inBuf || !outBuf) return 0;

    for (unsigned int i = 0; i < nBufferFrames; i++) {
        _this->processingBufferIn[i] = (inBuf[i * 2] + inBuf[i * 2 + 1]) * 0.5f;
    }

    if (_this->nrEnabled && _this->specbleach) {
        if (_this->isCapturing) {
            specbleach_process(_this->specbleach, nBufferFrames,
                             _this->processingBufferIn.data(), 
                             _this->processingBufferOut.data());

            memcpy(outBuf, inBuf, nBufferFrames * sizeof(dsp::stereo_t));

            _this->captureSamples += nBufferFrames;
            _this->captureProgress = static_cast<float>(_this->captureSamples) / 
                                   _this->totalCaptureSamples;

            if (_this->captureSamples >= _this->totalCaptureSamples) {
                _this->isCapturing = false;
                _this->params.learn_noise = 0;
                specbleach_load_parameters(_this->specbleach, _this->params);
                flog::info("Noise capture completed");
            }
        }
        else if (specbleach_noise_profile_available(_this->specbleach)) {
            bool nrSuccess = specbleach_process(_this->specbleach, nBufferFrames,
                                             _this->processingBufferIn.data(), 
                                             _this->processingBufferOut.data());

            if (nrSuccess) {
                // Apply filters to mono signal
                if (_this->highpassEnabled) {
                    _this->highpassFilter->process(_this->processingBufferOut.data(), nBufferFrames);
                }
                if (_this->lowpassEnabled) {
                    _this->lowpassFilter->process(_this->processingBufferOut.data(), nBufferFrames);
                }
                if (_this->notchEnabled) {
                    _this->notchFilter->process(_this->processingBufferOut.data(), nBufferFrames);
                }

                if (_this->gateEnabled) {
                    // Apply gate after filtering
                    for (unsigned int i = 0; i < nBufferFrames; i++) {
                        float processed = _this->processGate(_this->processingBufferOut[i], 
                                                         _this->currentGainL);
                        outBuf[i * 2] = outBuf[i * 2 + 1] = processed;
                    }
                } else {
                    // No gate, just copy processed audio
                    for (unsigned int i = 0; i < nBufferFrames; i++) {
                        outBuf[i * 2] = outBuf[i * 2 + 1] = _this->processingBufferOut[i];
                    }
                }
                
                // Apply crossfade
                _this->crossfade->run(nBufferFrames * 2, (const float*)inBuf, (float*)outBuf, true);
            } else {
                memcpy(outBuf, inBuf, nBufferFrames * sizeof(dsp::stereo_t));
            }
        } else {
            // No profile, apply only filters and gate
            memcpy(_this->processingBufferOut.data(), _this->processingBufferIn.data(), 
                   nBufferFrames * sizeof(float));

            // Apply filters
            if (_this->highpassEnabled) {
                _this->highpassFilter->process(_this->processingBufferOut.data(), nBufferFrames);
            }
            if (_this->lowpassEnabled) {
                _this->lowpassFilter->process(_this->processingBufferOut.data(), nBufferFrames);
            }
            if (_this->notchEnabled) {
                _this->notchFilter->process(_this->processingBufferOut.data(), nBufferFrames);
            }

            if (_this->gateEnabled) {
                for (unsigned int i = 0; i < nBufferFrames; i++) {
                    float processed = _this->processGate(_this->processingBufferOut[i], 
                                                     _this->currentGainL);
                    outBuf[i * 2] = outBuf[i * 2 + 1] = processed;
                }
            } else {
                for (unsigned int i = 0; i < nBufferFrames; i++) {
                    outBuf[i * 2] = outBuf[i * 2 + 1] = _this->processingBufferOut[i];
                }
            }

            _this->crossfade->run(nBufferFrames * 2, (const float*)inBuf, (float*)outBuf, 
                                 _this->highpassEnabled || _this->lowpassEnabled || 
                                 _this->notchEnabled || _this->gateEnabled);
        }
    } else {
        memcpy(_this->processingBufferOut.data(), _this->processingBufferIn.data(), 
               nBufferFrames * sizeof(float));

        if (_this->highpassEnabled) {
            _this->highpassFilter->process(_this->processingBufferOut.data(), nBufferFrames);
        }
        if (_this->lowpassEnabled) {
            _this->lowpassFilter->process(_this->processingBufferOut.data(), nBufferFrames);
        }
        if (_this->notchEnabled) {
            _this->notchFilter->process(_this->processingBufferOut.data(), nBufferFrames);
        }

        if (_this->gateEnabled) {
            for (unsigned int i = 0; i < nBufferFrames; i++) {
                float processed = _this->processGate(_this->processingBufferOut[i], 
                                                 _this->currentGainL);
                outBuf[i * 2] = outBuf[i * 2 + 1] = processed;
            }
        } else {
            for (unsigned int i = 0; i < nBufferFrames; i++) {
                outBuf[i * 2] = outBuf[i * 2 + 1] = _this->processingBufferOut[i];
            }
        }

        _this->crossfade->run(nBufferFrames * 2, (const float*)inBuf, (float*)outBuf, 
                             _this->highpassEnabled || _this->lowpassEnabled || 
                             _this->notchEnabled || _this->gateEnabled);
    }

    _this->stereoPacker.out.flush();
    return 0;
}

void SDAudioSink::updateSpecBleachParams() {
    if (!specbleach) return;

    params.reduction_amount = reductionAmount;
    params.smoothing_factor = smoothingFactor;
    params.transient_protection = transientProtection;
    params.whitening_factor = whiteningFactor;
    params.noise_scaling_type = noiseScalingType;
    params.noise_rescale = noiseRescale;
    params.post_filter_threshold = postFilterThreshold;
    
    if (!specbleach_load_parameters(specbleach, params)) {
        flog::error("Failed to update parameters");
    }
}

void SDAudioSink::startNoiseCapture() {
    if (!specbleach || isCapturing) return;

    isCapturing = true;
    captureProgress = 0.0f;
    captureSamples = 0;
    totalCaptureSamples = static_cast<int>(5.0f * sampleRate);

    if (!specbleach_reset_noise_profile(specbleach)) {
        flog::error("Failed to reset noise profile");
        isCapturing = false;
        return;
    }

    params.learn_noise = 1;
    if (!specbleach_load_parameters(specbleach, params)) {
        flog::error("Failed to set capture parameters");
        isCapturing = false;
        return;
    }

    flog::info("Starting noise capture...");
}

void SDAudioSink::releaseNoiseProfile() {
    if (!specbleach) return;
    specbleach_reset_noise_profile(specbleach);
    flog::info("Noise profile released");
}

void SDAudioSink::HelpMarker(const char* desc) {
    ImGui::TextDisabled("(?)");
    if (ImGui::IsItemHovered()) {
        ImGui::BeginTooltip();
        ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
        ImGui::TextUnformatted(desc);
        ImGui::PopTextWrapPos();
        ImGui::EndTooltip();
    }
}

void SDAudioSink::doStop() {
    s2m.stop();
    stereoPacker.stop();
    stereoPacker.out.stopReader();
    audio.stopStream();
    audio.closeStream();
    stereoPacker.out.clearReadStop();

    if (highpassFilter) highpassFilter->reset();
    if (lowpassFilter) lowpassFilter->reset();
    if (notchFilter) notchFilter->reset();
}

bool SDAudioSink::doStart() {
    RtAudio::StreamParameters parameters;
    parameters.deviceId = deviceIds[devId];
    parameters.nChannels = 2;
    
    unsigned int bufferFrames = frameBufferSize;
    RtAudio::StreamOptions opts;
    opts.flags = RTAUDIO_MINIMIZE_LATENCY;
    opts.streamName = _streamName;

    try {
        audio.openStream(&parameters, NULL, RTAUDIO_FLOAT32,
                        sampleRate, &bufferFrames, &audioCallback, this, &opts);

        stereoPacker.setSampleCount(bufferFrames);
        stereoPacker.start();
        s2m.start();
        audio.startStream();
        return true;
    }
    catch (const std::exception& e) {
        flog::error("Could not open audio device: {}", e.what());
        return false;
    }
}

void SDAudioSink::selectByName(std::string name) {
    for (int i = 0; i < devList.size(); i++) {
        if (devList[i].name == name) {
            selectById(i);
            return;
        }
    }
    selectById(defaultDevId);
}

void SDAudioSink::selectById(int id) {
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
    bool found = false;
    unsigned int defaultSr = devList[id].preferredSampleRate;
    for (int i = 0; i < sampleRates.size(); i++) {
        if (sampleRates[i] == sampleRate) {
            found = true;
            srId = i;
        }
        if (sampleRates[i] == defaultSr) {
            defaultId = i;
        }
    }
    if (!found) {
        sampleRate = defaultSr;
        srId = defaultId;
    }

    _stream->setSampleRate(sampleRate);
    calculateGateCoefficients();

    crossfade = std::make_unique<SignalCrossfade>(sampleRate);
    if (highpassFilter) highpassFilter->setSampleRate(sampleRate);
    if (lowpassFilter) lowpassFilter->setSampleRate(sampleRate);
    if (notchFilter) notchFilter->setSampleRate(sampleRate);

    if (running) {
        doStop();
        doStart();
    }
}

class SDAudioSinkModule : public ModuleManager::Instance {
public:
    SDAudioSinkModule(std::string name) {
        this->name = name;
        provider.create = create_sink;
        provider.ctx = this;
        sigpath::sinkManager.registerSinkProvider("Spectral Denoise Audio", provider);
        flog::info("SDAudioSink module initialized");
    }

    ~SDAudioSinkModule() {
        sigpath::sinkManager.unregisterSinkProvider("sdaudio_sink");
    }

    void postInit() {}
    void enable() { enabled = true; }
    void disable() { enabled = false; }
    bool isEnabled() { return enabled; }

private:
    static SinkManager::Sink* create_sink(SinkManager::Stream* stream, std::string streamName, void* ctx) {
        return (SinkManager::Sink*)(new SDAudioSink(stream, streamName));
    }

    std::string name;
    bool enabled = true;
    SinkManager::SinkProvider provider;
};

MOD_EXPORT void _INIT_() {
    json def = json({
        {"device", ""},
        {"devices", json::object()},
        {"nr_enabled", false},
        {"reduction_amount", 10.0f},
        {"smoothing_factor", 0.0f},
        {"whitening_factor", 0.0f},
        {"noise_rescale", 2.0f},
        {"post_filter_threshold", -10.0f},
        {"transient_protection", false},
        {"noise_scaling_type", 0},
        {"highpass_enabled", false},
        {"lowpass_enabled", false},
        {"notch_enabled", false},
        {"highpass_freq", 20.0f},
        {"lowpass_freq", 15000.0f},
        {"notch_freq", 50.0f},
        {"notch_q", 30.0f},
        {"gate_enabled", false},
        {"gate_threshold", -50.0f},
        {"gate_ratio", 10.0f},
        {"gate_attack", 10.0f},
        {"gate_release", 100.0f}
    });
    
    config.setPath(core::args["root"].s() + "/sdaudio_sink_config.json");
    config.load(def);
    config.enableAutoSave();
    flog::info("SDAudioSink config initialized");
}

MOD_EXPORT void* _CREATE_INSTANCE_(std::string name) {
    try {
        SDAudioSinkModule* instance = new SDAudioSinkModule(name);
        return instance;
    }
    catch (const std::exception& e) {
        flog::error("Failed to create SDAudioSink instance: {}", e.what());
        return nullptr;
    }
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    if (instance) {
        delete (SDAudioSinkModule*)instance;
    }
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
    flog::info("SDAudioSink module cleaned up");
}