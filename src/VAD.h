#ifndef VAD_H
#define VAD_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "VADConfig.h"

// VAD Configuration
class VADConfig {
public:
    // Audio parameters
    uint32_t sampleRate = VAD_SAMPLE_RATE;
    uint16_t frameSize = VAD_FRAME_SIZE;
    uint16_t prefixFrames = VAD_PREFIX_FRAMES;
    uint16_t silenceFrames = VAD_SILENCE_FRAMES;

    // Energy thresholds (these may need tuning based on your microphone)
    float speechThreshold = VAD_SPEECH_THRESHOLD;
    float silenceThreshold = VAD_SILENCE_THRESHOLD;

    // Smoothing
    float energySmoothingFactor = VAD_ENERGY_SMOOTHING;

    // Minimum speech duration to avoid false positives
    uint16_t minSpeechFrames = VAD_MIN_SPEECH_FRAMES;
};

// VAD States
enum VADState {
    VAD_SILENCE,        // No speech detected
    VAD_SPEECH_START,   // Speech just started (sending prefix + current)
    VAD_SPEECH_ACTIVE,  // Speech is active - continues until external reset
    VAD_SPEECH_END      // Speech ended, waiting for silence confirmation
};

// Circular buffer for audio frames
class AudioFrameBuffer {
private:
    int16_t** frames;
    uint16_t frameSize;
    uint16_t maxFrames;
    uint16_t writeIndex;
    uint16_t frameCount;
    
public:
    AudioFrameBuffer(uint16_t frameSize, uint16_t maxFrames);
    ~AudioFrameBuffer();
    
    bool addFrame(const int16_t* frame);
    bool getFrame(uint16_t index, int16_t* frame);
    uint16_t getFrameCount() const { return frameCount; }
    void clear();
};

// Main VAD class
class VoiceActivityDetector {
private:
    VADConfig config;
    VADState currentState;
    AudioFrameBuffer* prefixBuffer;
    
    // Energy tracking
    float currentEnergy;
    float smoothedEnergy;
    
    // Frame counters
    uint16_t speechFrameCount;
    uint16_t silenceFrameCount;

    // Calibration variables
    bool calibrationActive;
    unsigned long calibrationStartTime;
    unsigned long calibrationDuration;
    float calibrationEnergySum;
    float calibrationMaxEnergy;
    uint16_t calibrationFrameCount;
    
    // Internal methods
    float calculateRMSEnergy(const int16_t* frame, uint16_t frameSize);
    void updateEnergySmoothing(float newEnergy);
    bool isSpeechEnergy(float energy);
    bool isSilenceEnergy(float energy);
    
public:
    VoiceActivityDetector(const VADConfig& cfg = VADConfig());
    ~VoiceActivityDetector();
    
    // Main processing function
    // Returns true if audio should be transmitted
    bool processFrame(const int16_t* frame, bool& sendPrefixBuffer);
    
    // State queries
    VADState getState() const { return currentState; }
    float getCurrentEnergy() const { return smoothedEnergy; }
    bool isSpeechActive() const { return currentState == VAD_SPEECH_ACTIVE || currentState == VAD_SPEECH_START; }
    uint16_t getSpeechFrameCount() const { return speechFrameCount; }
    uint16_t getSilenceFrameCount() const { return silenceFrameCount; }
    
    // Configuration
    void updateConfig(const VADConfig& cfg);
    void setThresholds(float speechThreshold, float silenceThreshold);
    
    // Get prefix buffer for transmission
    bool getPrefixFrames(int16_t* buffer, uint16_t& frameCount);
    
    // Reset VAD state
    void reset();
    
    // Debug info
    void printDebugInfo();

    // Calibration helper
    void startCalibration(uint16_t durationMs = 5000);
    bool isCalibrating() const { return calibrationActive; }
    void getCalibrationResults(float& avgSilence, float& maxSilence, float& suggestedSpeechThreshold);
};

#endif // VAD_H
