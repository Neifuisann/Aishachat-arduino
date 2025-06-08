#include "VAD.h"
#include <cmath>

// AudioFrameBuffer Implementation
AudioFrameBuffer::AudioFrameBuffer(uint16_t frameSize, uint16_t maxFrames) 
    : frameSize(frameSize), maxFrames(maxFrames), writeIndex(0), frameCount(0) {
    
    // Allocate memory for frame pointers
    frames = new int16_t*[maxFrames];
    
    // Allocate memory for each frame
    for (uint16_t i = 0; i < maxFrames; i++) {
        frames[i] = new int16_t[frameSize];
        memset(frames[i], 0, frameSize * sizeof(int16_t));
    }
}

AudioFrameBuffer::~AudioFrameBuffer() {
    if (frames) {
        for (uint16_t i = 0; i < maxFrames; i++) {
            delete[] frames[i];
        }
        delete[] frames;
    }
}

bool AudioFrameBuffer::addFrame(const int16_t* frame) {
    if (!frame || !frames) return false;
    
    // Copy frame data
    memcpy(frames[writeIndex], frame, frameSize * sizeof(int16_t));
    
    // Update indices
    writeIndex = (writeIndex + 1) % maxFrames;
    if (frameCount < maxFrames) {
        frameCount++;
    }
    
    return true;
}

bool AudioFrameBuffer::getFrame(uint16_t index, int16_t* frame) {
    if (!frame || !frames || index >= frameCount) return false;
    
    // Calculate actual index (considering circular buffer)
    uint16_t actualIndex;
    if (frameCount < maxFrames) {
        actualIndex = index;
    } else {
        actualIndex = (writeIndex + index) % maxFrames;
    }
    
    memcpy(frame, frames[actualIndex], frameSize * sizeof(int16_t));
    return true;
}

void AudioFrameBuffer::clear() {
    writeIndex = 0;
    frameCount = 0;
}

// VoiceActivityDetector Implementation
VoiceActivityDetector::VoiceActivityDetector(const VADConfig& cfg)
    : config(cfg), currentState(VAD_SILENCE), currentEnergy(0.0f),
      smoothedEnergy(0.0f), speechFrameCount(0), silenceFrameCount(0),
      calibrationActive(false), calibrationStartTime(0), calibrationDuration(0),
      calibrationEnergySum(0.0f), calibrationMaxEnergy(0.0f), calibrationFrameCount(0) {
    
    // Create prefix buffer
    prefixBuffer = new AudioFrameBuffer(config.frameSize, config.prefixFrames);
}

VoiceActivityDetector::~VoiceActivityDetector() {
    delete prefixBuffer;
}

float VoiceActivityDetector::calculateRMSEnergy(const int16_t* frame, uint16_t frameSize) {
    if (!frame) return 0.0f;
    
    float sum = 0.0f;
    for (uint16_t i = 0; i < frameSize; i++) {
        float sample = static_cast<float>(frame[i]);
        sum += sample * sample;
    }
    
    return sqrt(sum / frameSize);
}

void VoiceActivityDetector::updateEnergySmoothing(float newEnergy) {
    currentEnergy = newEnergy;
    
    // Apply exponential smoothing
    smoothedEnergy = (config.energySmoothingFactor * newEnergy) + 
                     ((1.0f - config.energySmoothingFactor) * smoothedEnergy);
}

bool VoiceActivityDetector::isSpeechEnergy(float energy) {
    return energy > config.speechThreshold;
}

bool VoiceActivityDetector::isSilenceEnergy(float energy) {
    return energy < config.silenceThreshold;
}

bool VoiceActivityDetector::processFrame(const int16_t* frame, bool& sendPrefixBuffer) {
    sendPrefixBuffer = false;
    
    if (!frame) return false;
    
    // Calculate energy for current frame
    float frameEnergy = calculateRMSEnergy(frame, config.frameSize);
    updateEnergySmoothing(frameEnergy);

    // Debug: Print energy levels occasionally
    static unsigned long lastEnergyDebug = 0;
    if (millis() - lastEnergyDebug > 2000) {  // Every 2 seconds
        Serial.printf("VAD Energy Debug: Raw=%.1f, Smoothed=%.1f, Thresholds=%.1f/%.1f\n",
                     frameEnergy, smoothedEnergy, config.speechThreshold, config.silenceThreshold);
        lastEnergyDebug = millis();
    }

    // Handle calibration mode
    if (calibrationActive) {
        calibrationEnergySum += frameEnergy;
        if (frameEnergy > calibrationMaxEnergy) {
            calibrationMaxEnergy = frameEnergy;
        }
        calibrationFrameCount++;

        // Check if calibration is complete
        if (millis() - calibrationStartTime >= calibrationDuration) {
            calibrationActive = false;
            float avgSilence, maxSilence, suggestedThreshold;
            getCalibrationResults(avgSilence, maxSilence, suggestedThreshold);

            Serial.printf("VAD Calibration complete:\n");
            Serial.printf("  Average silence energy: %.2f\n", avgSilence);
            Serial.printf("  Maximum silence energy: %.2f\n", maxSilence);
            Serial.printf("  Suggested speech threshold: %.2f\n", suggestedThreshold);
            Serial.printf("  Suggested silence threshold: %.2f\n", suggestedThreshold * 0.6f);
        }

        // During calibration, don't process speech detection
        prefixBuffer->addFrame(frame);
        return false;
    }

    // Always add frame to prefix buffer (for potential future use)
    prefixBuffer->addFrame(frame);
    
    // State machine for VAD
    switch (currentState) {
        case VAD_SILENCE:
            if (isSpeechEnergy(smoothedEnergy)) {
                currentState = VAD_SPEECH_START;
                speechFrameCount = 1;
                silenceFrameCount = 0;
                sendPrefixBuffer = true;  // Send prefix buffer when speech starts
                Serial.printf("VAD: Speech started! Energy=%.1f\n", smoothedEnergy);
                return true;  // Also send current frame
            }
            return false;  // No speech, don't send
            
        case VAD_SPEECH_START:
            speechFrameCount++;
            if (speechFrameCount >= config.minSpeechFrames) {
                currentState = VAD_SPEECH_ACTIVE;
            }

            if (isSilenceEnergy(smoothedEnergy)) {
                silenceFrameCount++;
                if (silenceFrameCount == 1) {
                    Serial.printf("VAD: Silence detection started, energy=%.1f\n", smoothedEnergy);
                }
                if (silenceFrameCount >= config.silenceFrames) {
                    currentState = VAD_SILENCE;
                    speechFrameCount = 0;
                    silenceFrameCount = 0;
                    Serial.printf("VAD: Speech ended (silence detected) after %d frames\n", config.silenceFrames);
                    return false;  // End of speech - stop sending
                }
            } else {
                if (silenceFrameCount > 0) {
                    Serial.printf("VAD: Silence counter reset (was %d), energy=%.1f\n", silenceFrameCount, smoothedEnergy);
                }
                silenceFrameCount = 0;  // Reset silence counter
            }
            return true;  // Continue sending during speech start

        case VAD_SPEECH_ACTIVE:
            if (isSilenceEnergy(smoothedEnergy)) {
                silenceFrameCount++;
                if (silenceFrameCount == 1) {
                    Serial.printf("VAD: Silence detection started, energy=%.1f\n", smoothedEnergy);
                }
                if (silenceFrameCount >= config.silenceFrames) {
                    currentState = VAD_SILENCE;
                    speechFrameCount = 0;
                    silenceFrameCount = 0;
                    Serial.printf("VAD: Speech ended (silence detected) after %d frames\n", config.silenceFrames);
                    return false;  // End of speech - stop sending
                }
            } else {
                if (silenceFrameCount > 0) {
                    Serial.printf("VAD: Silence counter reset (was %d), energy=%.1f\n", silenceFrameCount, smoothedEnergy);
                }
                silenceFrameCount = 0;  // Reset silence counter
            }
            return true;  // Continue sending during active speech
            
        case VAD_SPEECH_END:
            // This state is not used in current implementation
            // but could be used for more complex state transitions
            currentState = VAD_SILENCE;
            return false;
    }
    
    return false;
}

bool VoiceActivityDetector::getPrefixFrames(int16_t* buffer, uint16_t& frameCount) {
    if (!buffer || !prefixBuffer) {
        frameCount = 0;
        return false;
    }
    
    frameCount = prefixBuffer->getFrameCount();
    
    // Copy all frames from prefix buffer
    for (uint16_t i = 0; i < frameCount; i++) {
        if (!prefixBuffer->getFrame(i, buffer + (i * config.frameSize))) {
            frameCount = i;  // Return partial count if error
            return false;
        }
    }
    
    return true;
}

void VoiceActivityDetector::updateConfig(const VADConfig& cfg) {
    config = cfg;
    
    // Recreate prefix buffer if frame size or count changed
    delete prefixBuffer;
    prefixBuffer = new AudioFrameBuffer(config.frameSize, config.prefixFrames);
    
    reset();
}

void VoiceActivityDetector::setThresholds(float speechThreshold, float silenceThreshold) {
    config.speechThreshold = speechThreshold;
    config.silenceThreshold = silenceThreshold;
}

void VoiceActivityDetector::reset() {
    currentState = VAD_SILENCE;
    currentEnergy = 0.0f;
    smoothedEnergy = 0.0f;
    speechFrameCount = 0;
    silenceFrameCount = 0;

    // Reset calibration state
    calibrationActive = false;
    calibrationStartTime = 0;
    calibrationEnergySum = 0.0f;
    calibrationMaxEnergy = 0.0f;
    calibrationFrameCount = 0;

    if (prefixBuffer) {
        prefixBuffer->clear();
    }
}

void VoiceActivityDetector::printDebugInfo() {
    Serial.printf("VAD State: %d, Energy: %.2f, Smoothed: %.2f, Speech: %d, Silence: %d, Thresholds: %.1f/%.1f\n",
                  currentState, currentEnergy, smoothedEnergy, speechFrameCount, silenceFrameCount,
                  config.speechThreshold, config.silenceThreshold);
}

void VoiceActivityDetector::startCalibration(uint16_t durationMs) {
    calibrationActive = true;
    calibrationStartTime = millis();
    calibrationDuration = durationMs;
    calibrationEnergySum = 0.0f;
    calibrationMaxEnergy = 0.0f;
    calibrationFrameCount = 0;

    Serial.printf("VAD Calibration started - please remain silent for %d seconds\n", durationMs / 1000);
}

void VoiceActivityDetector::getCalibrationResults(float& avgSilence, float& maxSilence, float& suggestedSpeechThreshold) {
    if (calibrationFrameCount > 0) {
        avgSilence = calibrationEnergySum / calibrationFrameCount;
        maxSilence = calibrationMaxEnergy;
        // Suggest speech threshold as 3x the maximum silence energy
        suggestedSpeechThreshold = maxSilence * 3.0f;
    } else {
        avgSilence = 0.0f;
        maxSilence = 0.0f;
        suggestedSpeechThreshold = config.speechThreshold;
    }
}
