/*******************************************************************************
 * Audio.h
 * (Combined: includes your working definitions plus camera/vision function declarations)
 *******************************************************************************/

#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "Print.h"
#include "Config.h"
#include "AudioTools.h"
#include "AudioTools/AudioCodecs/CodecOpus.h"
#include <WebSocketsClient.h>
#include "VAD.h"
#include "ADPCM.h"  // Add ADPCM support

// -------------- External Declarations --------------
extern SemaphoreHandle_t wsMutex;
extern SemaphoreHandle_t mp3Mutex;
extern WebSocketsClient webSocket;

extern TaskHandle_t speakerTaskHandle;
extern TaskHandle_t micTaskHandle;
extern TaskHandle_t networkTaskHandle;

extern bool scheduleListeningRestart;
extern unsigned long scheduledTime;
extern unsigned long speakingStartTime;

extern int currentVolume;
extern const int CHANNELS;         
extern const int BITS_PER_SAMPLE;

// For TTS output
constexpr size_t AUDIO_BUFFER_SIZE = 1024 * 10;  
constexpr size_t AUDIO_CHUNK_SIZE  = 1024;

// Tools objects
extern BufferRTOS<uint8_t> audioBuffer;
extern OpusAudioDecoder opusDecoder;
extern I2SStream i2s; 
extern VolumeStream volume;
extern QueueStream<uint8_t> queue;
extern StreamCopy copier;

// NEW for pitch shift
extern VolumeStream volumePitch;
extern StreamCopy pitchCopier;

extern AudioInfo info;

// For microphone input with VAD
extern VoiceActivityDetector vad;
extern int16_t* vadFrameBuffer;
extern uint16_t vadFrameIndex;
extern bool vadDebugEnabled;

// Microphone streaming objects
extern I2SStream i2sInput;
extern StreamCopy micToAdpcmCopier;  // Updated to use ADPCM

// ADPCM encoder/decoder objects
extern ADPCMEncoderStream adpcmEncoder;

// -------------- Functions --------------

// VAD functions
void setVADThresholds(float speechThreshold, float silenceThreshold);
void enableVADDebug(bool enable);
void startVADCalibration(uint16_t durationMs = 5000);
void getVADCalibrationResults(float& avgSilence, float& maxSilence, float& suggestedSpeechThreshold);
void printVADStatus();
void autoTuneVADThresholds();

// Audio gain control functions
void setMicrophoneGain(float gainFactor);
float getMicrophoneGain();
void enableHighPassFilter(bool enable);
bool isHighPassFilterEnabled();

// PCM raw binary transmission function
void sendPCMFrameRaw(const int16_t* frame, size_t frameSize);
void sendPCMFrameADPCM(const int16_t* frame, size_t frameSize);  // ADPCM compressed version

// Speaker / listening transitions
unsigned long getSpeakingDuration();
void transitionToSpeaking();
void transitionToListening();
void audioStreamTask(void *parameter);
void micTask(void *parameter);

// WebSocket
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length);
void websocketSetup(String server_domain, int port, String path);
void networkTask(void *parameter);

// -------------- Camera/Vision --------------
void transitionToTakingPhoto();           // Switch deviceState, capture photo
void captureAndSendPhotoBase64();         // Actually do the camera snap & base64

#endif // AUDIO_H
