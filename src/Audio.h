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
extern AudioInfo info;

// For microphone input
// (We define them in Audio.cpp; no need for extern if you prefer.)
//
// However, if you want to share them in other .cpp, you can mark them extern here.
// extern I2SStream i2sInput;
// extern StreamCopy micToWsCopier;

// -------------- Functions --------------

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
