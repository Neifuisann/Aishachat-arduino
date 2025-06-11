/*******************************************************************************
 * Audio.cpp - Combined
 * Includes your working microphone/speaker code AND the photo capture/vision code.
 *******************************************************************************/

 #include "Print.h"
 #include "Config.h"
 #include "AudioTools.h"
 #include "AudioTools/AudioCodecs/CodecOpus.h"
 #include "AudioTools/AudioCodecs/CodecMP3Helix.h"
 #include <WebSocketsClient.h>
 #include <ArduinoJson.h>
 #include "Audio.h"
 #include "ADPCM.h"  // Add ADPCM support
 #include "PitchShift.h"
 #include "VADConfig.h"
 #include <math.h>
 #include "SPIFFS.h"
 #include <stdlib.h> // Required for rand() and srand()
 #include <time.h>   // Required for time()
 #include <algorithm> // Required for min() function
 #include <esp_task_wdt.h> // Required for watchdog

 // For camera:
 #include "base64.h"
 #include "esp_camera.h"
 
 // -------------- WEBSOCKET --------------
 SemaphoreHandle_t wsMutex;
 WebSocketsClient webSocket;
 SemaphoreHandle_t mp3Mutex; // <<< ADD MUTEX FOR MP3 DECODER ACCESS
 
 // -------------- TASK HANDLES --------------
 TaskHandle_t speakerTaskHandle = NULL;
 TaskHandle_t micTaskHandle = NULL;
 TaskHandle_t networkTaskHandle = NULL;

 // Event groups for better task coordination
 EventGroupHandle_t audioEventGroup = NULL;
 
 // -------------- TIMING --------------
 bool scheduleListeningRestart = false;
 unsigned long scheduledTime = 0;
 unsigned long speakingStartTime = 0;
 unsigned long transitionToSpeakingTimestamp = 0;
 bool realAudioArrived = false;
 bool isPlayingProcessingSound = false;
 bool processingSoundFinished = false;
 bool isPlayingLimitSound = false;
 unsigned long lastLimitSoundStartTime = 0; // To prevent instant restart if a file fails/finishes
 const unsigned long LIMIT_SOUND_RETRY_DELAY = 10000; // ms delay between limit sounds
 
 // -------------- AUDIO SETTINGS --------------
 int currentVolume = 100;
 float currentPitchFactor = 1.0f;
 const int CHANNELS = 1;         // Mono
 const int BITS_PER_SAMPLE = 16; // 16-bit audio
 
 // -------------- OUTPUT (TTS) --------------
 BufferRTOS<uint8_t> audioBuffer(AUDIO_BUFFER_SIZE, AUDIO_CHUNK_SIZE);
 OpusAudioDecoder opusDecoder;  // Keep for incoming TTS audio
 MP3DecoderHelix mp3Decoder;
 File processingSoundFile;
 
 void playSoundFile(const char* filename);

 // We'll push TTS data to this "bufferPrint," which then writes into "audioBuffer."
 class BufferPrint : public Print {
 public:
   BufferPrint(BufferRTOS<uint8_t>& buf) : _buffer(buf) {}
 
   // Write single byte
   virtual size_t write(uint8_t data) override {
     bool isPlayingMp3 = isPlayingProcessingSound || isPlayingLimitSound; // Check if any MP3 is active
     // Only proceed if SPEAKING and NOT playing an MP3 sound
     if (webSocket.isConnected() && deviceState == SPEAKING && !isPlayingMp3) {
         size_t written = _buffer.writeArray(&data, 1);
         return written;
     }
     return 0;
   }
 
   // Write buffer
   virtual size_t write(const uint8_t *buffer, size_t size) override {
     bool isPlayingMp3 = isPlayingProcessingSound || isPlayingLimitSound; // Check if any MP3 is active
     // Only proceed if SPEAKING and NOT playing an MP3 sound
     if (webSocket.isConnected() && deviceState == SPEAKING && !isPlayingMp3) {
         size_t written = _buffer.writeArray(buffer, size);
         return written;
     }
     return 0;
   }
 
 private:
   BufferRTOS<uint8_t>& _buffer;
 };
 
 static BufferPrint bufferPrint(audioBuffer);
 
 // The pipeline for speaker:
 I2SStream i2s; 
 VolumeStream volume(i2s);
 QueueStream<uint8_t> queue(audioBuffer);
 StreamCopy copier(volume, queue);
 
 // NEW for pitch shift (lossy)Add commentMore actions
PitchShiftFixedOutput pitchShift(i2s);
VolumeStream volumePitch(pitchShift); //access from audioStreamTask only
StreamCopy pitchCopier(volumePitch, queue);
 
 // Audio info for speaker - Use 24kHz to match Gemini TTS output
 AudioInfo info(24000, CHANNELS, BITS_PER_SAMPLE);
 
 // -------------- MICROPHONE INPUT WITH ADPCM --------------
 // ADPCM-compressed WebSocket stream
 class ADPCMWebsocketStream : public Print {
 public:
   virtual size_t write(uint8_t b) override {
     if (!webSocket.isConnected() || deviceState != LISTENING) {
       return 1;
     }

     // Check if mutex is valid before using it
     if (wsMutex == NULL) {
       Serial.println("ERROR: wsMutex is NULL in ADPCMWebsocketStream::write(byte)");
       return 0;
     }

     if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
       webSocket.sendBIN(&b, 1);
       xSemaphoreGive(wsMutex);
     } else {
       Serial.println("WARNING: Failed to acquire wsMutex in ADPCMWebsocketStream::write(byte)");
       return 0;
     }
     return 1;
   }

   virtual size_t write(const uint8_t *buffer, size_t size) override {
     if (!webSocket.isConnected() || deviceState != LISTENING) {
       return size;
     }

     // Check if mutex is valid before using it
     if (wsMutex == NULL) {
       Serial.println("ERROR: wsMutex is NULL in ADPCMWebsocketStream::write(buffer)");
       return 0;
     }

     if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
       webSocket.sendBIN(buffer, size);
       xSemaphoreGive(wsMutex);
     } else {
       Serial.println("WARNING: Failed to acquire wsMutex in ADPCMWebsocketStream::write(buffer)");
       return 0;
     }
     return size;
   }
 };

 static ADPCMWebsocketStream adpcmWsStream;  // For ADPCM compressed transmission
 ADPCMEncoderStream adpcmEncoder(&adpcmWsStream, 1024); // ADPCM encoder with 1KB buffer (extern in header)

 // Memory monitoring for ADPCM
 void checkADPCMMemory() {
   static unsigned long lastMemCheck = 0;
   if (millis() - lastMemCheck > 30000) { // Check every 30 seconds
     Serial.printf("ADPCM Memory: Free heap: %d bytes, Largest block: %d bytes\n",
                   ESP.getFreeHeap(), ESP.getMaxAllocHeap());
     lastMemCheck = millis();
   }
 }
 I2SStream i2sInput;  // Remove static since it's declared extern in header
 static const int MIC_COPY_SIZE = 64; // chunk size for mic -> ADPCM encoder
 StreamCopy micToAdpcmCopier(adpcmEncoder, i2sInput); // mic -> ADPCM encoder -> WebSocket

 // VAD (Voice Activity Detection) components
 VoiceActivityDetector vad;
 int16_t* vadFrameBuffer = nullptr;
 uint16_t vadFrameIndex = 0;
 bool vadDebugEnabled = false;  // Temporarily enabled for debugging
 static unsigned long lastVadDebugTime = 0;
 static const unsigned long VAD_DEBUG_INTERVAL = VAD_DEBUG_INTERVAL_MS;

 // VAD state tracking for audio streaming
 static bool vadShouldStream = false;
 static bool vadPrefixSent = false;

 void sendPCMFrameADPCM(const int16_t* frame, size_t frameSize) {
   if (!webSocket.isConnected() || deviceState != LISTENING) {
     return;
   }

   // Send PCM data through ADPCM encoder
   size_t pcmDataSize = frameSize * sizeof(int16_t);
   const uint8_t* pcmBytes = (const uint8_t*)frame;

   // Send to ADPCM encoder which will compress and forward to WebSocket
   adpcmEncoder.write(pcmBytes, pcmDataSize);
 }

 // Keep the original function for compatibility/debugging
 void sendPCMFrameRaw(const int16_t* frame, size_t frameSize) {
   if (!webSocket.isConnected() || deviceState != LISTENING) {
     return;
   }

   // Send raw PCM data directly as binary WebSocket message
   size_t pcmDataSize = frameSize * sizeof(int16_t);
   const uint8_t* pcmBytes = (const uint8_t*)frame;

   const size_t PCM_CHUNK_SIZE = 64;
   size_t bytesRemaining = pcmDataSize;
   size_t offset = 0;

   while (bytesRemaining > 0) {
     size_t chunkSize = (bytesRemaining > PCM_CHUNK_SIZE) ? PCM_CHUNK_SIZE : bytesRemaining;

     if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
       webSocket.sendBIN(pcmBytes + offset, chunkSize);
       xSemaphoreGive(wsMutex);
     }

     offset += chunkSize;
     bytesRemaining -= chunkSize;
   }
 }

// Audio gain control variables
static float currentMicGain = 2.0f;  // Default PDM gain factor
static bool highPassEnabled = true;  // Default high-pass filter enabled
 
 // -----------------------------------------------------------------------------
 // UTILITY: how long have we been speaking?
 // -----------------------------------------------------------------------------
 unsigned long getSpeakingDuration() {
   if (deviceState == SPEAKING && speakingStartTime > 0) {
     return millis() - speakingStartTime;
   }
   return 0;
 }
 
 // -----------------------------------------------------------------------------
 // TRANSITIONS
 // -----------------------------------------------------------------------------
 // Simple transitionToSpeaking like old implementation (no mutex - called from networkTask that already holds it)
 void transitionToSpeaking() {
   vTaskDelay(50);

   // Stop any current sounds
   if (deviceState != SPEAKING && deviceState != QUOTA_EXCEEDED) {
      isPlayingProcessingSound = false;
      isPlayingLimitSound = false;
      if (processingSoundFile) {
          processingSoundFile.close();
      }
   } else if (deviceState == QUOTA_EXCEEDED) {
      isPlayingLimitSound = false;
      if (processingSoundFile) {
          processingSoundFile.close();
      }
   }

   // Reset TTS buffer
   audioBuffer.reset();
   i2s.flush();
   volume.flush();
   volumePitch.flush();
   queue.flush();

   // Change state
   deviceState = SPEAKING;
   speakingStartTime = millis();
   transitionToSpeakingTimestamp = millis();
   realAudioArrived = false;
   isPlayingProcessingSound = false;
   isPlayingLimitSound = false;
   processingSoundFinished = false;

   opusDecoder.setOutput(bufferPrint);

   //webSocket.enableHeartbeat(10000, 4000, 2); // 10s ping, 4s timeout, 2 retries

   Serial.println("Transitioned to speaking mode");
 }
 
 // Simple transitionToListening like old implementation (no mutex - called from networkTask that already holds it)
 void transitionToListening() {
   // Stop any MP3 sounds
   if (deviceState == QUOTA_EXCEEDED || isPlayingProcessingSound || isPlayingLimitSound) {
       Serial.println("Stopping MP3 sound due to transitionToListening.");
       isPlayingLimitSound = false;
       isPlayingProcessingSound = false;
       if (processingSoundFile) {
           processingSoundFile.close();
       }
   }

   realAudioArrived = false;
   processingSoundFinished = false;
   scheduleListeningRestart = false;

   deviceState = PROCESSING; // Intermediate state before LISTENING
   Serial.println("Transitioning to listening mode");

   // Flush streams and reset TTS buffer
   i2s.flush();
   volume.flush();
   volumePitch.flush();
   queue.flush();
   i2sInput.flush(); // Mic input
   audioBuffer.reset();

   Serial.println("Transitioned to listening mode");

   deviceState = LISTENING;

   //webSocket.disableHeartbeat();

   // Reset VAD when transitioning to listening
   vad.reset();
   vadFrameIndex = 0;
   vadShouldStream = false;
   vadPrefixSent = false;
 }
 
 void transitionToTakingPhoto() {
   // Set state early? Or after sound? Let's set early.
   deviceState = TAKING_PHOTO;
   Serial.println("Transitioning to take photo...");

   // Play camera shutter sound synchronously
   playSoundFile("/camera.mp3"); // <<< PLAY CAMERA SOUND

   // Now capture the photo
   captureAndSendPhotoBase64();
   Serial.println("Photo capture done");

   // Immediately transition to speaking (server expects image data then TTS)
   transitionToSpeaking();
 }
 
 /*******************************************************************************
  * CAMERA / VISION
  * For "REQUEST.PHOTO" => transitionToTakingPhoto -> captureAndSendPhotoBase64
  *******************************************************************************/
 void captureAndSendPhotoBase64() {
   Serial.println("=== PHOTO CAPTURE DEBUG ===");
   Serial.println("Step 1: Preparing camera...");

   sensor_t* sensor = esp_camera_sensor_get();
   if (!sensor) {
     Serial.println("ERROR: Camera sensor not available!");
     return;
   }

   sensor->set_reg(sensor, 0x3008, 0xFF, 0x02); // from standby to ready
   Serial.println("Step 2: Camera set to ready mode");

   delay(1000);

   Serial.println("Step 3: Capturing image...");
   camera_fb_t *fb = nullptr;

   fb = esp_camera_fb_get();

   // Critical: Check if camera capture was successful
   if (!fb) {
     Serial.println("ERROR: Camera capture failed! fb is NULL");
     sensor->set_reg(sensor, 0x3008, 0xFF, 0x42); // return to standby
     return;
   }

   if (fb->len == 0) {
     Serial.println("ERROR: Camera captured empty image!");
     esp_camera_fb_return(fb);
     sensor->set_reg(sensor, 0x3008, 0xFF, 0x42); // return to standby
     return;
   }

   Serial.printf("Step 4: Image captured successfully - %d bytes\n", fb->len);

   // base64-encode
   size_t encodedLen = base64_enc_len(fb->len);
   char *b64buf = (char*)malloc(encodedLen + 1);

   if (!b64buf) {
     Serial.println("ERROR: Failed to allocate memory for base64 encoding!");
     esp_camera_fb_return(fb);
     sensor->set_reg(sensor, 0x3008, 0xFF, 0x42); // return to standby
     return;
   }

   Serial.println("Step 5: Converting to base64...");
   base64_encode(b64buf, (char*)fb->buf, fb->len);
   esp_camera_fb_return(fb);

   Serial.printf("Step 6: Base64 encoding complete - %d characters\n", strlen(b64buf));
   Serial.printf("Free heap before chunking: %d bytes\n", ESP.getFreeHeap());

   // Calculate chunking parameters
   size_t base64Length = strlen(b64buf);
   size_t totalChunks = (base64Length + IMAGE_CHUNK_SIZE - 1) / IMAGE_CHUNK_SIZE; // Ceiling division

   Serial.printf("Step 7: Chunking image - %d bytes into %d chunks of %d bytes each\n",
                 base64Length, totalChunks, IMAGE_CHUNK_SIZE);

   // Send chunks
   bool allChunksSent = true;
   for (size_t chunkIndex = 0; chunkIndex < totalChunks; chunkIndex++) {
     size_t chunkStart = chunkIndex * IMAGE_CHUNK_SIZE;
     size_t chunkSize = min(IMAGE_CHUNK_SIZE, base64Length - chunkStart);

     // Create chunk message
     StaticJsonDocument<512> chunkDoc;
     chunkDoc["type"] = "image_chunk";
     chunkDoc["chunk_index"] = chunkIndex;
     chunkDoc["total_chunks"] = totalChunks;

     // Extract chunk data
     char chunkData[IMAGE_CHUNK_SIZE + 1];
     strncpy(chunkData, b64buf + chunkStart, chunkSize);
     chunkData[chunkSize] = '\0';
     chunkDoc["data"] = chunkData;

     String chunkJson;
     serializeJson(chunkDoc, chunkJson);

     // Send chunk
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
       webSocket.sendTXT(chunkJson);
     xSemaphoreGive(wsMutex);
       Serial.printf("Sent chunk %d/%d (%d bytes)\n", chunkIndex + 1, totalChunks, chunkSize);
       delay(10); // Small delay between chunks to prevent overwhelming
   } else {
       Serial.printf("ERROR: Failed to acquire WebSocket mutex for chunk %d!\n", chunkIndex);
       allChunksSent = false;
       break;
     }
   }

   // Send completion message
   if (allChunksSent) {
     StaticJsonDocument<256> completeDoc;
     completeDoc["type"] = "image_complete";
     completeDoc["total_chunks"] = totalChunks;
     completeDoc["mime"] = "image/jpeg";

     String completeJson;
     serializeJson(completeDoc, completeJson);

     if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
       webSocket.sendTXT(completeJson);
       xSemaphoreGive(wsMutex);
       Serial.println("Step 8: Image streaming complete!");
     } else {
       Serial.println("ERROR: Failed to send completion message!");
     }
   }

   free(b64buf);

   Serial.printf("Photo capture complete => %d chunks sent\n", totalChunks);

   sensor->set_reg(sensor, 0x3008, 0xFF, 0x42); // return to standby
   delay(1000);
   Serial.println("=== PHOTO CAPTURE COMPLETE ===");
 }
 
 // -----------------------------------------------------------------------------
 // AUDIO TASKS
 // -----------------------------------------------------------------------------
 void audioStreamTask(void *parameter) {
   Serial.println("Starting speaker pipeline...");
   srand(time(NULL));
 
   // Initialize Opus decoder immediately (like old implementation - no WiFi wait)
   Serial.printf("Free heap before Opus decoder init: %d bytes\n", ESP.getFreeHeap());

   // Use 24kHz to match Gemini TTS output
   OpusSettings cfg;
   cfg.sample_rate = 24000;
   cfg.channels = CHANNELS;
   cfg.bits_per_sample = BITS_PER_SAMPLE;
   cfg.max_buffer_size = 6144; // Adjust if needed
   opusDecoder.setOutput(bufferPrint); // Default output
   opusDecoder.begin(cfg);

   Serial.printf("Opus decoder initialized successfully. Free heap: %d bytes\n", ESP.getFreeHeap());
 
   // MP3 Setup (Output initially NULL, will be set dynamically)
   mp3Decoder.begin();
   uint8_t mp3ReadBuffer[1024];
   char randomSoundPath[30];
 
   // Output Pipeline Setup (i2s, volume, queue, copier)
   queue.begin();
   auto i2sCfg = i2s.defaultConfig(TX_MODE);
   i2sCfg.bits_per_sample = BITS_PER_SAMPLE;
   i2sCfg.sample_rate = 24000;
   i2sCfg.channels = CHANNELS;
   i2sCfg.pin_bck = I2S_BCK_OUT;
   i2sCfg.pin_ws = I2S_WS_OUT;
   i2sCfg.pin_data = I2S_DATA_OUT;
   i2sCfg.port_no = I2S_PORT_OUT;
   i2sCfg.copyFrom(info);
   i2s.begin(i2sCfg);
 
   auto vcfg = volume.defaultConfig();
   vcfg.copyFrom(info);
   vcfg.allow_boost = true; // Or false if not needed
   volume.begin(vcfg);
   volume.setVolume(currentVolume / 100.0f); // Set initial volume
 
   auto vcfgPitch = volumePitch.defaultConfig();
   vcfgPitch.copyFrom(info);
   vcfgPitch.allow_boost = true;
   volumePitch.begin(vcfgPitch);
 
   // Main Loop
   while (1) {
     bool playingMp3ThisIteration = false; // Track if we decoded MP3 in this loop pass
 
     // --- Acquire MP3 Mutex if planning to use MP3 Decoder ---
     bool mightPlayMp3 = (deviceState == SPEAKING && !isPlayingProcessingSound && !realAudioArrived && !processingSoundFinished && transitionToSpeakingTimestamp > 0) ||
                        (deviceState == SPEAKING && isPlayingProcessingSound) ||
                        (deviceState == QUOTA_EXCEEDED && !isPlayingLimitSound) ||
                        (deviceState == QUOTA_EXCEEDED && isPlayingLimitSound);

     bool mp3MutexTaken = false;
     if (mightPlayMp3) {
         // Try to take mutex with a short timeout. If fails, skip MP3 logic this iteration.
         if (xSemaphoreTake(mp3Mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
              mp3MutexTaken = true;
         } else {
              // Serial.println("Audio Task: Failed to get mp3Mutex quickly, skipping MP3 check.");
         }
     }

     // --- Handle SPEAKING State (only if mutex acquired) ---
     if (deviceState == SPEAKING && mp3MutexTaken) {
         // 1. Check if we should START playing processing sound
         if (!isPlayingProcessingSound && !realAudioArrived && !processingSoundFinished &&
             transitionToSpeakingTimestamp > 0 &&
             (millis() - transitionToSpeakingTimestamp > 1000)) { // Removed buffer check
 
             int randomSoundIndex = rand() % 4 + 1;
             snprintf(randomSoundPath, sizeof(randomSoundPath), "/processing_%d.mp3", randomSoundIndex);
             Serial.printf("Attempting to start processing sound: %s...\n", randomSoundPath);
 
             if (processingSoundFile) processingSoundFile.close(); // Close previous just in case
             processingSoundFile = SPIFFS.open(randomSoundPath, "r");
 
             if (!processingSoundFile || processingSoundFile.isDirectory()) {
                 Serial.printf("Failed to open %s\n", randomSoundPath);
                 processingSoundFinished = true; // Don't try again this cycle
             } else {
                 Serial.printf("Opened %s, starting MP3->Volume pipe.\n", randomSoundPath);
                 isPlayingProcessingSound = true;
                 mp3Decoder.setOutput(volume); // <<< Direct MP3 decoded output to volume stream
                 mp3Decoder.begin(); // Reset decoder state for new file
             }
         }
 
         // 2. Check if we should STOP playing processing sound (due to real audio)
         if (isPlayingProcessingSound && realAudioArrived) {
             Serial.println("Stopping processing sound MP3 (real audio detected)...");
             isPlayingProcessingSound = false;
             if (processingSoundFile) processingSoundFile.close();
             opusDecoder.setOutput(bufferPrint);
         }
 
         // 3. Decode and DIRECTLY OUTPUT processing sound chunk if active
         if (isPlayingProcessingSound && processingSoundFile) {
             // Mutex is already held here
             size_t bytesRead = processingSoundFile.read(mp3ReadBuffer, sizeof(mp3ReadBuffer));
              if (bytesRead > 0) {
                  size_t written = mp3Decoder.write(mp3ReadBuffer, bytesRead);
                  // Optional: Check 'written' if needed
                  // if (written != bytesRead) { Serial.printf("Warn: Processing MP3 direct write %d/%d\n", written, bytesRead); }
                  playingMp3ThisIteration = true; // We decoded MP3 data
              } else {
                  // End of file
                  Serial.println("Processing sound MP3 finished.");
                  isPlayingProcessingSound = false;
                  processingSoundFinished = true; // Mark finished for this SPEAKING cycle
                  if (processingSoundFile) processingSoundFile.close();
                  mp3Decoder.begin(); // Reset decoder state
                  // No need to change output, it's already volume
              }
         }
     } // --- End of SPEAKING State Logic ---
 
     // --- Handle QUOTA_EXCEEDED State (only if mutex acquired) ---
     else if (deviceState == QUOTA_EXCEEDED && mp3MutexTaken) {
         // 1. Check if we should START playing a limit sound
         if (!isPlayingLimitSound && (millis() - lastLimitSoundStartTime > LIMIT_SOUND_RETRY_DELAY)) {
             int randomSoundIndex = rand() % 2 + 1;
             snprintf(randomSoundPath, sizeof(randomSoundPath), "/limit_%d.mp3", randomSoundIndex);
             Serial.printf("Attempting to play limit sound: %s...\n", randomSoundPath);
 
             if (processingSoundFile) processingSoundFile.close(); // Close previous
             processingSoundFile = SPIFFS.open(randomSoundPath, "r");
 
             if (!processingSoundFile || processingSoundFile.isDirectory()) {
                 Serial.printf("Failed to open %s\n", randomSoundPath);
                 isPlayingLimitSound = false; // Ensure flag is false
                 lastLimitSoundStartTime = millis(); // Record time to enforce delay before next try
             } else {
                 Serial.printf("Opened %s, starting MP3->Volume pipe.\n", randomSoundPath);
                 isPlayingLimitSound = true;
                 mp3Decoder.setOutput(volume); // <<< Direct MP3 to volume
                 mp3Decoder.begin(); // Reset decoder
                 lastLimitSoundStartTime = millis(); // Reset time for next potential retry after this one finishes
             }
         }
 
         // 2. Decode and DIRECTLY OUTPUT limit sound chunk if active
         if (isPlayingLimitSound && processingSoundFile) {
             // Mutex is already held here
             size_t bytesRead = processingSoundFile.read(mp3ReadBuffer, sizeof(mp3ReadBuffer));
             if (bytesRead > 0) {
                 size_t written = mp3Decoder.write(mp3ReadBuffer, bytesRead);
                 // if (written != bytesRead) { Serial.printf("Warn: Limit MP3 direct write %d/%d\n", written, bytesRead); }
                 playingMp3ThisIteration = true; // We decoded MP3 data
             } else {
                 // End of file
                 Serial.println("Limit sound MP3 finished.");
                 if (processingSoundFile) processingSoundFile.close();
                 isPlayingLimitSound = false; // Ready to play next one after delay
                 lastLimitSoundStartTime = millis(); // Record finish time for delay calculation
                 mp3Decoder.begin(); // Reset decoder state
                 // No need to change output, it's already volume
             }
         }
     } // --- End of QUOTA_EXCEEDED State Logic ---


     // --- Release MP3 Mutex if taken ---
     if (mp3MutexTaken) {
         // Reset decoder if no MP3 is active anymore
         if (!isPlayingProcessingSound && !isPlayingLimitSound) { 
              mp3Decoder.begin();
         }
         xSemaphoreGive(mp3Mutex);
     }

     // --- Handle non-MP3 states or TTS part of SPEAKING ---
     if (!playingMp3ThisIteration) {
         // Ensure Opus decoder is connected to bufferPrint
          opusDecoder.setOutput(bufferPrint);
 
         // Copy data from the Opus/TTS buffer to the output
         // This handles the actual TTS playback
          if (currentPitchFactor != 1.0f && currentPitchFactor >= 0.5f && currentPitchFactor <= 2.0f) {
              pitchCopier.copy();
          } else {
              copier.copy();  // Use normal path for extreme pitch factors or 1.0f
          }
 
         // Adjust delays based on state
         if (deviceState != SPEAKING && deviceState != QUOTA_EXCEEDED) { 
              vTaskDelay(pdMS_TO_TICKS(10)); // Idle delay
              // Clean up just in case state changed abruptly
              if (isPlayingProcessingSound || isPlayingLimitSound) { 
                 isPlayingProcessingSound = false;
                 isPlayingLimitSound = false;
                 if (processingSoundFile) processingSoundFile.close();
              }
         } else {
              // Shorter delay during active playback (TTS, MP3)
              vTaskDelay(pdMS_TO_TICKS(1));
         }
     } else {
          // If we *did* play MP3 this iteration, just add a short delay
          vTaskDelay(pdMS_TO_TICKS(1));
     }
 
     esp_task_wdt_reset(); // Reset watchdog

   } // End while(1)
 }
 
 void micTask(void *parameter) {
   Serial.println("Starting microphone pipeline with VAD...");

   // Check initial stack usage
   UBaseType_t stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
   Serial.printf("Microphone task initial stack free: %d bytes\n", stackHighWaterMark * sizeof(StackType_t));

   // Initialize immediately (like old implementation - no WiFi wait)
   Serial.printf("Free heap: %d bytes\n", ESP.getFreeHeap());

   // Use aligned allocation for VAD frame buffer
   VADConfig vadConfig;
   vadFrameBuffer = (int16_t*)heap_caps_aligned_alloc(4, sizeof(int16_t) * vadConfig.frameSize, MALLOC_CAP_DEFAULT);
   if (!vadFrameBuffer) {
     Serial.println("Failed to allocate aligned VAD frame buffer!");
     vTaskDelete(NULL);
     return;
   }
   Serial.printf("VAD frame buffer allocated (%d samples for 20ms, 4-byte aligned). Free heap: %d bytes\n",
                 vadConfig.frameSize, ESP.getFreeHeap());

   // Configure I2S for PDM microphone with proper settings
   auto i2sConfig = i2sInput.defaultConfig(RX_MODE);
   i2sConfig.bits_per_sample = BITS_PER_SAMPLE;
   i2sConfig.sample_rate = SAMPLE_RATE;  // Use constant instead of hardcoded value
   i2sConfig.channels = CHANNELS;
   i2sConfig.i2s_format = I2S_PCM;
   i2sConfig.signal_type = PDM;
   i2sConfig.pin_data = PDM_DATA;
   i2sConfig.pin_ws   = PDM_CLK;
   i2sConfig.pin_bck  = I2S_PIN_NO_CHANGE;
   i2sConfig.port_no  = I2S_PORT_IN;
   i2sInput.begin(i2sConfig);

   Serial.println("PDM microphone configured with optimized settings");
   Serial.println("VAD initialized - 200ms prefix, 800ms silence detection (20ms frames)");
   Serial.printf("Audio processing: Raw PCM 16kHz 16-bit direct streaming\n");
   Serial.printf("PCM frames: %d samples/frame (20ms), Sample rate: %d Hz\n",
                 vadConfig.frameSize, VAD_SAMPLE_RATE);

   // Buffer for VAD processing (separate from streaming)
   uint8_t vadSampleBuffer[MIC_COPY_SIZE];

   // Audio level monitoring
   static unsigned long lastAudioLevelReport = 0;
   static float maxAudioLevel = 0.0f;
   static float avgAudioLevel = 0.0f;
   static int audioSampleCount = 0;

   // Stack monitoring variables
   static unsigned long lastStackCheck = 0;
   static const unsigned long STACK_CHECK_INTERVAL = 30000; // Check every 30 seconds
 
   while (1) {
     // Periodic stack monitoring
     if (millis() - lastStackCheck > STACK_CHECK_INTERVAL) {
       UBaseType_t stackFree = uxTaskGetStackHighWaterMark(NULL);
       Serial.printf("Microphone task stack free: %d bytes\n", stackFree * sizeof(StackType_t));
       lastStackCheck = millis();
     }

     // Check to see if a transition to listening mode is scheduled.
     if (scheduleListeningRestart && millis() >= scheduledTime) {
       transitionToListening();
     }

     if (deviceState == LISTENING && webSocket.isConnected()) {
       // Simple VAD processing for speech detection
       size_t bytesRead = i2sInput.readBytes(vadSampleBuffer, MIC_COPY_SIZE);

       if (bytesRead > 0) {
         // Convert bytes to 16-bit samples for VAD analysis only
         for (size_t i = 0; i < bytesRead && i < MIC_COPY_SIZE; i += 2) {
           if (vadFrameIndex < vadConfig.frameSize) {
             // Convert little-endian bytes to int16_t
             int16_t sample = (int16_t)((vadSampleBuffer[i+1] << 8) | vadSampleBuffer[i]);
             vadFrameBuffer[vadFrameIndex] = sample;
             vadFrameIndex++;
           }
         }

         // Process complete frames through VAD
         if (vadFrameIndex >= vadConfig.frameSize) {
           bool sendPrefixBuffer = false;
           bool shouldSendFrame = vad.processFrame(vadFrameBuffer, sendPrefixBuffer);

           // Update VAD streaming state
           if (sendPrefixBuffer && !vadPrefixSent) {
             vadShouldStream = true;
             vadPrefixSent = true;
             Serial.printf("VAD: Speech started - enabling audio stream\n");
           }

           if (!shouldSendFrame && vadShouldStream) {
             vadShouldStream = false;
             vadPrefixSent = false;
             Serial.printf("VAD: Speech ended - disabling audio stream\n");
           }

           // Debug output
           if (vadDebugEnabled && (millis() - lastVadDebugTime) > 1000) {
             Serial.printf("VAD: State=%d, Energy=%.1f, Streaming=%s\n",
                          vad.getState(), vad.getCurrentEnergy(),
                          vadShouldStream ? "YES" : "NO");
             lastVadDebugTime = millis();
           }

           // Reset frame buffer
           vadFrameIndex = 0;
         }
       }

       // Use StreamCopy for efficient ADPCM-compressed audio transmission when VAD allows
       if (vadShouldStream) {
         micToAdpcmCopier.copyBytes(MIC_COPY_SIZE);
         checkADPCMMemory(); // Monitor memory usage
       }

       vTaskDelay(1);
     } else {
       vTaskDelay(10);
     }

     // Always reset watchdog regardless of device state to prevent timeout
     esp_task_wdt_reset();
   }
 }
 
 // -----------------------------------------------------------------------------
 // WEBSOCKET EVENTS
 // -----------------------------------------------------------------------------
 void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
   switch (type) {
     case WStype_DISCONNECTED:
       Serial.println("[WSc] Disconnected");
       playSoundFile("/WS_restart.mp3");
       deviceState = IDLE;
       break;

     case WStype_CONNECTED:
       Serial.printf("[WSc] Connected => %s\n", payload);
       deviceState = PROCESSING;
       break;
 
     case WStype_TEXT: {
       Serial.printf("[WSc] TEXT: %s\n", payload);
       StaticJsonDocument<512> doc; 
       DeserializationError error = deserializeJson(doc, payload);
       if (error) {
         Serial.printf("JSON parse error: %s\n", error.c_str());
         deviceState = IDLE;
         return;
       }
 
       String typeS = doc["type"];
       if (typeS == "auth") {
          currentVolume = doc["volume_control"].as<int>();
          currentPitchFactor = doc["pitch_factor"].as<float>();

          bool is_ota = doc["is_ota"].as<bool>();
          bool is_reset = doc["is_reset"].as<bool>();

          // Update volumes on both streams
         volume.setVolume(currentVolume / 100.0f);
          volumePitch.setVolume(currentVolume / 100.0f);
          
          // Only initialize pitch shift if needed and within safe range
          if (currentPitchFactor != 1.0f && currentPitchFactor >= 0.5f && currentPitchFactor <= 2.0f) {
              auto pcfg = pitchShift.defaultConfig();
              pcfg.copyFrom(info);
              pcfg.pitch_shift = currentPitchFactor;
              pcfg.buffer_size = 1024;  // Match GRAINSIZE in PitchShift.cpp
              pitchShift.begin(pcfg);
              Serial.printf("Pitch shift initialized with factor: %.2f\n", currentPitchFactor);
          } else if (currentPitchFactor != 1.0f) {
              Serial.printf("Pitch factor %.2f out of safe range (0.5-2.0), using normal audio\n", currentPitchFactor);
          }
       }
       else if (typeS == "server") {
         String msg = doc["msg"] | "";
         Serial.println(msg);
 
         if (msg == "RESPONSE.COMPLETE" || msg == "RESPONSE.ERROR") {
           Serial.println("Server => done speaking => Listening");
           if (doc.containsKey("volume_control")) {
             int newVol = doc["volume_control"] | 100;
             volume.setVolume(newVol / 100.0f);
           }
           scheduleListeningRestart = true;
           scheduledTime = millis() + 1000; 
         }
         else if (msg == "AUDIO.COMMITTED") {
           deviceState = PROCESSING;
         }
         else if (msg == "RESPONSE.CREATED") {
           Serial.println("Server => TTS => transitionToSpeaking");
           transitionToSpeaking();
         }
         else if (msg == "REQUEST.PHOTO") {
           Serial.println("Server => capture photo => transitionToTakingPhoto");
           transitionToTakingPhoto();
         }
         else if (msg == "QUOTA.EXCEEDED") {
           Serial.println("Server => quota exceeded => Playing limit sounds");
           if (deviceState != QUOTA_EXCEEDED) {
              deviceState = QUOTA_EXCEEDED;
              isPlayingProcessingSound = false;
              processingSoundFinished = false;
              realAudioArrived = false;
              if (processingSoundFile) {
                  processingSoundFile.close();
              }
              audioBuffer.reset();
              i2s.flush();
              volume.flush();
              volumePitch.flush();
              queue.flush();
              isPlayingLimitSound = false;
              lastLimitSoundStartTime = 0;
           }
         }
       }
     }
     break;
 
     case WStype_BIN:
       if (scheduleListeningRestart || deviceState != SPEAKING) {
         Serial.println("Ignoring inbound TTS because not in SPEAKING mode");
         break;
       }
       if (!realAudioArrived) {
           Serial.println("First real audio packet arrived.");
           realAudioArrived = true;
       }
       {
         size_t processed = opusDecoder.write(payload, length);
         if (processed != length) {
           Serial.printf("Warning: wrote %d/%d bytes to Opus\n", processed, length);
         }
       }
       break;
 
     case WStype_ERROR:
       Serial.printf("[WSc] Error: %s\n", payload);
       break;
 
     default:
       break;
   }
 }
 
 // -----------------------------------------------------------------------------
 // WEBSOCKET SETUP + NETWORK TASK
 // -----------------------------------------------------------------------------
 // Simple robust websocketSetup like the old implementation
 void websocketSetup(String server_domain, int port, String path) {
   Serial.printf("=== WEBSOCKET SETUP ===\n");
   Serial.printf("Server: %s:%d%s\n", server_domain.c_str(), port, path.c_str());

   String headers = "Authorization: Bearer " + String(authTokenGlobal);

   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
     Serial.println("ERROR: Failed to acquire WebSocket mutex for setup");
     return;
   }

   webSocket.setExtraHeaders(headers.c_str());
   webSocket.onEvent(webSocketEvent);
   webSocket.setReconnectInterval(1000);

   // Disable heartbeat to prevent connection hanging issues
   webSocket.disableHeartbeat();

   #ifdef DEV_MODE
   Serial.println("DEV_MODE: Using non-SSL WebSocket connection");
   webSocket.begin(server_domain.c_str(), port, path.c_str());
   #else
   Serial.println("PROD_MODE: Using SSL WebSocket connection");
   webSocket.beginSslWithCA(server_domain.c_str(), port, path.c_str(), CA_cert);
   #endif

   xSemaphoreGive(wsMutex);
   Serial.println("WebSocket setup completed");
 }
 
 // Robust networkTask with watchdog protection
 void networkTask(void *parameter) {
   Serial.println("Network task started");

   unsigned long lastWatchdogReset = 0;
   const unsigned long WATCHDOG_RESET_INTERVAL = 1000; // Reset every 1 second minimum

   while (1) {
     // Always reset watchdog first, regardless of WebSocket state
     unsigned long currentTime = millis();
     if (currentTime - lastWatchdogReset >= WATCHDOG_RESET_INTERVAL) {
       esp_task_wdt_reset();
       lastWatchdogReset = currentTime;
     }

     // Try to process WebSocket with timeout protection
     if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
       // Call webSocket.loop() but ensure we don't block too long
       webSocket.loop();
       xSemaphoreGive(wsMutex);
     } else {
       // If we can't get mutex, still continue and reset watchdog
       Serial.println("Network task: WebSocket mutex busy, continuing...");
     }

     // Always delay to prevent tight loop and allow other tasks to run
     vTaskDelay(pdMS_TO_TICKS(50));
   }
 }
 
 // -----------------------------------------------------------------------------
 // SYNCHRONOUS SOUND PLAYBACK UTILITY
 // -----------------------------------------------------------------------------
 void playSoundFile(const char* filename) {
     Serial.printf("Attempting to play sound file: %s\n", filename);

     // 1. Acquire MP3 Decoder Mutex (wait indefinitely)
     if (xSemaphoreTake(mp3Mutex, portMAX_DELAY) != pdTRUE) {
         Serial.printf("Failed to acquire mp3Mutex for %s\n", filename);
         return;
     }

     // 2. Open File
     File soundFile = SPIFFS.open(filename, "r");
     if (!soundFile || soundFile.isDirectory()) {
         Serial.printf("Failed to open sound file: %s\n", filename);
         xSemaphoreGive(mp3Mutex); // Release mutex before returning
         return;
     }
     Serial.printf("Opened %s, size: %d\n", filename, soundFile.size());

     // 3. Prepare Decoder and Output
     mp3Decoder.setOutput(volume); // Target the volume stream directly
     mp3Decoder.begin();           // Reset decoder state

     // 4. Playback Loop
     uint8_t buffer[1024];
     size_t bytesRead;
     while ((bytesRead = soundFile.read(buffer, sizeof(buffer))) > 0) {
         size_t written = mp3Decoder.write(buffer, bytesRead);
         // Optional: Check 'written' if needed, but usually okay for direct synchronous playback
         vTaskDelay(pdMS_TO_TICKS(1)); // Small delay to prevent starving other tasks
     }

     // 5. Cleanup
     soundFile.close();
     // mp3Decoder.setOutput(nullptr); // Not needed/valid - Task loop will reset if necessary
     Serial.printf("Finished playing %s\n", filename);

     // Call begin again AFTER playback to ensure decoder is fully reset for next use
     mp3Decoder.begin();

     // 6. Release Mutex
     xSemaphoreGive(mp3Mutex);
 }

 // -----------------------------------------------------------------------------
 // VAD UTILITY FUNCTIONS
 // -----------------------------------------------------------------------------
 void setVADThresholds(float speechThreshold, float silenceThreshold) {
   vad.setThresholds(speechThreshold, silenceThreshold);
   Serial.printf("VAD thresholds updated: Speech=%.1f, Silence=%.1f\n",
                speechThreshold, silenceThreshold);
 }

 void enableVADDebug(bool enable) {
   vadDebugEnabled = enable;
   Serial.printf("VAD debug %s\n", enable ? "enabled" : "disabled");
 }

 void startVADCalibration(uint16_t durationMs) {
   vad.startCalibration(durationMs);
 }

 void getVADCalibrationResults(float& avgSilence, float& maxSilence, float& suggestedSpeechThreshold) {
   vad.getCalibrationResults(avgSilence, maxSilence, suggestedSpeechThreshold);
 }

 void printVADStatus() {
   vad.printDebugInfo();
 }

 void autoTuneVADThresholds() {
   // Based on current background noise level (~1295), set appropriate thresholds
   float currentEnergy = vad.getCurrentEnergy();
   float speechThreshold = currentEnergy + 500.0f;  // 500 above background
   float silenceThreshold = currentEnergy + 100.0f; // 100 above background

   setVADThresholds(speechThreshold, silenceThreshold);
   Serial.printf("VAD auto-tuned based on current energy %.1f\n", currentEnergy);
 }

// Audio gain control functions
void setMicrophoneGain(float gainFactor) {
  if (gainFactor >= 0.1f && gainFactor <= 20.0f) {
    currentMicGain = gainFactor;
    Serial.printf("Microphone gain set to %.2f\n", gainFactor);
  } else {
    Serial.printf("Invalid gain factor %.2f (must be 0.1-20.0)\n", gainFactor);
  }
}

float getMicrophoneGain() {
  return currentMicGain;
}

void enableHighPassFilter(bool enable) {
  highPassEnabled = enable;
  Serial.printf("High-pass filter %s\n", enable ? "enabled" : "disabled");
}

bool isHighPassFilterEnabled() {
  return highPassEnabled;
 }
 