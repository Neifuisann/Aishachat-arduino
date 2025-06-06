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
 #include "Audio.h"
 #include <math.h>
 #include "SPIFFS.h"
 #include <stdlib.h> // Required for rand() and srand()
 #include <time.h>   // Required for time()
 
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
 int currentVolume = 70;
 const int CHANNELS = 1;         // Mono
 const int BITS_PER_SAMPLE = 16; // 16-bit audio
 
 // -------------- OUTPUT (TTS) --------------
 BufferRTOS<uint8_t> audioBuffer(AUDIO_BUFFER_SIZE, AUDIO_CHUNK_SIZE);
 OpusAudioDecoder opusDecoder;
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
 
 // Audio info for speaker
 AudioInfo info(24000, CHANNELS, BITS_PER_SAMPLE);
 
 // -------------- MICROPHONE INPUT --------------
 // We define the same approach you had:
 class WebsocketStream : public Print {
 public:
   virtual size_t write(uint8_t b) override {
     if (!webSocket.isConnected() || deviceState != LISTENING) {
       return 1;
     }
     if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
       webSocket.sendBIN(&b, 1);
       xSemaphoreGive(wsMutex);
     }
     return 1;
   }
 
   virtual size_t write(const uint8_t *buffer, size_t size) override {
     if (!webSocket.isConnected() || deviceState != LISTENING) {
       return size;
     }
     if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
       webSocket.sendBIN(buffer, size);
       xSemaphoreGive(wsMutex);
     }
     return size;
   }
 };
 
 static WebsocketStream wsStream;
 static I2SStream i2sInput;
 static StreamCopy micToWsCopier(wsStream, i2sInput);
 static const int MIC_COPY_SIZE = 64; // chunk size for mic -> WS
 
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
 void transitionToSpeaking() {
   // 1. Signal audioStreamTask to stop playing any current sound
   bool needsTransition = false;
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (deviceState != SPEAKING && deviceState != QUOTA_EXCEEDED) {
           isPlayingProcessingSound = false;
           isPlayingLimitSound = false;
           needsTransition = true;
           if (processingSoundFile) {
               processingSoundFile.close();
           }
       } else if (deviceState == QUOTA_EXCEEDED) {
           isPlayingLimitSound = false;
           needsTransition = true;
           if (processingSoundFile) {
               processingSoundFile.close();
           }
       }
       xSemaphoreGive(wsMutex);
   }

   if (!needsTransition) {
     return; // Already speaking or failed mutex
   }

   // 3. Now perform the main transition under wsMutex lock
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {

       // *** Reset only the TTS buffer ***
       audioBuffer.reset();
       i2s.flush();
       volume.flush();
       queue.flush();

       // *** Now change state ***
       deviceState = SPEAKING;
       speakingStartTime = millis();
       transitionToSpeakingTimestamp = millis();
       realAudioArrived = false;
       isPlayingProcessingSound = false;
       isPlayingLimitSound = false;
       processingSoundFinished = false;

       opusDecoder.setOutput(bufferPrint);

       webSocket.enableHeartbeat(60000, 30000, 3);
       xSemaphoreGive(wsMutex);

   } else {
       Serial.println("TransitionToSpeaking: Failed to get ws mutex!");
   }
 }
 
 void transitionToListening() {
   bool wasPlayingMp3 = false;
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
       if (deviceState == QUOTA_EXCEEDED || isPlayingProcessingSound || isPlayingLimitSound) {
           wasPlayingMp3 = true;
           Serial.println("Stopping MP3 sound due to transitionToListening.");
           isPlayingLimitSound = false;
           isPlayingProcessingSound = false;
           if (processingSoundFile) {
               processingSoundFile.close();
           }
       }
       realAudioArrived = false; // Reset this too
       processingSoundFinished = false;
       xSemaphoreGive(wsMutex);
   }
 
   // Give audio task time to stop if it was playing MP3
   if (wasPlayingMp3) {
       vTaskDelay(pdMS_TO_TICKS(10));
   }
 
 
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      deviceState = PROCESSING; // Intermediate state before LISTENING
      scheduleListeningRestart = false;
      Serial.println("Transitioning to listening mode");
 
      // Flush streams and reset TTS buffer
      i2s.flush();
      volume.flush();
      queue.flush();
      i2sInput.flush(); // Mic input
      audioBuffer.reset();
 
      webSocket.disableHeartbeat();
      xSemaphoreGive(wsMutex);
 
      // Set final state after releasing mutex
      deviceState = LISTENING;
      Serial.println("Transitioned to listening mode");
 
   } else {
      Serial.println("TransitionToListening: Failed to get wsMutex");
      // Attempt to force state anyway? Or just log error? For now, log.
      // Maybe set state directly if mutex fails? Risky.
      deviceState = LISTENING; // Force state change if mutex fails?
   }
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
   sensor_t* sensor = esp_camera_sensor_get();  
   sensor->set_reg(sensor, 0x3008, 0xFF, 0x02); // from standby to ready
 
   delay(1000);
 
   camera_fb_t *fb = nullptr;
   
   fb = esp_camera_fb_get();
 
   // base64-encode
   size_t encodedLen = base64_enc_len(fb->len);
   char *b64buf = (char*)malloc(encodedLen + 1);
 
   base64_encode(b64buf, (char*)fb->buf, fb->len);
   esp_camera_fb_return(fb);
 
   StaticJsonDocument<256> doc;
   doc["type"] = "image";
   doc["mime"] = "image/jpeg";
   doc["data"] = b64buf;
 
   String json;
   serializeJson(doc, json);
   free(b64buf);
 
   // Send to server
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
     webSocket.sendTXT(json);
     xSemaphoreGive(wsMutex);
   }
   Serial.printf("Photo sent => %u bytes of JSON\n", json.length());
 
   sensor->set_reg(sensor, 0x3008, 0xFF, 0x42); // if you want standby again
   delay(1000);
 }
 
 // -----------------------------------------------------------------------------
 // AUDIO TASKS
 // -----------------------------------------------------------------------------
 void audioStreamTask(void *parameter) {
   Serial.println("Starting speaker pipeline...");
   srand(time(NULL));
 
   // Opus Setup (Output initially to bufferPrint)
   OpusSettings cfg;
   cfg.sample_rate = 24000;
   cfg.channels = CHANNELS;
   cfg.bits_per_sample = BITS_PER_SAMPLE;
   cfg.max_buffer_size = 6144; // Adjust if needed
   opusDecoder.setOutput(bufferPrint); // Default output
   opusDecoder.begin(cfg);
 
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
   vcfg.copyFrom(i2sCfg);
   vcfg.allow_boost = true; // Or false if not needed
   volume.begin(vcfg);
   volume.setVolume(currentVolume / 100.0f); // Set initial volume
 
 
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
 
             int randomSoundIndex = rand() % 10 + 1;
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
         copier.copy();
 
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
 
   } // End while(1)
 }
 
 void micTask(void *parameter) {
   Serial.println("Starting microphone pipeline...");
 
   auto i2sConfig = i2sInput.defaultConfig(RX_MODE);
   i2sConfig.bits_per_sample = BITS_PER_SAMPLE;
   i2sConfig.sample_rate = 16000; 
   i2sConfig.channels = CHANNELS;
   i2sConfig.i2s_format = I2S_PCM;
   i2sConfig.signal_type = PDM;
   i2sConfig.pin_data = PDM_DATA;
   i2sConfig.pin_ws   = PDM_CLK;
   i2sConfig.pin_bck  = I2S_PIN_NO_CHANGE;
   i2sConfig.port_no  = I2S_PORT_IN;
   i2sInput.begin(i2sConfig);
 
   while (1) {
     if (scheduleListeningRestart && millis() >= scheduledTime) {
       transitionToListening();
     }
 
     if (deviceState == LISTENING && webSocket.isConnected()) {
       micToWsCopier.copyBytes(MIC_COPY_SIZE);
       vTaskDelay(1);
     } else {
       vTaskDelay(10);
     }
   }
 }
 
 // -----------------------------------------------------------------------------
 // WEBSOCKET EVENTS
 // -----------------------------------------------------------------------------
 void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
   switch (type) {
     case WStype_DISCONNECTED:
       Serial.println("[WSc] Disconnected");
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
         currentVolume = doc["volume_control"] | 70;
         volume.setVolume(currentVolume / 100.0f);
       }
       else if (typeS == "server") {
         String msg = doc["msg"] | "";
         Serial.println(msg);
 
         if (msg == "RESPONSE.COMPLETE" || msg == "RESPONSE.ERROR") {
           Serial.println("Server => done speaking => Listening");
           if (doc.containsKey("volume_control")) {
             int newVol = doc["volume_control"] | 70;
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
 void websocketSetup(String server_domain, int port, String path) {
   // wsMutex = xSemaphoreCreateMutex(); // <<< REMOVED Initialization
   // mp3Mutex = xSemaphoreCreateMutex(); // <<< REMOVED Initialization

   Serial.println("=== WEBSOCKET SETUP ===");
   Serial.println("Server: " + server_domain + ":" + String(port) + path);

   if (authTokenGlobal.isEmpty()) {
     Serial.println("WARNING: Auth token is empty! WebSocket connection will likely fail.");
     Serial.println("Authorization header: Bearer [EMPTY]");
   } else {
     Serial.println("Auth token: " + authTokenGlobal.substring(0, 20) + "...");
   }

   String headers = "Authorization: Bearer " + String(authTokenGlobal);
   webSocket.setExtraHeaders(headers.c_str());
   webSocket.onEvent(webSocketEvent);
   webSocket.setReconnectInterval(1000);
 
   webSocket.enableHeartbeat(60000, 30000, 3);
 
 #ifdef DEV_MODE
   webSocket.begin(server_domain.c_str(), port, path.c_str());
 #else
   webSocket.beginSslWithCA(server_domain.c_str(), port, path.c_str(), CA_cert);
 #endif

   Serial.println("WebSocket connection initiated");
   Serial.println("======================");
 }
 
 void networkTask(void *parameter) {
   while (1) {
     webSocket.loop();
     vTaskDelay(1);
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
 