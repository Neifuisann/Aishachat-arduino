/*******************************************************************************
 * Audio.cpp - Combined
 * Includes your working microphone/speaker code AND the photo capture/vision code.
 *******************************************************************************/

 #include "OTA.h"
 #include "Print.h"
 #include "Config.h"
 #include "AudioTools.h"
 #include "AudioTools/AudioCodecs/CodecOpus.h"
 #include <WebSocketsClient.h>
 #include "Audio.h"
 
 // For camera:
 #include "base64.h"
 #include "esp_camera.h"
 
 // -------------- WEBSOCKET --------------
 SemaphoreHandle_t wsMutex;
 WebSocketsClient webSocket;
 
 // -------------- TASK HANDLES --------------
 TaskHandle_t speakerTaskHandle = NULL;
 TaskHandle_t micTaskHandle = NULL;
 TaskHandle_t networkTaskHandle = NULL;
 
 // -------------- TIMING --------------
 bool scheduleListeningRestart = false;
 unsigned long scheduledTime = 0;
 unsigned long speakingStartTime = 0;
 
 // -------------- AUDIO SETTINGS --------------
 int currentVolume = 70;
 const int CHANNELS = 1;         // Mono
 const int BITS_PER_SAMPLE = 16; // 16-bit audio
 
 // -------------- OUTPUT (TTS) --------------
 BufferRTOS<uint8_t> audioBuffer(AUDIO_BUFFER_SIZE, AUDIO_CHUNK_SIZE);
 OpusAudioDecoder opusDecoder;
 
 // We'll push TTS data to this "bufferPrint," which then writes into "audioBuffer."
 class BufferPrint : public Print {
 public:
   BufferPrint(BufferRTOS<uint8_t>& buf) : _buffer(buf) {}
 
   // Write single byte
   virtual size_t write(uint8_t data) override {
     if (webSocket.isConnected() && deviceState == SPEAKING) {
       return _buffer.writeArray(&data, 1);
     }
     return 0;
   }
 
   // Write buffer
   virtual size_t write(const uint8_t *buffer, size_t size) override {
     if (webSocket.isConnected() && deviceState == SPEAKING) {
       return _buffer.writeArray(buffer, size);
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
   vTaskDelay(50);
 
   // flush mic input so we don't read old data
   i2sInput.flush();
 
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
     deviceState = SPEAKING;
     speakingStartTime = millis();
 
     webSocket.enableHeartbeat(60000, 30000, 3);
     xSemaphoreGive(wsMutex);
   }
 
   Serial.println("Transitioned to speaking mode");
 }
 
 void transitionToListening() {
   deviceState = PROCESSING;
   scheduleListeningRestart = false;
   Serial.println("Transitioning to listening mode");
 
   // flush streams
   i2s.flush();
   volume.flush();
   queue.flush();
   i2sInput.flush();
   audioBuffer.reset();
 
   Serial.println("Transitioned to listening mode");
 
   if (xSemaphoreTake(wsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
     deviceState = LISTENING;
     webSocket.disableHeartbeat();
     xSemaphoreGive(wsMutex);
   }
 }

 void transitionToTakingPhoto() {
  deviceState = TAKING_PHOTO;

  captureAndSendPhotoBase64();

  Serial.println("Photo capture done");

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
 
   // Setup Opus decoder for TTS
   OpusSettings cfg;
   cfg.sample_rate = 24000; 
   cfg.channels = CHANNELS;
   cfg.bits_per_sample = BITS_PER_SAMPLE;
   cfg.max_buffer_size = 6144;
   opusDecoder.setOutput(bufferPrint);
   opusDecoder.begin(cfg);
 
   queue.begin();
 
   // I2S config for speaker
   auto i2sCfg = i2s.defaultConfig(TX_MODE);
   i2sCfg.bits_per_sample = BITS_PER_SAMPLE;
   i2sCfg.sample_rate = 24000;
   i2sCfg.channels = CHANNELS;
   i2sCfg.pin_bck  = I2S_BCK_OUT;
   i2sCfg.pin_ws   = I2S_WS_OUT;
   i2sCfg.pin_data = I2S_DATA_OUT;
   i2sCfg.port_no  = I2S_PORT_OUT;
   i2sCfg.copyFrom(info);
   i2s.begin(i2sCfg);
 
   auto vcfg = volume.defaultConfig();
   vcfg.copyFrom(i2sCfg);
   vcfg.allow_boost = true;
   volume.begin(vcfg);
 
   while (1) {
     if (webSocket.isConnected() && deviceState == SPEAKING) {
       // Copy TTS from queue -> speaker
       copier.copy();
     }
     vTaskDelay(1);
   }
 }
 
 void micTask(void *parameter) {
   Serial.println("Starting microphone pipeline...");
 
   // Configure I2S input for PDM mic
   auto i2sConfig = i2sInput.defaultConfig(RX_MODE);
   i2sConfig.bits_per_sample = BITS_PER_SAMPLE;
   i2sConfig.sample_rate = 16000; 
   i2sConfig.channels = CHANNELS;
   i2sConfig.i2s_format = I2S_PCM;
   i2sConfig.signal_type = PDM;
   i2sConfig.pin_data = PDM_DATA;
   i2sConfig.pin_ws   = PDM_CLK;
   i2sConfig.pin_bck  = I2S_PIN_NO_CHANGE; // Not used for PDM
   i2sConfig.port_no  = I2S_PORT_IN;
   i2sInput.begin(i2sConfig);
 
   while (1) {
     if (scheduleListeningRestart && millis() >= scheduledTime) {
       transitionToListening();
     }
 
     if (deviceState == LISTENING && webSocket.isConnected()) {
       // read mic -> forward to WS
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
         // e.g. currentVolume, OTA, reset, etc.
         currentVolume = doc["volume_control"] | 70;
         volume.setVolume(currentVolume / 100.0f);
 
         bool is_ota = doc["is_ota"] | false;
         bool is_reset = doc["is_reset"] | false;
         if (is_ota) {
           Serial.println("OTA update requested => handle");
           setOTAStatusInNVS(OTA_IN_PROGRESS);
           ESP.restart();
         }
         if (is_reset) {
           Serial.println("Factory reset requested => handle");
           // ...
           ESP.restart();
         }
       }
       else if (typeS == "server") {
         String msg = doc["msg"] | "";
         Serial.println(msg);
 
         if (msg == "RESPONSE.COMPLETE" || msg == "RESPONSE.ERROR") {
           // wrap up => go listening
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
       }
     }
     break;
 
     case WStype_BIN:
       // TTS opus data from server
       if (scheduleListeningRestart || deviceState != SPEAKING) {
         Serial.println("Ignoring inbound TTS because not in SPEAKING mode");
         break;
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
   String headers = "Authorization: Bearer " + String(authTokenGlobal);
   webSocket.setExtraHeaders(headers.c_str());
   webSocket.onEvent(webSocketEvent);
   webSocket.setReconnectInterval(1000);
 
   // Heartbeat pings
   webSocket.enableHeartbeat(60000, 30000, 3);
 
 #ifdef DEV_MODE
   webSocket.begin(server_domain.c_str(), port, path.c_str());
 #else
   webSocket.beginSslWithCA(server_domain.c_str(), port, path.c_str(), CA_cert);
 #endif
 }
 
 void networkTask(void *parameter) {
   while (1) {
     webSocket.loop();
     vTaskDelay(1);
   }
 }
