#include <Arduino.h>
#include <driver/rtc_io.h>
#include "Config.h"
#include "SPIFFS.h"
#include "WifiManager.h"
#include "FactoryReset.h"
#include "Audio.h"

#include "esp_camera.h"              
#define CAMERA_MODEL_XIAO_ESP32S3      
#include "camera_pins.h"      

// #define WEBSOCKETS_DEBUG_LEVEL WEBSOCKETS_LEVEL_ALL  

AsyncWebServer webServer(80);
WIFIMANAGER WifiManager;
esp_err_t getErr = ESP_OK;


void printOutESP32Error(esp_err_t err)
{
    switch (err)
    {
    case ESP_OK:
        Serial.println("ESP_OK no errors");
        break;
    case ESP_ERR_INVALID_ARG:
        Serial.println("ESP_ERR_INVALID_ARG if the selected GPIO is not an RTC GPIO, or the mode is invalid");
        break;
    case ESP_ERR_INVALID_STATE:
        Serial.println("ESP_ERR_INVALID_STATE if wakeup triggers conflict or wireless not stopped");
        break;
    default:
        Serial.printf("Unknown error code: %d\n", err);
        break;
    }
}

void getAuthTokenFromNVS()
{
    preferences.begin("auth", false);
    authTokenGlobal = preferences.getString("auth_token", "");
    preferences.end();

    Serial.println("=== AUTH TOKEN STATUS ===");
    if (authTokenGlobal.isEmpty()) {
        Serial.println("No auth token found in NVS storage");
    } else {
        Serial.println("Auth token loaded from NVS: " + authTokenGlobal.substring(0, 20) + "...");
    }
    Serial.println("========================");
}

void setupWiFi()
{
    WifiManager.startBackgroundTask("Aisha Device");        // Run the background task to take care of our Wifi
    WifiManager.fallbackToSoftAp(true);       // Run a SoftAP if no known AP can be reached
    WifiManager.attachWebServer(&webServer);  // Attach our API to the Webserver 
    WifiManager.attachUI();                   // Attach the UI to the Webserver
  
    // Run the Webserver and add your webpages to it
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/wifi");
    });
    webServer.onNotFound([&](AsyncWebServerRequest *request) {
      request->send(404, "text/plain", "Not found");
    });
    webServer.begin();
}


static bool camera_ok = false;

static void init_camera_once()
{
  if (camera_ok) return;

  camera_config_t cfg;
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0  = Y2_GPIO_NUM;
  cfg.pin_d1  = Y3_GPIO_NUM;
  cfg.pin_d2  = Y4_GPIO_NUM;
  cfg.pin_d3  = Y5_GPIO_NUM;
  cfg.pin_d4  = Y6_GPIO_NUM;
  cfg.pin_d5  = Y7_GPIO_NUM;
  cfg.pin_d6  = Y8_GPIO_NUM;
  cfg.pin_d7  = Y9_GPIO_NUM;
  cfg.pin_xclk = XCLK_GPIO_NUM;
  cfg.pin_pclk = PCLK_GPIO_NUM;
  cfg.pin_vsync = VSYNC_GPIO_NUM;
  cfg.pin_href  = HREF_GPIO_NUM;
  cfg.pin_sscb_sda = SIOD_GPIO_NUM;
  cfg.pin_sscb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn  = PWDN_GPIO_NUM;
  cfg.pin_reset = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 10000000;
  cfg.grab_mode = CAMERA_GRAB_LATEST;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size = FRAMESIZE_XGA;     // 1024×768 = good compromise 
  cfg.jpeg_quality = 24;              // 0–63 (lower = better quality)
  cfg.fb_location = CAMERA_FB_IN_PSRAM;
  cfg.fb_count = 2;

  if (esp_camera_init(&cfg) == ESP_OK) {
    camera_ok = true;
    Serial.println("Camera initialised");
  } else {
    Serial.println("Camera init failed - vision disabled");
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_hmirror(s, 1);
  s->set_saturation(s, 4);

  s->set_reg(s, 0x3008, 0xFF, 0x42);   //camera to standby
  delay(1000);
}

void setupDeviceMetadata() {
    // Initialize mutexes early, before tasks start
    wsMutex = xSemaphoreCreateMutex();
    mp3Mutex = xSemaphoreCreateMutex(); // <<< Initialize MP3 Mutex here

    // factoryResetDevice(); // Call this if needed before state checks

    getAuthTokenFromNVS();

    Serial.printf("Initial Device State: %d\n", deviceState); // Add log
}



void setup()
{
    Serial.begin(115200);
    vTaskDelay(500);

    // SETUP
    setupDeviceMetadata();

    // Mount SPIFFS *before* creating tasks that might use it
    if(!SPIFFS.begin(true)) { // Use format SPIFFS if not mounted
        Serial.println("SPIFFS Mount Failed! Halting.");
        while(true) { delay(10); } // Stop execution if SPIFFS fails
    } else {
        Serial.println("SPIFFS Mounted.");
    }

    xTaskCreatePinnedToCore(
        audioStreamTask,   // Function
        "Speaker Task",    // Name
        4096,              // Stack size
        NULL,              // Parameters
        3,                 // Priority
        NULL,              // Handle
        1                  // Core 1 (application core)
    );

    xTaskCreatePinnedToCore(
        micTask,           // Function
        "Microphone Task", // Name
        4096,              // Stack size
        NULL,              // Parameters
        4,                 // Priority
        NULL,              // Handle
        1                  // Core 1 (application core)
    );

    // Pin network task to Core 0 (protocol core)
    xTaskCreatePinnedToCore(
        networkTask,       // Function
        "Websocket Task",  // Name
        8192,              // Stack size
        NULL,              // Parameters
        configMAX_PRIORITIES-1, // Highest priority
        &networkTaskHandle,// Handle
        0                  // Core 0 (protocol core)
    );
    


    // WIFI
    setupWiFi();

    // Camera (can run after tasks are created)
    init_camera_once();
}

void loop(){
}