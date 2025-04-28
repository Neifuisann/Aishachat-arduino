#include "OTA.h"
#include <Arduino.h>
#include <driver/rtc_io.h>
#include "Config.h"
#include "SPIFFS.h"
#include "WifiManager.h"
#include "Button.h"
#include "FactoryReset.h"

#include "esp_camera.h"              
#define CAMERA_MODEL_XIAO_ESP32S3      
#include "camera_pins.h"     
#include "ESP32_OV5640_AF.h"    

// #define WEBSOCKETS_DEBUG_LEVEL WEBSOCKETS_LEVEL_ALL  

// ----- Touch-button / Wake pin ---------------------------------
#define WAKE_PIN         GPIO_NUM_1    // D0 on XIAO
#define WAKE_ACTIVE_HIGH 1             // TTP-223 default output
#define LONG_PRESS_MS    5000           // keep -> sleep
#define DEBOUNCE_MS      60

OV5640 ov5640;
AsyncWebServer webServer(80);
WIFIMANAGER WifiManager;
esp_err_t getErr = ESP_OK;

void goToSleep() {
    // wait until finger is released, or we’ll bounce straight back
    while (digitalRead((int)WAKE_PIN) == WAKE_ACTIVE_HIGH) vTaskDelay(10);
  
    // give RTC domain control over the pin
    gpio_pulldown_en(WAKE_PIN);            // keep it LOW while sleeping
    esp_sleep_enable_ext0_wakeup(WAKE_PIN, WAKE_ACTIVE_HIGH);  // EXT0
  
    Serial.println("Deep-sleep now…");

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO); 
    delayMicroseconds(200);// wait for the pull-down to take effect

    esp_deep_sleep_start();
  }
  

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

static void onButtonLongPressUpEventCb(void *button_handle, void *usr_data)
{
    Serial.println("Button long press end");
    vTaskDelay(10);
    goToSleep();
}

static void onButtonDoubleClickCb(void *button_handle, void *usr_data)
{
    Serial.println("Button double click");
    vTaskDelay(10);
    goToSleep();
}

void getAuthTokenFromNVS()
{
    preferences.begin("auth", false);
    authTokenGlobal = preferences.getString("auth_token", "");
    preferences.end();
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
  cfg.xclk_freq_hz = 8000000;
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

  ov5640.start(s);
  if (ov5640.focusInit() == 0) 
  {
    Serial.println("OV5640 Focus Init Successful!");
  }

  if (ov5640.autoFocusMode() == 0) 
  {
    Serial.println("OV5640 Auto Focus Successful!");
  }

  s->set_reg(s, 0x3008, 0xFF, 0x42);   //camera to standby
  delay(1000);
}

void IRAM_ATTR touchTask(void*){
    bool pressed=false;
    unsigned long t0=0;
  
    while (true){
      bool now = digitalRead((int)WAKE_PIN)==WAKE_ACTIVE_HIGH;
      unsigned long ms = millis();
  
      if (now && !pressed) {           // touch-down
        pressed=true;  t0=ms;
      }
      if (!now && pressed) {           // release
        pressed=false;
        if (ms-t0 > LONG_PRESS_MS) goToSleep();   // long press => sleep
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
}
  


void setupDeviceMetadata() {
    // factoryResetDevice();
    deviceState = IDLE;

    getAuthTokenFromNVS();
    getOTAStatusFromNVS();

    if (otaState == OTA_IN_PROGRESS || otaState == OTA_COMPLETE) {
        deviceState = OTA;
    }
    if (factory_reset_status) {
        deviceState = FACTORY_RESET;
    }
}

void setup()
{
    Serial.begin(115200);
    vTaskDelay(500);

    // SETUP
    init_camera_once();
    setupDeviceMetadata();
    wsMutex = xSemaphoreCreateMutex();    

    pinMode((int)WAKE_PIN, INPUT_PULLDOWN);        // idle LOW
    rtc_gpio_pulldown_en(WAKE_PIN);
    
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) 
    {
        Serial.println("Woke up via TTP-223 touch");
    }    

    // INTERRUPT
    #ifdef TTP_GPIO_WAKE
        xTaskCreatePinnedToCore(touchTask,"touch",2048,nullptr,2,nullptr,1);

    #else
        getErr = esp_sleep_enable_ext0_wakeup(BUTTON_PIN, LOW);
        printOutESP32Error(getErr);
        Button *btn = new Button(BUTTON_PIN, false);
        btn->attachLongPressUpEventCb(&onButtonLongPressUpEventCb, NULL);
        btn->attachDoubleClickEventCb(&onButtonDoubleClickCb, NULL);
        btn->detachSingleClickEvent();
    #endif

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
}

void loop(){
    if (otaState == OTA_IN_PROGRESS)
    {
        loopOTA();
    }
}