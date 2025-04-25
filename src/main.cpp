#include "OTA.h"
#include <Arduino.h>
#include <driver/rtc_io.h>
#include "Config.h"
#include "SPIFFS.h"
#include "WifiManager.h"
#include "Button.h"
#include "FactoryReset.h"

// #define WEBSOCKETS_DEBUG_LEVEL WEBSOCKETS_LEVEL_ALL  

// ----- Touch-button / Wake pin ---------------------------------
#define WAKE_PIN         GPIO_NUM_1    // D0 on XIAO
#define WAKE_ACTIVE_HIGH 1             // TTP-223 default output
#define LONG_PRESS_MS    500           // keep -> sleep
#define DEBOUNCE_MS      60


AsyncWebServer webServer(80);
WIFIMANAGER WifiManager;
esp_err_t getErr = ESP_OK;

void goToSleep() {
    // wait until finger is released, or we’ll bounce straight back
    while (digitalRead((int)WAKE_PIN) == WAKE_ACTIVE_HIGH) delay(10);
  
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
    delay(10);
    goToSleep();
}

static void onButtonDoubleClickCb(void *button_handle, void *usr_data)
{
    Serial.println("Button double click");
    delay(10);
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
    WifiManager.startBackgroundTask("ELATO-DEVICE");        // Run the background task to take care of our Wifi
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
    delay(500);

    // SETUP
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