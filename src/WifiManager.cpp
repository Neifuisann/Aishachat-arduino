/**
 * Wifi Manager
 * (c) 2022-2024 Martin Verges
 *
 * Licensed under CC BY-NC-SA 4.0
 * (Attribution-NonCommercial-ShareAlike 4.0 International)
**/
#include "WifiManager.h"
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include <WiFi.h>
#include <Preferences.h>
#include <Config.h>
#include <esp_wifi.h> // add for power save and rate control
#include "esp_wifi_types.h"
#include "esp_private/wifi.h"

bool isDeviceRegistered() {
  if (!authTokenGlobal.isEmpty()) {
    return true;
  }

    HTTPClient http;
    WiFiClientSecure client;
    client.setCACert(Vercel_CA_cert);

    #ifdef DEV_MODE
    http.begin("http://" + String(backend_server) + ":" + String(backend_port) +
                 "/api/generate_auth_token?macAddress=" + WiFi.macAddress());
    #else
    http.begin(client, "https://" + String(backend_server) +
                 "/api/generate_auth_token?macAddress=" + WiFi.macAddress());
    #endif

    http.setTimeout(10000);

    int httpCode = http.GET();

    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
            Serial.print("JSON parsing failed: ");
            Serial.println(error.c_str());
            http.end();
            return false;
        }

        String authToken = doc["token"];
        if (!authToken.isEmpty()) {
            // Store the auth token in NVS
            preferences.begin("auth", false);
            preferences.putString("auth_token", authToken);
            preferences.end();

            authTokenGlobal = String(authToken);
            http.end();
            return true;
        }
    }

    // If we get here, either the request failed or no token was found
    http.end();
    return false;
}

void connectCb() {
  Serial.println("On connecting to Wifi");

  // Ensure device is registered and has a valid auth token before connecting WebSocket
  Serial.println("=== CHECKING DEVICE REGISTRATION ===");

  if (authTokenGlobal.isEmpty()) {
    Serial.println("No auth token found, attempting to register device...");

    if (isDeviceRegistered()) {
      Serial.println("Device registration successful!");
      Serial.println("New token: " + authTokenGlobal.substring(0, 20) + "...");
    } else {
      Serial.println("ERROR: Device registration failed!");
      Serial.println("Please ensure the device is registered on the server.");
      Serial.println("MAC Address: " + WiFi.macAddress());
      Serial.println("WebSocket connection will be attempted anyway...");
    }
  } else {
    Serial.println("Auth token exists: " + authTokenGlobal.substring(0, 20) + "...");
  }

  Serial.println("===================================");

  // Play connection success sound
  playSoundFile("/connecting.mp3");

  // Immediately setup WebSocket when WiFi connects (like old robust implementation)
  // Call websocketSetup immediately when WiFi connects (old robust pattern)
  websocketSetup(ws_server, ws_port, ws_path);
}

/**
 * @brief Write a message to the Serial interface
 * @param msg The message to be written
 *
 * This function is a simple wrapper around Serial.print() to write a message
 * to the serial console. It can be overwritten by a custom implementation for 
 * enhanced logging.
 */
void WIFIMANAGER::logMessage(String msg) {
  Serial.print(msg);
}

/**
 * @brief Background Task running as a loop forever
 * @param param needs to be a valid WIFIMANAGER instance
 */
void wifiTask(void* param) {
  yield();
  vTaskDelay(500); // wait a short time until everything is setup before executing the loop forever
  yield();
  const TickType_t xDelay = 10000 / portTICK_PERIOD_MS;
  WIFIMANAGER * wifimanager = (WIFIMANAGER *) param;

  for(;;) {
    yield();
    wifimanager->loop();
    yield();
    vTaskDelay(xDelay);
  }
}

/**
 * @brief Start the background task, which will take care of the Wifi connection
 *
 * This method will load the configuration from NVS, try to connect to the configured WIFI(s)
 * and then start a background task, which will keep monitoring and trying to reconnect
 * to the configured WIFI(s) in case the connection drops.
 */
void WIFIMANAGER::startBackgroundTask(String softApName, String softApPass) {
  if (softApName.length()) this->softApName = softApName;
  if (softApPass.length()) this->softApPass = softApPass;

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE); // keep RF awake - eliminates 400-3000ms jitter
  loadFromNVS();
  tryConnect();

  BaseType_t taskCreated = xTaskCreatePinnedToCore(
    wifiTask,
    "WifiManager",
    4096,   // Stack size in words
    this,   // Task input parameter
    1,      // Priority of the task
    &WifiCheckTask,  // Task handle.
    0       // Core where the task should run
  );

  if (taskCreated != pdPASS) {
    logMessage("[ERROR] WifiManager: Error creating background task\n");
  }
}

/**
 * @brief Construct a new WIFIMANAGER::WIFIMANAGER object
 * @details Puts the Wifi mode to AP+STA and registers Wifi Events
 * @param ns Namespace for the preferences non volatile storage (NVS)
 */
WIFIMANAGER::WIFIMANAGER(const char * ns) {
  NVS = (char *)ns;

  // AP on/off
  WiFi.onEvent([&](WiFiEvent_t event, WiFiEventInfo_t info) {
    logMessage("[WIFI] onEvent() AP mode started!\n");
    playSoundFile("/No_wifi_startAP.mp3");
    softApRunning = true;
#if ESP_ARDUINO_VERSION_MAJOR >= 2
    }, ARDUINO_EVENT_WIFI_AP_START); // arduino-esp32 2.0.0 and later
#else
    }, SYSTEM_EVENT_AP_START); // arduino-esp32 1.0.6
#endif
  WiFi.onEvent([&](WiFiEvent_t event, WiFiEventInfo_t info) {
    logMessage("[WIFI] onEvent() AP mode stopped!\n");
    softApRunning = false;
#if ESP_ARDUINO_VERSION_MAJOR >= 2
    }, ARDUINO_EVENT_WIFI_AP_STOP); // arduino-esp32 2.0.0 and later
#else
    }, SYSTEM_EVENT_AP_STOP); // arduino-esp32 1.0.6
#endif
  // AP client join/leave
  WiFi.onEvent([&](WiFiEvent_t event, WiFiEventInfo_t info) {
    logMessage("[WIFI] onEvent() new client connected to softAP!\n");
#if ESP_ARDUINO_VERSION_MAJOR >= 2
    }, ARDUINO_EVENT_WIFI_AP_STACONNECTED); // arduino-esp32 2.0.0 and later
#else
    }, SYSTEM_EVENT_AP_STACONNECTED); // arduino-esp32 1.0.6
#endif
  WiFi.onEvent([&](WiFiEvent_t event, WiFiEventInfo_t info) {
    logMessage("[WIFI] onEvent() Client disconnected from softAP!\n");
#if ESP_ARDUINO_VERSION_MAJOR >= 2
    }, ARDUINO_EVENT_WIFI_AP_STADISCONNECTED); // arduino-esp32 2.0.0 and later
#else
    }, SYSTEM_EVENT_AP_STADISCONNECTED); // arduino-esp32 1.0.6
#endif
}

/**
 * @brief Destroy the WIFIMANAGER::WIFIMANAGER object
 * @details will stop the background task as well but not cleanup the AsyncWebserver
 */
WIFIMANAGER::~WIFIMANAGER() {
  vTaskDelete(WifiCheckTask);
  // FIXME: get rid of the registered Webserver AsyncCallbackWebHandlers
}

/**
 * @brief If no WIFI is available, fallback to create an AP on the ESP32
 * @param state boolean true (create AP) or false (don't create an AP)
 */
void WIFIMANAGER::fallbackToSoftAp(const bool state) {
  createFallbackAP = state;
}

/**
 * @brief Get the current configured fallback state
 * @return true
 * @return false
 */
bool WIFIMANAGER::getFallbackState() {
  return createFallbackAP;
}

/**
 * @brief Remove all entries from the current known and configured Wifi list
 * @details This only affects memory, not the storage!
 * @details If you wan't to persist this, you need to call writeToNVS()
 */
void WIFIMANAGER::clearApList() {
  for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
    apList[i].apName = "";
    apList[i].apPass = "";
  }
}

/**
 * @brief Load last saved configuration from the NVS into the memory
 * @return true on success
 * @return false on error
 */
bool WIFIMANAGER::loadFromNVS() {
  configuredSSIDs = 0;
  if (preferences.begin(NVS, true)) {
    clearApList();
    char tmpKey[10] = { 0 };
    for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
      sprintf(tmpKey, "apName%d", i);
      if (preferences.getType(tmpKey) == PT_STR) {
        String apName = preferences.getString(tmpKey, "");
        if (apName.length() > 0) {
          sprintf(tmpKey, "apPass%d", i);
          String apPass = preferences.getString(tmpKey);
          logMessage(String("[WIFI] Load SSID '") + apName + "' to " + String(i+1) + ". slot.\n");
          apList[i].apName = apName;
          apList[i].apPass = apPass;
          configuredSSIDs++;
        }
      }
    }
    preferences.end();
    return true;
  }
  logMessage("[WIFI] Unable to load data from NVS, giving up...\n");
  return false;
}

/**
 * @brief Write the current in memory configuration to the non volatile storage
 * @return true on success
 * @return false on error with the NVS
 */
bool WIFIMANAGER::writeToNVS() {
  if (!preferences.begin(NVS, false)) {
    logMessage("[WIFI] Unable to write data to NVS, giving up...");
    return false;
  }

  preferences.clear();
  char tmpKey[10];
  for(uint8_t i = 0; i < WIFIMANAGER_MAX_APS; i++) {
    if (apList[i].apName.isEmpty()) continue;

    snprintf(tmpKey, sizeof(tmpKey), "apName%d", i);
    preferences.putString(tmpKey, apList[i].apName);

    snprintf(tmpKey, sizeof(tmpKey), "apPass%d", i);
    preferences.putString(tmpKey, apList[i].apPass);
  }

  preferences.end();
  return true;
}

/**
 * @brief Add a new WIFI SSID to the known credentials list
 * @param apName Name of the SSID to connect to
 * @param apPass Password (or empty) to connect to the SSID
 * @param updateNVS Write the new entry directly to NVS
 * @return true on success
 * @return false on failure
 */
bool WIFIMANAGER::addWifi(String apName, String apPass, bool updateNVS) {
  if(apName.length() < 1 || apName.length() > 31) {
    logMessage("[WIFI] No SSID given or ssid too long");
    return false;
  }

  if(apPass.length() > 63) {
    logMessage("[WIFI] Passphrase too long");
    return false;
  }

  for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
    if (apList[i].apName == "") {
      logMessage(String("[WIFI] Found unused slot Nr. ") + String(i) + " to store the new SSID '" + apName + "' credentials.\n");
      apList[i].apName = apName;
      apList[i].apPass = apPass;
      configuredSSIDs++;
      if (updateNVS) return writeToNVS();
      else return true;
    }
  }
  logMessage("[WIFI] No slot available to store SSID credentials");
  return false; // max entries reached
}

/**
 * @brief Drop a known SSID entry ID from the known list and write change to NVS
 * @param apId ID of the SSID within the array
 * @return true on success
 * @return false on error
 */
bool WIFIMANAGER::delWifi(uint8_t apId) {
  if (apId < WIFIMANAGER_MAX_APS) {
    apList[apId].apName.clear();
    apList[apId].apPass.clear();
    return writeToNVS();
  }
  return false;
}

/**
 * @brief Drop a known SSID name from the known list and write change to NVS
 * @param apName SSID name
 * @return true on success
 * @return false on error
 */
bool WIFIMANAGER::delWifi(String apName) {
  int num = 0;
  for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
    if (apList[i].apName == apName) {
      if (delWifi(i)) num++;
    }
  }
  return num > 0;
}

/**
 * @brief Provides information about the current configuration state
 * @details When at least 1 SSID is configured, the return value will be true, otherwise false
 * @return true if one or more SSIDs stored
 * @return false if no configuration is available
 */
bool WIFIMANAGER::configAvailable() {
    return configuredSSIDs != 0;
}

/**
 * @brief Provides the apList element id of the first configured slot
 * @details It's used to speed up connection by getting the first available configuration
 * @note only call this function when you have configuredSSIDs > 0, otherwise it will return 0 as well and fail!
 * @return uint8_t apList element id
 */
uint8_t WIFIMANAGER::getApEntry() {
  for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
    if (apList[i].apName.length()) return i;
  }
  logMessage("[WIFI][ERROR] We did not find a valid entry!\n");
  logMessage("[WIFI][ERROR] Make sure to not call this function if configuredSSIDs != 1.\n");
  return 0;
}

/**
 * @brief Background loop function running inside the task
 * @details regulary check if the connection is up&running, try to reconnect or create a fallback AP
 */
void WIFIMANAGER::loop() {
  if (millis() - lastWifiCheckMillis < intervalWifiCheckMillis) return;
  lastWifiCheckMillis = millis();

  if(WiFi.waitForConnectResult() == WL_CONNECTED) {
    // Check if we are connected to a well known SSID
    for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
      if (WiFi.SSID() == apList[i].apName) {
        logMessage(String("[WIFI][STATUS] Connected to known SSID: '") + WiFi.SSID() + "' with IP " + WiFi.localIP().toString() + "\n");
        return;
      }
    }
    // looks like we are connected to something else, strange!?
    logMessage("[WIFI] We are connected to an unknown SSID ignoring. Connected to: " + WiFi.SSID() + "\n");
  } else {
    if (softApRunning) {
      logMessage("[WIFI] Not trying to connect to a known SSID. SoftAP has " + String(WiFi.softAPgetStationNum()) + " clients connected!\n");
    } else {
      // let's try to connect to some WiFi in Range
      if (!tryConnect()) {
        if (createFallbackAP) runSoftAP();
        else logMessage("[WIFI] Auto creation of SoftAP is disabled, no starting AP!\n");
      }
    }
  }

  if (softApRunning && millis() - startApTimeMillis > timeoutApMillis) {
    if (WiFi.softAPgetStationNum() > 0) {
      logMessage("[WIFI] SoftAP has " + String(WiFi.softAPgetStationNum()) + " clients connected!\n");
      startApTimeMillis = millis(); // reset timeout as someone is connected
      return;
    }
    logMessage("[WIFI] Running in AP mode but timeout reached. Closing AP!\n");
    stopSoftAP();
    vTaskDelay(100);
  }
}

/**
 * @brief Try to connect to one of the configured SSIDs (if available).
 * @details If more than 2 SSIDs configured, scan for available WIFIs and connect to the strongest
 * @return true on success
 * @return false on error or no configuration
 */
bool WIFIMANAGER::tryConnect() {
  if (!configAvailable()) {
    logMessage("[WIFI] No SSIDs configured in NVS, unable to connect\n");
    if (createFallbackAP) runSoftAP();
    return false;
  }

  if (softApRunning) {
    logMessage("[WIFI] Not trying to connect. SoftAP has " + String(WiFi.softAPgetStationNum()) + " clients connected!\n");
    return false;
  }

  int choosenAp = INT_MIN;
  if (configuredSSIDs == 1) {
    // only one configured SSID, skip scanning and try to connect to this specific one.
    choosenAp = getApEntry();
  } else {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_ps(WIFI_PS_NONE); // keep RF awake during scanning
    int8_t scanResult = WiFi.scanNetworks(false, true);
    if(scanResult <= 0) {
      logMessage("[WIFI] Unable to find WIFI networks in range to this device!\n");
      return false;
    }
    logMessage(String("[WIFI] Found ") + String(scanResult) + " networks in range\n");
    int choosenRssi = INT_MIN;  // we want to select the strongest signal with the highest priority if we have multiple SSIDs available
    for(int8_t x = 0; x < scanResult; ++x) {
      String ssid;
      uint8_t encryptionType;
      int32_t rssi;
      uint8_t* bssid;
      int32_t channel;
      WiFi.getNetworkInfo(x, ssid, encryptionType, rssi, bssid, channel);
      for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
        if (apList[i].apName.length() == 0 || apList[i].apName != ssid) continue;

        if (rssi > choosenRssi) {
          if(encryptionType == WIFI_AUTH_OPEN || apList[i].apPass.length() > 0) { // open wifi or we do know a password
            choosenAp = i;
            choosenRssi = rssi;
          }
        } // else lower wifi signal
      }
    }
    WiFi.scanDelete();
  }

  if (choosenAp == INT_MIN) {
    logMessage("[WIFI] Unable to find an SSID to connect to!\n");
    return false;
  } else {
    logMessage(String("[WIFI] Trying to connect to SSID ") + apList[choosenAp].apName 
      + " with password " + (apList[choosenAp].apPass.length() > 0 ? "'***'" : "''") + "\n"
    );

    WiFi.begin(apList[choosenAp].apName.c_str(), apList[choosenAp].apPass.c_str());

    // Set max TX power and robust rate for fewer lost frames near edge of coverage
    WiFi.setTxPower(WIFI_POWER_19_5dBm); // max legal power in most regions
    esp_wifi_internal_set_fix_rate(WIFI_IF_STA, true, WIFI_PHY_RATE_MCS0_SGI); // legacy 6 Mbps, robust
    wl_status_t status = (wl_status_t)WiFi.waitForConnectResult(5000UL);

    auto startTime = millis();
    // wait for connection, fail, or timeout
    while(status != WL_CONNECTED && status != WL_NO_SSID_AVAIL && status != WL_CONNECT_FAILED && (millis() - startTime) <= 10000) {
        vTaskDelay(10);
        status = (wl_status_t)WiFi.waitForConnectResult(5000UL);
    }
    switch(status) {
      case WL_IDLE_STATUS:
        logMessage("[WIFI] Connecting failed (0): Idle\n");
        break;
      case WL_NO_SSID_AVAIL:
        logMessage("[WIFI] Connecting failed (1): The AP can't be found\n");
        break;
      case WL_SCAN_COMPLETED:
        logMessage("[WIFI] Connecting failed (2): Scan completed\n");
        break;
      case WL_CONNECTED: // 3
        logMessage("[WIFI] Connection successful\n");
        logMessage("[WIFI] SSID   : " + WiFi.SSID() + "\n");
        logMessage("[WIFI] IP     : " + WiFi.localIP().toString() + "\n");
        connectCb();
        stopSoftAP();
        return true;
        break;
      case WL_CONNECT_FAILED:
        logMessage("[WIFI] Connecting failed (4): Unknown reason\n");
        break;
      case WL_CONNECTION_LOST:
        logMessage("[WIFI] Connecting failed (5): Connection lost\n");
        break;
      case WL_DISCONNECTED:
        logMessage("[WIFI] Connecting failed (6): Disconnected\n");
        break;
      case WL_NO_SHIELD:
        logMessage("[WIFI] Connecting failed (7): No Wifi shield found\n");
        break;
      default:
        logMessage("[WIFI] Connecting failed (" + String(status) + "): Unknown status code\n");
        break;
    }
  }
  return false;
}

void WIFIMANAGER::configueSoftAp(String apName, String apPass) {
  this->softApName = apName;
  this->softApPass = apPass;
}



/**
 * @brief Start a SoftAP for direct client access
 * @param apName name of the AP to create (default is ESP_XXXXXXXX)
 * @return true on success
 * @return false o error or if a SoftAP already runs
 */
bool WIFIMANAGER::runSoftAP(String apName, String apPass) {
  if (apName.length()) this->softApName = apName;
  if (apPass.length()) this->softApPass = apPass;

  if (softApRunning) return true;
  startApTimeMillis = millis();

  if (this->softApName == "") this->softApName = "ESP_" + String((uint32_t)ESP.getEfuseMac());
  logMessage("[WIFI] Starting configuration portal on AP SSID " + this->softApName + "\n");

  WiFi.mode(WIFI_AP);
  bool state = WiFi.softAP(this->softApName.c_str(), (this->softApPass.length() ? this->softApPass.c_str() : NULL));
  if (state) {
    IPAddress IP = WiFi.softAPIP();
    logMessage("[WIFI] AP created. My IP is: " + String(IP) + "\n");
    return true;
  } else {
    logMessage("[WIFI] Unable to create SoftAP!\n");
    return false;
  }
}

/**
 * @brief Stop/Disconnect a current running SoftAP
 */
void WIFIMANAGER::stopSoftAP() {
  WiFi.softAPdisconnect();
  WiFi.mode(WIFI_STA);
}

/**
 * @brief Stop/Disconnect a current wifi connection
 */
void WIFIMANAGER::stopClient() {
  WiFi.disconnect();
}

/**
 * @brief Stop/Disconnect all running wifi activies and optionally kill the background task as well
 * @param killTask true to kill the background task to prevent reconnects
 */
void WIFIMANAGER::stopWifi(bool killTask) {
  if (killTask) vTaskDelete(WifiCheckTask);
  stopSoftAP();
  stopClient();
}

/**
 * @brief Attach the WebServer to the WifiManager to register the RESTful API
 * @param srv WebServer object
 */
#if ASYNC_WEBSERVER == true
void WIFIMANAGER::attachWebServer(AsyncWebServer * srv) {
#else
void WIFIMANAGER::attachWebServer(WebServer * srv) {
#endif
  webServer = srv; // store it in the class for later use

#if ASYNC_WEBSERVER == true
  // not required
#else
  // just for debugging
  webServer->onNotFound([&]() {
    String uri = WebServer::urlDecode(webServer->uri());  // required to read paths with blanks

     // Dump debug data
    String message;
    message.reserve(100);
    message = F("Error: File not found\n\nURI: ");
    message += uri;
    message += F("\nMethod: ");
    message += (webServer->method() == HTTP_GET) ? "GET" : "POST";
    message += F("\nArguments: ");
    message += webServer->args();
    message += '\n';
    for (uint8_t i = 0; i < webServer->args(); i++) {
      message += F(" NAME:");
      message += webServer->argName(i);
      message += F("\n VALUE:");
      message += webServer->arg(i);
      message += '\n';
    }
    message += "path=";
    message += webServer->arg("path");
    message += '\n';
    logMessage(message);
  });
#endif

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/softap/start").c_str(), HTTP_POST, [&](AsyncWebServerRequest * request) {
    request->send(200, "application/json", "{\"message\":\"Soft AP stopped\"}");
#else
  webServer->on((apiPrefix + "/softap/start").c_str(), HTTP_POST, [&]() {
    webServer->send(200, "application/json", "{\"message\":\"Soft AP stopped\"}");
#endif
    yield();
    vTaskDelay(250);
    runSoftAP();
  });
  
#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/softap/stop").c_str(), HTTP_POST, [&](AsyncWebServerRequest * request) {
    request->send(200, "application/json", "{\"message\":\"Soft AP stopped\"}");
#else
  webServer->on((apiPrefix + "/softap/stop").c_str(), HTTP_POST, [&]() {
    webServer->send(200, "application/json", "{\"message\":\"Soft AP stopped\"}");
#endif
    yield();
    vTaskDelay(250); // It's likely that this message won't go trough, but we give it a short time
    stopSoftAP();
  });

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/client/stop").c_str(), HTTP_POST, [&](AsyncWebServerRequest * request) {
    request->send(200, "application/json", "{\"message\":\"Terminating current Wifi connection\"}");
#else
  webServer->on((apiPrefix + "/client/stop").c_str(), HTTP_POST, [&]() {
    webServer->send(200, "application/json", "{\"message\":\"Terminating current Wifi connection\"}");
#endif
    yield();
    vTaskDelay(500); // It's likely that this message won't go trough, but we give it a short time
    stopClient();
  });

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/add").c_str(), HTTP_POST, [&](AsyncWebServerRequest * request){}, NULL,
    [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    JsonDocument jsonBuffer;
    deserializeJson(jsonBuffer, (const char*)data);
    auto resp = request;
#else
  webServer->on((apiPrefix + "/add").c_str(), HTTP_POST, [&]() {
    if (webServer->args() != 1) {
      webServer->send(400, "application/json", "{\"message\":\"Bad Request. Only accepting one json body in request!\"}");
    }
    JsonDocument jsonBuffer;
    deserializeJson(jsonBuffer, webServer->arg(0));
    auto resp = webServer;
#endif
    if (!jsonBuffer["apName"].is<String>() || !jsonBuffer["apPass"].is<String>()) {
      resp->send(422, "application/json", "{\"message\":\"Invalid data\"}");
      return;
    }
    if (!addWifi(jsonBuffer["apName"].as<String>(), jsonBuffer["apPass"].as<String>())) {
      resp->send(500, "application/json", "{\"message\":\"Unable to process data\"}");
    } else {resp->send(200, "application/json", "{\"message\":\"New network added\"");}
  });

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/id").c_str(), HTTP_DELETE, [&](AsyncWebServerRequest * request){}, NULL,
    [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    JsonDocument jsonBuffer;
    deserializeJson(jsonBuffer, (const char*)data);
    auto resp = request;
#else
  webServer->on((apiPrefix + "/id").c_str(), HTTP_DELETE, [&]() {
    if (webServer->args() != 1) {
      webServer->send(400, "application/json", "{\"message\":\"Bad Request. Only accepting one json body in request!\"}");
    }
    JsonDocument jsonBuffer;
    deserializeJson(jsonBuffer, webServer->arg(0));
    auto resp = webServer;
#endif
    int idValue = jsonBuffer["id"].as<String>().toInt();
    if (idValue < 0 || idValue >= WIFIMANAGER_MAX_APS) {
      resp->send(422, "application/json", "{\"message\":\"Invalid data\"}");
      return;
    }
    if (!delWifi(jsonBuffer["id"].as<uint8_t>())) {
      resp->send(500, "application/json", "{\"message\":\"Unable to delete entry\"}");
    } else resp->send(200, "application/json", "{\"message\":\"AP deleted\"}");
  });

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/apName").c_str(), HTTP_DELETE, [&](AsyncWebServerRequest * request){}, NULL,
    [&](AsyncWebServerRequest * request, uint8_t *data, size_t len, size_t index, size_t total) {
    JsonDocument jsonBuffer;
    deserializeJson(jsonBuffer, (const char*)data);
    auto resp = request;
#else
  webServer->on((apiPrefix + "/apName").c_str(), HTTP_DELETE, [&]() {
    if (webServer->args() != 1) {
      webServer->send(400, "application/json", "{\"message\":\"Bad Request. Only accepting one json body in request!\"}");
    }
    JsonDocument jsonBuffer;
    deserializeJson(jsonBuffer, webServer->arg(0));
    auto resp = webServer;
#endif
    if (!jsonBuffer["apName"].is<String>()) {
      resp->send(422, "application/json", "{\"message\":\"Invalid data\"}");
      return;
    }
    if (!delWifi(jsonBuffer["apName"].as<String>())) {
      resp->send(500, "application/json", "{\"message\":\"Unable to delete entry\"}");
    } else resp->send(200, "application/json", "{\"message\":\"AP deleted\"}");
  });

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/configlist").c_str(), HTTP_GET, [&](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
#else
  webServer->on((apiPrefix + "/configlist").c_str(), HTTP_GET, [&]() {
    String buffer;
#endif
    JsonDocument jsonDoc;
    auto jsonArray = jsonDoc.to<JsonArray>();
    for(uint8_t i=0; i<WIFIMANAGER_MAX_APS; i++) {
      if (apList[i].apName.length() > 0) {
        JsonObject wifiNet = jsonArray.createNestedObject();
        wifiNet["id"] = i;
        wifiNet["apName"] = apList[i].apName;
        wifiNet["apPass"] = apList[i].apPass.length() > 0 ? true : false;
      }
    }
#if ASYNC_WEBSERVER == true
    serializeJson(jsonArray, *response);
    response->setCode(200);
    response->setContentLength(measureJson(jsonDoc));
    request->send(response);
#else
    // Improve me: not that efficient without the stream response
    serializeJson(jsonArray, buffer);
    webServer->send(200, "application/json", (buffer.equals("null") ? "{}" : buffer));
#endif
  });

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/scan").c_str(), HTTP_GET, [&](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
#else
  webServer->on((apiPrefix + "/scan").c_str(), HTTP_GET, [&]() {
    String buffer;
#endif
    JsonDocument jsonDoc;

    int scanResult;
    String ssid;
    uint8_t encryptionType;
    int32_t rssi;
    uint8_t* bssid;
    int32_t channel;

    scanResult = WiFi.scanComplete();
    if (scanResult == WIFI_SCAN_FAILED) {
      scanResult = WiFi.scanNetworks(true, true);   // FIXME: scanNetworks is disconnecting clients!
      jsonDoc["status"] = "scanning";
    } else if (scanResult > 0) {
      for (int8_t i = 0; i < scanResult; i++) {
        WiFi.getNetworkInfo(i, ssid, encryptionType, rssi, bssid, channel);

        JsonObject wifiNet = jsonDoc.createNestedObject();
        wifiNet["ssid"] = ssid;
        wifiNet["encryptionType"] = encryptionType;
        wifiNet["rssi"] = rssi;
        wifiNet["channel"] = channel;
        yield();
      }
      WiFi.scanDelete();
    }
#if ASYNC_WEBSERVER == true
    serializeJson(jsonDoc, *response);
    response->setCode(200);
    response->setContentLength(measureJson(jsonDoc));
    request->send(response);
#else
    // Improve me: not that efficient without the stream response
    serializeJson(jsonDoc, buffer);
    webServer->send(200, "application/json", buffer);
#endif
  });

#if ASYNC_WEBSERVER == true
  webServer->on((apiPrefix + "/status").c_str(), HTTP_GET, [&](AsyncWebServerRequest *request) {
    AsyncResponseStream *response = request->beginResponseStream("application/json");
#else
  webServer->on((apiPrefix + "/status").c_str(), HTTP_GET, [&]() {
    String buffer;
#endif
    JsonDocument jsonDoc;

    jsonDoc["ssid"] = WiFi.SSID();
    jsonDoc["signalStrengh"] = WiFi.RSSI();

    jsonDoc["ip"] = WiFi.localIP().toString();
    jsonDoc["gw"] = WiFi.gatewayIP().toString();
    jsonDoc["nm"] = WiFi.subnetMask().toString();

    jsonDoc["hostname"] = WiFi.getHostname();

    jsonDoc["chipModel"] = ESP.getChipModel();
    jsonDoc["chipRevision"] = ESP.getChipRevision();
    jsonDoc["chipCores"] = ESP.getChipCores();

    jsonDoc["getHeapSize"] = ESP.getHeapSize();
    jsonDoc["freeHeap"] = ESP.getFreeHeap();

#if ASYNC_WEBSERVER == true
    serializeJson(jsonDoc, *response);
    response->setCode(200);
    response->setContentLength(measureJson(jsonDoc));
    request->send(response);
#else
    // Improve me: not that efficient without the stream response
    serializeJson(jsonDoc, buffer);
    webServer->send(200, "application/json", buffer);
#endif
  });
}

/**
 * @brief Attach the WebServer to the WifiManager to register the RESTful API
 * @param srv WebServer object
 */
void WIFIMANAGER::attachUI() {

#if ASYNC_WEBSERVER == true
  webServer->on((uiPrefix).c_str(), HTTP_GET, [](AsyncWebServerRequest* request) {
#else
  webServer->on((uiPrefix).c_str(), HTTP_GET, [&]() {
#endif

    String html = R"html(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>AISHA Device Setup</title>
    <style>
        :root {
            --primary-color: #2563eb;
            --primary-hover: #1d4ed8;
            --secondary-color: #64748b;
            --bg-color: #f8fafc;
            --card-bg: #ffffff;
            --text-color: #1e293b;
            --text-secondary: #64748b;
            --border-color: #e2e8f0;
            --success-color: #059669;
            --success-bg: #ecfdf5;
            --error-color: #dc2626;
            --error-bg: #fef2f2;
            --warning-color: #d97706;
            --warning-bg: #fffbeb;
            --shadow-sm: 0 1px 2px 0 rgba(0, 0, 0, 0.05);
            --shadow-md: 0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06);
            --shadow-lg: 0 10px 15px -3px rgba(0, 0, 0, 0.1), 0 4px 6px -2px rgba(0, 0, 0, 0.05);
        }

        * {
            box-sizing: border-box;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: var(--text-color);
            margin: 0;
            padding: 20px;
            line-height: 1.6;
            min-height: 100vh;
        }

        .container {
            max-width: 640px;
            margin: 0 auto;
        }

        .header {
            text-align: center;
            margin-bottom: 32px;
        }

        .logo {
            background: var(--card-bg);
            border-radius: 16px;
            padding: 24px;
            margin-bottom: 24px;
            box-shadow: var(--shadow-lg);
            border: 1px solid var(--border-color);
        }

        .logo h1 {
            margin: 0;
            font-size: 2.5rem;
            font-weight: 700;
            background: linear-gradient(135deg, var(--primary-color), #7c3aed);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            background-clip: text;
        }

        .logo p {
            margin: 8px 0 0 0;
            color: var(--text-secondary);
            font-size: 1.1rem;
        }

        .card {
            background: var(--card-bg);
            border-radius: 16px;
            padding: 24px;
            margin-bottom: 24px;
            box-shadow: var(--shadow-lg);
            border: 1px solid var(--border-color);
            backdrop-filter: blur(10px);
        }

        .card h2 {
            margin: 0 0 20px 0;
            color: var(--text-color);
            font-size: 1.5rem;
            font-weight: 600;
            display: flex;
            align-items: center;
            gap: 8px;
        }

        .action-buttons {
            display: flex;
            gap: 12px;
            flex-wrap: wrap;
            margin-bottom: 20px;
        }

        .btn {
            background: var(--primary-color);
            color: white;
            border: none;
            padding: 12px 24px;
            border-radius: 8px;
            cursor: pointer;
            font-size: 0.95rem;
            font-weight: 500;
            transition: all 0.2s ease;
            display: inline-flex;
            align-items: center;
            gap: 8px;
            text-decoration: none;
            min-height: 44px;
        }

        .btn:hover {
            background: var(--primary-hover);
            transform: translateY(-1px);
            box-shadow: var(--shadow-md);
        }

        .btn:active {
            transform: translateY(0);
        }

        .btn-secondary {
            background: var(--bg-color);
            color: var(--text-color);
            border: 1px solid var(--border-color);
        }

        .btn-secondary:hover {
            background: #e2e8f0;
        }

        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
            transform: none;
        }

        .network-list {
            list-style: none;
            padding: 0;
            margin: 0;
        }

        .network-item {
            display: flex;
            align-items: center;
            padding: 16px;
            border-bottom: 1px solid var(--border-color);
            cursor: pointer;
            transition: all 0.2s ease;
            border-radius: 8px;
            margin-bottom: 8px;
        }

        .network-item:last-child {
            border-bottom: none;
            margin-bottom: 0;
        }

        .network-item:hover {
            background: var(--bg-color);
            transform: translateX(4px);
        }

        .network-info {
            flex-grow: 1;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }

        .network-details {
            flex-grow: 1;
        }

        .ssid {
            font-weight: 600;
            margin-bottom: 4px;
            color: var(--text-color);
            font-size: 1rem;
        }

        .signal {
            font-size: 0.875rem;
            color: var(--text-secondary);
            display: flex;
            align-items: center;
            gap: 4px;
        }

        .btn-small {
            padding: 8px 16px;
            font-size: 0.875rem;
            min-height: 36px;
        }

        .btn-danger {
            background: var(--error-color);
            color: white;
        }

        .btn-danger:hover {
            background: #b91c1c;
        }

        .status {
            padding: 16px;
            border-radius: 12px;
            margin-bottom: 20px;
            display: none;
            font-weight: 500;
            border-left: 4px solid;
        }

        .status.error {
            background: var(--error-bg);
            color: var(--error-color);
            border-left-color: var(--error-color);
            display: block;
        }

        .status.success {
            background: var(--success-bg);
            color: var(--success-color);
            border-left-color: var(--success-color);
            display: block;
        }

        .status.info {
            background: var(--warning-bg);
            color: var(--warning-color);
            border-left-color: var(--warning-color);
            display: block;
        }

        .modal {
            display: none;
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.6);
            align-items: center;
            justify-content: center;
            z-index: 1000;
            backdrop-filter: blur(4px);
        }

        .modal-content {
            background: var(--card-bg);
            padding: 32px;
            border-radius: 16px;
            width: 90%;
            max-width: 480px;
            box-shadow: var(--shadow-lg);
            border: 1px solid var(--border-color);
        }

        .modal h2 {
            margin: 0 0 24px 0;
            color: var(--text-color);
            font-size: 1.5rem;
            font-weight: 600;
        }

        .form-group {
            margin-bottom: 20px;
        }

        .form-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: 500;
            color: var(--text-color);
        }

        input {
            width: 100%;
            padding: 12px 16px;
            border: 2px solid var(--border-color);
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.2s ease;
            background: var(--card-bg);
        }

        input:focus {
            outline: none;
            border-color: var(--primary-color);
            box-shadow: 0 0 0 3px rgba(37, 99, 235, 0.1);
        }

        .button-group {
            display: flex;
            gap: 12px;
            justify-content: flex-end;
            margin-top: 24px;
        }

        .empty-state {
            text-align: center;
            padding: 40px 20px;
            color: var(--text-secondary);
        }

        .empty-state-icon {
            font-size: 3rem;
            margin-bottom: 16px;
            opacity: 0.5;
        }

        .loading {
            display: inline-block;
            width: 20px;
            height: 20px;
            border: 2px solid var(--border-color);
            border-radius: 50%;
            border-top-color: var(--primary-color);
            animation: spin 1s ease-in-out infinite;
        }

        @keyframes spin {
            to { transform: rotate(360deg); }
        }

        .signal-strength {
            display: inline-flex;
            align-items: center;
            gap: 4px;
        }

        .signal-bars {
            display: flex;
            gap: 1px;
            align-items: end;
        }

        .signal-bar {
            width: 3px;
            background: var(--text-secondary);
            border-radius: 1px;
        }

        .signal-bar.active {
            background: var(--success-color);
        }

        .signal-bar:nth-child(1) { height: 4px; }
        .signal-bar:nth-child(2) { height: 8px; }
        .signal-bar:nth-child(3) { height: 12px; }
        .signal-bar:nth-child(4) { height: 16px; }

        @media (max-width: 640px) {
            body {
                padding: 16px;
            }

            .card {
                padding: 20px;
                margin-bottom: 20px;
            }

            .action-buttons {
                flex-direction: column;
            }

            .btn {
                width: 100%;
                justify-content: center;
            }

            .network-info {
                flex-direction: column;
                align-items: flex-start;
                gap: 12px;
            }

            .modal-content {
                padding: 24px;
                margin: 20px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <div class="logo">
                <h1>🤖 AISHA</h1>
                <p>Thiết bị thông minh của bạn</p>
            </div>
        </div>

        <div class="card">
            <div id="status"></div>
            <div class="action-buttons">
                <button class="btn" onclick="scanNetworks()">
                    🔍 Quét mạng WiFi
                </button>
                <button class="btn btn-secondary" onclick="showConnectModal()">
                    ➕ Kết nối thủ công
                </button>
            </div>
        </div>

        <div class="card">
            <h2>💾 Mạng đã lưu</h2>
            <div id="savedNetworks" class="network-list"></div>
        </div>

        <div class="card">
            <h2>📶 Mạng khả dụng</h2>
            <div id="networkList" class="network-list"></div>
        </div>
    </div>

    <div id="connectModal" class="modal">
        <div class="modal-content">
            <h2>🔗 Kết nối mạng WiFi</h2>
            <form id="connectForm" onsubmit="connectToNetwork(event)">
                <div class="form-group">
                    <label for="apName">Tên mạng (SSID):</label>
                    <input type="text" id="apName" required placeholder="Nhập tên mạng WiFi">
                </div>

                <div class="form-group">
                    <label for="apPass">Mật khẩu:</label>
                    <input type="password" id="apPass" required placeholder="Nhập mật khẩu WiFi">
                </div>

                <div class="button-group">
                    <button type="button" class="btn btn-secondary" onclick="closeModal()">Hủy</button>
                    <button type="submit" class="btn">Kết nối</button>
                </div>
            </form>
        </div>
    </div>

    <script>
        const API_BASE = '/api';
        let networks = {};

        // Load saved networks when page loads
        window.addEventListener('load', () => {
            loadSavedNetworks();
            scanNetworks();
        });

        async function loadSavedNetworks() {
            try {
                const response = await fetch(`${API_BASE}/wifi/configlist`);
                if (!response.ok) throw new Error('Không thể tải danh sách mạng đã lưu');

                const savedNetworks = await response.json();
                displaySavedNetworks(savedNetworks);
            } catch (error) {
                showStatus('❌ Lỗi tải mạng đã lưu: ' + error.message, 'error');
            }
        }

        function displaySavedNetworks(networks) {
            const networkList = document.getElementById('savedNetworks');
            const networkArray = Object.values(networks);

            if (networkArray.length === 0) {
                networkList.innerHTML = `
                    <div class="empty-state">
                        <div class="empty-state-icon">📱</div>
                        <p>Chưa có mạng WiFi nào được lưu</p>
                    </div>
                `;
                return;
            }

            networkList.innerHTML = networkArray.map(network => `
                <div class="network-item">
                    <div class="network-info">
                        <div class="network-details">
                            <div class="ssid">🔗 ${network.apName}</div>
                            <div class="signal">Đã lưu • ${network.apPass ? 'Có mật khẩu' : 'Không có mật khẩu'}</div>
                        </div>
                        <button class="btn btn-small btn-danger" onclick="deleteNetwork('${network.id}')">
                            🗑️ Xóa
                        </button>
                    </div>
                </div>
            `).join('');
        }

        async function scanNetworks() {
            const scanBtn = document.querySelector('button[onclick="scanNetworks()"]');
            const originalText = scanBtn.innerHTML;

            try {
                scanBtn.innerHTML = '<span class="loading"></span> Đang quét...';
                scanBtn.disabled = true;
                showStatus('🔍 Đang quét mạng WiFi xung quanh...', 'info');

                const response = await fetch(`${API_BASE}/wifi/scan`);
                if (!response.ok) throw new Error('Quét mạng thất bại');

                // Wait 5 seconds for the scan to complete
                await new Promise(resolve => setTimeout(resolve, 5000));

                const scanResponse = await fetch(`${API_BASE}/wifi/scan`);
                if (!scanResponse.ok) throw new Error('Không thể lấy kết quả quét');

                networks = await scanResponse.json();
                displayNetworks(networks);
                showStatus('✅ Đã tìm thấy ' + Object.keys(networks).length + ' mạng WiFi', 'success');
            } catch (error) {
                showStatus('❌ Lỗi: ' + error.message, 'error');
            } finally {
                scanBtn.innerHTML = originalText;
                scanBtn.disabled = false;
            }
        }

        function displayNetworks(networks) {
            const networkList = document.getElementById('networkList');
            const networkArray = Object.values(networks || {})
              .filter(network => network && network.ssid && network.ssid.length > 0);

            if (networkArray.length === 0) {
                networkList.innerHTML = `
                    <div class="empty-state">
                        <div class="empty-state-icon">📡</div>
                        <p>Không tìm thấy mạng WiFi nào</p>
                        <button class="btn btn-secondary" onclick="scanNetworks()">Quét lại</button>
                    </div>
                `;
                return;
            }

            // Sort networks by RSSI
            networkArray.sort((a, b) => b.rssi - a.rssi);

            networkList.innerHTML = networkArray
                .map(network => `
                    <div class="network-item" onclick="showConnectModal('${network.ssid}')">
                        <div class="network-info">
                            <div class="network-details">
                                <div class="ssid">
                                    ${network.encryptionType > 0 ? '🔒' : '🌐'} ${network.ssid}
                                </div>
                                <div class="signal">
                                    <span class="signal-strength">
                                        ${getSignalBars(network.rssi)}
                                        ${getSignalStrength(network.rssi)}
                                    </span>
                                    • ${network.encryptionType > 0 ? 'Bảo mật' : 'Mở'}
                                </div>
                            </div>
                        </div>
                    </div>
                `).join('');
        }

        function getSignalStrength(rssi) {
            if (rssi >= -50) return 'Tuyệt vời';
            if (rssi >= -60) return 'Rất tốt';
            if (rssi >= -70) return 'Tốt';
            if (rssi >= -80) return 'Khá';
            return 'Yếu';
        }

        function getSignalBars(rssi) {
            let bars = 0;
            if (rssi >= -50) bars = 4;
            else if (rssi >= -60) bars = 3;
            else if (rssi >= -70) bars = 2;
            else if (rssi >= -80) bars = 1;

            let barsHtml = '<span class="signal-bars">';
            for (let i = 1; i <= 4; i++) {
                barsHtml += `<span class="signal-bar ${i <= bars ? 'active' : ''}"></span>`;
            }
            barsHtml += '</span>';
            return barsHtml;
        }

        function showConnectModal(apName = '') {
            document.getElementById('apName').value = apName;
            document.getElementById('apName').readOnly = !!apName;
            document.getElementById('apPass').value = '';
            document.getElementById('connectModal').style.display = 'flex';
        }

        function closeModal() {
            document.getElementById('connectModal').style.display = 'none';
        }

        async function deleteNetwork(deleteId) {
            if (!confirm('Bạn có chắc chắn muốn xóa mạng này?')) {
                return;
            }

            try {
                showStatus('🗑️ Đang xóa mạng...', 'info');
                const response = await fetch(`${API_BASE}/wifi/id`, {
                    method: 'DELETE',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ id: deleteId }),
                });

                if (!response.ok) throw new Error('Không thể xóa mạng');

                showStatus('✅ Đã xóa mạng thành công', 'success');
                await loadSavedNetworks(); // Refresh the list
            } catch (error) {
                showStatus('❌ Lỗi khi xóa mạng: ' + error.message, 'error');
            }
        }

        async function connectToNetwork(event) {
            event.preventDefault();
            const apName = document.getElementById('apName').value;
            const apPass = document.getElementById('apPass').value;
            const submitBtn = event.target.querySelector('button[type="submit"]');
            const originalText = submitBtn.innerHTML;

            try {
                submitBtn.innerHTML = '<span class="loading"></span> Đang kết nối...';
                submitBtn.disabled = true;
                showStatus('🔗 Đang kết nối tới mạng ' + apName + '...', 'info');

                const response = await fetch(`${API_BASE}/wifi/add`, {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({ apName, apPass }),
                });

                if (!response.ok) throw new Error('Kết nối thất bại');

                closeModal();
                showStatus('🎉 Kết nối thành công tới mạng ' + apName + '!', 'success');

                // Refresh saved networks list
                await loadSavedNetworks();
            } catch (error) {
                showStatus('❌ Lỗi kết nối: ' + error.message, 'error');
            } finally {
                submitBtn.innerHTML = originalText;
                submitBtn.disabled = false;
            }
        }

        function showStatus(message, type) {
            const statusElement = document.getElementById('status');
            statusElement.innerHTML = message;
            statusElement.className = `status ${type}`;
        }
    </script>
</body>
</html>
)html";

#if ASYNC_WEBSERVER == true
    request->send(200, "text/html", html);
#else
    webServer->send(200, "text/html", html);
#endif

  });
}
