#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <driver/i2s.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include "FS.h"
#include "SPIFFS.h"

extern Preferences preferences;
extern bool factory_reset_status;

enum DeviceState
{
    SETUP,
    IDLE,
    LISTENING,
    SPEAKING,
    TAKING_PHOTO,
    PROCESSING,
    WAITING,
    QUOTA_EXCEEDED,
    FACTORY_RESET
};

extern DeviceState deviceState;

// Atomic device state management
extern portMUX_TYPE stateMux;
void setDeviceState(DeviceState newState);
DeviceState getDeviceState();

// OTA-safe heap management
#define HIGH_WATERLINE (50 * 1024)  // 50KB minimum free heap
bool checkHeapSafety();
File safeOpenSPIFFS(const char* path, const char* mode);
void* safeMalloc(size_t size);

// WiFi credentials
extern const char *EAP_IDENTITY;
extern const char *EAP_USERNAME;
extern const char *EAP_PASSWORD;
extern const char *ssid;

extern const char *ssid_peronal;
extern const char *password_personal;

extern String authTokenGlobal;

// WebSocket server details
extern const char *ws_server;
extern const uint16_t ws_port;
extern const char *ws_path;

// Backend server details
extern const char *backend_server;
extern const uint16_t backend_port;

// I2S and Audio parameters
extern const uint32_t SAMPLE_RATE;

// Image Streaming Configuration
extern const size_t IMAGE_CHUNK_SIZE;

// ---------- Development ------------
#define DEV_MODE

// ─── HIGH-LEVEL SWITCHES ─────────────────────────────────────────
#define TTP_GPIO_WAKE            // we have a capacitive key, not a push-button
#define USE_XIAO_S3_SENSE     // our pin-map
#define USE_PDM_MIC           // mic is PDM, not classic I²S

// ----------------- Pin Definitions -----------------
// #define USE_NORMAL_ESP32

//extern const gpio_num_t BUTTON_PIN;

// I2S Microphone pins
extern const int I2S_SD;
extern const int I2S_WS;
extern const int I2S_SCK;
extern const i2s_port_t I2S_PORT_IN;

#ifdef USE_PDM_MIC        // only needed when the PDM mic option is active
extern const int PDM_CLK;   // GPIO 42 on XIAO-S3 Sense
extern const int PDM_DATA;  // GPIO 41 on XIAO-S3 Sense
#endif


// I2S Speaker pins
extern const int I2S_WS_OUT;
extern const int I2S_BCK_OUT;
extern const int I2S_DATA_OUT;
extern const i2s_port_t I2S_PORT_OUT;
extern const int I2S_SD_OUT;

// SSL certificate
extern const char *CA_cert;
extern const char *Vercel_CA_cert;
void factoryResetDevice();

#endif