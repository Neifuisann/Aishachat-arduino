#include "Config.h"
#include <nvs_flash.h>

// ! define preferences
Preferences preferences;
OtaStatus otaState = OTA_IDLE;
bool factory_reset_status = false;

/**
 * Configuration for Elato Firmware
 * 
 * DEVELOPMENT vs PRODUCTION SETUP:
 * --------------------------------
 * 1. Define `DEV_MODE` in your config.h file to use local development servers
 * 2. `DEV_MODE` requires updating the IP addresses to your local network IP
 * 3. Without `DEV_MODE` defined, the firmware will use your production servers
 *
 * Development Mode (find your local IP address using ifconfig):
 *   - WebSocket: Your local IP (e.g., 192.168.1.100:8000)
 *   - Backend: Your local IP (e.g., 192.168.1.100:3000)
 *   - No SSL certificates required
 *
 * Production Mode:
 *   - WebSocket: <your-websocket-server>.deno.dev (port 443)
 *   - Backend: <your-vercel-backend-server> (port 3000)
 *   - Uses pre-configured SSL certificates (set in Config.cpp)
 */

#ifdef DEV_MODE
const char *ws_server = "192.168.1.70";
const uint16_t ws_port = 8000;
const char *ws_path = "/";
// Backend server details 
const char *backend_server = "192.168.1.70";
const uint16_t backend_port = 3000;

#else
// PROD
const char *ws_server = "neichat.deno.dev";
const uint16_t ws_port = 443;
const char *ws_path = "/";
// Backend server details 
const char *backend_server = "www.elatoai.com";
const uint16_t backend_port = 3000;

#endif

String authTokenGlobal;
DeviceState deviceState = IDLE;

// ─── AUDIO SAMPLE RATE ───────────────────────────────────────────
const uint32_t SAMPLE_RATE = 16000;    

// ----------------- Pin Definitions -----------------
#ifdef USE_NORMAL_ESP32

const i2s_port_t I2S_PORT_IN = I2S_NUM_1;
const i2s_port_t I2S_PORT_OUT = I2S_NUM_0;

const int I2S_SD = 41; 
const int I2S_SCK = 42;

const int I2S_WS_OUT = A1;
const int I2S_BCK_OUT = A2;
const int I2S_DATA_OUT = A3;
const int I2S_SD_OUT = D10;

//const gpio_num_t BUTTON_PIN = GPIO_NUM_2; // Only RTC IO are allowed - ESP32 Pin example

#endif

#ifdef USE_XIAO_S3_SENSE

// Speaker (MAX98357)
const int I2S_WS_OUT   = 2;   // silk-screen A1
const int I2S_BCK_OUT  = 3;   // A2
const int I2S_DATA_OUT = 4;   // A3
const i2s_port_t I2S_PORT_OUT = I2S_NUM_1;

// Microphone (ICS-43434 – PDM)
const int PDM_CLK = 42;
const int PDM_DATA = 41;
const i2s_port_t I2S_PORT_IN  = I2S_NUM_0;
#endif

const char *Vercel_CA_cert = R"EOF(
-----BEGIN CERTIFICATE-----
<YOUR VERCEL CERTIFICATE HERE>
-----END CERTIFICATE-----
)EOF";

// Supabase Edge Functions CA cert
// const char *CA_cert = R"EOF(
// -----BEGIN CERTIFICATE-----
// <YOUR HOST CERTIFICATE HERE>
// -----END CERTIFICATE-----
// )EOF";

// Deno Edge Functions CA cert
const char *CA_cert = R"EOF(
-----BEGIN CERTIFICATE-----
<YOUR TALKEDGE CERTIFICATE HERE>
-----END CERTIFICATE-----
)EOF";
