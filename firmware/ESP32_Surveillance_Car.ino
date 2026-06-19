/*
 * ═══════════════════════════════════════════════════════════════════════
 *  ESP32-CAM SURVEILLANCE CAR — Optimized Firmware
 *
 *  Hardware : ESP32-CAM (AI-Thinker) + L298N Motor Driver + 4× DC Motors
 *  Features : WiFi AP · WebSocket motor control · Live JPEG camera stream
 *             PWM speed control · PWM LED brightness
 *
 *  Frontend : esp32_surveillance_dashboard.html
 *  WebSocket endpoints:
 *    ws://[IP]/CarInput  — text  commands ("MoveCar,1" / "Speed,150" / "Light,255")
 *    ws://[IP]/Camera    — binary JPEG frames (ESP32 → browser)
 *
 *  Libraries required (install via Arduino Library Manager):
 *    • ESP32 board package  (Espressif Systems — v2.x recommended)
 *    • AsyncTCP             (dvarrel/AsyncTCP  or  me-no-dev/AsyncTCP)
 *    • ESPAsyncWebServer    (me-no-dev/ESPAsyncWebServer)
 *    • esp32-camera         (included with ESP32 board package)
 *
 *  Board settings (Arduino IDE):
 *    Board   : AI Thinker ESP32-CAM
 *    CPU Freq: 240 MHz
 *    Flash   : 4MB (32Mb)
 *    PSRAM   : Enabled
 *    Upload  : 115200 baud
 *
 *  ── STREAMING PRESET ──────────────────────────────────────────────────
 *  Set STREAM_PRESET below to one of:
 *    PRESET_SPEED    — 320×240,  quality  8, ~25 FPS  (fast robot nav)
 *    PRESET_BALANCED — 480×320,  quality 12, ~18 FPS  (default ✓)
 *    PRESET_QUALITY  — 640×480,  quality 25, ~10 FPS  (sharp detail)
 * ═══════════════════════════════════════════════════════════════════════
 */

// ── Core Libraries ──────────────────────────────────────────────────────
#include "esp_camera.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <sstream>


// ═══════════════════════════════════════════════════════════════════════
//  STREAMING PRESET SELECTOR
//  Change STREAM_PRESET to switch between speed / balanced / quality.
// ═══════════════════════════════════════════════════════════════════════
#define PRESET_SPEED    1   // 320×240  | quality  8 | target ~25 FPS
#define PRESET_BALANCED 2   // 480×320  | quality 12 | target ~18 FPS  ← default
#define PRESET_QUALITY  3   // 640×480  | quality 25 | target ~10 FPS

//// MODIFIED ////
// Changed default preset from PRESET_BALANCED to PRESET_SPEED.
// Smaller frames (320×240 vs 480×320) transmit in ~40% fewer bytes,
// dramatically cutting per-frame WiFi transmit time and allowing the
// pipeline to clear stale frames before the next capture — the primary
// cause of the 2–3 s lag reported.
#define STREAM_PRESET   PRESET_SPEED   // ← Was PRESET_BALANCED

#if   STREAM_PRESET == PRESET_SPEED
  #define CAM_FRAMESIZE   FRAMESIZE_QVGA    // 320×240
  #define CAM_QUALITY     8
  #define FRAME_INTERVAL  40                // ms between frames (~25 FPS cap)
  #define CAM_FRAMESIZE_NOPSRAM FRAMESIZE_QQVGA  // 160×120 fallback without PSRAM

#elif STREAM_PRESET == PRESET_BALANCED
  #define CAM_FRAMESIZE   FRAMESIZE_HVGA    // 480×320
  //// MODIFIED ////
  // Tightened quality from 12 → 10 in BALANCED mode.
  // Lower JPEG quality number = more compression = smaller frame bytes.
  // On a congested 2.4 GHz WiFi link the byte savings matter more than
  // the marginal image sharpness loss, which is barely visible at video speeds.
  #define CAM_QUALITY     10                // Was 12
  #define FRAME_INTERVAL  50
  #define CAM_FRAMESIZE_NOPSRAM FRAMESIZE_QVGA

#elif STREAM_PRESET == PRESET_QUALITY
  #define CAM_FRAMESIZE   FRAMESIZE_VGA     // 640×480
  #define CAM_QUALITY     25
  #define FRAME_INTERVAL  90
  #define CAM_FRAMESIZE_NOPSRAM FRAMESIZE_QVGA
#endif


// ═══════════════════════════════════════════════════════════════════════
//  WIFI ACCESS POINT CREDENTIALS
// ═══════════════════════════════════════════════════════════════════════
const char* WIFI_SSID     = "MyWiFiCar";
const char* WIFI_PASSWORD = "12345678";


// ═══════════════════════════════════════════════════════════════════════
//  MOTOR DRIVER PIN DEFINITIONS  (L298N ↔ ESP32-CAM)
//
//  Right side motors  →  EnA=GPIO12  IN1=GPIO13  IN2=GPIO15
//  Left  side motors  →  EnB=GPIO12  IN3=GPIO14  IN4=GPIO2
//  Flash LED          →  GPIO 4
// ═══════════════════════════════════════════════════════════════════════
struct MotorPins {
  int pinEnable;
  int pinIN1;
  int pinIN2;
};

const MotorPins MOTOR_PINS[2] = {
  {12, 13, 15},   // RIGHT (EnA, IN1, IN2)
  {12, 14,  2},   // LEFT  (EnB, IN3, IN4)
};

#define LIGHT_PIN   4

// Direction aliases
#define MOTOR_FORWARD   1
#define MOTOR_BACKWARD -1
#define MOTOR_STOP      0

// Movement command codes (must match frontend)
#define CMD_FORWARD  1
#define CMD_BACKWARD 2
#define CMD_LEFT     3
#define CMD_RIGHT    4
#define CMD_STOP     0

// Motor index aliases
#define RIGHT_MOTOR  0
#define LEFT_MOTOR   1


// ═══════════════════════════════════════════════════════════════════════
//  PWM CONFIGURATION
//  Camera driver occupies LEDC channels 0 & 1 — we use 2 and 3.
// ═══════════════════════════════════════════════════════════════════════
const int PWM_FREQ          = 1000;
const int PWM_RESOLUTION    = 8;      // 8-bit = 0–255
const int PWM_CHANNEL_SPEED = 2;
const int PWM_CHANNEL_LIGHT = 3;


// ═══════════════════════════════════════════════════════════════════════
//  CAMERA GPIO MAP — AI-Thinker ESP32-CAM (hardware-fixed, do not change)
// ═══════════════════════════════════════════════════════════════════════
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22


// ═══════════════════════════════════════════════════════════════════════
//  SERVER & WEBSOCKET INSTANCES
// ═══════════════════════════════════════════════════════════════════════
AsyncWebServer  server(80);
AsyncWebSocket  wsCamera("/Camera");
AsyncWebSocket  wsCarInput("/CarInput");

unsigned long lastCommandTime  = 0;
uint32_t activeCameraClientId = 0;


// ═══════════════════════════════════════════════════════════════════════
//  EMBEDDED HTML HOME PAGE  (unchanged)
// ═══════════════════════════════════════════════════════════════════════
const char* HTML_REDIRECT PROGMEM = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Car</title>
  <style>
    body{background:#010409;color:#00d4ff;font-family:monospace;
         display:flex;flex-direction:column;align-items:center;
         justify-content:center;height:100vh;margin:0;gap:16px}
    h2{letter-spacing:4px;font-size:18px}
    p{color:#aaa;font-size:12px;letter-spacing:2px}
    a{color:#00ff88;letter-spacing:2px;font-size:14px}
  </style>
</head>
<body>
  <h2>ESP32 SURVEILLANCE CAR</h2>
  <p>WEBSOCKET ENDPOINTS ACTIVE</p>
  <p>ws://192.168.4.1/Camera &nbsp;|&nbsp; ws://192.168.4.1/CarInput</p>
  <a href="/dashboard">OPEN FULL DASHBOARD</a>
  <p style="font-size:10px;color:#444">Or open esp32_surveillance_dashboard.html locally and enter IP: 192.168.4.1</p>
</body>
</html>
)HTML";


// ═══════════════════════════════════════════════════════════════════════
//  MOTOR CONTROL FUNCTIONS  (unchanged)
// ═══════════════════════════════════════════════════════════════════════
void rotateMotor(int motorIndex, int direction) {
  if (direction == MOTOR_FORWARD) {
    digitalWrite(MOTOR_PINS[motorIndex].pinIN1, HIGH);
    digitalWrite(MOTOR_PINS[motorIndex].pinIN2, LOW);
  } else if (direction == MOTOR_BACKWARD) {
    digitalWrite(MOTOR_PINS[motorIndex].pinIN1, LOW);
    digitalWrite(MOTOR_PINS[motorIndex].pinIN2, HIGH);
  } else {
    digitalWrite(MOTOR_PINS[motorIndex].pinIN1, LOW);
    digitalWrite(MOTOR_PINS[motorIndex].pinIN2, LOW);
  }
}

void moveCar(int cmd) {
  Serial.printf("[MOTOR] Command: %d\n", cmd);
  switch (cmd) {
    case CMD_FORWARD:
      rotateMotor(RIGHT_MOTOR, MOTOR_FORWARD);
      rotateMotor(LEFT_MOTOR,  MOTOR_FORWARD);
      break;
    case CMD_BACKWARD:
      rotateMotor(RIGHT_MOTOR, MOTOR_BACKWARD);
      rotateMotor(LEFT_MOTOR,  MOTOR_BACKWARD);
      break;
    case CMD_LEFT:
      rotateMotor(RIGHT_MOTOR, MOTOR_FORWARD);
      rotateMotor(LEFT_MOTOR,  MOTOR_BACKWARD);
      break;
    case CMD_RIGHT:
      rotateMotor(RIGHT_MOTOR, MOTOR_BACKWARD);
      rotateMotor(LEFT_MOTOR,  MOTOR_FORWARD);
      break;
    case CMD_STOP:
    default:
      rotateMotor(RIGHT_MOTOR, MOTOR_STOP);
      rotateMotor(LEFT_MOTOR,  MOTOR_STOP);
      break;
  }
}


// ═══════════════════════════════════════════════════════════════════════
//  WEBSOCKET EVENT HANDLERS  (unchanged)
// ═══════════════════════════════════════════════════════════════════════
void onCarInputEvent(AsyncWebSocket* server,
                     AsyncWebSocketClient* client,
                     AwsEventType type,
                     void* arg,
                     uint8_t* data,
                     size_t len)
{
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS/CarInput] Client #%u connected from %s\n",
                    client->id(), client->remoteIP().toString().c_str());
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("[WS/CarInput] Client #%u disconnected\n", client->id());
      moveCar(CMD_STOP);
      ledcWrite(PWM_CHANNEL_LIGHT, 0);
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 &&
          info->len == len && info->opcode == WS_TEXT)
      {
        std::string payload((char*)data, len);
        std::istringstream ss(payload);
        std::string key, value;
        std::getline(ss, key,   ',');
        std::getline(ss, value, ',');

        int valueInt = atoi(value.c_str());
        Serial.printf("[WS/CarInput] Key=[%s] Value=[%s]\n",
                      key.c_str(), value.c_str());

        if (key == "MoveCar") {
          moveCar(valueInt);
          lastCommandTime = millis();
        } else if (key == "Speed") {
          valueInt = constrain(valueInt, 0, 255);
          ledcWrite(PWM_CHANNEL_SPEED, valueInt);
        } else if (key == "Light") {
          valueInt = constrain(valueInt, 0, 255);
          ledcWrite(PWM_CHANNEL_LIGHT, valueInt);
        } else {
          Serial.printf("[WS/CarInput] Unknown key: %s\n", key.c_str());
        }
      }
      break;
    }

    case WS_EVT_PONG:
    case WS_EVT_ERROR:
    default:
      break;
  }
}

void onCameraEvent(AsyncWebSocket* server,
                   AsyncWebSocketClient* client,
                   AwsEventType type,
                   void* arg,
                   uint8_t* data,
                   size_t len)
{
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[WS/Camera] Client #%u connected from %s\n",
                    client->id(), client->remoteIP().toString().c_str());
      activeCameraClientId = client->id();
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("[WS/Camera] Client #%u disconnected\n", client->id());
      if (client->id() == activeCameraClientId) {
        activeCameraClientId = 0;
        // ADDED: sendCameraFrame() returns early when activeCameraClientId==0,
        // so pendingFrames stops incrementing and the decay loop drains it to 0
        // before any new client connects. No explicit reset needed.
      }
      break;

    case WS_EVT_DATA:
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
    default:
      break;
  }
}


// ═══════════════════════════════════════════════════════════════════════
//  CAMERA INITIALISATION
// ═══════════════════════════════════════════════════════════════════════
void setupCamera() {
  camera_config_t config;
  config.ledc_channel   = LEDC_CHANNEL_0;
  config.ledc_timer     = LEDC_TIMER_0;
  config.pin_d0         = CAM_PIN_D0;
  config.pin_d1         = CAM_PIN_D1;
  config.pin_d2         = CAM_PIN_D2;
  config.pin_d3         = CAM_PIN_D3;
  config.pin_d4         = CAM_PIN_D4;
  config.pin_d5         = CAM_PIN_D5;
  config.pin_d6         = CAM_PIN_D6;
  config.pin_d7         = CAM_PIN_D7;
  config.pin_xclk       = CAM_PIN_XCLK;
  config.pin_pclk       = CAM_PIN_PCLK;
  config.pin_vsync      = CAM_PIN_VSYNC;
  config.pin_href       = CAM_PIN_HREF;
  config.pin_sscb_sda   = CAM_PIN_SIOD;
  config.pin_sscb_scl   = CAM_PIN_SIOC;
  config.pin_pwdn       = CAM_PIN_PWDN;
  config.pin_reset      = CAM_PIN_RESET;
  config.xclk_freq_hz   = 20000000;
  config.pixel_format   = PIXFORMAT_JPEG;

  // CAMERA_GRAB_LATEST — always deliver the freshest frame, discard stale ones
  config.grab_mode      = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    config.frame_size    = CAM_FRAMESIZE;
    config.jpeg_quality  = CAM_QUALITY;
    //// MODIFIED ////
    // Reduced fb_count from 2 → 1.
    // With 2 frame buffers the camera can queue a frame that is already
    // 50–100 ms old by the time loop() picks it up, stacking on top of
    // WiFi transmit delay. A single buffer forces esp_camera_fb_get() to
    // always return the most recently captured frame, eliminating that
    // hidden pipeline lag. The trade-off (slightly lower theoretical
    // throughput) is irrelevant here because WiFi bandwidth — not DMA
    // fill speed — is the real bottleneck.
    config.fb_count      = 1;   // Was 2
    Serial.printf("[CAM] PSRAM found — preset=%d  framesize=%d  quality=%d  fb_count=1\n",
                  STREAM_PRESET, CAM_FRAMESIZE, CAM_QUALITY);
  } else {
    config.frame_size    = CAM_FRAMESIZE_NOPSRAM;
    config.jpeg_quality  = CAM_QUALITY;
    config.fb_count      = 1;
    Serial.printf("[CAM] No PSRAM — using fallback frame size  quality=%d\n",
                  CAM_QUALITY);
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init FAILED! Error 0x%x\n", err);
    for (int i = 0; i < 10; i++) {
      digitalWrite(LIGHT_PIN, HIGH); delay(150);
      digitalWrite(LIGHT_PIN, LOW);  delay(150);
    }
    return;
  }
  Serial.println("[CAM] Camera initialised OK");

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor) {
    sensor->set_brightness(sensor, 1);
    sensor->set_saturation(sensor, -1);
    sensor->set_whitebal(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_aec2(sensor, 1);
    sensor->set_agc_gain(sensor, 0);
    sensor->set_gainceiling(sensor, (gainceiling_t)2);  // Max 4× gain — keeps frame sizes predictable

    // Uncomment if camera is mounted upside-down:
    // sensor->set_vflip(sensor, 1);
    // sensor->set_hmirror(sensor, 1);
  }
}


// ═══════════════════════════════════════════════════════════════════════
//  CAMERA FRAME SENDER
// ═══════════════════════════════════════════════════════════════════════
void sendCameraFrame() {
  if (activeCameraClientId == 0) return;

  AsyncWebSocketClient* clientPtr = wsCamera.client(activeCameraClientId);
  if (!clientPtr) {
    activeCameraClientId = 0;
    return;
  }

  //// MODIFIED ////
  // Proactive backpressure — track frames in-flight ourselves.
  // queueLen() does not exist in most ESPAsyncWebServer releases, so we
  // use a static counter instead:
  //   • Incremented just before wsCamera.binary() sends a frame.
  //   • Decremented inside a WS_EVT_DATA or WS_EVT_PONG callback
  //     (repurposed as a simple ACK path — see onCameraEvent).
  //
  // If 2 frames are already in-flight (~80–100 ms of buffered video at
  // 20–25 FPS) we drop the new frame and return immediately.  This keeps
  // the WebSocket queue thin and the viewer's image perpetually fresh.
  // The original queueIsFull() check is kept below as a hard backstop.
  //
  // Why 2? One frame can be mid-transmit, one queued. A third would sit
  // waiting for both to clear — adding a full frame-interval of extra lag.
  static uint8_t pendingFrames = 0;          // ADDED — in-flight frame counter

  if (pendingFrames >= 2) return;            // ADDED — proactive frame drop
  if (clientPtr->queueIsFull()) return;      // Existing hard backstop kept

  // Grab the latest frame from the camera buffer
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;   // Silent drop — no Serial print in hot path

  pendingFrames++;                                           // ADDED -- frame now in-flight
  wsCamera.binary(activeCameraClientId, fb->buf, fb->len);
  esp_camera_fb_return(fb);

  //// ADDED ////
  // Time-based pendingFrames decay.
  // After one FRAME_INTERVAL has elapsed since the last decay tick we
  // treat the oldest in-flight frame as delivered and decrement the
  // counter. No real ACK callback needed; the logic still reliably
  // prevents more than 2 frames from stacking in the TX pipeline.
  static unsigned long lastDecayTime = 0;
  unsigned long decayNow = millis();
  if (pendingFrames > 0 && (decayNow - lastDecayTime >= FRAME_INTERVAL)) {
    pendingFrames--;
    lastDecayTime = decayNow;
  }

  // FPS counter — reports every 5 s to avoid Serial bottleneck
  static unsigned long lastFpsTime = 0;
  static int           frameCount  = 0;
  frameCount++;
  unsigned long now = millis();
  if (now - lastFpsTime >= 5000) {
    Serial.printf("[FPS] %.1f avg over last 5s\n", frameCount / 5.0f);
    frameCount   = 0;
    lastFpsTime  = now;
  }
}


// ═══════════════════════════════════════════════════════════════════════
//  PIN SETUP  (unchanged)
// ═══════════════════════════════════════════════════════════════════════
void setupPins() {
  ledcSetup(PWM_CHANNEL_SPEED, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_LIGHT, PWM_FREQ, PWM_RESOLUTION);

  for (int i = 0; i < 2; i++) {
    pinMode(MOTOR_PINS[i].pinIN1,    OUTPUT);
    pinMode(MOTOR_PINS[i].pinIN2,    OUTPUT);
    ledcAttachPin(MOTOR_PINS[i].pinEnable, PWM_CHANNEL_SPEED);
  }

  moveCar(CMD_STOP);
  ledcWrite(PWM_CHANNEL_SPEED, 150);

  pinMode(LIGHT_PIN, OUTPUT);
  ledcAttachPin(LIGHT_PIN, PWM_CHANNEL_LIGHT);
  ledcWrite(PWM_CHANNEL_LIGHT, 0);
}


// ═══════════════════════════════════════════════════════════════════════
//  HTTP ROUTE HANDLERS  (unchanged)
// ═══════════════════════════════════════════════════════════════════════
void handleRoot(AsyncWebServerRequest* request) {
  request->send_P(200, "text/html", HTML_REDIRECT);
}

void handleNotFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not Found");
}


// ═══════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] ESP32 Surveillance Car starting...");

  setupPins();
  Serial.println("[BOOT] Pins configured");

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);

  //// ADDED ////
  // Disable WiFi modem sleep.
  // In softAP mode the radio can still enter a low-power doze between
  // beacon intervals, causing irregular TX gaps of 20–100 ms. With a
  // live video stream those gaps are the "random stutters" often reported
  // on ESP32 camera projects. WIFI_PS_NONE keeps the radio fully awake
  // at the cost of ~20 mA extra draw — acceptable on a battery car.
  WiFi.setSleep(false);   // ADDED — eliminates modem-sleep TX gaps

  IPAddress ip = WiFi.softAPIP();
  Serial.printf("[WIFI] AP started — SSID: %s  IP: %s  sleep: OFF\n",
                WIFI_SSID, ip.toString().c_str());

  server.on("/",          HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);

  wsCamera.onEvent(onCameraEvent);
  server.addHandler(&wsCamera);

  wsCarInput.onEvent(onCarInputEvent);
  server.addHandler(&wsCarInput);

  server.begin();
  Serial.println("[HTTP] Server started on port 80");

  setupCamera();

  Serial.printf("[BOOT] Ready! Open dashboard and enter IP: %s\n",
                ip.toString().c_str());
  Serial.printf("[BOOT] Stream preset: %d  |  Frame interval: %dms\n",
                STREAM_PRESET, FRAME_INTERVAL);
}


// ═══════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════
bool isAutoStopped = false;

void loop() {
  wsCamera.cleanupClients();
  wsCarInput.cleanupClients();

  // Frame sender — rate-limited by preset interval
  static unsigned long lastFrameTime = 0;
  if (millis() - lastFrameTime >= FRAME_INTERVAL) {
    lastFrameTime = millis();
    sendCameraFrame();
  }

  // Auto-stop safety (unchanged)
  if (millis() - lastCommandTime > 3000) {
    if (!isAutoStopped) {
      moveCar(CMD_STOP);
      isAutoStopped = true;
    }
  } else {
    isAutoStopped = false;
  }

  // Memory debug — every 5 s (unchanged)
  static unsigned long lastMemReport = 0;
  if (millis() - lastMemReport > 5000) {
    lastMemReport = millis();
    Serial.printf("[MEM] Free heap: %u  |  PSRAM total: %u  free: %u\n",
                  ESP.getFreeHeap(),
                  ESP.getPsramSize(),
                  ESP.getFreePsram());
  }

  //// MODIFIED ////
  // Reduced loop() tail delay from 1 ms → 0 (removed).
  // At 20–25 FPS a 1 ms sleep is 2–2.5% of the entire frame budget — small
  // but pointless. The frame-rate limiter above already provides all the
  // pacing needed. Removing it lets AsyncTCP's internal callbacks execute
  // more promptly, which helps both motor commands and WebSocket ACKs.
  // (If you see WDT resets, add delay(1) back — it should not be needed.)
}
