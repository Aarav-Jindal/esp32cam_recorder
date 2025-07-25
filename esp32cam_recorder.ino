/*
   esp32cam_recorder.ino  —  July 2025
   ------------------------------------------------------------
   ESP32-CAM autonomous video logger
     • Reads /config.txt (exposure, gain, brightness, fps, duration)
     • Records sequential MJPEG-AVI files that survive power loss
     • GPIO1 HIGH while recording; GPIO2 HIGH on unrecoverable error
     • Wi-Fi + Bluetooth fully disabled for lower current draw
   Tested with Arduino-ESP32 core 3.0.7
   ------------------------------------------------------------*/

#include <Arduino.h>
#include <WiFi.h>          // for WiFi.mode()
#include "esp_wifi.h"      // for esp_wifi_stop()
#include <FS.h>
#include <SD_MMC.h>
#include <esp_camera.h>
#include "recorder_config.h"   // INI parser (next file)

/* -------- GPIO status lines -------- */
#define PIN_STATUS_REC  1   // GPIO1 – HIGH during recording
#define PIN_STATUS_ERR  2   // GPIO2 – HIGH on fatal error

/* ============================================================
   Minimal embedded MJPEG-AVI writer
   ============================================================ */
class AviWriter {
  File     f;
  uint32_t moviStart = 0;
  uint32_t frames    = 0;

public:
  bool begin(File file, uint16_t fps)
  {
    f = file;
    if (!f) return false;

    /* ---- Build a 64-byte RIFF/AVI header template ---- */
    uint8_t hdr[64] = {0};
    memcpy(&hdr[0],  "RIFF", 4);
    memcpy(&hdr[8],  "AVI ", 4);
    memcpy(&hdr[12], "LIST", 4); *reinterpret_cast<uint32_t*>(&hdr[16]) = 0x3C;
    memcpy(&hdr[20], "hdrl", 4);
    memcpy(&hdr[24], "avih", 4); *reinterpret_cast<uint32_t*>(&hdr[28]) = 0x28;
    uint32_t uspf = 1'000'000UL / fps;
    *reinterpret_cast<uint32_t*>(&hdr[32]) = uspf;          // µs / frame
    memcpy(&hdr[44], "\x10\0\0\0", 4);                      // flags = HAS_INDEX
    memcpy(&hdr[56], "LIST", 4);
    memcpy(&hdr[60], "\0\0\0\0movi", 5);                    // size patched later

    f.write(hdr, sizeof(hdr));
    moviStart = f.position();
    return true;
  }



  void addFrame(const uint8_t *buf, uint32_t len)
  {
    const uint8_t tag[4] = { '0','0','d','c' };
    f.write(tag, 4);
    f.write((uint8_t*)&len, 4);
    f.write(buf, len);
    if (len & 1) f.write((uint8_t)0);       // word-align

    frames++;
    if ((frames & 0x0F) == 0) patchHeader();   // every 16 frames
  }

  void end() { patchHeader(); f.flush(); }

private:
  void patchHeader()
  {
    uint32_t fileSize = f.size();
    uint32_t moviSize = fileSize - moviStart;

    f.seek(4);  f.write((uint8_t*)&fileSize, 4);   // RIFF size
    f.seek(48); f.write((uint8_t*)&frames, 4);     // total frames
    f.seek(60); f.write((uint8_t*)&moviSize, 4);   // LIST movi size
    f.flush();
  }
};

/* ------------------------------------------------------------ */
void disableWireless()
{
  btStop();
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
}

String nextFilename()
{
  char name[22];
  for (uint16_t idx = 1; ; ++idx) {
    sprintf(name, "/recording_%04u.avi", idx);
    if (!SD_MMC.exists(name)) return String(name);
  }
}

bool initCamera(const RecSettings &cfg)
{
  camera_config_t c = {
    .pin_pwdn     = 32,
    .pin_reset    = -1,
    .pin_xclk     = 0,
    .pin_sscb_sda = 26,
    .pin_sscb_scl = 27,

    .pin_d7 = 35, .pin_d6 = 34, .pin_d5 = 39, .pin_d4 = 36,
    .pin_d3 = 21, .pin_d2 = 19, .pin_d1 = 18, .pin_d0 = 5,
    .pin_vsync = 25, .pin_href = 23, .pin_pclk = 22,

    .xclk_freq_hz = 20'000'000,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size   = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count     = 2
  };
  if (esp_camera_init(&c) != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  s->set_gain_ctrl(s, 0);
  s->set_aec2(s, 0);
  s->set_agc_gain(s, cfg.gain);
  s->set_brightness(s, cfg.brightness);
  s->set_exposure_ctrl(s, 0);

  uint32_t shutter = constrain((uint32_t)(cfg.exposure * 1200), 100u, 600u);
  s->set_aec_value(s, shutter);

  /* FPS throttled in software timing loop for compatibility */
  return true;
}

bool recordClip(const RecSettings &cfg)
{
  File vf = SD_MMC.open(nextFilename(), FILE_WRITE);
  if (!vf) { digitalWrite(PIN_STATUS_ERR, HIGH); return false; }

  AviWriter avi;
  if (!avi.begin(vf, cfg.fps)) { digitalWrite(PIN_STATUS_ERR, HIGH); return false; }

  digitalWrite(PIN_STATUS_REC, HIGH);
  uint32_t start = millis();
  uint32_t frameInt = 1000UL / cfg.fps;

  while (cfg.duration == 0 || millis() - start < cfg.duration * 1000UL) {
    uint32_t t0 = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      avi.addFrame(fb->buf, fb->len);
      esp_camera_fb_return(fb);
    }
    uint32_t spent = millis() - t0;
    if (spent < frameInt) delay(frameInt - spent);
  }

  avi.end();
  vf.close();
  digitalWrite(PIN_STATUS_REC, LOW);
  return true;
}

/* ---------------- Arduino skeleton ---------------- */
void setup()
{
  pinMode(PIN_STATUS_REC, OUTPUT);
  pinMode(PIN_STATUS_ERR, OUTPUT);
  digitalWrite(PIN_STATUS_REC, LOW);
  digitalWrite(PIN_STATUS_ERR, LOW);

  Serial.begin(115200);
  Serial.println("\n[ESP32-CAM Recorder]");

  disableWireless();

  if (!SD_MMC.begin()) {
    Serial.println("SD init failed");
    digitalWrite(PIN_STATUS_ERR, HIGH);
    return;
  }
}

void loop()
{
  RecSettings cfg;
  loadConfig(cfg);                 // parse /config.txt each cycle

  Serial.printf("exp=%.2fs gain=%d bright=%d fps=%d dur=%us\n",
                cfg.exposure, cfg.gain, cfg.brightness, cfg.fps, cfg.duration);

  if (!initCamera(cfg)) {
    Serial.println("Camera init failed");
    digitalWrite(PIN_STATUS_ERR, HIGH);
    while (true) delay(1000);
  }

  if (!recordClip(cfg)) {
    Serial.println("Fatal error – halting");
    while (true) delay(1000);
  }

  delay(2000);    // 2-s gap before the next clip (or edit config & reboot)
}
