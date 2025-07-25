/*  esp32cam_recorder.ino – rev‑B (2025‑07‑25)
    ------------------------------------------------------------
    • Buffer‑overflow bug fixed (AVI header now 68 bytes)
    • Status LEDs moved off UART pins (GPIO14/15)
    • Extra sanity prints so you know each stage succeeded
*/

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <FS.h>
#include <SD_MMC.h>
#include <esp_camera.h>
#include "recorder_config.h"

/* -------- GPIO status lines -------- */
#define PIN_STATUS_REC  14  //  GPIO14 – HIGH during recording
#define PIN_STATUS_ERR  15  //  GPIO15 – HIGH on fatal error

/* ============================================================
   Minimal embedded MJPEG‑AVI writer (stack‑safe)
   ============================================================ */
class AviWriter {
  File       f;
  uint32_t   moviStart = 0;
  uint32_t   frames    = 0;
  uint16_t   width     = 320, height = 240;   // QVGA default
  uint32_t   uspf      = 33333;               // 30 fps default
public:
  bool begin(File file, uint16_t fps,
             uint16_t w = 320, uint16_t h = 240)
  {
    f = file;
    if (!f) return false;

    width  = w;
    height = h;
    uspf   = 1'000'000UL / fps;

    /* ---- Build header ---- */
    struct  __attribute__((packed)) {
      char riff_id[4]   = {'R','I','F','F'};
      uint32_t riff_sz  = 0;              // patched later
      char riff_ty[4]   = {'A','V','I',' '};

      char hdrl_id[4]   = {'L','I','S','T'};
      uint32_t hdrl_sz  = 0xC4;           // fixed
      char hdrl_ty[4]   = {'h','d','r','l'};

      /* avih */
      char avih_id[4]   = {'a','v','i','h'};
      uint32_t avih_sz  = 0x38;
      uint32_t usec_per_frame;
      uint32_t max_bytes_per_sec;
      uint32_t padding = 0;
      uint32_t flags   = 0x10;            // HAS_INDEX
      uint32_t total_frames = 0;          // patched
      uint32_t initial_frames = 0;
      uint32_t streams = 1;
      uint32_t buf_suggest = 0;
      uint32_t w, h;
      uint32_t reserved[4] = {0};

      /* LIST strl */
      char strl_id[4]  = {'L','I','S','T'};
      uint32_t strl_sz = 0x7C;
      char strl_ty[4]  = {'s','t','r','l'};

      /* strh */
      char strh_id[4]  = {'s','t','r','h'};
      uint32_t strh_sz = 0x38;
      char fcc_type[4] = {'v','i','d','s'};
      char fcc_hdlr[4] = {'M','J','P','G'};
      uint32_t flags2  = 0;
      uint16_t priority = 0, language = 0;
      uint32_t init_frames = 0;
      uint32_t scale;
      uint32_t rate;
      uint32_t start = 0;
      uint32_t length = 0;                // patched
      uint32_t buf_suggest2 = 0;
      uint32_t quality = -1;
      uint32_t sample_sz = 0;
      int16_t  rc_left = 0, rc_top = 0, rc_right, rc_bottom;

      /* strf (BITMAPINFOHEADER) */
      char strf_id[4] = {'s','t','r','f'};
      uint32_t strf_sz = 0x28;
      uint32_t bi_size = 40;
      int32_t  bi_width;
      int32_t  bi_height;
      uint16_t bi_planes = 1;
      uint16_t bi_bitcount = 24;
      uint32_t bi_compression = 0x47504A4D; // 'MJPG'
      uint32_t bi_size_image = 0;
      int32_t  bi_x_ppm = 0, bi_y_ppm = 0;
      uint32_t bi_clr_used = 0, bi_clr_important = 0;

      /* LIST movi (header only, size patched later) */
      char movi_id[4] = {'L','I','S','T'};
      uint32_t movi_sz = 0;               // patched
      char movi_ty[4] = {'m','o','v','i'};
    } hdr;

    /* Fill dynamic fields */
    hdr.usec_per_frame = uspf;
    hdr.max_bytes_per_sec = 1024 * 1024;          // generous
    hdr.w = hdr.rc_right = width;
    hdr.h = hdr.rc_bottom = height;
    hdr.scale = uspf;
    hdr.rate  = 1'000'000;
    hdr.bi_width  = width;
    hdr.bi_height = height;

    f.write((uint8_t*)&hdr, sizeof(hdr));
    moviStart = f.position();
    return true;
  }

  void addFrame(const uint8_t *buf, uint32_t len)
  {
    static const uint8_t tag[4] = { '0','0','d','c' };
    f.write(tag, 4);
    f.write((uint8_t*)&len, 4);
    f.write(buf, len);
    if (len & 1) f.write((uint8_t)0);

    frames++;
    if ((frames & 0x07) == 0) patch();    // every 8 frames
  }

  void end() { patch(); }

private:
  void patch()
  {
    uint32_t fileSize = f.size();
    uint32_t moviSize = fileSize - moviStart;

    /* RIFF size */
    f.seek(4);  f.write((uint8_t*)&fileSize, 4);

    /* total frames in avih */
    f.seek(48); f.write((uint8_t*)&frames, 4);

    /* length in strh */
    f.seek(140); f.write((uint8_t*)&frames, 4);

    /* LIST movi size */
    f.seek(moviStart - 8);   // start of 'LIST'
    f.write((uint8_t*)&moviSize, 4);

    f.flush();
  }
};


/* ------------------------------------------------------------ */
void disableWireless() {
  btStop();
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
}

String nextFilename() {
  char name[22];
  for (uint16_t idx = 1; ; ++idx) {
    sprintf(name, "/recording_%04u.avi", idx);
    if (!SD_MMC.exists(name)) return String(name);
  }
}

bool initCamera(const RecSettings &cfg) {
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

  uint32_t shutter = constrain((uint32_t)(cfg.exposure * 1200), 100U, 600U);
  s->set_aec_value(s, shutter);
  return true;   // FPS throttled in software
}

bool recordClip(const RecSettings &cfg) {
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
void setup() {
  pinMode(PIN_STATUS_REC, OUTPUT);
  pinMode(PIN_STATUS_ERR, OUTPUT);
  digitalWrite(PIN_STATUS_REC, LOW);
  digitalWrite(PIN_STATUS_ERR, LOW);

  Serial.begin(115200);
  delay(50);                 // give USB time to enumerate
  Serial.println("[ESP32‑CAM Recorder ‑ revB]");

  disableWireless();

  if (!SD_MMC.begin()) {
    Serial.println("SD init failed");
    digitalWrite(PIN_STATUS_ERR, HIGH);
    return;
  }
  Serial.println("SD card mounted");
}

void loop() {
  RecSettings cfg; loadConfig(cfg);
  Serial.printf("exp=%.2fs gain=%d bright=%d fps=%d dur=%us", cfg.exposure, cfg.gain, cfg.brightness, cfg.fps, cfg.duration);

  if (!initCamera(cfg)) {
    Serial.println("Camera init failed");
    digitalWrite(PIN_STATUS_ERR, HIGH);
    while (true) delay(1000);
  }
  Serial.println("Camera OK, recording…");

  if (!recordClip(cfg)) {
    Serial.println("Fatal error – halting");
    while (true) delay(1000);
  }

  esp_camera_deinit();
  Serial.println("Clip finished ✅");
  delay(2000);
}
