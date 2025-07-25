/*  esp32cam_recorder.ino  –  rev-C (HD option, static helpers)  July 2025
    ------------------------------------------------------------
    • Reads /config.txt (exposure, gain, brightness, fps, duration, resolution)
    • MJPEG-AVI with idx1 (opens silently in FFmpeg / VLC)
    • Default QVGA @15 fps;  set  resolution=HD   for 1280×720 @5 fps
    ------------------------------------------------------------*/

#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include <FS.h>
#include <SD_MMC.h>
#include <esp_camera.h>

/* ───────── Status LED ───────── */
#define PIN_STATUS_REC 33     // on-board white LED

struct ResInfo;
/* ───────── Config structure ───────── */
struct RecSettings {
  float   exposure   = 0.5f;
  uint8_t gain       = 5;
  int8_t  brightness = 0;
  uint8_t fps        = 15;
  uint32_t duration  = 60;
  String  res        = "QVGA";
};

/* ───────── Disable Wi-Fi / BT ───────── */
static void disableWireless() {
  btStop();
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
}

/* ───────── INI reader ───────── */
static bool readConfig(RecSettings &cfg) {
  File f = SD_MMC.open("/config.txt", "r");
  if (!f) return false;
  while (f.available()) {
    String ln = f.readStringUntil('\n'); ln.trim();
    if (!ln.length() || ln[0]=='#') continue;
    int eq = ln.indexOf('='); if (eq < 1) continue;
    String k = ln.substring(0,eq); k.trim();
    String v = ln.substring(eq+1); v.trim();
    if      (k.equalsIgnoreCase("exposure"))   cfg.exposure   = v.toFloat();
    else if (k.equalsIgnoreCase("gain"))       cfg.gain       = v.toInt();
    else if (k.equalsIgnoreCase("brightness")) cfg.brightness = v.toInt();
    else if (k.equalsIgnoreCase("framerate"))  cfg.fps        = v.toInt();
    else if (k.equalsIgnoreCase("duration"))   cfg.duration   = v.toInt();
    else if (k.equalsIgnoreCase("resolution")) cfg.res        = v;
  }
  f.close(); return true;
}

/* ───────── Resolution helper ───────── */
struct ResInfo { framesize_t fs; uint16_t w, h; };   // ①  <— must come first

static ResInfo resFromString(const String &k) {      // ②  now the type is known
  if (k.equalsIgnoreCase("HD"))   return {FRAMESIZE_HD ,1280, 720};
  if (k.equalsIgnoreCase("SVGA")) return {FRAMESIZE_SVGA, 800, 600};
  if (k.equalsIgnoreCase("XGA"))  return {FRAMESIZE_XGA ,1024, 768};
  if (k.equalsIgnoreCase("SXGA")) return {FRAMESIZE_SXGA,1280,1024};
  if (k.equalsIgnoreCase("UXGA")) return {FRAMESIZE_UXGA,1600,1200};
  if (k.equalsIgnoreCase("VGA"))  return {FRAMESIZE_VGA , 640, 480};
  return                          {FRAMESIZE_QVGA,      320, 240};
}

/* ───────── Compact MJPEG-AVI writer ───────── */
class AviWriter {
  File     f;
  uint32_t moviStart = 0;
  uint32_t frames    = 0;
  uint16_t width, height;
  uint32_t uspf;
public:
  bool begin(File file, uint16_t fps, uint16_t w, uint16_t h);
  void addFrame(const uint8_t *buf, uint32_t len);
  void end();
private:
  void patch();
  void writeIdx();
};

bool AviWriter::begin(File file, uint16_t fps, uint16_t w, uint16_t h) {
  f = file; if (!f) return false;
  width = w; height = h; uspf = 1'000'000UL / fps;

  struct __attribute__((packed)) HDR {
    char RIFF[4]={'R','I','F','F'}; uint32_t riff_sz=0; char AVI_[4]={'A','V','I',' '};
    char LIST_hdrl[4]={'L','I','S','T'}; uint32_t hdrl_sz=0xC4; char hdrl_ty[4]={'h','d','r','l'};
    char avih[4]={'a','v','i','h'}; uint32_t avih_sz=0x38;
    uint32_t us_pf; uint32_t max_bps; uint32_t pad=0; uint32_t flags=0x10;
    uint32_t tot_fr=0; uint32_t init_fr=0; uint32_t streams=1; uint32_t buf_sug=0;
    uint32_t w; uint32_t h; uint32_t rsv[4]={0};
    char LIST_strl[4]={'L','I','S','T'}; uint32_t strl_sz=0x7C; char strl_ty[4]={'s','t','r','l'};
    char strh[4]={'s','t','r','h'}; uint32_t strh_sz=0x38;
    char fcc_type[4]={'v','i','d','s'}; char fcc_hdlr[4]={'M','J','P','G'};
    uint32_t flags2=0; uint16_t prio=0, lang=0; uint32_t init_fr2=0;
    uint32_t scale; uint32_t rate; uint32_t start=0; uint32_t length=0;
    uint32_t buf_sug2=0; uint32_t quality=0xFFFFFFFF; uint32_t sample_sz=0;
    int16_t rcL=0, rcT=0, rcR, rcB;
    char strf[4]={'s','t','r','f'}; uint32_t strf_sz=0x28;
    uint32_t bi_size=40; int32_t bi_w; int32_t bi_h; uint16_t planes=1;
    uint16_t bit_cnt=24; uint32_t compress=0x47504A4D; uint32_t bi_sz_img=0;
    int32_t xppm=0, yppm=0; uint32_t clr_used=0, clr_imp=0;
    char LIST_movi[4]={'L','I','S','T'}; uint32_t movi_sz=0; char movi_ty[4]={'m','o','v','i'};
  } hdr;

  hdr.us_pf = uspf; hdr.max_bps = 1024*1024;
  hdr.w = hdr.rcR = width; hdr.h = hdr.rcB = height;
  hdr.scale = uspf; hdr.rate = 1'000'000;
  hdr.bi_w = width; hdr.bi_h = height;

  f.write((uint8_t*)&hdr, sizeof(hdr));
  moviStart = f.position();
  return true;
}

void AviWriter::addFrame(const uint8_t *buf, uint32_t len) {
  static const uint8_t tag_00dc[4] = {'0','0','d','c'};
  f.write(tag_00dc,4); f.write((uint8_t*)&len,4);
  f.write(buf,len); if(len&1) f.write((uint8_t)0);
  frames++; if((frames&0x07)==0) patch();
}

void AviWriter::end() { patch(); writeIdx(); }

void AviWriter::patch() {
  uint32_t fs = f.size(), movi = fs - moviStart;
  f.seek(4);   f.write((uint8_t*)&fs,4);
  f.seek(48);  f.write((uint8_t*)&frames,4);
  f.seek(140); f.write((uint8_t*)&frames,4);
  f.seek(moviStart-8); f.write((uint8_t*)&movi,4);
  f.flush();
}

void AviWriter::writeIdx() {
  uint32_t idx_sz = frames*16;
  static const uint8_t tag_idx1[4] = {'i','d','x','1'};
  static const uint8_t tag_00dc[4] = {'0','0','d','c'};
  f.write(tag_idx1,4); f.write((uint8_t*)&idx_sz,4);

  uint32_t off = moviStart + 4;
  f.seek(moviStart);
  for(uint32_t i=0;i<frames;i++){
    f.write(tag_00dc,4); uint32_t fl=0x10; f.write((uint8_t*)&fl,4);
    f.write((uint8_t*)&off,4);
    f.seek(off+4); uint32_t len; f.read((uint8_t*)&len,4);
    f.write((uint8_t*)&len,4);
    off += 8 + len + (len&1);
  }
  f.flush();
}

/* ───────── Camera init ───────── */
static bool initCamera(const RecSettings &cfg,
                       uint16_t &w, uint16_t &h, framesize_t &fs)
{
  ResInfo ri = resFromString(cfg.res);
  w = ri.w; h = ri.h; fs = ri.fs;

  camera_config_t c = {
    .pin_pwdn=32,.pin_reset=-1,.pin_xclk=0,
    .pin_sscb_sda=26,.pin_sscb_scl=27,
    .pin_d7=35,.pin_d6=34,.pin_d5=39,.pin_d4=36,
    .pin_d3=21,.pin_d2=19,.pin_d1=18,.pin_d0=5,
    .pin_vsync=25,.pin_href=23,.pin_pclk=22,
    .xclk_freq_hz=20'000'000,
    .pixel_format=PIXFORMAT_JPEG,
    .frame_size=FRAMESIZE_HD,
    .jpeg_quality=12,
    .fb_count=2
  };
  if (esp_camera_init(&c)!=ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  s->set_gain_ctrl(s,0); s->set_aec2(s,0);
  s->set_agc_gain(s,cfg.gain);
  s->set_brightness(s,cfg.brightness);
  s->set_exposure_ctrl(s,0);
  uint32_t shut = constrain((uint32_t)(cfg.exposure*1200),100U,600U);
  s->set_aec_value(s, shut);
  return true;
}

/* ───────── Filename helper ───────── */
static String nextFilename() {
  char buf[24]; uint16_t idx=1;
  while(true){ sprintf(buf,"/recording_%04u.avi",idx);
    if(!SD_MMC.exists(buf)) return String(buf); idx++; }
}

/* ───────── Record one clip ───────── */
static bool recordClip(const RecSettings &cfg) {
  uint16_t w,h; framesize_t fs;
  if(!initCamera(cfg,w,h,fs)) { Serial.println("Camera init failed"); return false; }

  File vf = SD_MMC.open(nextFilename(), FILE_WRITE);
  if(!vf){ Serial.println("Open file failed"); return false; }

  AviWriter avi;
  if(!avi.begin(vf,cfg.fps,w,h)){ Serial.println("AVI begin failed"); return false; }

  Serial.println("Camera OK, recording…");
  digitalWrite(PIN_STATUS_REC, HIGH);

  uint32_t start=millis(), frameInt=1000UL/cfg.fps;
  while(cfg.duration==0 || millis()-start < cfg.duration*1000UL) {
    uint32_t t0=millis();
    camera_fb_t *fb = esp_camera_fb_get();
    if(fb){ avi.addFrame(fb->buf, fb->len); esp_camera_fb_return(fb); }
    uint32_t spent = millis()-t0; if(spent<frameInt) delay(frameInt-spent);
  }

  avi.end(); vf.close();
  digitalWrite(PIN_STATUS_REC, LOW);
  esp_camera_deinit();
  Serial.println("Clip finished ✅");
  return true;
}

/* ───────── Arduino skeleton ───────── */
void setup() {
  Serial.begin(115200);
  pinMode(PIN_STATUS_REC, OUTPUT); digitalWrite(PIN_STATUS_REC, LOW);
  disableWireless();

  if(!SD_MMC.begin()){ Serial.println("SD card init failed"); return; }
  Serial.println("[ESP32-CAM Recorder – revC]  SD card mounted");

  RecSettings cfg; readConfig(cfg);
  Serial.printf("exp=%.2fs gain=%d bright=%d fps=%d dur=%us res=%s\n",
                cfg.exposure,cfg.gain,cfg.brightness,cfg.fps,cfg.duration,
                cfg.res.c_str());

  recordClip(cfg);
}

void loop() { /* one clip per boot */ }
