#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SD_MMC.h>

struct RecSettings {
  float    exposure  = 0.5f;  // seconds (0.2–1.0)
  uint8_t  gain      = 5;     // 0–30
  int8_t   brightness= 0;     // –2…+2
  uint8_t  fps       = 15;    // frames/sec
  uint32_t duration  = 60;    // seconds (0 = unlimited)
};

inline void loadConfig(RecSettings &s)
{
  File f = SD_MMC.open("/config.txt");
  if (!f) return;                         // keep defaults

  while (f.available()) {
    String ln = f.readStringUntil('\n');  ln.trim();
    if (!ln.length() || ln[0]=='#') continue;

    int eq = ln.indexOf('=');  if (eq < 1) continue;
    String key = ln.substring(0, eq);  key.trim();
    String val = ln.substring(eq+1);   val.trim();

    if      (key.equalsIgnoreCase("exposure"))   s.exposure   = val.toFloat();
    else if (key.equalsIgnoreCase("gain"))       s.gain       = val.toInt();
    else if (key.equalsIgnoreCase("brightness")) s.brightness = val.toInt();
    else if (key.equalsIgnoreCase("framerate"))  s.fps        = val.toInt();
    else if (key.equalsIgnoreCase("duration"))   s.duration   = val.toInt();
  }
  f.close();
}
