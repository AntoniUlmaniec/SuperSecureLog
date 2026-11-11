/*
  SuperSecureLog – tryb czuwania: sen + wybudzanie + analiza ruchu i dźwięku
  Płytka: ESP32 Dev Module, Serial Monitor 115200

  Pliki w tym folderze:
    ssl_czuwanie.ino – sprzęt: sen, wybudzanie, odczyt czujników (ten plik)
    algorytm.h       – PARAMETRY i LOGIKA ALARMÓW (do edycji przez zespół od algorytmu)
    dsp.h            – FFT i cechy dźwięku

  Jak działa:
    ESP śpi. Budzi się, gdy:
      A) akcelerometr wykryje ruch (INT1 -> GPIO33),
      B) minie LISTEN_INTERVAL_S (domyślnie 10 min) – nasłuch silnika,
      C) minie RECHECK_S po wyniku „podejrzane” – ponowne sprawdzenie.
    Po pobudce: włącza mikrofon, przez 5 s zbiera ruch (100 Hz) i dźwięk (4 kHz, FFT co 0,256 s),
    wywołuje alg::decide() i wypisuje wynik. Potem znów śpi.

  POŁĄCZENIA – jak w ssl_ruch_dzwiek, z JEDNĄ zmianą:
    mikrofon VCC -> GPIO25 (zamiast szyny +). ESP włącza mikrofon tylko na czas analizy.
    (DOUT -> GPIO32 może zostać, ten szkic go nie używa.)

  RADIO nRF905 (pliki ssl_radio.h, ssl_protocol.h, nrf905.h, ssl_klucz.h w tym folderze):
    VCC 3V3, GND, PWR_UP G4, TRX_CE G16, TX_EN G17, CSN G5, SCK G18, MISO G19, MOSI G23, DR G27.
    Wysyłane do odbiornika (szkic ssl_odbiornik):
      ALARM (reguły 1–3), PODEJRZANE (4–5), INFO: silnik w pobliżu (6),
      ZYJE po włączeniu i potem co HEARTBEAT_S.
    Alarm bez potwierdzenia (ACK) jest powtarzany przy każdej kolejnej pobudce, aż dojdzie.
    Klucz w ssl_klucz.h musi być taki sam jak w odbiorniku.

  Pierwsze uruchomienie (lub przycisk EN): kalibracja położenia spoczynku i ciszy –
  czujnik musi leżeć nieruchomo, w pobliżu cicho, ok. 4 s.
*/
#include <Wire.h>
#include <math.h>
#include <sys/time.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "dsp.h"
#include "algorytm.h"
#include "ssl_radio.h"

// ======================================================= KONFIGURACJA
const uint32_t LISTEN_INTERVAL_S = 60;    // co ile nasłuch (B). Do testów ustaw np. 60
const uint32_t RECHECK_S         = 60;     // ponowne sprawdzenie po „podejrzane”
const uint32_t ANALYSIS_MS       = 5000;   // długość analizy po pobudce
const uint32_t MIC_SETTLE_MS     = 400;    // czas na ustabilizowanie mikrofonu po włączeniu
const float    WAKE_THRESH_G     = 0.10f;  // czułość wybudzania akcelerometru
const bool     PRINT_FRAMES      = true;   // wypisuj cechy każdej ramki dźwięku (do strojenia)
const bool     LED_ON_ALARM      = true;

// ======================================================= RADIO
const uint8_t  NODE_ID           = 1;      // numer czujnika 1..254 (każdy czujnik inny)
const uint32_t HEARTBEAT_S       = 6 * 3600;  // „żyję” co 6 h. Do testów np. 300
const bool     SEND_SUSPICIOUS   = true;   // wysyłać też „podejrzane” (reguły 4–5)
const bool     SEND_ENGINE_INFO  = true;   // wysyłać „silnik w pobliżu” (reguła 6)

// ======================================================= PINY
const int PIN_SDA     = 21;
const int PIN_SCL     = 22;
const int PIN_INT1    = 33;
const int PIN_MIC_A   = 34;
const int PIN_MIC_PWR = 25;
const int PIN_LED     = 2;
const gpio_num_t GPIO_INT1    = GPIO_NUM_33;
const gpio_num_t GPIO_MIC_PWR = GPIO_NUM_25;

// ======================================================= PAMIĘĆ PRZEŻYWAJĄCA SEN (RTC)
const uint32_t MAGIC = 0x55A15E01;
struct State {
  uint32_t magic;
  uint8_t  chip;          // 0 brak, 1 LIS3DSH, 2 LIS3DH
  uint8_t  addr;
  float    restX, restY, restZ;   // położenie spoczynku [g]
  float    noiseRms;              // poziom ciszy mikrofonu
  int      suspectCount;
  bool     recheckPending;
  uint32_t wakes, wakesMotion, wakesTimer, alarms;
  uint32_t awakeMsTotal;          // łączny czas pracy (do szacowania baterii)
  // radio
  bool     alarmPending;          // alarm nie dostał ACK – powtórzyć przy następnej pobudce
  uint8_t  pendRule, pendWake;
  int16_t  pendTilt;
  uint16_t pendMotionMs, pendPeakHz;
  uint8_t  pendEnginePct;
  uint32_t lastHeartbeatS;        // czas systemowy ostatniego „żyję” (liczy się też w czasie snu)
  uint32_t radioSent, radioAcked;
};
RTC_DATA_ATTR State st;

// ======================================================= AKCELEROMETR
enum Chip { NONE = 0, LIS3DSH = 1, LIS3DH = 2 };

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(st.addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int readReg(uint8_t reg) {
  Wire.beginTransmission(st.addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)st.addr, 1) != 1) return -1;
  return Wire.read();
}

bool readXYZ(float& x, float& y, float& z) {
  Wire.beginTransmission(st.addr);
  Wire.write(st.chip == LIS3DH ? (0x28 | 0x80) : 0x28);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)st.addr, 6) != 6) return false;
  int16_t r[3];
  for (int i = 0; i < 3; i++) {
    uint8_t lo = Wire.read(), hi = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  const float k = (st.chip == LIS3DH) ? (0.001f / 16.0f) : 0.00006f;
  x = r[0] * k; y = r[1] * k; z = r[2] * k;
  return true;
}

bool detectChip() {
  const uint8_t cands[] = {0x18, 0x19, 0x1E, 0x1D};
  for (uint8_t a : cands) {
    st.addr = a;
    int who = readReg(0x0F);
    if (who == 0x33) { st.chip = LIS3DH;  return true; }
    if (who == 0x3F) { st.chip = LIS3DSH; return true; }
  }
  st.chip = NONE;
  return false;
}

// czyta i kasuje zatrzaśnięte przerwanie
void accelClearInt() {
  if (st.chip == LIS3DH)  readReg(0x31);   // INT1_SRC
  if (st.chip == LIS3DSH) readReg(0x7F);   // OUTS2
}

// tryb analizy: 100 Hz, ±2 g, kolejka FIFO (32 próbki) w trybie strumieniowym
void accelConfigAnalysis() {
  if (st.chip == LIS3DH) {
    writeReg(0x30, 0x00);   // INT1_CFG off
    writeReg(0x22, 0x00);   // CTRL_REG3: nic na INT1
    writeReg(0x21, 0x00);   // CTRL_REG2: bez filtra
    writeReg(0x20, 0x57);   // CTRL_REG1: 100 Hz, XYZ
    writeReg(0x23, 0x88);   // CTRL_REG4: BDU, ±2 g, wysoka rozdzielczość
    writeReg(0x24, 0x40);   // CTRL_REG5: FIFO_EN
    writeReg(0x2E, 0x00);   // FIFO: bypass (czyści)
    writeReg(0x2E, 0x80);   // FIFO: stream
  } else if (st.chip == LIS3DSH) {
    writeReg(0x22, 0x00);   // CTRL_REG2: SM2 off
    writeReg(0x21, 0x00);   // CTRL_REG1: SM1 off
    writeReg(0x23, 0x00);   // CTRL_REG3: przerwania off
    writeReg(0x20, 0x67);   // CTRL_REG4: 100 Hz, XYZ
    writeReg(0x24, 0xC0);   // CTRL_REG5: filtr 50 Hz, ±2 g
    writeReg(0x25, 0x50);   // CTRL_REG6: FIFO_EN + autoinkrementacja
    writeReg(0x2E, 0x00);   // FIFO: bypass
    writeReg(0x2E, 0x40);   // FIFO: stream
  }
  accelClearInt();
}

// tryb czuwania: mała częstotliwość, przerwanie na INT1 przy ruchu
void accelConfigWake() {
  if (st.chip == LIS3DH) {
    writeReg(0x2E, 0x00);                  // FIFO bypass
    writeReg(0x24, 0x08);                  // CTRL_REG5: zatrzaskiwanie INT1, FIFO off
    writeReg(0x20, 0x2F);                  // CTRL_REG1: 10 Hz, tryb niskiego poboru, XYZ
    writeReg(0x21, 0x01);                  // CTRL_REG2: filtr górnoprzepustowy dla przerwania 1
    writeReg(0x23, 0x80);                  // CTRL_REG4: BDU, ±2 g
    writeReg(0x25, 0x00);                  // CTRL_REG6: INT aktywne wysokim
    int ths = (int)lroundf(WAKE_THRESH_G / 0.016f);
    if (ths < 1) ths = 1;
    writeReg(0x32, (uint8_t)ths);          // INT1_THS (16 mg/LSB przy ±2 g)
    writeReg(0x33, 0x00);                  // INT1_DURATION: wystarczy 1 próbka
    readReg(0x26);                         // REFERENCE: zeruje filtr
    writeReg(0x30, 0x2A);                  // INT1_CFG: X, Y lub Z powyżej progu
    writeReg(0x22, 0x40);                  // CTRL_REG3: przerwanie 1 na pin INT1
  } else if (st.chip == LIS3DSH) {
    // UWAGA: wersja dla LIS3DSH nie była testowana na sprzęcie – sprawdzić z notą ST AN3393
    writeReg(0x25, 0x10);                  // FIFO off
    writeReg(0x2E, 0x00);
    writeReg(0x22, 0x00);
    writeReg(0x20, 0x37);                  // 12,5 Hz
    writeReg(0x24, 0xC0);
    const float k = 16667.0f / 260.0f;     // g -> jednostki CS (1 g ~ 64)
    writeReg(0x13, (uint8_t)(int8_t)lroundf(st.restX * k));
    writeReg(0x14, (uint8_t)(int8_t)lroundf(st.restY * k));
    writeReg(0x15, (uint8_t)(int8_t)lroundf(st.restZ * k));
    writeReg(0x77, (uint8_t)lroundf(WAKE_THRESH_G / 0.0156f));   // THRS1_2
    writeReg(0x7A, 0xFC);                  // MASK2_A
    writeReg(0x79, 0xFC);                  // MASK2_B
    writeReg(0x7B, 0x19);                  // SETT2
    writeReg(0x60, 0x05);                  // ST2_1: GNTH1
    writeReg(0x61, 0x11);                  // ST2_2: CONT
    writeReg(0x23, 0x48);                  // INT1, aktywny wysoki, zatrzaskiwany
    writeReg(0x22, 0x01);                  // SM2 on, wyjście na INT1
  }
  delay(300);
  accelClearInt();
}

// przetwarzanie próbek ruchu: bloki po 10 próbek (100 ms)
struct AccelStats {
  float bx, by, bz, bsq; int bn;
  int blocks, movingBlocks;
  float maxDyn, maxTilt;
  float lastX[5], lastY[5], lastZ[5]; int li, ln;
  int samples;
} as;

float angleDeg(float x, float y, float z, float rx, float ry, float rz) {
  float d = (x * rx + y * ry + z * rz) /
            (sqrtf(x * x + y * y + z * z) * sqrtf(rx * rx + ry * ry + rz * rz) + 1e-9f);
  return acosf(fminf(1.0f, fmaxf(-1.0f, d))) * 180.0f / (float)M_PI;
}

void accelSample(float x, float y, float z) {
  as.samples++;
  as.bx += x; as.by += y; as.bz += z; as.bsq += x * x + y * y + z * z; as.bn++;
  if (as.bn < 10) return;
  float mx = as.bx / 10, my = as.by / 10, mz = as.bz / 10;
  float var = as.bsq / 10 - (mx * mx + my * my + mz * mz);
  float dyn = sqrtf(fmaxf(0, var));
  as.blocks++;
  if (dyn >= alg::MOTION_RMS_G) as.movingBlocks++;
  if (dyn > as.maxDyn) as.maxDyn = dyn;
  float t = angleDeg(mx, my, mz, st.restX, st.restY, st.restZ);
  if (t > as.maxTilt) as.maxTilt = t;
  as.lastX[as.li] = mx; as.lastY[as.li] = my; as.lastZ[as.li] = mz;
  as.li = (as.li + 1) % 5; if (as.ln < 5) as.ln++;
  as.bx = as.by = as.bz = as.bsq = 0; as.bn = 0;
}

void accelDrainFifo() {
  if (st.chip == NONE) return;
  int src = readReg(0x2F);                 // FIFO_SRC
  if (src < 0 || (src & 0x20)) return;     // błąd albo pusto
  int n = (src & 0x1F) + ((src & 0x40) ? 1 : 0);
  if (n == 0) n = 1;
  for (int i = 0; i < n; i++) {
    float x, y, z;
    if (readXYZ(x, y, z)) accelSample(x, y, z);
  }
}

void currentOrientation(float& x, float& y, float& z) {
  x = y = z = 0;
  for (int i = 0; i < as.ln; i++) { x += as.lastX[i]; y += as.lastY[i]; z += as.lastZ[i]; }
  if (as.ln) { x /= as.ln; y /= as.ln; z /= as.ln; }
}

// ======================================================= MIKROFON
float re[dsp::N], im[dsp::N], hann[dsp::N];

void micPower(bool on) {
  if (on) {
    rtc_gpio_hold_dis(GPIO_MIC_PWR);       // zwolnij blokadę z czasu snu
    rtc_gpio_deinit(GPIO_MIC_PWR);         // z powrotem zwykły pin
    pinMode(PIN_MIC_PWR, OUTPUT);
    digitalWrite(PIN_MIC_PWR, HIGH);
  } else {
    digitalWrite(PIN_MIC_PWR, LOW);
    // w czasie snu pin trzyma układ RTC – stan niski zablokowany na pewno
    rtc_gpio_init(GPIO_MIC_PWR);
    rtc_gpio_set_direction(GPIO_MIC_PWR, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(GPIO_MIC_PWR, 0);
    rtc_gpio_hold_en(GPIO_MIC_PWR);
  }
}

void captureFrame() {
  uint32_t t = micros();
  for (int i = 0; i < dsp::N; i++) {
    while ((int32_t)(micros() - t) < 0) {}
    re[i] = analogRead(PIN_MIC_A);
    t += 250;                               // 4000 próbek/s
  }
}

float medianOf(float* v, int n) {
  for (int i = 1; i < n; i++) {             // sortowanie przez wstawianie (n <= 32)
    float x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return n ? v[n / 2] : 0;
}

// ======================================================= ANALIZA 5 s
alg::Observations analyze(alg::WakeCause cause) {
  alg::Observations o = {};
  o.cause = cause;
  o.accelOk = st.chip != NONE;
  o.noiseRms = st.noiseRms;
  o.suspectCount = st.suspectCount;
  memset(&as, 0, sizeof(as));

  float rmsList[32]; int nr = 0;
  float f0sum = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ANALYSIS_MS) {
    captureFrame();
    accelDrainFifo();                       // ruch czytamy między ramkami (FIFO trzyma 0,32 s)
    dsp::FrameFeatures f;
    dsp::analyzeFrame(re, im, hann, alg::LOW_BAND_MIN_HZ, alg::LOW_BAND_MAX_HZ,
                      alg::TOTAL_BAND_MAX_HZ, f);
    bool eng = alg::frameIsEngine(f, st.noiseRms);
    o.frames++;
    if (eng) { o.engineFrames++; f0sum += f.f0; }
    if (nr < 32) rmsList[nr++] = f.rms;
    if (PRINT_FRAMES)
      Serial.printf("  ramka %2d: glosnosc %6.1f (%4.1fx cisza)  niskie %.2f  pik %5.1fx @ %4.0f Hz  harm %d%s -> %s\n",
                    o.frames, f.rms, f.rms / st.noiseRms, f.lowRatio, f.prominence, f.f0, f.harmonics,
                    f.clipped ? "  OBCIETY" : "", eng ? "SILNIK" : "-");
  }
  accelDrainFifo();

  o.engine = alg::engineDetected(o.engineFrames, o.frames);
  o.engineF0Hz = o.engineFrames ? f0sum / o.engineFrames : 0;
  o.medianRms = medianOf(rmsList, nr);
  if (o.accelOk && as.ln) {
    float x, y, z;
    currentOrientation(x, y, z);
    o.finalTiltDeg = angleDeg(x, y, z, st.restX, st.restY, st.restZ);
    o.maxTiltDeg = as.maxTilt;
    o.motionSec = as.movingBlocks * 0.1f;
    o.maxDynG = as.maxDyn;
  } else {
    o.accelOk = false;
  }
  return o;
}

// ======================================================= RAPORT I DZIAŁANIA
uint32_t nowS() {                            // czas systemowy ESP32 biegnie też w deep sleep
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return (uint32_t)tv.tv_sec;
}

uint8_t wakeCode(alg::WakeCause c) {
  switch (c) {
    case alg::WAKE_MOTION:  return ssl::W_MOTION;
    case alg::WAKE_LISTEN:  return ssl::W_LISTEN_TIMER;
    case alg::WAKE_RECHECK: return ssl::W_CHECK_TIMER;
    default:                return ssl::W_POWER_ON;
  }
}

bool radioSend(ssl::Frame f, const char* what) {
  f.wakes = (uint16_t)st.wakes;
  sslradio::Result r = sslradio::send(NODE_ID, f);
  st.radioSent++;
  if (r.acked) st.radioAcked++;
  if (!r.radioOk)
    Serial.printf("  RADIO:  %s - nRF905 NIE ODPOWIADA (VCC, GND, SPI G5/G18/G19/G23, PWR_UP G4)\n", what);
  else
    Serial.printf("  RADIO:  %s - %s, nadane %u/%u, %lu ms%s\n", what, r.acked ? "ACK OK" : "BRAK ACK",
                  r.txDone, r.attempts, (unsigned long)r.ms,
                  r.txDone == 0 ? "  <- modul nie potwierdza nadania: sprawdz DR (G27) i TX_EN (G17)" : "");
  return r.acked;
}

// wysyła wynik decyzji (alarm / podejrzane / info); niepotwierdzony alarm zapamiętuje do powtórki
void sendRadio(alg::Verdict v, int rule, const alg::Observations& o) {
  ssl::Frame f;
  f.type = v == alg::V_THEFT_ALARM ? ssl::T_ALARM : v == alg::V_SUSPICIOUS ? ssl::T_SUSPECT : ssl::T_INFO;
  f.reason = (uint8_t)rule;
  f.wake = wakeCode(o.cause);
  if (o.accelOk) {
    f.tilt_ddeg = (int16_t)lroundf(o.finalTiltDeg * 10);
    f.motion_ms = (uint16_t)lroundf(fminf(o.motionSec, 60.0f) * 1000);
  }
  f.engine_pct = o.frames ? (uint8_t)(100 * o.engineFrames / o.frames) : 0;
  f.peak_hz = (uint16_t)lroundf(o.engineF0Hz);

  bool ok = radioSend(f, alg::verdictName(v));
  if (v == alg::V_THEFT_ALARM) {
    st.alarmPending = !ok;
    st.pendRule = f.reason; st.pendWake = f.wake; st.pendTilt = f.tilt_ddeg;
    st.pendMotionMs = f.motion_ms; st.pendPeakHz = f.peak_hz; st.pendEnginePct = f.engine_pct;
  }
}

// po decyzji: powtórka niepotwierdzonego alarmu i cykliczne „żyję”
void radioHousekeeping(alg::WakeCause cause, bool alarmSentNow) {
  if (st.alarmPending && !alarmSentNow) {
    ssl::Frame f;
    f.type = ssl::T_ALARM;
    f.flags = ssl::FLAG_RETRY;
    f.reason = st.pendRule; f.wake = st.pendWake; f.tilt_ddeg = st.pendTilt;
    f.motion_ms = st.pendMotionMs; f.peak_hz = st.pendPeakHz; f.engine_pct = st.pendEnginePct;
    if (radioSend(f, "powtorka niepotwierdzonego alarmu")) st.alarmPending = false;
  }
  if (nowS() - st.lastHeartbeatS >= HEARTBEAT_S) {
    ssl::Frame f;
    f.type = ssl::T_HEARTBEAT;
    f.wake = wakeCode(cause);
    if (radioSend(f, "zyje")) st.lastHeartbeatS = nowS();
  }
}

void report(const alg::Observations& o, alg::Verdict v) {
  Serial.println("  ----------------------------------------------");
  if (o.accelOk)
    Serial.printf("  RUCH:   kat teraz %.1f st. (max %.1f), drgania %.1f s (max %.2f g)\n",
                  o.finalTiltDeg, o.maxTiltDeg, o.motionSec, o.maxDynG);
  else
    Serial.println("  RUCH:   brak akcelerometru");
  Serial.printf("  DZWIEK: %d ramek, silnikowych %d (%.0f%%)%s, cisza %.1f, mediana %.1f\n",
                o.frames, o.engineFrames, o.frames ? 100.0f * o.engineFrames / o.frames : 0,
                o.engine ? " -> SILNIK" : "", o.noiseRms, o.medianRms);
  if (o.engineFrames) Serial.printf("          sredni pik silnika ok. %.0f Hz\n", o.engineF0Hz);
  Serial.printf("  WYNIK:  %s\n", alg::verdictName(v));
}

// co zrobić po decyzji; zwraca czas snu w sekundach
uint32_t applyVerdict(const alg::Observations& o, alg::Verdict v, int rule) {
  float x, y, z;
  currentOrientation(x, y, z);
  bool haveOrient = o.accelOk && as.ln > 0;

  switch (v) {
    case alg::V_THEFT_ALARM:
      st.alarms++;
      sendRadio(v, rule, o);
      if (LED_ON_ALARM)
        for (int i = 0; i < 6; i++) { digitalWrite(PIN_LED, i % 2 == 0); delay(150); }
      // przyjmij nowe położenie, żeby ten sam stan nie alarmował w kółko
      if (haveOrient) { st.restX = x; st.restY = y; st.restZ = z; }
      st.suspectCount = 0;
      st.recheckPending = true;            // sprawdź, co dzieje się dalej
      return RECHECK_S;

    case alg::V_SUSPICIOUS:
      if (SEND_SUSPICIOUS) sendRadio(v, rule, o);
      st.suspectCount++;
      st.recheckPending = true;
      return RECHECK_S;

    case alg::V_ENGINE_NEARBY:
      if (SEND_ENGINE_INFO) sendRadio(v, rule, o);
      st.suspectCount = 0;
      st.recheckPending = false;
      return LISTEN_INTERVAL_S;

    default:  // V_NOTHING
      st.suspectCount = 0;
      st.recheckPending = false;
      // spokój -> uaktualnij położenie spoczynku i poziom ciszy
      if (haveOrient && o.motionSec < 0.5f) { st.restX = x; st.restY = y; st.restZ = z; }
      if (!o.engine && o.medianRms > 0) {
        st.noiseRms = 0.8f * st.noiseRms + 0.2f * o.medianRms;
        if (st.noiseRms < 1.0f) st.noiseRms = 1.0f;
      }
      return LISTEN_INTERVAL_S;
  }
}

// ======================================================= SEN
void goToSleep(uint32_t seconds) {
  micPower(false);
  digitalWrite(PIN_LED, LOW);
  bool wakeOnMotion = false;
  if (st.chip != NONE) {
    accelConfigWake();
    // Filtr w akcelerometrze po przestawieniu potrzebuje chwili. Jeśli INT1 dalej jest aktywne,
    // ESP obudziłby się natychmiast i mikrofon wyglądałby na włączony cały czas.
    for (int i = 0; i < 10 && digitalRead(PIN_INT1); i++) { delay(200); accelClearInt(); }
    if (digitalRead(PIN_INT1)) {
      Serial.println("  UWAGA: INT1 nadal aktywne (ruch trwa?) - tym razem budzi tylko timer.");
      if (seconds > RECHECK_S) seconds = RECHECK_S;
    } else {
      esp_sleep_enable_ext0_wakeup(GPIO_INT1, 1);
      wakeOnMotion = true;
    }
  }
  esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
  sslradio::sleep();                        // nRF905 wyłączony, jego piny zamrożone na czas snu
  st.awakeMsTotal += millis();
  Serial.printf("  Mikrofon WYLACZONY. Sen: %s%u s. Czas pracy dotad: %.1f s w %u pobudkach.\n\n",
                wakeOnMotion ? "do ruchu albo " : "", seconds, st.awakeMsTotal / 1000.0f, st.wakes);
  Serial.flush();
  esp_deep_sleep_start();
}

// ======================================================= PIERWSZE URUCHOMIENIE
void firstBoot() {
  memset(&st, 0, sizeof(st));
  st.magic = MAGIC;
  Serial.println("\n=== SuperSecureLog: czuwanie – pierwsze uruchomienie ===");
  Wire.beginTransmission(0);  // nic – tylko żeby Wire był gotowy
  Wire.endTransmission();
  if (detectChip())
    Serial.printf("Akcelerometr: %s pod 0x%02X\n", st.chip == LIS3DH ? "LIS3DH" : "LIS3DSH", st.addr);
  else
    Serial.println("Akcelerometr: NIE ZNALEZIONO – dziala tylko nasluch dzwieku co 10 min.");

  Serial.println("Kalibracja: nie ruszaj czujnikiem, badz cicho (ok. 4 s)...");
  st.restX = 0; st.restY = 0; st.restZ = 1;
  st.noiseRms = 1.0f;
  if (st.chip != NONE) accelConfigAnalysis();
  delay(MIC_SETTLE_MS);
  // 2 s: położenie spoczynku i cisza
  memset(&as, 0, sizeof(as));
  float rmsList[32]; int nr = 0;
  uint32_t t0 = millis();
  float sx = 0, sy = 0, sz = 0; int n = 0;
  while (millis() - t0 < 2500) {
    captureFrame();
    if (st.chip != NONE) {
      int src = readReg(0x2F);
      int cnt = (src >= 0 && !(src & 0x20)) ? (src & 0x1F) + ((src & 0x40) ? 1 : 0) : 0;
      for (int i = 0; i < cnt; i++) { float x, y, z; if (readXYZ(x, y, z)) { sx += x; sy += y; sz += z; n++; } }
    }
    dsp::FrameFeatures f;
    dsp::analyzeFrame(re, im, hann, alg::LOW_BAND_MIN_HZ, alg::LOW_BAND_MAX_HZ, alg::TOTAL_BAND_MAX_HZ, f);
    if (nr < 32) rmsList[nr++] = f.rms;
  }
  if (n > 0) { st.restX = sx / n; st.restY = sy / n; st.restZ = sz / n; }
  st.noiseRms = fmaxf(1.0f, medianOf(rmsList, nr));
  Serial.printf("Spoczynek: X=%.2f Y=%.2f Z=%.2f g (|a|=%.2f), cisza mikrofonu: %.1f\n",
                st.restX, st.restY, st.restZ,
                sqrtf(st.restX * st.restX + st.restY * st.restY + st.restZ * st.restZ), st.noiseRms);
  if (st.noiseRms < 1.5f)
    Serial.println("UWAGA: mikrofon prawie bez sygnalu – sprawdz VCC->GPIO25 i AOUT->GPIO34.");
  Serial.printf("Gotowe. Nasluch co %u s, ponowne sprawdzenie po %u s.\n", LISTEN_INTERVAL_S, RECHECK_S);

  // pierwsza ramka „żyję”: odbiornik od razu widzi, że czujnik wystartował i radio działa
  ssl::Frame f;
  f.type = ssl::T_HEARTBEAT;
  f.wake = ssl::W_POWER_ON;
  radioSend(f, "zyje (start)");
  st.lastHeartbeatS = nowS();
  goToSleep(LISTEN_INTERVAL_S);
}

// ======================================================= START
void setup() {
  gpio_deep_sleep_hold_dis();               // zwolnij piny, gdyby poprzedni program zamroził je na czas snu
  setCpuFrequencyMhz(80);                   // 3x mniej prądu niż 240 MHz, FFT i tak zdąży
  micPower(true);                           // mikrofon startuje od razu, równolegle z resztą
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  rtc_gpio_deinit(GPIO_INT1);               // po pobudce od ruchu pin jest jeszcze w trybie RTC
  pinMode(PIN_INT1, INPUT);
  analogReadResolution(12);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  dsp::makeHann(hann);

  esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
  if (st.magic != MAGIC || (wc != ESP_SLEEP_WAKEUP_EXT0 && wc != ESP_SLEEP_WAKEUP_TIMER)) {
    firstBoot();
    return;
  }

  st.wakes++;
  alg::WakeCause cause;
  if (wc == ESP_SLEEP_WAKEUP_EXT0) { cause = alg::WAKE_MOTION; st.wakesMotion++; }
  else { cause = st.recheckPending ? alg::WAKE_RECHECK : alg::WAKE_LISTEN; st.wakesTimer++; }
  Serial.printf("\n=== Pobudka #%u: %s ===\n", st.wakes, alg::causeName(cause));

  if (st.chip != NONE) {
    accelClearInt();
    accelConfigAnalysis();
  }
  uint32_t settle = millis();
  while (millis() - settle < MIC_SETTLE_MS) {}   // mikrofon dochodzi do stanu ustalonego

  alg::Observations o = analyze(cause);
  int rule = alg::decideRule(o);
  alg::Verdict v = alg::verdictOfRule(rule);
  report(o, v);
  Serial.printf("  REGULA: %d\n", rule);
  uint32_t sleepS = applyVerdict(o, v, rule);
  radioHousekeeping(cause, v == alg::V_THEFT_ALARM);
  goToSleep(sleepS);
}

void loop() {}   // nieużywane – po każdej pobudce ESP idzie spać w setup()
