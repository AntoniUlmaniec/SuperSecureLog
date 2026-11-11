// algorytm.h – PARAMETRY I LOGIKA ALARMÓW
// To jest plik do edycji przez zespół od algorytmu. Nie zawiera nic sprzętowego,
// więc można go też skompilować i przetestować na PC (np. na nagraniach z mikrofonu).
#pragma once
#include "dsp.h"

namespace alg {

// ======================================================= PARAMETRY: DŹWIĘK
const float LOW_BAND_MIN_HZ     = 40.0f;   // pasmo „silnikowe” od...
const float LOW_BAND_MAX_HZ     = 300.0f;  // ...do
const float TOTAL_BAND_MAX_HZ   = 2000.0f; // całe analizowane pasmo: LOW_BAND_MIN_HZ..2000 Hz
const float LOW_RATIO_MIN       = 0.5f;    // min. udział energii pasma niskiego
const float PROMINENCE_MIN      = 8.0f;    // pik min. 8x ponad swoje otoczenie ±40 Hz (ok. 9 dB) = „ton”, nie szum
const int   HARMONICS_MIN       = 1;       // ile harmonicznych (2*f0, 3*f0) musi być widać – silnik ma ich wiele, szum nie
const float LEVEL_FACTOR        = 2.0f;    // ramka min. 2x głośniejsza niż cisza
const float ENGINE_FRACTION_MIN = 0.6f;    // silnik = min. 60% ramek „silnikowych” w oknie analizy
const int   ENGINE_MIN_FRAMES   = 5;       // poniżej tylu ramek nie oceniamy

// ======================================================= PARAMETRY: RUCH
const float TILT_ALARM_DEG   = 15.0f;  // zmiana orientacji = kradzież
const float TILT_SUSPECT_DEG = 5.0f;   // 5..15 st. = podejrzane
const float MOTION_RMS_G     = 0.1f;   // blok 100 ms liczy się jako „drgania”, gdy RMS >= 0,1 g
const float MOTION_ALARM_S   = 2.0f;   // drgania łącznie >= 2 s w oknie analizy
const int   MOTION_REPEATS_ALARM = 3;  // długie drgania bez silnika: alarm dopiero za tym razem z rzędu

// ======================================================= TYPY
enum WakeCause { WAKE_FIRST_BOOT, WAKE_MOTION, WAKE_LISTEN, WAKE_RECHECK };
enum Verdict   { V_NOTHING, V_SUSPICIOUS, V_THEFT_ALARM, V_ENGINE_NEARBY };

struct Observations {
  WakeCause cause;
  bool  accelOk;          // czy akcelerometr działał
  float finalTiltDeg;     // kąt względem położenia spoczynku na końcu analizy
  float maxTiltDeg;       // największy kąt w trakcie
  float motionSec;        // łączny czas drgań (bloki 100 ms z RMS >= MOTION_RMS_G)
  float maxDynG;          // największy RMS drgań [g]
  int   frames;           // ile ramek dźwięku przeanalizowano
  int   engineFrames;     // ile z nich uznano za „silnikowe”
  bool  engine;           // decyzja: pracuje silnik
  float engineF0Hz;       // średnia częstotliwość piku w ramkach silnikowych
  float medianRms;        // mediana głośności ramek (do aktualizacji poziomu ciszy)
  float noiseRms;         // aktualny poziom ciszy
  int   suspectCount;     // ile razy z rzędu poprzednio wyszło „podejrzane”
};

// ======================================================= LOGIKA

// Czy jedna ramka (0,256 s) brzmi jak silnik?
inline bool frameIsEngine(const dsp::FrameFeatures& f, float noiseRms) {
  if (f.clipped) {
    // obcięta = bardzo głośno; oceniamy normalnie, ale widmo jest zniekształcone
  }
  bool loud  = f.rms >= noiseRms * LEVEL_FACTOR;
  bool low   = f.lowRatio >= LOW_RATIO_MIN;
  bool tonal = f.prominence >= PROMINENCE_MIN && f.harmonics >= HARMONICS_MIN;  // wyraźny ton + harmoniczna
  return loud && low && tonal;
}

// Czy w całym oknie analizy (5 s) pracował silnik?
inline bool engineDetected(int engineFrames, int frames) {
  if (frames < ENGINE_MIN_FRAMES) return false;
  return (float)engineFrames / frames >= ENGINE_FRACTION_MIN;
}

// GŁÓWNA DECYZJA – tabela z ustaleń zespołu.
// Zwraca numer reguły 1..7 (pierwsza pasująca od góry). Ten numer idzie też radiem do odbiornika.
inline int decideRule(const Observations& o) {
  if (o.accelOk) {
    // 1. kłoda zmieniła położenie
    if (o.finalTiltDeg >= TILT_ALARM_DEG) return 1;
    // 2. długie drgania + silnik (np. załadunek dźwigiem)
    if (o.motionSec >= MOTION_ALARM_S && o.engine) return 2;
    // 3. długie drgania bez silnika trzeci raz z rzędu (dwa ponowne sprawdzenia też wykazały ruch)
    if (o.motionSec >= MOTION_ALARM_S && o.suspectCount >= MOTION_REPEATS_ALARM - 1) return 3;
    // 4. długie drgania bez silnika -> sprawdź ponownie za chwilę
    if (o.motionSec >= MOTION_ALARM_S) return 4;
    // 5. lekkie przechylenie 5..15 st. -> sprawdź ponownie; jeśli przy ponownym sprawdzeniu
    //    kłoda leży spokojnie, uznajemy to za osiadanie stosu (nowe położenie spoczynku)
    if (o.finalTiltDeg >= TILT_SUSPECT_DEG && o.cause != WAKE_RECHECK) return 5;
  }
  // 6. bez ruchu, ale słychać silnik -> informacja
  if (o.engine) return 6;
  // 7. nic ciekawego (krótkie drgania: zwierzę, gałąź, wiatr)
  return 7;
}

inline Verdict verdictOfRule(int rule) {
  if (rule <= 3) return V_THEFT_ALARM;
  if (rule <= 5) return V_SUSPICIOUS;
  if (rule == 6) return V_ENGINE_NEARBY;
  return V_NOTHING;
}

inline Verdict decide(const Observations& o) { return verdictOfRule(decideRule(o)); }

inline const char* verdictName(Verdict v) {
  switch (v) {
    case V_THEFT_ALARM:   return "ALARM: KRADZIEZ";
    case V_SUSPICIOUS:    return "PODEJRZANE - ponowne sprawdzenie";
    case V_ENGINE_NEARBY: return "INFORMACJA: silnik w poblizu";
    default:              return "NIC";
  }
}

inline const char* causeName(WakeCause c) {
  switch (c) {
    case WAKE_FIRST_BOOT: return "pierwsze uruchomienie";
    case WAKE_MOTION:     return "RUCH (INT1 z akcelerometru)";
    case WAKE_LISTEN:     return "timer - nasluch";
    default:              return "timer - ponowne sprawdzenie";
  }
}

}  // namespace alg
