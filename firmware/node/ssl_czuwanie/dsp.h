// dsp.h – FFT i cechy jednej ramki dźwięku (czysty C++, bez Arduino – można testować na PC)
#pragma once
#include <math.h>
#include <stdint.h>

namespace dsp {

const int   N      = 1024;          // próbek w ramce
const float FS     = 4000.0f;       // próbek na sekundę
const float BIN_HZ = FS / N;        // ok. 3,9 Hz na prążek FFT

struct FrameFeatures {
  float rms;          // głośność ramki (RMS, jednostki ADC, bez składowej stałej)
  float lowRatio;     // energia w paśmie niskim / energia w całym paśmie
  float prominence;   // najwyraźniejszy pik w paśmie niskim / jego otoczenie (±40 Hz)
  float f0;           // częstotliwość tego piku [Hz]
  int   harmonics;    // ile z 2*f0 i 3*f0 też jest wyraźnymi pikami (0..2)
  bool  clipped;      // sygnał obcięty (za głośno / za duże wzmocnienie)
};

// okno Hanna – liczone raz
inline void makeHann(float* w) {
  for (int i = 0; i < N; i++) w[i] = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * i / (N - 1));
}

// FFT radix-2 w miejscu
inline void fft(float* re, float* im, int n) {
  for (int i = 1, j = 0; i < n; i++) {
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * (float)M_PI / len;
    float wr = cosf(ang), wi = sinf(ang);
    int half = len >> 1;
    for (int i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (int k = 0; k < half; k++) {
        int a = i + k, b = a + half;
        float tr = re[b] * cr - im[b] * ci;
        float ti = re[b] * ci + im[b] * cr;
        re[b] = re[a] - tr; im[b] = im[a] - ti;
        re[a] += tr;        im[a] += ti;
        float ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = ncr;
      }
    }
  }
}

inline int hzToBin(float hz) {
  int k = (int)lroundf(hz / BIN_HZ);
  if (k < 1) k = 1;
  if (k > N / 2 - 1) k = N / 2 - 1;
  return k;
}

// średnia mocy w otoczeniu prążka k (±10 prążków), z pominięciem samego piku (±2)
inline float localBackground(const float* p, int k) {
  double s = 0; int n = 0;
  for (int j = k - 10; j <= k + 10; j++) {
    if (j < 1 || j >= N / 2 || (j >= k - 2 && j <= k + 2)) continue;
    s += p[j]; n++;
  }
  return n ? (float)(s / n) : 1e-9f;
}

// re[] = surowe próbki ADC (nadpisywane), im[] = bufor roboczy, w[] = okno Hanna
inline void analyzeFrame(float* re, float* im, const float* w,
                         float lowMinHz, float lowMaxHz, float totalMaxHz,
                         FrameFeatures& f) {
  // składowa stała, głośność, obcięcie
  double sum = 0;
  int clip = 0;
  for (int i = 0; i < N; i++) {
    sum += re[i];
    if (re[i] <= 5 || re[i] >= 4090) clip++;
  }
  float mean = (float)(sum / N);
  double sq = 0;
  for (int i = 0; i < N; i++) {
    float v = re[i] - mean;
    sq += (double)v * v;
    re[i] = v * w[i];
    im[i] = 0;
  }
  f.rms = (float)sqrt(sq / N);
  f.clipped = clip > N / 100;

  fft(re, im, N);
  // widmo mocy zapisujemy w re[k]
  for (int k = 0; k < N / 2; k++) re[k] = re[k] * re[k] + im[k] * im[k];

  int kLo = hzToBin(lowMinHz), kLoMax = hzToBin(lowMaxHz), kTotMax = hzToBin(totalMaxHz);
  double eLow = 0, eTot = 0;
  int kPeak = kLo;
  for (int k = kLo; k <= kTotMax; k++) {
    eTot += re[k];
    if (k <= kLoMax) {
      eLow += re[k];
      if (re[k] > re[kPeak]) kPeak = k;
    }
  }
  f.lowRatio = eTot > 0 ? (float)(eLow / eTot) : 0;

  // „ton”: prążek wyraźnie wyższy od SWOJEGO OTOCZENIA (±10 prążków, ok. ±40 Hz).
  // Porównanie z otoczeniem, a nie ze średnią całego pasma, żeby szum wiatru
  // (dużo energii przy najniższych częstotliwościach, ale bez pików) nie udawał silnika.
  f.prominence = 0;
  kPeak = kLo;
  for (int k = kLo; k <= kLoMax; k++) {
    float p = re[k] / (localBackground(re, k) + 1e-9f);
    if (p > f.prominence) { f.prominence = p; kPeak = k; }
  }
  // dokładniejsze położenie piku: najwyższy prążek w pobliżu + interpolacja paraboliczna
  for (int k = kPeak - 2; k <= kPeak + 2; k++)
    if (k >= 1 && k < N / 2 - 1 && re[k] > re[kPeak]) kPeak = k;
  float kf = (float)kPeak;
  {
    float a = re[kPeak - 1], b = re[kPeak], c = re[kPeak + 1];
    float den = a - 2 * b + c;
    if (den < 0) kf += 0.5f * (a - c) / den;
  }
  f.f0 = kf * BIN_HZ;

  // harmoniczne: czy w okolicy 2*f0 i 3*f0 też jest pik ponad lokalne tło
  f.harmonics = 0;
  for (int h = 2; h <= 3; h++) {
    int kh = (int)lroundf(kf * h);
    if (kh + 3 >= N / 2) break;
    int kmax = kh;
    for (int k = kh - 2; k <= kh + 2; k++) if (re[k] > re[kmax]) kmax = k;
    if (re[kmax] > 4.0f * localBackground(re, kmax)) f.harmonics++;
  }
}

}  // namespace dsp
