// SuperSecureLog – wysyłanie z czujnika (nadajnik) z potwierdzeniem ACK.
//
// Użycie w szkicu czujnika:
//   #include <ssl_radio.h>
//   ssl::Frame f; f.type = ssl::T_ALARM; f.reason = 1; f.tilt_ddeg = 173; ...
//   sslradio::Result r = sslradio::send(NODE_ID, f);   // radio włącza się samo
//   ...
//   sslradio::sleep();          // ZAWSZE tuż przed esp_deep_sleep_start()
//
// Licznik włączeń (boot) jest w NVS, numer ramki (seq) w pamięci RTC – przeżywa
// deep sleep, zeruje się po resecie razem ze wzrostem boot.

#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "ssl_protocol.h"
#include "ssl_klucz.h"
#include "nrf905.h"

#ifndef SSL_BAT_PIN
#define SSL_BAT_PIN -1          // -1 = brak dzielnika (zasilanie z USB). Z dzielnikiem 1 MΩ/1 MΩ na GPIO35:
                                // #define SSL_BAT_PIN 35 przed #include <ssl_radio.h>
#endif
#ifndef SSL_BAT_DIV
#define SSL_BAT_DIV 2.0f
#endif
#ifndef SSL_LOW_BATT_MV
#define SSL_LOW_BATT_MV 3400    // ogniwo Li-ion: poniżej ok. 3,4 V zostaje kilka % pojemności
#endif
#ifndef SSL_ACK_TIMEOUT_MS
#define SSL_ACK_TIMEOUT_MS 40
#endif

namespace sslradio {

RTC_DATA_ATTR static uint16_t rtc_boot = 0;      // 0 = jeszcze nie odczytany z NVS
RTC_DATA_ATTR static uint16_t rtc_seq = 0;
RTC_DATA_ATTR static bool rtc_firstSent = false;

static Nrf905 g_radio;
static uint8_t g_node = 0;

struct Result {
  bool acked = false;       // odbiornik potwierdził
  bool radioOk = false;     // moduł odpowiedział na SPI
  uint8_t attempts = 0;     // ile razy próbowano nadać
  uint8_t txDone = 0;       // ile razy moduł potwierdził nadanie pinem DR
  uint32_t ms = 0;          // czas całej wysyłki
};

inline uint16_t bumpBoot() {
  Preferences p;
  p.begin("sslradio", false);
  uint16_t b = (uint16_t)(p.getUShort("boot", 0) + 1);
  if (b == 0) b = 1;
  p.putUShort("boot", b);
  p.end();
  return b;
}

// Włącza radio jako czujnik nr node. Wywoływane automatycznie przez send().
inline bool begin(uint8_t node) {
  if (rtc_boot == 0) { rtc_boot = bumpBoot(); rtc_seq = 0; }
  if (g_radio.ready() && g_node == node) return true;
  g_node = node;
  const uint8_t myAddr[4] = {'S', 'S', 'L', node};
  return g_radio.begin(myAddr);
}

// Napięcie baterii w mV (średnia z 16 odczytów), 0 = dzielnik niepodłączony.
inline uint16_t batteryMv() {
#if SSL_BAT_PIN < 0
  return 0;
#else
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) sum += analogReadMilliVolts(SSL_BAT_PIN);
  uint32_t mv = (uint32_t)((sum / 16) * SSL_BAT_DIV);
  return mv < 1500 ? 0 : (uint16_t)mv;
#endif
}

inline uint16_t nextSeq() {
  if (rtc_seq == 0xFFFF) { rtc_boot = bumpBoot(); rtc_seq = 0; }
  return ++rtc_seq;
}

// Wysyła ramkę i czeka na ACK. Uzupełnia sama: node, boot, seq, bat_mv, flagi baterii.
// maxAttempts = 0 -> domyślnie wg typu (alarm 6, podejrzane/info 3, żyje 2, test 1).
inline Result send(uint8_t node, ssl::Frame f, uint8_t maxAttempts = 0) {
  Result r;
  uint32_t t0 = millis();
  if (maxAttempts == 0) {
    maxAttempts = f.type == ssl::T_ALARM ? 6 :
                  (f.type == ssl::T_SUSPECT || f.type == ssl::T_INFO) ? 3 :
                  f.type == ssl::T_HEARTBEAT ? 2 : 1;
  }
  r.radioOk = begin(node);
  if (!r.radioOk) { r.ms = millis() - t0; return r; }

  f.node = node;
  f.boot = rtc_boot;
  f.seq = nextSeq();
  if (f.batt_mv == 0) f.batt_mv = batteryMv();
  if (f.batt_mv == 0) f.flags |= ssl::FLAG_NO_BATT_MEAS;
  else if (f.batt_mv < SSL_LOW_BATT_MV) f.flags |= ssl::FLAG_LOW_BATT;
  if (!rtc_firstSent) { f.flags |= ssl::FLAG_COLD_BOOT; rtc_firstSent = true; }

  const uint8_t gw[4] = {'S', 'S', 'L', ssl::GATEWAY_ID};
  uint8_t tx[ssl::FRAME_BYTES], rx[ssl::FRAME_BYTES];

  for (uint8_t a = 0; a < maxAttempts; a++) {
    if (a > 0) f.flags |= ssl::FLAG_RETRY;
    ssl::pack(f, SSL_KEY, tx);
    r.attempts++;
    if (!g_radio.send(tx, gw)) continue;          // brak DR: moduł nie potwierdził nadania
    r.txDone++;
    g_radio.startRx();
    uint32_t tw = millis();
    while (millis() - tw < SSL_ACK_TIMEOUT_MS) {
      if (!g_radio.available()) continue;
      g_radio.read(rx, true);
      ssl::Frame ack;
      if (ssl::unpack(rx, SSL_KEY, ack) == ssl::RES_OK && ack.type == ssl::T_ACK &&
          ack.node == node && ack.boot == f.boot && ack.seq == f.seq) {
        r.acked = true;
        break;
      }
    }
    g_radio.standby();
    if (r.acked) break;
    delay(random(20, 150));                       // losowe odstępy: mniej kolizji między czujnikami
  }
  r.ms = millis() - t0;
  return r;
}

// Tuż przed esp_deep_sleep_start(). Wyłącza moduł i zamraża jego piny.
inline void sleep() { g_radio.prepareDeepSleep(); }

inline uint16_t bootCount() { return rtc_boot; }

}  // namespace sslradio
