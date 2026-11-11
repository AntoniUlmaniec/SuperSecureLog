// SuperSecureLog – format ramki radiowej (wersja 1)
//
// Jedna ramka = dokładnie 32 bajty = pełny ładunek nRF905.
// Liczby wielobajtowe: little-endian (najpierw młodszy bajt).
//
//  bajt  pole        typ    opis
//  0     ver_typ     u8     starsze 4 bity: wersja (1), młodsze 4 bity: typ ramki
//  1     wezel       u8     numer czujnika 1..254 (0 = odbiornik)
//  2-3   boot        u16    licznik włączeń czujnika (NVS, rośnie po wymianie baterii/resecie)
//  4-5   seq         u16    numer ramki od włączenia; (boot, seq) chroni przed powtórką
//  6     regula      u8     numer reguły z tabeli decide() (1..7), 0 = nie dotyczy
//  7     flagi       u8     F_* niżej
//  8-9   bat_mv      u16    napięcie baterii w mV, 0 = brak pomiaru
//  10-11 kat         i16    zmiana położenia względem spoczynku, w 0,1°
//  12-13 drgania_ms  u16    łączny czas drgań w oknie 5 s, ms
//  14    silnik_pct  u8     % ramek dźwięku uznanych za silnik
//  15    pobudka     u8     przyczyna pobudki W_*
//  16-17 pik_hz      u16    częstotliwość piku silnika, Hz (0 = brak)
//  18-19 pobudki     u16    liczba pobudek od włączenia
//  20-23 extra       4×u8   zarezerwowane (zera)
//  24-31 mac         8×u8   pierwsze 8 bajtów HMAC-SHA256(klucz, bajty 0..23)
//
// Ramka ACK (odbiornik -> czujnik): typ = T_ACK, wezel/boot/seq = kopia
// potwierdzanej ramki, reszta zera, ten sam MAC.
//
// Plik nie zależy od Arduino (kompiluje się też na PC z mbedtls).
// Nazwy FLAG_* i RES_* celowo: F_TEST i OK są już zajęte przez rdzeń ESP32 (unistd.h, ets_sys.h).

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "mbedtls/md.h"

namespace ssl {

constexpr uint8_t PROTO_VER  = 1;
constexpr uint8_t FRAME_BYTES  = 32;
constexpr uint8_t MAC_BYTES    = 8;
constexpr uint8_t MAC_POS = FRAME_BYTES - MAC_BYTES;   // 24
constexpr uint8_t KEY_BYTES    = 16;
constexpr uint8_t GATEWAY_ID = 0;

enum Type : uint8_t {
  T_ALARM     = 1,   // kradzież (reguły 1–3)
  T_SUSPECT   = 2,   // podejrzane (reguły 4–5)
  T_INFO      = 3,   // silnik w pobliżu (reguła 6)
  T_HEARTBEAT = 4,   // "jestem żywy"
  T_TEST      = 5,   // test zasięgu
  T_ACK       = 8,   // potwierdzenie od odbiornika
};

enum Flag : uint8_t {
  FLAG_RETRY        = 0x01,  // ponowienie tej samej ramki (to samo seq)
  FLAG_LOW_BATT     = 0x02,  // bateria poniżej progu
  FLAG_TEST         = 0x04,  // ramka ze szkicu testowego – serwer może ją pominąć
  FLAG_COLD_BOOT    = 0x08,  // pierwsza ramka po włączeniu zasilania / resecie
  FLAG_NO_BATT_MEAS = 0x10,  // dzielnik baterii niepodłączony
};

enum Wake : uint8_t {
  W_UNKNOWN = 0, W_MOTION = 1, W_LISTEN_TIMER = 2, W_CHECK_TIMER = 3, W_POWER_ON = 4,
};

struct Frame {
  uint8_t  type       = 0;
  uint8_t  node       = 0;
  uint16_t boot       = 0;
  uint16_t seq        = 0;
  uint8_t  reason     = 0;
  uint8_t  flags      = 0;
  uint16_t batt_mv    = 0;
  int16_t  tilt_ddeg  = 0;
  uint16_t motion_ms  = 0;
  uint8_t  engine_pct = 0;
  uint8_t  wake       = 0;
  uint16_t peak_hz    = 0;
  uint16_t wakes      = 0;
  uint8_t  extra[4]   = {0, 0, 0, 0};
};

enum Result : uint8_t { RES_OK = 0, RES_BAD_VERSION = 1, RES_BAD_MAC = 2, RES_HMAC_ERROR = 3 };

inline void put16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline uint16_t get16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

inline bool hmac8(const uint8_t key[KEY_BYTES], const uint8_t* data, size_t n, uint8_t out[MAC_BYTES]) {
  uint8_t full[32];
  const mbedtls_md_info_t* md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  if (md == nullptr || mbedtls_md_hmac(md, key, KEY_BYTES, data, n, full) != 0) return false;
  memcpy(out, full, MAC_BYTES);
  return true;
}

// Zwraca false tylko przy błędzie biblioteki kryptograficznej.
inline bool pack(const Frame& f, const uint8_t key[KEY_BYTES], uint8_t out[FRAME_BYTES]) {
  memset(out, 0, FRAME_BYTES);
  out[0] = (uint8_t)((PROTO_VER << 4) | (f.type & 0x0F));
  out[1] = f.node;
  put16(out + 2, f.boot);
  put16(out + 4, f.seq);
  out[6] = f.reason;
  out[7] = f.flags;
  put16(out + 8, f.batt_mv);
  put16(out + 10, (uint16_t)f.tilt_ddeg);
  put16(out + 12, f.motion_ms);
  out[14] = f.engine_pct;
  out[15] = f.wake;
  put16(out + 16, f.peak_hz);
  put16(out + 18, f.wakes);
  memcpy(out + 20, f.extra, 4);
  return hmac8(key, out, MAC_POS, out + MAC_POS);
}

inline Result unpack(const uint8_t in[FRAME_BYTES], const uint8_t key[KEY_BYTES], Frame& f) {
  if ((in[0] >> 4) != PROTO_VER) return RES_BAD_VERSION;
  uint8_t mac[MAC_BYTES];
  if (!hmac8(key, in, MAC_POS, mac)) return RES_HMAC_ERROR;
  uint8_t diff = 0;                                  // porównanie w stałym czasie
  for (uint8_t i = 0; i < MAC_BYTES; i++) diff |= (uint8_t)(mac[i] ^ in[MAC_POS + i]);
  if (diff != 0) return RES_BAD_MAC;
  f.type       = in[0] & 0x0F;
  f.node       = in[1];
  f.boot       = get16(in + 2);
  f.seq        = get16(in + 4);
  f.reason     = in[6];
  f.flags      = in[7];
  f.batt_mv    = get16(in + 8);
  f.tilt_ddeg  = (int16_t)get16(in + 10);
  f.motion_ms  = get16(in + 12);
  f.engine_pct = in[14];
  f.wake       = in[15];
  f.peak_hz    = get16(in + 16);
  f.wakes      = get16(in + 18);
  memcpy(f.extra, in + 20, 4);
  return RES_OK;
}

// -1 = starsza niż (lastBoot, lastSeq), 0 = ta sama, 1 = nowsza
inline int compareCounter(uint16_t boot, uint16_t seq, uint16_t lastBoot, uint16_t lastSeq) {
  uint32_t a = ((uint32_t)boot << 16) | seq, b = ((uint32_t)lastBoot << 16) | lastSeq;
  return a < b ? -1 : (a == b ? 0 : 1);
}

inline const char* typeName(uint8_t t) {
  switch (t) {
    case T_ALARM: return "alarm";
    case T_SUSPECT: return "podejrzane";
    case T_INFO: return "info";
    case T_HEARTBEAT: return "zyje";
    case T_TEST: return "test";
    case T_ACK: return "ack";
    default: return "nieznany";
  }
}

// Numery reguł = wiersze tabeli decide() z dokumentu o algorytmie czuwania.
inline const char* reasonName(uint8_t r) {
  switch (r) {
    case 0: return "";
    case 1: return "kat >= 15 st.";
    case 2: return "drgania >= 2 s i silnik";
    case 3: return "drgania >= 2 s trzeci raz z rzedu";
    case 4: return "drgania >= 2 s bez silnika";
    case 5: return "kat 5-15 st.";
    case 6: return "silnik w poblizu";
    case 7: return "nic";
    default: return "?";
  }
}

inline const char* wakeName(uint8_t w) {
  switch (w) {
    case W_MOTION: return "ruch";
    case W_LISTEN_TIMER: return "nasluch";
    case W_CHECK_TIMER: return "kontrola";
    case W_POWER_ON: return "wlaczenie";
    default: return "?";
  }
}

}  // namespace ssl
