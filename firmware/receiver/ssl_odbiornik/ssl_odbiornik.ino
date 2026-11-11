// SuperSecureLog – odbiornik (bramka): ESP32 + nRF905, zasilany z USB komputera.
//
// Odbiera ramki z czujników, sprawdza HMAC i licznik (boot, seq), odsyła ACK
// i wypisuje każde zdarzenie jako JEDNĄ linię JSON na USB (115200 bodów).
//   linie zaczynające się od '{'  – dane dla serwera
//   linie zaczynające się od '#'  – diagnostyka dla człowieka
//
// Okablowanie nRF905 identyczne jak w czujniku (domyślne piny w nrf905.h).

#include <Preferences.h>
#include "ssl_protocol.h"
#include "ssl_klucz.h"
#include "nrf905.h"

// ======================================================= OKABLOWANIE nRF905 – wybierz jedno
//  0 = jak w czujniku:            TX_EN -> 17, TRX_CE -> 16, PWR_UP -> 4,  DR -> 27
//  1 = jak w przykladach nopnop2002/esp-idf-nrf905:
//                                 TX_EN -> 15, TRX_CE -> 17, PWR_UP -> 16, DR niepodlaczony
//  W obu: CSN 5, SCK 18, MISO 19, MOSI 23, VCC 3V3, GND.
#ifndef UKLAD_ODBIORNIKA
#define UKLAD_ODBIORNIKA 0
#endif

Nrf905Pins radioPins() {
  Nrf905Pins p;                                   // domyślnie układ 0
#if UKLAD_ODBIORNIKA == 1
  p.txen = 15; p.ce = 17; p.pwr = 16; p.dr = -1;
#endif
  return p;
}

Nrf905 radio;
Preferences prefs;

struct NodeState {
  bool loaded = false, known = false;
  uint16_t boot = 0, seq = 0;
  uint32_t received = 0, lost = 0, retries = 0, lastMs = 0;
};
NodeState nodes[256];

uint32_t badMac = 0, lastStatusMs = 0, amEdges = 0, frames = 0;
bool amPrev = false, drWarned = false;

void printHex(const uint8_t* b, size_t n) {
  for (size_t i = 0; i < n; i++) Serial.printf("%02x", b[i]);
}

void loadNode(uint8_t id) {
  NodeState& n = nodes[id];
  if (n.loaded) return;
  char kb[6], ks[6];
  snprintf(kb, sizeof kb, "b%u", id);
  snprintf(ks, sizeof ks, "s%u", id);
  n.known = prefs.isKey(kb);
  n.boot = prefs.getUShort(kb, 0);
  n.seq = prefs.getUShort(ks, 0);
  n.loaded = true;
}

void saveNode(uint8_t id) {
  char kb[6], ks[6];
  snprintf(kb, sizeof kb, "b%u", id);
  snprintf(ks, sizeof ks, "s%u", id);
  prefs.putUShort(kb, nodes[id].boot);
  prefs.putUShort(ks, nodes[id].seq);
}

void sendAck(const ssl::Frame& f) {
  ssl::Frame a;
  a.type = ssl::T_ACK;
  a.node = f.node;
  a.boot = f.boot;
  a.seq = f.seq;
  uint8_t buf[ssl::FRAME_BYTES];
  ssl::pack(a, SSL_KEY, buf);
  const uint8_t addr[4] = {'S', 'S', 'L', f.node};
  radio.send(buf, addr);
}

void printFlags(uint8_t fl) {
  const char* names[] = {"ponowienie", "niska_bateria", "test", "wlaczenie", "brak_pomiaru_baterii"};
  bool first = true;
  Serial.print('[');
  for (int i = 0; i < 5; i++) {
    if (fl & (1 << i)) {
      Serial.printf("%s\"%s\"", first ? "" : ",", names[i]);
      first = false;
    }
  }
  Serial.print(']');
}

void printEvent(const ssl::Frame& f, const uint8_t* raw, uint32_t lostNow, bool restart) {
  Serial.printf("{\"ev\":\"%s\",\"wezel\":%u,\"boot\":%u,\"seq\":%u,\"regula\":%u,\"opis\":\"%s\","
                "\"bat_mv\":%u,\"kat\":%.1f,\"drgania_ms\":%u,\"silnik_pct\":%u,\"pik_hz\":%u,"
                "\"pobudka\":\"%s\",\"pobudki\":%u,\"flagi\":",
                ssl::typeName(f.type), f.node, f.boot, f.seq, f.reason, ssl::reasonName(f.reason),
                f.batt_mv, f.tilt_ddeg / 10.0f, f.motion_ms, f.engine_pct, f.peak_hz,
                ssl::wakeName(f.wake), f.wakes);
  printFlags(f.flags);
  Serial.printf(",\"utracone\":%lu,\"restart\":%s,\"t_ms\":%lu,\"raw\":\"",
                (unsigned long)lostNow, restart ? "true" : "false", (unsigned long)millis());
  printHex(raw, ssl::FRAME_BYTES);
  Serial.println("\"}");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  prefs.begin("sslgw", false);
  const uint8_t myAddr[4] = {'S', 'S', 'L', ssl::GATEWAY_ID};
  while (!radio.begin(myAddr, radioPins())) {
    Serial.println("{\"ev\":\"blad\",\"opis\":\"nRF905 nie odpowiada - sprawdz zasilanie 3V3, CSN/SCK/MISO/MOSI, PWR_UP\"}");
    delay(2000);
  }
  radio.startRx();
  Nrf905Pins p = radioPins();
  Serial.printf("{\"ev\":\"start\",\"opis\":\"odbiornik gotowy, 433.2 MHz, uklad %d: TX_EN %d, TRX_CE %d, PWR_UP %d, DR %d\"}\n",
                UKLAD_ODBIORNIKA, p.txen, p.ce, p.pwr, p.dr);
}

void loop() {
  // 'R' w Serial Monitorze: zapomnij zapisane liczniki czujników. Potrzebne, gdy czujnik
  // dostał nową płytkę albo wyczyszczony flash (boot zaczyna się od 1 i odbiornik
  // odrzucałby jego ramki jako powtórki).
  if (Serial.available() && Serial.read() == 'R') {
    prefs.clear();
    for (auto& n : nodes) n = NodeState();
    Serial.println("{\"ev\":\"reset_licznikow\"}");
  }

  if (millis() - lastStatusMs > 20000) {          // co 20 s: czy moduł nadal odpowiada
    lastStatusMs = millis();
    bool ok = radio.verify();
    Serial.printf("# odbiornik zyje, radio %s, ramek: %lu, zle MAC: %lu, AM (pasujacy adres w eterze): %lu\n",
                  ok ? "ok" : "BLAD", (unsigned long)frames, (unsigned long)badMac, (unsigned long)amEdges);
    if (!ok) {
      const uint8_t myAddr[4] = {'S', 'S', 'L', ssl::GATEWAY_ID};
      radio.begin(myAddr, radioPins());
      radio.startRx();
    }
  }

  // AM = moduł słyszy pakiet z naszym adresem (jeszcze przed sprawdzeniem CRC).
  // AM bez późniejszych ramek = pakiety dochodzą, ale z błędami (zakłócenia, za daleko, inny kwarc).
  bool am = radio.status() & 0x80;
  if (am && !amPrev) amEdges++;
  amPrev = am;

  if (!radio.available()) return;
  if (radio.drPinMisses && !drWarned) {
    drWarned = true;
    Serial.println("# UWAGA: pakiet wykryty tylko przez SPI - przewod DR nie dochodzi do G27 (lewa strona ESP). Odbieram mimo to.");
  }
  frames++;

  uint8_t raw[ssl::FRAME_BYTES];
  radio.read(raw, false);                         // zostajemy w standby do czasu odpowiedzi
  ssl::Frame f;
  ssl::Result res = ssl::unpack(raw, SSL_KEY, f);

  if (res != ssl::RES_OK) {
    radio.startRx();
    badMac++;
    Serial.printf("{\"ev\":\"odrzucona\",\"powod\":\"%s\",\"raw\":\"",
                  res == ssl::RES_BAD_VERSION ? "wersja" : "mac");
    printHex(raw, ssl::FRAME_BYTES);
    Serial.println("\"}");
    return;
  }
  if (f.type == ssl::T_ACK || f.node == ssl::GATEWAY_ID) { radio.startRx(); return; }

  NodeState& n = nodes[f.node];
  loadNode(f.node);
  int cmp = n.known ? ssl::compareCounter(f.boot, f.seq, n.boot, n.seq) : 1;

  if (cmp < 0) {                                  // stara ramka: powtórka (atak) – bez ACK
    radio.startRx();
    Serial.printf("{\"ev\":\"powtorka_odrzucona\",\"wezel\":%u,\"boot\":%u,\"seq\":%u,\"ostatni_boot\":%u,\"ostatni_seq\":%u}\n",
                  f.node, f.boot, f.seq, n.boot, n.seq);
    return;
  }

  sendAck(f);                                     // ACK najpierw – czujnik czeka tylko 40 ms
  radio.startRx();

  if (cmp == 0) {                                 // ta sama ramka – nasz ACK nie doszedł
    n.retries++;
    Serial.printf("# wezel %u seq %u: ponowienie, ACK wyslany jeszcze raz\n", f.node, f.seq);
    return;
  }

  uint32_t lostNow = 0;
  bool restart = n.known && f.boot != n.boot;
  if (n.known && f.boot == n.boot && f.seq > n.seq + 1) lostNow = f.seq - n.seq - 1;
  n.lost += lostNow;
  n.received++;
  n.known = true;
  n.boot = f.boot;
  n.seq = f.seq;
  n.lastMs = millis();
  saveNode(f.node);

  printEvent(f, raw, lostNow, restart);
}
