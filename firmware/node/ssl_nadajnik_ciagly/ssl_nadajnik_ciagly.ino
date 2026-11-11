/*
  SuperSecureLog – nadajnik ciągły do testu łącza.
  Co 0,5 s wysyła ramkę testową do odbiornika (szkic ssl_odbiornik) i przez 40 ms czeka na ACK.
  Bez akcelerometru, mikrofonu i snu – tylko radio. Serial Monitor 115200.

  Połączenia nRF905 jak w ssl_czuwanie (łącz według napisów na ESP):
    VCC 3V3, GND, PWR_UP 4, TRX_CE 16, TX_EN 17, CSN 5, SCK 18, MISO 19, MOSI 23, DR 27

  Odbiornik powinien wypisywać co 0,5 s linię {"ev":"test","wezel":99,...}.
*/
#include <Preferences.h>
#include "ssl_protocol.h"
#include "ssl_klucz.h"
#include "nrf905.h"

const uint8_t  NODE_ID   = 99;      // osobny numer, żeby nie mieszać liczników z czujnikiem nr 1
const uint32_t PERIOD_MS = 500;

Nrf905 radio;
uint16_t boot = 0, seq = 0;
uint32_t sent = 0, txOk = 0, acks = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Nadajnik ciagly (test lacza) ===");

  Preferences p;                      // licznik włączeń rośnie przy każdym starcie,
  p.begin("ciagly", false);           // więc odbiornik nie uzna ramek za powtórki
  boot = (uint16_t)(p.getUShort("boot", 0) + 1);
  p.putUShort("boot", boot);
  p.end();

  const uint8_t myAddr[4] = {'S', 'S', 'L', NODE_ID};
  while (!radio.begin(myAddr)) {
    Serial.println("nRF905 NIE ODPOWIADA (VCC 3V3, GND, CSN 5, SCK 18, MISO 19, MOSI 23, PWR_UP 4). Ponawiam...");
    delay(2000);
  }
  Serial.printf("nRF905 OK, wezel %u, boot %u. Wysylam co %lu ms.\n", NODE_ID, boot, (unsigned long)PERIOD_MS);
}

void loop() {
  uint32_t t0 = millis();
  ssl::Frame f;
  f.type = ssl::T_TEST;
  f.node = NODE_ID;
  f.boot = boot;
  f.seq = ++seq;
  f.flags = ssl::FLAG_TEST | ssl::FLAG_NO_BATT_MEAS;
  f.wakes = (uint16_t)sent;
  uint8_t tx[ssl::FRAME_BYTES], rx[ssl::FRAME_BYTES];
  ssl::pack(f, SSL_KEY, tx);

  const uint8_t gw[4] = {'S', 'S', 'L', ssl::GATEWAY_ID};
  uint32_t drMissBefore = radio.drPinMisses;
  bool ok = radio.send(tx, gw);
  sent++;
  if (ok) txOk++;
  const char* how = !ok ? "NIE (brak DR)" : (radio.drPinMisses > drMissBefore ? "TAK (tylko przez SPI - sprawdz przewod DR -> 27)" : "TAK");

  bool acked = false;
  if (ok) {
    radio.startRx();
    uint32_t tw = millis();
    while (millis() - tw < 40) {
      if (!radio.available()) continue;
      radio.read(rx, true);
      ssl::Frame a;
      if (ssl::unpack(rx, SSL_KEY, a) == ssl::RES_OK && a.type == ssl::T_ACK && a.node == NODE_ID && a.seq == f.seq) {
        acked = true;
        break;
      }
    }
    radio.standby();
  }
  if (acked) acks++;

  Serial.printf("#%lu seq %u: nadana %s, ACK %s   | razem nadane %lu/%lu, ACK %lu\n",
                (unsigned long)sent, seq, how, acked ? "TAK" : "nie",
                (unsigned long)txOk, (unsigned long)sent, (unsigned long)acks);

  while (millis() - t0 < PERIOD_MS) {}
}
