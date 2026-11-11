/*
  SuperSecureLog – surowy test radia nRF905 (bez podpisu, bez protokołu).
  Ten sam szkic na OBIE płytki. Serial Monitor 115200, wysyłaj znaki z klawiatury:

    t   ta płytka NADAJE (co 0,3 s pakiet "SSLTEST nnnn")
    r   ta płytka ODBIERA (domyślnie) – wypisuje każdy pakiet i co 1 s liczniki
    k   przełącz kanał:  433,2 MHz (jak nasz kod)  <->  423,4 MHz (jak nopnop2002)
    a   przełącz adres:  "S S L 00" (jak nasz kod) <->  E7 E7 E7 E7 (jak nopnop2002)
    p   wypisz rejestr konfiguracji i stan modułu
    ?   pomoc

  Kanał i adres muszą być ustawione TAK SAMO na obu płytkach.
  Okablowanie jak w ssl_czuwanie: TX_EN TX2(17), TRX_CE RX2(16), PWR_UP D4, DR D27,
  CSN D5, SCK D18, MISO D19, MOSI D23, VCC 3V3, GND.
*/
#include "nrf905.h"

Nrf905 radio;
bool txRole = false, chanNopnop = false, addrNopnop = false;
const uint8_t ADDR_SSL[4] = {'S', 'S', 'L', 0x00};
const uint8_t ADDR_E7[4]  = {0xE7, 0xE7, 0xE7, 0xE7};
uint32_t txCount = 0, txDr = 0, rxCount = 0, amEdges = 0, lastPrint = 0, lastTx = 0;
bool amPrev = false;

const uint8_t* addr() { return addrNopnop ? ADDR_E7 : ADDR_SSL; }

void apply() {
  Nrf905Config c;
  c.channel = chanNopnop ? 10 : 108;
  bool ok = radio.begin(addr(), Nrf905Pins(), c);
  Serial.printf("\n>> %s | kanal %s | adres %s | nRF905 %s\n",
                txRole ? "NADAJE" : "ODBIERA",
                chanNopnop ? "10 (423,4 MHz)" : "108 (433,2 MHz)",
                addrNopnop ? "E7 E7 E7 E7" : "S S L 00",
                ok ? "OK" : "NIE ODPOWIADA (sprawdz VCC, GND, SPI)");
  if (!txRole) radio.startRx();
  txCount = txDr = rxCount = amEdges = 0;
}

void printConfig() {
  uint8_t c[10];
  radio.readConfig(c);
  Serial.print(">> konfiguracja:");
  for (int i = 0; i < 10; i++) Serial.printf(" %02X", c[i]);
  uint8_t st = radio.status();
  Serial.printf("\n>> status 0x%02X: DR=%d AM=%d, pin DR=%d\n", st, (st >> 5) & 1, (st >> 7) & 1, radio.drPin());
  Serial.println(">> oczekiwane: 6C 0C 44 20 20 53 53 4C 00 D8 (nasze) albo 0A 0C 44 20 20 E7 E7 E7 E7 D8 (nopnop)");
}

void help() {
  Serial.println("\nt = nadawaj, r = odbieraj, k = kanal, a = adres, p = konfiguracja, ? = pomoc");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== test_radio ===");
  help();
  apply();
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 't': txRole = true;  apply(); break;
      case 'r': txRole = false; apply(); break;
      case 'k': chanNopnop = !chanNopnop; apply(); break;
      case 'a': addrNopnop = !addrNopnop; apply(); break;
      case 'p': printConfig(); break;
      case '?': help(); break;
      default: break;
    }
  }

  if (txRole) {
    if (millis() - lastTx >= 300) {
      lastTx = millis();
      uint8_t buf[Nrf905::PAYLOAD] = {0};
      snprintf((char*)buf, sizeof buf, "SSLTEST %04lu", (unsigned long)(txCount % 10000));
      bool ok = radio.send(buf, addr());
      txCount++;
      if (ok) txDr++;
      Serial.printf("TX #%lu %s  (zakonczone %lu/%lu)\n", (unsigned long)txCount, ok ? "nadany" : "BRAK DR",
                    (unsigned long)txDr, (unsigned long)txCount);
    }
    return;
  }

  bool am = radio.status() & 0x80;
  if (am && !amPrev) amEdges++;
  amPrev = am;
  if (radio.available()) {
    uint8_t buf[Nrf905::PAYLOAD];
    radio.read(buf, true);
    rxCount++;
    buf[31] = 0;
    Serial.printf("RX #%lu: \"%s\"\n", (unsigned long)rxCount, (const char*)buf);
  }
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    Serial.printf("   odebrane %lu, AM (adres pasuje) %lu\n", (unsigned long)rxCount, (unsigned long)amEdges);
  }
}
