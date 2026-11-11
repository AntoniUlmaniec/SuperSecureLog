/*
  SuperSecureLog – gdzie jest akcelerometr? (bez multimetru)
  Serial Monitor 115200, potem EN. Nic nie trzeba przepinać.

  A. Który pin ma stan wysoki z zewnątrz. Linie SDA i SCL modułu mają na płytce modułu
     rezystory podciągające do jego VCC – jeśli moduł jest zasilony i przewód SDA/SCL
     dochodzi do ESP, ten pin pokaże 1 mimo podciągania w dół po stronie ESP.
  B. Próba rozmowy I2C na każdej parze pinów – znajduje, gdzie naprawdę są SDA i SCL.

  Pomijane: G0, G2, G12, G15 (piny startowe), G6–G11 (pamięć flash), G1/G3 (USB),
  G34–G39 (tylko wejścia).
*/
#include <Wire.h>

const int PINS[] = {4, 5, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
const int NP = sizeof(PINS) / sizeof(PINS[0]);
const uint8_t ADDRS[] = {0x18, 0x19, 0x1D, 0x1E};

bool probe(int sda, int scl, uint8_t& foundAddr, int& who) {
  Wire.begin(sda, scl, 100000);
  Wire.setTimeOut(10);
  bool ok = false;
  for (uint8_t a : ADDRS) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      foundAddr = a;
      Wire.beginTransmission(a);
      Wire.write(0x0F);
      who = -1;
      if (Wire.endTransmission(false) == 0 && Wire.requestFrom((int)a, 1) == 1) who = Wire.read();
      ok = true;
      break;
    }
  }
  Wire.end();
  pinMode(sda, INPUT_PULLDOWN);
  pinMode(scl, INPUT_PULLDOWN);
  return ok;
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== Szukam akcelerometru ===");

  // A
  Serial.println("A. Piny ze stanem wysokim z zewnatrz (ESP ciagnie je w dol):");
  int highs[NP], nh = 0;
  for (int i = 0; i < NP; i++) pinMode(PINS[i], INPUT_PULLDOWN);
  delay(20);
  for (int i = 0; i < NP; i++) {
    if (digitalRead(PINS[i])) {
      highs[nh++] = PINS[i];
      Serial.printf("   G%d = 1\n", PINS[i]);
    }
  }
  if (nh == 0) Serial.println("   zaden. Jesli G21 i G22 tu nie ma, modul jest bez zasilania albo SDA/SCL nie dochodza do ESP.");
  bool sdaHigh = false, sclHigh = false;
  for (int i = 0; i < nh; i++) { if (highs[i] == 21) sdaHigh = true; if (highs[i] == 22) sclHigh = true; }
  Serial.printf("   -> G21 (SDA): %s, G22 (SCL): %s\n", sdaHigh ? "OK, modul podciaga" : "BRAK", sclHigh ? "OK, modul podciaga" : "BRAK");

  // B
  Serial.println("B. Proba I2C na parach pinow (moze potrwac do ok. 30 s)...");
  int found = 0;
  for (int i = 0; i < NP; i++) {
    for (int j = 0; j < NP; j++) {
      if (i == j) continue;
      uint8_t a; int who;
      if (probe(PINS[i], PINS[j], a, who)) {
        found++;
        Serial.printf("   ZNALEZIONO: SDA = G%d, SCL = G%d, adres 0x%02X, WHO_AM_I = 0x%02X%s\n",
                      PINS[i], PINS[j], a, who & 0xFF,
                      who == 0x33 ? " (LIS3DH)" : who == 0x3F ? " (LIS3DSH)" : "");
      }
    }
  }
  if (!found) {
    Serial.println("   Nigdzie nie odpowiada -> modul NIE MA ZASILANIA (VCC/GND) albo CS nie jest na 3,3 V.");
  } else {
    Serial.println("   Szkice SSL oczekuja SDA = G21, SCL = G22. Jesli wyszlo inaczej, przepnij przewody.");
  }
  Serial.println("=== koniec ===");
}

void loop() {}
