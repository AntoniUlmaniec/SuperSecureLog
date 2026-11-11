/*
  SuperSecureLog – diagnostyka akcelerometru LIS3DH (ten sam układ co w ssl_czuwanie)
  Płytka: ESP32 Dev Module, Serial Monitor 115200. Połączenia jak w ssl_czuwanie:
    SDA G21, SCL G22, INT1 G33, CS -> 3,3 V, SDO -> GND.

  Kroki (wszystko wypisuje się samo, wystarczy patrzeć):
    1. skan magistrali I2C
    2. WHO_AM_I
    3. zapis rejestrów i odczyt kontrolny (czy zapisy w ogóle działają)
    4. 10 odczytów wprost (bez kolejki FIFO)
    5. kolejka FIFO – tak jak czyta ją ssl_czuwanie
    6. przerwanie ruchu na INT1 – 30 s na stukanie i przechylanie
    7. potem ciągły podgląd X/Y/Z
*/
#include <Wire.h>
#include <math.h>

const int PIN_SDA = 21, PIN_SCL = 22, PIN_INT1 = 33;
const uint32_t I2C_HZ = 400000;   // jak w ssl_czuwanie; przy błędach spróbuj 100000
uint8_t addr = 0;

int rd(uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((int)addr, 1) != 1) return -1;
  return Wire.read();
}

bool wr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool wrCheck(uint8_t reg, uint8_t val, const char* name) {
  bool ack = wr(reg, val);
  int back = rd(reg);
  bool ok = ack && back == val;
  Serial.printf("  %-14s 0x%02X <- 0x%02X : %s (odczyt 0x%02X)\n", name, reg, val,
                ok ? "OK" : (ack ? "ZAPIS NIE ZOSTAL" : "BRAK ACK PRZY ZAPISIE"), back < 0 ? 0xEE : back);
  return ok;
}

bool xyz(float& x, float& y, float& z) {
  Wire.beginTransmission(addr);
  Wire.write(0x28 | 0x80);                       // autoinkrementacja
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 6) != 6) return false;
  int16_t r[3];
  for (int i = 0; i < 3; i++) {
    uint8_t lo = Wire.read(), hi = Wire.read();
    r[i] = (int16_t)((hi << 8) | lo);
  }
  const float k = 0.001f / 16.0f;                // ±2 g, tryb wysokiej rozdzielczości
  x = r[0] * k; y = r[1] * k; z = r[2] * k;
  return true;
}

void printXYZ(const char* prefix) {
  float x, y, z;
  if (xyz(x, y, z))
    Serial.printf("%sX=%6.3f  Y=%6.3f  Z=%6.3f g   |a|=%.3f\n", prefix, x, y, z, sqrtf(x * x + y * y + z * z));
  else
    Serial.printf("%sblad odczytu I2C\n", prefix);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  pinMode(PIN_INT1, INPUT);
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Serial.println("\n=== Diagnostyka LIS3DH ===");

  // 1. skan
  Serial.print("1. Urzadzenia I2C:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "" : " BRAK – sprawdz SDA/SCL, VCC i GND akcelerometru");

  // 2. WHO_AM_I
  const uint8_t cands[] = {0x18, 0x19};
  for (uint8_t a : cands) {
    addr = a;
    if (rd(0x0F) == 0x33) break;
    addr = 0;
  }
  if (!addr) { Serial.println("2. WHO_AM_I: nie znaleziono LIS3DH (0x33) pod 0x18/0x19. Koniec."); return; }
  Serial.printf("2. WHO_AM_I = 0x33 pod adresem 0x%02X (LIS3DH)\n", addr);

  // 3. zapis i odczyt kontrolny
  Serial.println("3. Zapis rejestrow:");
  bool w = true;
  w &= wrCheck(0x20, 0x57, "CTRL_REG1");         // 100 Hz, XYZ
  w &= wrCheck(0x23, 0x88, "CTRL_REG4");         // BDU, ±2 g, HR
  w &= wrCheck(0x24, 0x00, "CTRL_REG5");         // FIFO off
  if (!w) Serial.println("   -> zapisy nie dzialaja: sprawdz przewody, sprobuj I2C_HZ = 100000");

  // 4. odczyty wprost
  Serial.println("4. Odczyty wprost (plytka lezy plasko -> jedna os ok. +-1 g, |a| ok. 1):");
  delay(50);
  for (int i = 0; i < 10; i++) {
    uint32_t t0 = millis();
    while (!(rd(0x27) & 0x08) && millis() - t0 < 100) {}   // STATUS_REG: nowe dane
    printXYZ("   ");
    delay(100);
  }

  // 5. FIFO tak jak w ssl_czuwanie
  Serial.println("5. Kolejka FIFO:");
  wrCheck(0x24, 0x40, "CTRL_REG5 FIFO");
  wr(0x2E, 0x00);
  wrCheck(0x2E, 0x80, "FIFO_CTRL str.");
  delay(200);                                    // 100 Hz -> ok. 20 probek
  int src = rd(0x2F);
  Serial.printf("   FIFO_SRC = 0x%02X: pusta=%d, przepelniona=%d, probek=%d (oczekiwane ok. 20)\n",
                src, (src >> 5) & 1, (src >> 6) & 1, src & 0x1F);
  int n = (src >= 0 && !(src & 0x20)) ? (src & 0x1F) + ((src & 0x40) ? 1 : 0) : 0;
  for (int i = 0; i < n; i++) {
    float x, y, z;
    xyz(x, y, z);
    if (i == 0) Serial.printf("   pierwsza probka z FIFO: X=%.3f Y=%.3f Z=%.3f\n", x, y, z);
  }
  Serial.printf("   odczytano %d probek z FIFO\n", n);

  // 6. przerwanie ruchu – identycznie jak tryb czuwania w ssl_czuwanie
  Serial.println("6. Przerwanie ruchu na INT1 (prog 0,1 g). STUKAJ i PRZECHYLAJ przez 30 s:");
  wr(0x2E, 0x00);
  wrCheck(0x24, 0x08, "CTRL_REG5 LIR");          // zatrzaskiwanie INT1
  wrCheck(0x20, 0x2F, "CTRL_REG1 10Hz");         // 10 Hz, niski pobór
  wrCheck(0x21, 0x01, "CTRL_REG2 HP");           // filtr górnoprzepustowy dla przerwania 1
  wrCheck(0x23, 0x80, "CTRL_REG4");
  wrCheck(0x25, 0x00, "CTRL_REG6");
  wrCheck(0x32, 6, "INT1_THS");                  // 6 x 16 mg = 0,1 g
  wrCheck(0x33, 0x00, "INT1_DURATION");
  rd(0x26);                                      // REFERENCE: zeruje filtr
  wrCheck(0x30, 0x2A, "INT1_CFG");               // X/Y/Z powyżej progu
  wrCheck(0x22, 0x40, "CTRL_REG3");              // przerwanie 1 na pin INT1
  delay(300);
  rd(0x31);                                      // skasuj ewentualne stare zdarzenie
  Serial.printf("   INT1 (G33) w spoczynku = %d (powinno byc 0)\n", digitalRead(PIN_INT1));
  int events = 0, pinHigh = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 30000) {
    int pin = digitalRead(PIN_INT1);             // zatrzask trzyma pin w stanie 1 do odczytu INT1_SRC
    int s = rd(0x31);                            // INT1_SRC – odczyt kasuje zatrzask
    if (s >= 0 && (s & 0x40)) {
      events++;
      if (pin) pinHigh++;
      Serial.printf("   %5.1f s: RUCH (INT1_SRC 0x%02X), pin G33 = %d\n", (millis() - t0) / 1000.0f, s, pin);
      delay(150);
    }
    delay(20);
  }
  Serial.printf("   Koniec: akcelerometr zglosil %d zdarzen, pin G33 widzial %d z nich.\n", events, pinHigh);
  if (events == 0) Serial.println("   -> brak zdarzen: akcelerometr nie wykrywa ruchu (patrz kroki 3-4)");
  else if (pinHigh == 0) Serial.println("   -> zdarzenia sa, ale nie docieraja do G33: sprawdz przewod INT1 (g29 -> j52)");
  else Serial.println("   -> przerwanie dziala");

  // 7. podgląd
  wr(0x22, 0x00); wr(0x30, 0x00); wr(0x21, 0x00);
  wr(0x20, 0x57); wr(0x23, 0x88);
  Serial.println("7. Podglad na zywo (co 0,3 s). Przechyl plytke – wartosci musza sie zmieniac:");
}

void loop() {
  if (!addr) return;
  printXYZ("   ");
  delay(300);
}
