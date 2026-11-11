// Minimalny sterownik nRF905 dla ESP32 (Arduino), tryb ShockBurst, ładunek 32 B.
//
// Domyślne piny = schemat z dokumentu "montaż na płytce stykowej" (21.10.2026):
//   CSN 5, SCK 18, MISO 19, MOSI 23, TRX_CE 16, TX_EN 17, PWR_UP 4, DR 27
//   (uCLK, CD, AM niepodłączone; rezystor na PWR_UP niepotrzebny)
//
// Ustawienia radia: 433,2 MHz (kanał 108), +10 dBm, adres 4 B, CRC16,
// kwarc 16 MHz. Jeśli na kwarcu modułu jest 20.000 -> xof = 4.

#pragma once
#include <Arduino.h>
#include <SPI.h>
#include "driver/gpio.h"
#include "driver/rtc_io.h"

struct Nrf905Pins {
  int8_t csn = 5, ce = 16, txen = 17, pwr = 4, dr = 27;
  int8_t sck = 18, miso = 19, mosi = 23;
};

struct Nrf905Config {
  uint16_t channel = 108;    // f = (422,4 + kanał/10) × (1 + hfreq) MHz -> 433,2 MHz
  bool     hfreq   = false;  // true dla modułów 868/915 MHz
  uint8_t  paPwr   = 3;      // 0: -10 dBm, 1: -2 dBm, 2: +6 dBm, 3: +10 dBm
  uint8_t  xof     = 3;      // kwarc: 0=4, 1=8, 2=12, 3=16, 4=20 MHz
  uint32_t spiHz   = 4000000;
};

class Nrf905 {
 public:
  static constexpr uint8_t PAYLOAD = 32;

  // Włącza moduł, wpisuje konfigurację i sprawdza ją odczytem.
  // false = moduł nie odpowiada albo odczyt się nie zgadza (okablowanie SPI/zasilanie).
  bool begin(const uint8_t rxAddr[4], const Nrf905Pins& pins = Nrf905Pins(),
             const Nrf905Config& cfg = Nrf905Config()) {
    p_ = pins;
    releaseHolds();
    pinMode(p_.csn, OUTPUT);  digitalWrite(p_.csn, HIGH);
    pinMode(p_.ce, OUTPUT);   digitalWrite(p_.ce, LOW);
    pinMode(p_.txen, OUTPUT); digitalWrite(p_.txen, LOW);
    pinMode(p_.pwr, OUTPUT);  digitalWrite(p_.pwr, LOW);
    pinMode(p_.dr, INPUT);
    SPI.begin(p_.sck, p_.miso, p_.mosi, -1);
    spi_ = SPISettings(cfg.spiHz, MSBFIRST, SPI_MODE0);
    spiOn_ = true;

    digitalWrite(p_.pwr, HIGH);   // power-down -> standby
    delay(4);                     // start kwarcu: 3 ms wg noty

    conf_[0] = (uint8_t)(cfg.channel & 0xFF);
    conf_[1] = (uint8_t)(((cfg.paPwr & 3) << 2) | (cfg.hfreq ? 0x02 : 0) | ((cfg.channel >> 8) & 1));
    conf_[2] = 0x44;              // adres TX i RX: 4 bajty
    conf_[3] = PAYLOAD;           // RX_PW
    conf_[4] = PAYLOAD;           // TX_PW
    memcpy(conf_ + 5, rxAddr, 4); // własny adres odbiorczy
    conf_[9] = (uint8_t)(0xC0 | ((cfg.xof & 7) << 3));  // CRC16 włączone, bez wyjścia zegara
    cmd(W_CONFIG, conf_, nullptr, 10);
    ready_ = verify();
    return ready_;
  }

  // Ponowny odczyt rejestru konfiguracji (test okablowania SPI).
  bool verify() {
    uint8_t rd[10];
    cmd(R_CONFIG, nullptr, rd, 10);
    return memcmp(rd, conf_, 10) == 0;
  }

  bool ready() const { return ready_; }

  // Rejestr stanu nRF905 (przychodzi przez SPI przy każdej komendzie): bit 5 = DR, bit 7 = AM.
  // Pozwala wykryć odebrany pakiet nawet przy niedziałającym przewodzie DR.
  uint8_t status() { return cmd(R_CONFIG, nullptr, nullptr, 0); }
  bool drPin() { return digitalRead(p_.dr) == HIGH; }
  uint32_t drPinMisses = 0;   // ile razy DR był w rejestrze, a pin DR milczał (zły przewód DR)

  // Wysyła 32 B na adres txAddr. Wraca do standby. false = brak DR w czasie (moduł nie nadał).
  bool send(const uint8_t buf[PAYLOAD], const uint8_t txAddr[4], uint32_t timeoutMs = 30) {
    digitalWrite(p_.ce, LOW);
    if (drPin() || (status() & 0x20)) {           // nieodebrany pakiet blokowałby DR
      uint8_t junk[PAYLOAD];
      cmd(R_RX_PAYLOAD, nullptr, junk, PAYLOAD);
    }
    cmd(W_TX_ADDRESS, txAddr, nullptr, 4);
    cmd(W_TX_PAYLOAD, buf, nullptr, PAYLOAD);
    digitalWrite(p_.txen, HIGH);
    digitalWrite(p_.ce, HIGH);                    // start nadawania (ok. 0,65 ms + 6 ms w eterze)
    uint32_t t0 = millis();
    bool ok = false;
    while (millis() - t0 < timeoutMs) {
      if (drPin()) { ok = true; break; }
      if (status() & 0x20) { if (!drPin()) drPinMisses++; ok = true; break; }
    }
    digitalWrite(p_.ce, LOW);                     // koniec: standby
    digitalWrite(p_.txen, LOW);
    return ok;
  }

  // Nasłuch na własnym adresie.
  void startRx() {
    digitalWrite(p_.ce, LOW);
    digitalWrite(p_.txen, LOW);
    digitalWrite(p_.ce, HIGH);
  }

  // DR = odebrano pakiet z poprawnym adresem i CRC.
  bool available() {
    if (drPin()) return true;
    if (status() & 0x20) { if (!drPin()) drPinMisses++; return true; }
    return false;
  }

  // Odczyt ładunku. resumeRx = po odczycie od razu wrócić do nasłuchu.
  void read(uint8_t buf[PAYLOAD], bool resumeRx) {
    digitalWrite(p_.ce, LOW);
    cmd(R_RX_PAYLOAD, nullptr, buf, PAYLOAD);
    if (resumeRx) digitalWrite(p_.ce, HIGH);
  }

  void standby() {
    digitalWrite(p_.ce, LOW);
    digitalWrite(p_.txen, LOW);
  }

  void powerDown() {
    standby();
    digitalWrite(p_.pwr, LOW);
  }

  // Wywołać tuż przed esp_deep_sleep_start(). Wyłącza moduł (ok. 2,5 µA) i trzyma
  // PWR_UP w stanie niskim przez cały sen (GPIO4 to pin RTC, więc da się go zablokować
  // pojedynczo, tak jak zasilanie mikrofonu). Działa też bez begin().
  // UWAGA: nie używamy gpio_deep_sleep_hold_en() – na ESP32 zamraża WSZYSTKIE zwykłe piny,
  // także I2C akcelerometru, i potrafią zostać zamrożone po pobudce.
  void prepareDeepSleep(const Nrf905Pins& pins = Nrf905Pins()) {
    if (!spiOn_) p_ = pins;
    if (spiOn_) { SPI.end(); spiOn_ = false; }
    releaseHolds();
    pinMode(p_.ce, OUTPUT);   digitalWrite(p_.ce, LOW);
    pinMode(p_.txen, OUTPUT); digitalWrite(p_.txen, LOW);
    pinMode(p_.pwr, OUTPUT);  digitalWrite(p_.pwr, LOW);
    pinMode(p_.csn, OUTPUT);  digitalWrite(p_.csn, HIGH);
    gpio_num_t pw = (gpio_num_t)p_.pwr;
    if (rtc_gpio_is_valid_gpio(pw)) {
      rtc_gpio_init(pw);
      rtc_gpio_set_direction(pw, RTC_GPIO_MODE_OUTPUT_ONLY);
      rtc_gpio_set_level(pw, 0);
      rtc_gpio_hold_en(pw);
    }
    ready_ = false;
  }

 private:
  enum : uint8_t {
    W_CONFIG = 0x00, R_CONFIG = 0x10, W_TX_PAYLOAD = 0x20, R_TX_PAYLOAD = 0x21,
    W_TX_ADDRESS = 0x22, R_TX_ADDRESS = 0x23, R_RX_PAYLOAD = 0x24,
  };

  uint8_t cmd(uint8_t c, const uint8_t* tx, uint8_t* rx, size_t n) {
    SPI.beginTransaction(spi_);
    digitalWrite(p_.csn, LOW);
    uint8_t status = SPI.transfer(c);
    for (size_t i = 0; i < n; i++) {
      uint8_t b = SPI.transfer(tx ? tx[i] : 0x00);
      if (rx) rx[i] = b;
    }
    digitalWrite(p_.csn, HIGH);
    SPI.endTransaction();
    return status;
  }

  // Zwalnia blokady zostawione przed snem (także przez starszą wersję tego pliku).
  void releaseHolds() {
    gpio_deep_sleep_hold_dis();
    const int8_t pins[] = {p_.ce, p_.txen, p_.csn, p_.sck, p_.mosi, p_.miso};
    for (int8_t g : pins) gpio_hold_dis((gpio_num_t)g);
    gpio_num_t pw = (gpio_num_t)p_.pwr;
    if (rtc_gpio_is_valid_gpio(pw)) { rtc_gpio_hold_dis(pw); rtc_gpio_deinit(pw); }
    else gpio_hold_dis(pw);
  }

  Nrf905Pins p_;
  SPISettings spi_;
  uint8_t conf_[10] = {0};
  bool spiOn_ = false;
  bool ready_ = false;
};
