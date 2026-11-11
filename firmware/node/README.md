# Node (sensor in the log)
ESP32 code (Arduino / PlatformIO): reading the LIS3DSH and the microphone, deep sleep and wake-up on interrupt,
the filtering algorithm, sending the nRF905 radio packet (format in `docs/`).
Owners: Igor, Filip, Łukasz.

## Sketches (Arduino IDE, board "ESP32 Dev Module")
| Folder | What it is |
|---|---|
| `ssl_czuwanie/` | main node firmware: deep sleep, wake on motion / timer, 5 s motion + sound analysis, `decide()`, radio alerts with ACK |
| `ssl_nadajnik_ciagly/` | link test: sends a test packet every 0.5 s and reports transmission and ACK |
| `test_lis3dh/` | accelerometer diagnostics: register writes, direct reads, FIFO, motion interrupt on INT1 |
| `szukaj_akcelerometru/` | finds which pins the accelerometer is really wired to and whether it is powered |

Before compiling a sketch that uses the radio, copy `ssl_klucz.example.h` to `ssl_klucz.h` in the same folder and
paste the team key (never commit it). Wiring and packet format: `docs/radio_nrf905.md`.

`algorytm.h` holds all thresholds and the decision table (`decideRule()` returns the rule number 1–7 that is also sent
over the radio); `dsp.h` holds the FFT and sound features.
