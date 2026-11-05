# SuperSecureLog (SSL)

An IoT system that detects theft of timber stored in the forest.
A sensor hidden inside a log at the bottom of the stack (ESP32 + LIS3DSH accelerometer + microphone) detects movement and noise.
An on-device algorithm tells theft (e.g. loading with a crane truck) apart from wind or passing cars,
and the alert travels over 433 MHz radio (nRF905) to a receiver, then to the server and a map dashboard in ThingsBoard.

Student team project, Internet of Things 2026/27, AGH University of Krakow.

## Repository structure

| Folder | Contents | Owner |
| --- | --- | --- |
| `firmware/node/` | ESP32 in the log: sensors, deep sleep, sending the radio packet | Igor, Filip, Łukasz |
| `firmware/receiver/` | ESP32 next to the laptop: receiving the packet, converting it to JSON | Filip |
| `algorithm/` | Python prototype of the filtering algorithm and its tests | Łukasz |
| `server/` | script receiving the data, local database, forwarding to ThingsBoard | Sebastian |
| `dashboard/` | ThingsBoard dashboard export and description | Maciej |
| `docs/` | data format, wiring diagram, power budget, security | Antoni, Mateusz |

## Rules
- Everyone works in their own folder. Changes in someone else's folder only after agreeing with them.
- The data format is described in `docs/` and changes only after agreeing with Antoni.
- Never commit passwords, keys or tokens (e.g. ThingsBoard tokens, the packet signing key).

## Team
Antoni Ulmaniec (PM), Igor Kosakowski, Filip Mach, Mateusz Miszczyński, Sebastian Bochenek, Maciej Kubasiak, Łukasz Bukowiec
