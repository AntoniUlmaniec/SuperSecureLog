# Receiver
ESP32 code next to the laptop: receiving the nRF905 packet, verifying its signature, converting it to JSON and forwarding it (USB or Wi-Fi).
Owner: Filip.

## Current receiver (added by Igor, 2026-11-11 – Filip, please review)
- `ssl_odbiornik/` – ESP32 + nRF905 next to the laptop: verifies the signature and the (boot, seq) counter,
  sends the ACK, prints one JSON line per event on USB (115200). Same wiring as the node, see `docs/radio_nrf905.md`.
- `tools/ssl_ramka.py` – decodes a raw packet (`dekoduj <hex>`) and logs the receiver output to CSV
  (`loguj --port COM6`). Needs `pyserial`; key from `SSL_KEY_HEX` or `ssl_klucz.txt`.

Copy `ssl_odbiornik/ssl_klucz.example.h` to `ssl_klucz.h` and paste the team key before compiling (never commit it).
