# nRF905 radio link: wiring, packet format, receiver output

Status 2026-11-11 (Igor). The node sends the result of `decide()` over nRF905 to the receiver;
the receiver acknowledges every packet (ACK) and prints one JSON line per event on USB.

> **To agree with Antoni:** `docs/README.md` mentions a 24-byte packet. This format has
> **24 bytes of data + an 8-byte signature = 32 bytes**, which is exactly one nRF905 payload.

## Radio settings
433.2 MHz (channel 108), +10 dBm, ShockBurst 50 kbit/s, CRC16, 4-byte addresses,
16 MHz crystal on the module. If a module's crystal reads `20.000`, set `xof = 4` in
`Nrf905Config` (`nrf905.h`). Never transmit without the antenna.

## Wiring (node and receiver identical)
Wire by the **pin labels printed on the ESP32 board**, not by breadboard column letters.

| nRF905 pin | ESP32 pin |
|---|---|
| 1 VCC | 3V3 (never 5V) |
| 13 GND | GND |
| 2 TX_EN | 17 |
| 3 TRX_CE | 16 |
| 4 PWR_UP | 4 |
| 8 DR | 27 |
| 9 MISO | 19 |
| 10 MOSI | 23 |
| 11 SCK | 18 |
| 12 CSN | 5 |
| 5 uCLK, 6 CD, 7 AM, 14 GND | not connected |

No extra resistors or capacitors are needed when powered from USB.
Node sensor pins for reference: SDA 21, SCL 22, INT1 33 (accelerometer), mic VCC 25, AOUT 34, DOUT 32.
Keep 0, 2, 12, 15 (boot pins) and the flash pins (CLK, SD0–SD3, CMD) free.

## Packet (32 bytes, little-endian)
Defined in `ssl_protocol.h` (same file in both sketches).

| byte | field | type | meaning |
|---|---|---|---|
| 0 | ver/type | u8 | high nibble: version 1, low nibble: type |
| 1 | node | u8 | sensor id 1..254 (0 = receiver) |
| 2–3 | boot | u16 | power-on counter (NVS) |
| 4–5 | seq | u16 | packet counter since boot; (boot, seq) blocks replays |
| 6 | rule | u8 | row of the `decide()` table 1..7, 0 = n/a |
| 7 | flags | u8 | 0x01 retry, 0x02 low battery, 0x04 test, 0x08 first after power-on, 0x10 no battery measurement |
| 8–9 | batt_mv | u16 | battery voltage, 0 = not measured |
| 10–11 | tilt | i16 | orientation change, 0.1° |
| 12–13 | motion_ms | u16 | vibration time in the 5 s window |
| 14 | engine_pct | u8 | % of sound frames classified as engine |
| 15 | wake | u8 | 1 motion, 2 listen timer, 3 recheck timer, 4 power-on |
| 16–17 | peak_hz | u16 | engine peak frequency |
| 18–19 | wakes | u16 | wake-ups since power-on |
| 20–23 | extra | 4 B | reserved, zero |
| 24–31 | mac | 8 B | first 8 bytes of HMAC-SHA256(key, bytes 0–23) |

Types: 1 alarm (rules 1–3), 2 suspicious (4–5), 3 info: engine nearby (6), 4 heartbeat, 5 test, 8 ACK.
ACK: type 8 with node/boot/seq copied from the acknowledged packet, rest zero, same MAC.
nRF905 addresses: receiver `S S L 0x00`, node N `S S L N`.

## Exchange
1. Node powers the radio up, sends (about 7 ms on air), switches to receive and waits 40 ms for the ACK.
2. No ACK: random 20–150 ms pause, same packet with the retry flag. Attempts: alarm 6, suspicious/info 3, heartbeat 2.
3. Receiver: bad MAC → `odrzucona`; older (boot, seq) → `powtorka_odrzucona`, no ACK; same seq → ACK only; newer → ACK and JSON.
4. An alarm that still has no ACK is resent on every following wake-up until acknowledged.
5. Heartbeat right after start-up and then every `HEARTBEAT_S` (6 h by default).

## Receiver output (USB, 115200 baud)
Lines starting with `{` are events, lines starting with `#` are diagnostics.
```
{"ev":"alarm","wezel":1,"boot":1,"seq":2,"regula":1,"opis":"kat >= 15 st.","bat_mv":0,"kat":17.3,"drgania_ms":0,"silnik_pct":0,"pik_hz":0,"pobudka":"ruch","pobudki":14,"flagi":["brak_pomiaru_baterii"],"utracone":0,"restart":false,"t_ms":1013,"raw":"..."}
```
Event names: `alarm`, `podejrzane`, `info`, `zyje`, `test`, plus `start`, `blad`, `odrzucona`,
`powtorka_odrzucona`, `reset_licznikow`. `utracone` = sequence numbers lost since the previous packet,
`restart` = the node rebooted. Pressing **R** in the receiver's serial monitor forgets the stored counters
(needed after re-flashing a node with "Erase All Flash").
Every 20 s: `# odbiornik zyje, radio ok, ramek: X, zle MAC: Y, AM: Z`.

## Key
Never committed. Each sketch folder has `ssl_klucz.example.h`: copy it to `ssl_klucz.h` and paste the team key
(`ssl_klucz.h` is in `.gitignore`). The Python tool reads the key from `SSL_KEY_HEX` or `ssl_klucz.txt`.
One key for all nodes means a captured node reveals it; per-node keys are an open question for Mateusz.

## Lessons from bring-up (2026-11-11)
- `F_TEST` and `OK` are already defined by the ESP32 core (`unistd.h`, `ets_sys.h`); hence the `FLAG_*`/`RES_*` names.
- `gpio_deep_sleep_hold_en()` freezes **all** digital pins on ESP32, including I2C, and they may stay frozen after waking.
  Only PWR_UP (GPIO4, an RTC pin) is held during sleep now.
- Upload failing with "Wrong boot mode (0x13)": hold BOOT while "Connecting..." is shown, release when writing starts.
- A loose SDA wire made the accelerometer answer intermittently (found with `szukaj_akcelerometru`).
