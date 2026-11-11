"""SuperSecureLog – narzędzia do ramek radiowych.

  python ssl_ramka.py dekoduj <32 bajty hex>       # rozkłada ramkę i sprawdza MAC
  python ssl_ramka.py loguj --port COM6 [--plik log.csv]
      # czyta odbiornik z USB, wypisuje zdarzenia, zapisuje do CSV z godziną
      # (do testu zasięgu i testu baterii; potrzebny pakiet pyserial)

Klucz (16 B hex, ten sam co w ssl_klucz.h) podaj w zmiennej SSL_KEY_HEX albo w pliku
ssl_klucz.txt obok skryptu. Klucza nie commitujemy (plik jest w .gitignore).
"""
import argparse
import csv
import datetime
import hashlib
import hmac
import json
import os
import struct
import sys

def _load_key() -> bytes:
    h = os.environ.get("SSL_KEY_HEX", "").strip()
    if not h:
        p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ssl_klucz.txt")
        if os.path.exists(p):
            h = open(p, encoding="utf-8").read().strip()
    if len(h) != 32:
        sys.exit("Brak klucza: ustaw SSL_KEY_HEX (32 znaki hex) albo utworz ssl_klucz.txt obok skryptu.")
    return bytes.fromhex(h)


KEY = None  # wczytywany przy starcie (main), zeby import modulu dzialal bez klucza

TYPES = {1: "alarm", 2: "podejrzane", 3: "info", 4: "zyje", 5: "test", 8: "ack"}
FLAGS = ["ponowienie", "niska_bateria", "test", "wlaczenie", "brak_pomiaru_baterii"]
WAKES = {0: "?", 1: "ruch", 2: "nasluch", 3: "kontrola", 4: "wlaczenie"}
FMT = "<BBHHBBHhHBBHH4s8s"   # 32 bajty, little-endian


def decode(raw: bytes, key: bytes = None) -> dict:
    key = key or KEY
    if len(raw) != 32:
        raise ValueError("ramka musi mieć 32 bajty")
    (vt, node, boot, seq, reason, flags, batt, tilt, motion, engine, wake,
     peak, wakes, extra, mac) = struct.unpack(FMT, raw)
    good = hmac.compare_digest(hmac.new(key, raw[:24], hashlib.sha256).digest()[:8], mac)
    return {
        "wersja": vt >> 4, "ev": TYPES.get(vt & 0x0F, "nieznany"), "wezel": node,
        "boot": boot, "seq": seq, "regula": reason,
        "flagi": [n for i, n in enumerate(FLAGS) if flags & (1 << i)],
        "bat_mv": batt, "kat": tilt / 10, "drgania_ms": motion, "silnik_pct": engine,
        "pobudka": WAKES.get(wake, "?"), "pik_hz": peak, "pobudki": wakes,
        "extra": extra.hex(), "mac_ok": good,
    }


def encode(fields: dict, key: bytes = None) -> bytes:
    """Odwrotność decode() – do testów i symulacji czujnika po stronie serwera."""
    key = key or KEY
    t = {v: k for k, v in TYPES.items()}[fields.get("ev", "test")]
    fl = sum(1 << FLAGS.index(n) for n in fields.get("flagi", []))
    body = struct.pack("<BBHHBBHhHBBHH4s", (1 << 4) | t, fields.get("wezel", 1),
                       fields.get("boot", 1), fields.get("seq", 1), fields.get("regula", 0), fl,
                       fields.get("bat_mv", 0), round(fields.get("kat", 0) * 10),
                       fields.get("drgania_ms", 0), fields.get("silnik_pct", 0),
                       {v: k for k, v in WAKES.items()}.get(fields.get("pobudka", "?"), 0),
                       fields.get("pik_hz", 0), fields.get("pobudki", 0), b"\0\0\0\0")
    return body + hmac.new(key, body, hashlib.sha256).digest()[:8]


def log(port: str, path: str) -> None:
    import serial  # pip install pyserial
    cols = ["czas", "ev", "wezel", "boot", "seq", "regula", "opis", "bat_mv", "kat",
            "drgania_ms", "silnik_pct", "pik_hz", "pobudka", "pobudki", "flagi", "utracone", "restart"]
    with serial.Serial(port, 115200, timeout=1) as s, open(path, "a", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        if f.tell() == 0:
            w.writeheader()
        print(f"Czytam {port}, zapis do {path}. Ctrl+C konczy.")
        while True:
            line = s.readline().decode("utf-8", "replace").strip()
            if not line:
                continue
            now = datetime.datetime.now().isoformat(timespec="seconds")
            if not line.startswith("{"):
                print(now, line)
                continue
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                print(now, "zla linia:", line)
                continue
            ev["czas"] = now
            ev["flagi"] = ",".join(ev.get("flagi", []))
            w.writerow(ev)
            f.flush()
            print(now, ev.get("ev"), "wezel", ev.get("wezel"), "seq", ev.get("seq"),
                  "bat", ev.get("bat_mv"), "mV", "utracone", ev.get("utracone"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("dekoduj"); d.add_argument("hex")
    l = sub.add_parser("loguj"); l.add_argument("--port", required=True); l.add_argument("--plik", default="ssl_log.csv")
    a = ap.parse_args()
    KEY = _load_key()
    if a.cmd == "dekoduj":
        print(json.dumps(decode(bytes.fromhex(a.hex)), ensure_ascii=False, indent=2))
    else:
        try:
            log(a.port, a.plik)
        except KeyboardInterrupt:
            sys.exit(0)
