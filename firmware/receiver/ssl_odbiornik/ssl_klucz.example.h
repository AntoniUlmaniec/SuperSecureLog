// TEMPLATE – the real key is never committed (repo rule: no keys in the repo).
// 1. Copy this file to ssl_klucz.h in the same folder (ssl_klucz.h is in .gitignore).
// 2. Paste the 16-byte team key (get it from Igor / Mateusz, not over a public channel).
//    New key:  python -c "import secrets;print(secrets.token_hex(16))"
// The node and the receiver must use the same key.
#pragma once
#include <stdint.h>

static const uint8_t SSL_KEY[16] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
