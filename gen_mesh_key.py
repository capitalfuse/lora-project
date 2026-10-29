#!/usr/bin/env python3
"""Generate the shared mesh command key.

Writes stm32_dx-lr30/src/mesh_secret.h (STM32 side) and prints the 32-character hex string
to enter as GATEWAY_MESH_KEY_HEX on the ESP32 side. Both sides must use the same key.

Run from the repository root:  python3 gen_mesh_key.py
Use --force to replace an existing mesh_secret.h (this changes the key: update the ESP32 and
every STM32 device afterwards).
"""
import os, secrets, sys

PATH = os.path.join("stm32_dx-lr30", "src", "mesh_secret.h")

if os.path.exists(PATH) and "--force" not in sys.argv:
    sys.exit(f"{PATH} already exists. Keep it, or run with --force to generate a new key.")
if not os.path.isdir(os.path.dirname(PATH)):
    sys.exit("Run this from the repository root (stm32_dx-lr30/src/ not found).")

key = secrets.token_bytes(16)
rows = [", ".join(f"0x{b:02x}" for b in key[i:i + 8]) for i in (0, 8)]
with open(PATH, "w", newline="\n") as f:
    f.write("// Shared mesh command key. DO NOT COMMIT. Must equal GATEWAY_MESH_KEY_HEX on the ESP32.\n")
    f.write("#pragma once\n#include <stdint.h>\n")
    f.write("static const uint8_t MESH_KEY[16] = {\n    " + ",\n    ".join(rows) + "\n};\n")

print(f"wrote {PATH}")
print("GATEWAY_MESH_KEY_HEX =", key.hex())
