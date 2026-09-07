#!/usr/bin/env python3
import struct
import sys
import time
import zlib
from pathlib import Path

MAGIC = 0x31564741  # AGV1
FORMAT = 1
APP_BASE = 0x08008000
APP_LIMIT = 0x08060000

image = Path(sys.argv[1]).read_bytes()
out = Path(sys.argv[2])
if not image or len(image) > APP_LIMIT - APP_BASE:
    raise SystemExit(f"invalid application size: {len(image)}")
app_crc = zlib.crc32(image) & 0xFFFFFFFF
head = struct.pack("<5I", MAGIC, FORMAT, APP_BASE, len(image), app_crc)
header_crc = zlib.crc32(head) & 0xFFFFFFFF
generation = int(time.time()) & 0xFFFFFFFF
manifest = head + struct.pack("<3I", header_crc, generation, 0)
out.write_bytes(manifest)
print(f"manifest size={len(manifest)} app_size={len(image)} app_crc=0x{app_crc:08X} header_crc=0x{header_crc:08X} gen={generation}")
