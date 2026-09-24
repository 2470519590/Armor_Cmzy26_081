"""Create a first-install image with bootloader, slot A, and confirmed metadata."""

from __future__ import annotations

import argparse
import struct
import zlib
from pathlib import Path

FLASH_SIZE = 128 * 1024
BOOT_SIZE = 16 * 1024
SLOT_A = 16 * 1024
SLOT_SIZE = 48 * 1024
META_A = 112 * 1024
META_MAGIC = 0x3141544D
CONFIRMED = 2


def build(boot: bytes, app: bytes) -> bytes:
    if len(boot) > BOOT_SIZE or len(app) < 8 or len(app) > SLOT_SIZE:
        raise ValueError("boot or slot A image size is outside its partition")
    image = bytearray([0xFF]) * FLASH_SIZE
    image[: len(boot)] = boot
    image[SLOT_A : SLOT_A + len(app)] = app
    record = struct.pack(
        "<9I", META_MAGIC, 1, 0x08004000, 0, len(app),
        zlib.crc32(app), 0, 0, CONFIRMED,
    )
    record += struct.pack("<I", zlib.crc32(record))
    image[META_A : META_A + len(record)] = record
    return bytes(image)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--boot", type=Path, required=True)
    parser.add_argument("--slot-a", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.write_bytes(build(args.boot.read_bytes(), args.slot_a.read_bytes()))
    print(f"wrote {args.output} ({FLASH_SIZE} bytes)")


if __name__ == "__main__":
    main()
