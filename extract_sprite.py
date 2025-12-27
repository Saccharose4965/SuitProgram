#!/usr/bin/env python3
"""
Extract a monochrome sprite from a PNG and emit a C header with bit rows.

Example:
  python extract_sprite.py --input bird.png --symbol flappy_bird \
         --out components/flappy/include/flappy_sprite.h

- PNG must be 8-bit RGB or RGBA.
- Pixels whose summed RGB exceed the threshold (default 128 per channel) are 1s.
- Leftmost pixel maps to the MSB of each row.
- Widths up to 32 bits are supported; uint16_t is used up to 16, else uint32_t.
"""
from __future__ import annotations
import argparse
import os
import struct
import zlib
from typing import List, Sequence


def paeth(a: int, b: int, c: int) -> int:
    p = a + b - c
    pa = abs(p - a)
    pb = abs(p - b)
    pc = abs(p - c)
    if pa <= pb and pa <= pc:
        return a
    if pb <= pc:
        return b
    return c


def decode_png(path: str) -> tuple[List[List[int]], int, int, int]:
    with open(path, "rb") as f:
        data = f.read()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise SystemExit("Not a PNG file")

    ptr = 8
    idat = bytearray()
    w = h = None
    bit_depth = color_type = None

    while ptr < len(data):
        ln = int.from_bytes(data[ptr:ptr + 4], "big")
        ptr += 4
        typ = data[ptr:ptr + 4]
        ptr += 4
        chunk = data[ptr:ptr + ln]
        ptr += ln
        ptr += 4  # CRC
        if typ == b"IHDR":
            w, h, bit_depth, color_type, comp, flt, inter = struct.unpack(
                ">IIBBBBB", chunk)
            if comp != 0 or flt != 0 or inter != 0:
                raise SystemExit("Unsupported PNG (filters/interlace)")
        elif typ == b"IDAT":
            idat.extend(chunk)
        elif typ == b"IEND":
            break

    if w is None or h is None or bit_depth is None or color_type is None:
        raise SystemExit("Missing IHDR in PNG")
    if bit_depth != 8 or color_type not in (2, 6):  # RGB or RGBA
        raise SystemExit("Expected 8-bit RGB or RGBA PNG")

    raw = zlib.decompress(idat)
    channels = 3 if color_type == 2 else 4
    stride = w * channels
    prev = [0] * stride
    pos = 0
    rows: List[List[int]] = []
    for _ in range(h):
        ftype = raw[pos]
        pos += 1
        row = list(raw[pos:pos + stride])
        pos += stride
        if ftype == 1:  # Sub
            for i in range(stride):
                row[i] = (row[i] + (row[i - channels] if i >= channels else 0)) & 0xFF
        elif ftype == 2:  # Up
            for i in range(stride):
                row[i] = (row[i] + prev[i]) & 0xFF
        elif ftype == 3:  # Average
            for i in range(stride):
                row[i] = (row[i] + ((row[i - channels] if i >= channels else 0) + prev[i]) // 2) & 0xFF
        elif ftype == 4:  # Paeth
            for i in range(stride):
                a = row[i - channels] if i >= channels else 0
                b = prev[i]
                c = prev[i - channels] if i >= channels else 0
                row[i] = (row[i] + paeth(a, b, c)) & 0xFF
        elif ftype != 0:
            raise SystemExit(f"Unsupported filter {ftype}")
        rows.append(row)
        prev = row

    return rows, w, h, channels  # rows[y][channels*x:channels*(x+1)]


def is_ink(px: Sequence[int], threshold: int) -> bool:
    if len(px) == 4:
        r, g, b, a = px
        if a == 0:
            return False
    else:
        r, g, b = px
    return (r + g + b) >= threshold * 3


def rows_to_bits(img: List[List[int]], w: int, h: int, channels: int, threshold: int) -> List[int]:
    rows_bits: List[int] = []
    for y in range(h):
        bits = 0
        for x in range(w):
            px = img[y][channels * x: channels * (x + 1)]
            if is_ink(px, threshold):
                bits |= 1 << (w - 1 - x)  # leftmost pixel -> MSB
        rows_bits.append(bits)
    return rows_bits


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="Input PNG")
    ap.add_argument("--out", required=True, help="Output header path")
    ap.add_argument("--symbol", default="flappy_bird", help="Base symbol name")
    ap.add_argument("--threshold", type=int, default=128, help="0-255 ink threshold")
    args = ap.parse_args()

    img, w, h, channels = decode_png(args.input)
    rows_bits = rows_to_bits(img, w, h, channels, args.threshold)

    sym = args.symbol
    sym_up = sym.upper()
    c_type = "uint16_t" if w <= 16 else "uint32_t"
    if w > 32:
        raise SystemExit("Width > 32 not supported")

    lines: List[str] = []
    lines.append("#pragma once")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"#define {sym_up}_W {w}")
    lines.append(f"#define {sym_up}_H {h}")
    lines.append("")
    lines.append(f"static const {c_type} {sym}_rows[{sym_up}_H] = {{")
    for r in rows_bits:
        lines.append(f"    0x{r:0{(w+3)//4}X},")
    lines.append("};")
    lines.append("")

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"Wrote {args.out} ({w}x{h}) as {c_type}")


if __name__ == "__main__":
    main()
