#!/usr/bin/env python3
"""Pack launcher PNG icons into 64x64 RGB565 C arrays."""

from __future__ import annotations

from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "assets" / "icons"
OUT_CPP = ROOT / "main" / "launcher_icons.cpp"

SIZE = 64
TRANSPARENT = 0xF81F
ICONS = (
    ("fluid", "kIconFluid"),
    ("cube", "kIconCube"),
    ("attitude", "kIconAttitude"),
    ("maze", "kIconMaze"),
    ("avalanche", "kIconAvalanche"),
)


def rgb565(r: int, g: int, b: int) -> int:
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)


def chroma_distance(pixel: tuple[int, int, int], key: tuple[int, int, int]) -> float:
    return (
        (pixel[0] - key[0]) ** 2
        + (pixel[1] - key[1]) ** 2
        + (pixel[2] - key[2]) ** 2
    ) ** 0.5


def punch_key(image: Image.Image) -> Image.Image:
    image = image.convert("RGBA")
    corners = (
        image.getpixel((0, 0))[:3],
        image.getpixel((image.width - 1, 0))[:3],
        image.getpixel((0, image.height - 1))[:3],
        image.getpixel((image.width - 1, image.height - 1))[:3],
    )
    key = tuple(sum(c[i] for c in corners) // 4 for i in range(3))
    pixels = image.load()
    assert pixels is not None
    for y in range(image.height):
        for x in range(image.width):
            r, g, b, _a = pixels[x, y]
            if chroma_distance((r, g, b), key) < 70:
                pixels[x, y] = (0, 0, 0, 0)
    return image


def pack_icon(path: Path) -> list[int]:
    image = Image.open(path).convert("RGBA")
    image = image.resize((256, 256), Image.Resampling.LANCZOS)
    image = punch_key(image)
    image = image.resize((SIZE, SIZE), Image.Resampling.LANCZOS)
    pixels = image.load()
    assert pixels is not None
    out: list[int] = []
    for y in range(SIZE):
        for x in range(SIZE):
            r, g, b, a = pixels[x, y]
            if a < 128:
                out.append(TRANSPARENT)
                continue
            value = rgb565(r, g, b)
            if value == TRANSPARENT:
                value ^= 1
            out.append(value)
    return out


def format_array(values: list[int]) -> str:
    lines = []
    for i in range(0, len(values), 12):
        chunk = ", ".join(f"0x{v:04X}" for v in values[i : i + 12])
        lines.append(f"    {chunk},")
    return "\n".join(lines)


def main() -> None:
    bodies = []
    for stem, symbol in ICONS:
        src = SRC_DIR / f"{stem}.png"
        if not src.exists():
            raise SystemExit(f"missing icon source: {src}")
        bodies.append(
            f"const uint16_t {symbol}[kLauncherIconPixels] = {{\n{format_array(pack_icon(src))}\n}};"
        )
    OUT_CPP.write_text(
        f"""#include "launcher_icons.hpp"

namespace fluid_demo {{

{chr(10).join(bodies)}

}}  // namespace fluid_demo
"""
    )
    print(f"wrote {OUT_CPP.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
