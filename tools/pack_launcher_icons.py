#!/usr/bin/env python3
"""Pack launcher PNG icons into 64x64 RGB565 C arrays."""

from __future__ import annotations

from pathlib import Path

from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
SOURCE_DIRECTORY = ROOT / "assets" / "icons"
OUTPUT_CPP = ROOT / "main" / "launcher_icons.cpp"

ICON_SIZE = 64
TRANSPARENT_COLOR = 0xF81F
ICON_SPECS = (
    ("fluid", "kIconFluid"),
    ("cube", "kIconCube"),
    ("attitude", "kIconAttitude"),
    ("maze", "kIconMaze"),
    ("avalanche", "kIconAvalanche"),
)


def rgb565(red: int, green: int, blue: int) -> int:
    return ((red >> 3) << 11) | ((green >> 2) << 5) | (blue >> 3)


def chroma_distance(pixel: tuple[int, int, int], chroma_key: tuple[int, int, int]) -> float:
    return (
        (pixel[0] - chroma_key[0]) ** 2
        + (pixel[1] - chroma_key[1]) ** 2
        + (pixel[2] - chroma_key[2]) ** 2
    ) ** 0.5


def punch_key(image: Image.Image) -> Image.Image:
    image = image.convert("RGBA")
    corner_colors = (
        image.getpixel((0, 0))[:3],
        image.getpixel((image.width - 1, 0))[:3],
        image.getpixel((0, image.height - 1))[:3],
        image.getpixel((image.width - 1, image.height - 1))[:3],
    )
    chroma_key = tuple(
        sum(color[channel_index] for color in corner_colors) // len(corner_colors)
        for channel_index in range(3)
    )
    pixels = image.load()
    assert pixels is not None
    for row in range(image.height):
        for column in range(image.width):
            red, green, blue, _alpha = pixels[column, row]
            if chroma_distance((red, green, blue), chroma_key) < 70:
                pixels[column, row] = (0, 0, 0, 0)
    return image


def pack_icon(source_path: Path) -> list[int]:
    image = Image.open(source_path).convert("RGBA")
    image = image.resize((256, 256), Image.Resampling.LANCZOS)
    image = punch_key(image)
    image = image.resize((ICON_SIZE, ICON_SIZE), Image.Resampling.LANCZOS)
    pixels = image.load()
    assert pixels is not None
    packed_pixels: list[int] = []
    for row in range(ICON_SIZE):
        for column in range(ICON_SIZE):
            red, green, blue, alpha = pixels[column, row]
            if alpha < 128:
                packed_pixels.append(TRANSPARENT_COLOR)
                continue
            packed_color = rgb565(red, green, blue)
            if packed_color == TRANSPARENT_COLOR:
                packed_color ^= 1
            packed_pixels.append(packed_color)
    return packed_pixels


def format_array(packed_pixels: list[int]) -> str:
    formatted_lines = []
    for line_start in range(0, len(packed_pixels), 12):
        formatted_values = ", ".join(
            f"0x{packed_color:04X}"
            for packed_color in packed_pixels[line_start : line_start + 12]
        )
        formatted_lines.append(f"    {formatted_values},")
    return "\n".join(formatted_lines)


def main() -> None:
    array_definitions = []
    for icon_name, symbol_name in ICON_SPECS:
        source_path = SOURCE_DIRECTORY / f"{icon_name}.png"
        if not source_path.exists():
            raise SystemExit(f"missing icon source: {source_path}")
        array_body = format_array(pack_icon(source_path))
        array_definitions.append(
            f"const uint16_t {symbol_name}[kLauncherIconPixels] = "
            f"{{\n{array_body}\n}};"
        )
    OUTPUT_CPP.write_text(
        f"""#include "launcher_icons.hpp"

namespace fluid_demo {{

{chr(10).join(array_definitions)}

}}
"""
    )
    print(f"wrote {OUTPUT_CPP.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
