from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / "assets"
SIZE = 2048


def font(name: str, size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    path = Path("C:/Windows/Fonts") / name
    try:
        return ImageFont.truetype(str(path), size=size)
    except OSError:
        return ImageFont.load_default()


def rounded_mask(size: int, radius: int) -> Image.Image:
    mask = Image.new("L", (size, size), 0)
    ImageDraw.Draw(mask).rounded_rectangle((0, 0, size - 1, size - 1), radius=radius, fill=255)
    return mask


def build_icon() -> Image.Image:
    image = Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 0))
    gradient = Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 255))
    pixels = gradient.load()
    for y in range(SIZE):
        for x in range(SIZE):
            t = min(1.0, max(0.0, (x * 0.66 + y * 0.34) / (SIZE - 1)))
            if t < 0.52:
                u = t / 0.52
                color = (
                    int(108 + (154 - 108) * u),
                    int(0 + (8 - 0) * u),
                    int(0 + (16 - 0) * u),
                    255,
                )
            else:
                u = (t - 0.52) / 0.48
                color = (
                    int(154 + (111 - 154) * u),
                    int(8 + (0 - 8) * u),
                    int(16 + (8 - 16) * u),
                    255,
                )
            pixels[x, y] = color
    image = gradient
    image.putalpha(rounded_mask(SIZE, 380))

    draw = ImageDraw.Draw(image)
    gold = (216, 181, 106, 255)
    light_gold = (240, 215, 155, 255)
    ivory = (255, 248, 232, 245)

    draw.rounded_rectangle((42, 42, SIZE - 43, SIZE - 43), radius=350, outline=gold, width=18)
    draw.rounded_rectangle((80, 80, SIZE - 81, SIZE - 81), radius=318, outline=(240, 215, 155, 112), width=6)

    for offset in range(-SIZE, SIZE * 2, 116):
        draw.line((offset, SIZE, offset + SIZE, 0), fill=(240, 215, 155, 22), width=5)

    center = SIZE // 2
    draw.ellipse((center - 330, 280, center + 330, 940), fill=(255, 248, 232, 20), outline=(240, 215, 155, 100), width=9)
    draw.ellipse((center - 260, 350, center + 260, 870), outline=(216, 181, 106, 92), width=6)

    # Clock tower.
    draw.polygon(
        [
            (center - 400, 590),
            (center, 255),
            (center + 400, 590),
            (center + 340, 590),
            (center + 340, 660),
            (center - 340, 660),
            (center - 340, 590),
        ],
        fill=(255, 248, 232, 32),
        outline=light_gold,
    )
    draw.polygon([(center - 325, 590), (center, 330), (center + 325, 590)], fill=(255, 248, 232, 26), outline=light_gold)
    draw.rounded_rectangle((center - 205, 610, center + 205, 1260), radius=24, fill=(255, 248, 232, 30), outline=ivory, width=14)
    draw.rectangle((center - 150, 550, center + 150, 635), fill=(255, 248, 232, 28), outline=ivory, width=12)
    draw.ellipse((center - 145, 720, center + 145, 1010), fill=(122, 0, 0, 86), outline=ivory, width=13)
    draw.line((center, 755, center, 870), fill=light_gold, width=15)
    draw.line((center, 870, center + 75, 920), fill=light_gold, width=15)

    # Open book.
    book_top = 1215
    draw.polygon([(center - 430, book_top), (center - 40, book_top + 75), (center - 40, 1535), (center - 485, 1460)], fill=(255, 248, 232, 38), outline=light_gold)
    draw.polygon([(center + 430, book_top), (center + 40, book_top + 75), (center + 40, 1535), (center + 485, 1460)], fill=(255, 248, 232, 38), outline=light_gold)
    for offset in (0, 55, 110):
        draw.line((center - 365, 1285 + offset, center - 95, 1335 + offset), fill=(240, 215, 155, 140), width=8)
        draw.line((center + 365, 1285 + offset, center + 95, 1335 + offset), fill=(240, 215, 155, 140), width=8)

    draw.text((center, 1665), "SUDA", font=font("arialbd.ttf", 150), fill=ivory, anchor="mm", stroke_width=3, stroke_fill=(122, 0, 0, 255))
    draw.text((center, 1805), "1900", font=font("georgiab.ttf", 58), fill=(240, 215, 155, 220), anchor="mm")
    return image


def main() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    image = build_icon()
    image.resize((1024, 1024), Image.Resampling.LANCZOS).save(ASSETS / "suda_app_icon.png")
    image.resize((512, 512), Image.Resampling.LANCZOS).save(
        ASSETS / "suda_app_icon.ico",
        format="ICO",
        sizes=[(256, 256), (128, 128), (64, 64), (48, 48), (32, 32), (16, 16)],
    )


if __name__ == "__main__":
    main()