import math, random
from PIL import Image, ImageDraw, ImageFilter

def generate_sar_aerial_scene(output_path="demo/sample_sar_aerial.jpg", width=2028, height=1520):
    # Create base terrain canvas (earthy wilderness tones)
    base = Image.new("RGB", (width, height), (78, 92, 54))
    draw = ImageDraw.Draw(base)

    # 1. Broad terrain elevation shading / hills
    for y in range(0, height, 8):
        v = math.sin(y * 0.003) * 15 + math.cos(y * 0.007) * 10
        col = (int(70 + v), int(85 + v * 1.1), int(48 + v * 0.8))
        draw.rectangle([0, y, width, y + 8], fill=col)

    # 2. Mountain rocky ridge on the upper-right
    ridge_poly = [(int(width * 0.55), 0), (width, 0), (width, int(height * 0.45)), (int(width * 0.7), int(height * 0.3))]
    draw.polygon(ridge_poly, fill=(110, 108, 98))
    # Rock textures
    for _ in range(300):
        rx = random.randint(int(width * 0.6), width)
        ry = random.randint(0, int(height * 0.42))
        rw = random.randint(20, 120)
        rh = random.randint(10, 40)
        c = random.randint(85, 130)
        draw.ellipse([rx, ry, rx + rw, ry + rh], fill=(c, c - 4, c - 10))

    # 3. Winding creek / river cutting through the valley
    creek_points = []
    cy = 0
    cx = width * 0.35
    while cy < height:
        creek_points.append((int(cx + math.sin(cy * 0.006) * 140 + math.cos(cy * 0.015) * 40), cy))
        cy += 15
    for i in range(len(creek_points) - 1):
        p1, p2 = creek_points[i], creek_points[i + 1]
        draw.line([p1, p2], fill=(42, 75, 88), width=36)
        draw.line([p1, p2], fill=(62, 105, 122), width=18)

    # 4. Dirt hiking trail / path meandering through trees
    trail_points = []
    ty = 0
    tx = width * 0.52
    while ty < height:
        trail_points.append((int(tx + math.sin(ty * 0.008) * 110 + math.sin(ty * 0.02) * 25), ty))
        ty += 12
    for i in range(len(trail_points) - 1):
        p1, p2 = trail_points[i], trail_points[i + 1]
        draw.line([p1, p2], fill=(142, 128, 98), width=14)
        draw.line([p1, p2], fill=(168, 154, 122), width=6)

    # 5. Tree clusters and forest patches (dense pine & deciduous clumps)
    for _ in range(1200):
        tx = random.randint(20, width - 20)
        ty = random.randint(20, height - 20)
        # Avoid direct river center
        tr = random.randint(18, 55)
        tree_tone = random.choice([
            (32, 58, 28),   # dark spruce
            (45, 78, 38),   # pine
            (56, 92, 44),   # fir
            (68, 104, 52),  # canopy highlight
            (88, 112, 58),  # deciduous brush
        ])
        draw.ellipse([tx, ty, tx + tr, ty + tr], fill=tree_tone)
        # Tree shadow
        draw.ellipse([tx + int(tr * 0.25), ty + int(tr * 0.2), tx + tr, ty + tr], fill=(22, 42, 20))

    # Apply slight natural blur to blend aerial perspective
    base = base.filter(ImageFilter.GaussianBlur(1.2))
    draw = ImageDraw.Draw(base)

    # 6. Add SAR Search Target: Lost person in high-vis blaze orange / red jacket near trail/creek
    # Position around (1180, 840)
    hx, hy = 1180, 840
    # Shadow
    draw.ellipse([hx + 4, hy + 8, hx + 28, hy + 22], fill=(20, 28, 18, 180))
    # High-vis SAR orange jacket
    draw.ellipse([hx, hy, hx + 22, hy + 20], fill=(245, 85, 20))
    draw.ellipse([hx + 3, hy + 2, hx + 19, hy + 18], fill=(255, 110, 30))
    # Head / cap (yellow high-vis or dark)
    draw.ellipse([hx + 6, hy - 6, hx + 16, hy + 4], fill=(235, 200, 40))
    # Blue jeans / pack
    draw.rectangle([hx + 2, hy + 16, hx + 10, hy + 26], fill=(40, 60, 95))
    draw.rectangle([hx + 12, hy + 16, hx + 20, hy + 26], fill=(40, 60, 95))
    # High-vis reflective stripes
    draw.line([(hx + 5, hy + 10), (hx + 17, hy + 10)], fill=(255, 255, 240), width=2)

    base.save(output_path, quality=90)
    print(f"Generated realistic SAR aerial test scene at {output_path} ({width}x{height})")

if __name__ == "__main__":
    generate_sar_aerial_scene("C:/Users/eyres/.gemini/antigravity/scratch/lora_image_studio/demo/sample_sar_aerial.jpg")
