"""
Draw the CBS scaling curve from bench.py's JSON output.

    python3 bench.py --trials 50 --json docs/bench.json
    python3 plot.py                                       # writes docs/scaling.png

Top panel is solve time on a log axis (median, p95, and the median-to-p95 band);
bottom panel is the fraction of instances solved inside the time budget. Pillow only.
"""

import json
import math
import sys

from PIL import Image, ImageDraw, ImageFont

W, H = 900, 648
L, R = 104, W - 40                      # plot area, x
T1, B1 = 78, 380                       # time panel, y
T2, B2 = 476, 578                      # success panel, y

BG    = (255, 255, 255)
INK   = (31, 36, 48)
MUTED = (107, 114, 128)
GRID  = (228, 232, 237)
AXIS  = (160, 168, 178)
MED   = (47, 111, 237)
P95   = (232, 131, 58)
BAND  = (250, 234, 220)
OK    = (34, 150, 94)
BAD   = (229, 72, 77)


def font(size, bold=False):
    for path in ("/System/Library/Fonts/Supplemental/Arial Bold.ttf" if bold
                 else "/System/Library/Fonts/Supplemental/Arial.ttf",
                 "/System/Library/Fonts/Helvetica.ttc"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default(size=size)


F_TITLE = font(20, bold=True)
F_SUB   = font(13)
F_AX    = font(12)
F_LBL   = font(12, bold=True)
F_TINY  = font(11)


def vtext(img, xy, text, fnt, fill):
    """Vertical (bottom-up) text, for the y-axis label."""
    tmp = Image.new("RGBA", (240, 28), (0, 0, 0, 0))
    ImageDraw.Draw(tmp).text((120, 14), text, font=fnt, fill=fill, anchor="mm")
    tmp = tmp.rotate(90, expand=True)
    img.paste(tmp, (xy[0] - tmp.width // 2, xy[1] - tmp.height // 2), tmp)


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "docs/bench.json"
    data = json.load(open(src))
    rows = data["rows"]
    timed = [r for r in rows if r["median"] is not None]
    if not timed:
        sys.exit("no solved instances to plot")

    xs = [r["agents"] for r in rows]
    xmin, xmax = min(xs), max(xs)
    lo = min((r["min"] or r["median"]) for r in timed)
    hi = max(r["p95"] for r in timed)
    ymin = 10 ** math.floor(math.log10(lo))
    ymax = 10 ** math.ceil(math.log10(hi))

    pad = 30                                   # keep end points off the frame
    def px(a):
        return L + pad + (a - xmin) / (xmax - xmin) * (R - L - 2 * pad)

    def py(sec):
        f = (math.log10(sec) - math.log10(ymin)) / (math.log10(ymax) - math.log10(ymin))
        return B1 - f * (B1 - T1)

    def py2(frac):
        return B2 - frac * (B2 - T2)

    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)

    d.text((40, 30), "CBS scaling on random 32×32 grids", font=F_TITLE, fill=INK, anchor="lm")
    d.text((40, 54),
           f"{data['density']:.0%} obstacles · {rows[0]['trials']} random instances per point "
           f"· {data['timeout']:g}s budget per instance",
           font=F_SUB, fill=MUTED, anchor="lm")

    # ---- time panel ----
    decade = int(round(math.log10(ymin)))
    while 10 ** decade <= ymax:
        v = 10 ** decade
        y = py(v)
        d.line([(L, y), (R, y)], fill=GRID, width=1)
        lab = f"{v:g}s" if v >= 1 else f"{v:g}".rstrip("0").rstrip(".") + "s"
        d.text((L - 12, y), lab, font=F_AX, fill=MUTED, anchor="rm")
        for m in range(2, 10):                      # minor log ticks
            mv = v * m
            if ymin <= mv <= ymax:
                my = py(mv)
                d.line([(L, my), (L + 5, my)], fill=GRID, width=1)
        decade += 1

    for a in xs:
        d.line([(px(a), T1), (px(a), B1)], fill=GRID, width=1)

    band = ([(px(r["agents"]), py(r["median"])) for r in timed] +
            [(px(r["agents"]), py(r["p95"])) for r in reversed(timed)])
    d.polygon(band, fill=BAND)

    for key, color in (("p95", P95), ("median", MED)):
        pts = [(px(r["agents"]), py(r[key])) for r in timed]
        d.line(pts, fill=color, width=3, joint="curve")
        for x, y in pts:
            d.ellipse([x - 5, y - 5, x + 5, y + 5], fill=color, outline=BG, width=2)

    d.line([(L, T1), (L, B1), (R, B1)], fill=AXIS, width=1)
    for a in xs:
        d.line([(px(a), B1), (px(a), B1 + 5)], fill=AXIS, width=1)
        d.text((px(a), B1 + 17), str(a), font=F_AX, fill=MUTED, anchor="mm")
    d.text(((L + R) / 2, B1 + 38), "number of agents", font=F_LBL, fill=INK, anchor="mm")
    vtext(img, (26, int((T1 + B1) / 2)), "solve time (seconds)", F_LBL, INK)

    # legend
    lx, ly = L + 30, T1 + 18
    d.rectangle([lx - 14, ly - 15, lx + 158, ly + 37], fill=BG, outline=GRID)
    for i, (name, color) in enumerate((("median", MED), ("95th percentile", P95))):
        y = ly + i * 22
        d.line([(lx, y), (lx + 26, y)], fill=color, width=3)
        d.ellipse([lx + 10, y - 4, lx + 18, y + 4], fill=color, outline=BG, width=1)
        d.text((lx + 34, y), name, font=F_AX, fill=INK, anchor="lm")

    # call out the headline point: most agents still solving >=95%
    best = max((r for r in timed if r["success"] >= 0.95),
               key=lambda r: r["agents"], default=None)
    if best:
        bx, by = px(best["agents"]), py(best["p95"])
        d.ellipse([bx - 11, by - 11, bx + 11, by + 11], outline=INK, width=2)
        note = (f"{best['agents']} agents · p95 {best['p95']:.1f}s "
                f"· {best['success']:.0%} solved")
        half = (d.textbbox((0, 0), note, font=F_TINY)[2]) / 2 + 4
        d.text((min(max(bx, L + half), R - half), by - 24), note,
               font=F_TINY, fill=INK, anchor="mm")

    # ---- success panel ----
    d.text((40, T2 - 26), "instances solved within the time budget",
           font=F_LBL, fill=INK, anchor="lm")
    for frac in (0, 0.5, 1.0):
        y = py2(frac)
        d.line([(L, y), (R, y)], fill=GRID, width=1)
        d.text((L - 12, y), f"{frac:.0%}", font=F_AX, fill=MUTED, anchor="rm")

    bw = max(10, int((R - L) / (len(xs) * 2.2)))
    for r in rows:
        x = px(r["agents"])
        y = py2(r["success"])
        color = OK if r["success"] >= 0.95 else (P95 if r["success"] >= 0.6 else BAD)
        d.rectangle([x - bw / 2, y, x + bw / 2, B2], fill=color)
        d.text((x, y - 11), f"{r['success']:.0%}", font=F_TINY, fill=color, anchor="mm")

    d.line([(L, T2), (L, B2), (R, B2)], fill=AXIS, width=1)
    for a in xs:
        d.text((px(a), B2 + 16), str(a), font=F_AX, fill=MUTED, anchor="mm")
    d.text(((L + R) / 2, B2 + 40), "number of agents", font=F_LBL, fill=INK, anchor="mm")

    out = "docs/scaling.png"
    img.save(out)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
