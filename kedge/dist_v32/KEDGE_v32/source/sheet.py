import json, sys
from PIL import Image, ImageDraw, ImageFont
OUT = sys.argv[1]; S = json.load(open(OUT + '/_shots.json'))
try: F = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 16)
except Exception: F = ImageFont.load_default()
def grid(items, cols, fn, title):
    ims = [(Image.open(f).convert('RGB'), n) for f, n in items]
    w, h = ims[0][0].size; rows = (len(ims) + cols - 1) // cols
    sh = Image.new('RGB', (w * cols, h * rows + 34), 'white'); d = ImageDraw.Draw(sh); d.text((8, 8), title, fill='black', font=F)
    for i, (im, n) in enumerate(ims):
        x, y = (i % cols) * w, (i // cols) * h + 34; sh.paste(im, (x, y)); d.text((x + 6, y + 4), n, fill=(200, 60, 30), font=F)
    sh.save(OUT + '/' + fn); print(fn, sh.size)
v = json.load(open(OUT + '/parts.json')).get('version', '')
grid(S['shots'], 6, 'audit_sheet.png', f'KEDGE {v} audit sheet: 24 views (8 folded, 8 deployed, 6 mid-motion, top, under)')
grid(S['secs'], 4, 'sections.png', f'KEDGE {v} section cuts')
