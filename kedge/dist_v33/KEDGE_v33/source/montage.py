import sys, glob, os
from PIL import Image, ImageDraw
d = sys.argv[1]; out = sys.argv[2]; files = sorted(glob.glob(os.path.join(d, '*.png')))[:int(sys.argv[3]) if len(sys.argv) > 3 else 99]
W, H = 700, 484; cols = 3; rows = (len(files) + cols - 1) // cols
M = Image.new('RGB', (W * cols, H * rows), 'white'); dr = ImageDraw.Draw(M)
for i, f in enumerate(files):
    im = Image.open(f).convert('RGB').resize((W, H)); M.paste(im, ((i % cols) * W, (i // cols) * H)); dr.text(((i % cols) * W + 6, (i // cols) * H + 4), os.path.basename(f), fill='red')
M.save(out); print(M.size)
