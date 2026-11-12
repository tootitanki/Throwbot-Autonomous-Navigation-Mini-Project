"""Writes the ground-truth static map of the room (walls + 4 fixed boxes, NOT the moving box) from the SDF geometry.
Map frame origin = robot start pose = world (-4.5, 0)."""
import sys, numpy as np
RES, OX, OY, W, H = 0.05, -2.0, -5.0, 280, 200
SHIFT = 4.5   # map x = world x + 4.5
BOXES = [(-2.5, 1.5, 1.0, 1.0), (-1.0, -2.0, 1.2, 0.8), (3.0, 2.0, 1.0, 1.5), (3.0, -2.5, 1.5, 1.0)]  # cx, cy, sx, sy (world)


def build():
    img = np.full((H, W), 254, np.uint8)           # free
    xs = OX + (np.arange(W) + 0.5) * RES           # map-frame x of each column
    ys = OY + (np.arange(H) + 0.5) * RES
    X, Y = np.meshgrid(xs, ys)
    wx = X - SHIFT
    occ = (np.abs(wx) > 6.0) | (np.abs(Y) > 4.0)
    for cx, cy, sx, sy in BOXES:
        occ |= (np.abs(wx - cx) <= sx / 2) & (np.abs(Y - cy) <= sy / 2)
    img[occ] = 0
    return img[::-1]                               # image row 0 = top = max y


if __name__ == '__main__':
    out = sys.argv[1]
    img = build()
    with open(out + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (W, H)); f.write(img.tobytes())
    open(out + '.yaml', 'w').write(
        f"image: {out.split('/')[-1]}.pgm\nresolution: {RES}\norigin: [{OX}, {OY}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
