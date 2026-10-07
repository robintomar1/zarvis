import sys, numpy as np
U = sys.argv[1]; H = float(sys.argv[2])
src = open('chosen.py').read(); src = src[:src.index("out = {'s'")]
sys.argv = ['x', U, '0.0', 'none']
exec(src)
Hb = 0.30; B = to_arm(world_boxes(Hb=Hb), H)
x0, x1 = BIN_X; y0, y1 = BIN_Y['L']; yc = (y0 + y1) / 2
res = []
for x in (x1 - 0.05, (x0 + x1) / 2, x0 + 0.05):
    for mode in ('vert30',):
        st, q = ik(np.array([x - xa, yc + 0.04, Hb + 0.03 - H]), mode, B, n=300)
        res.append((round(x, 2), st, jr(q)))
st, q = ik(np.array([x0 + 0.05 - xa, yc, Hb + WALL + 0.05 - H]), 'tilt30', B, n=300)
print('H', H, res, 'rear drop:', st, flush=True)
