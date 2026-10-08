import sys, numpy as np
U = sys.argv[1]
src = open('bin.py').read(); src = src[:src.index('D = np.array')]
sys.argv = ['x', U, '5']
exec(src)
def solve_free(p, H, s, Hb, n=80):
    def res(q):
        tcp, aa, _, _ = fk(q); return np.r_[tcp - p, 0.05 * max(0, aa[2] + 0.5)]   # approach >=30 deg below horizontal
    for _ in range(n):
        r = least_squares(res, rng.uniform(lo, hi), bounds=(lo, hi), max_nfev=300)
        tcp, aa, _, _ = fk(r.x)
        if np.linalg.norm(tcp - p) < 0.002 and aa[2] <= -0.49 and not coll(r.x, H, s, Hb): return True
    return False
for s in (0.12,):
    for Hb in (0.25, 0.30):
        for H in (0.20, 0.25, 0.30):
            for label, dz in (('bottom', 0.05), ('rim', WALL + 0.05)):
                ok = [f"{x:+.2f}/{y:+.2f}" for x in (-0.29, -0.34, -0.39) for y in (0.0, 0.10, -0.10) if solve_free(np.array([x + s, y, Hb + dz - H]), H, s, Hb)]
                print(f"s={s} Hb={Hb} H={H} {label}: {len(ok)}/9 {ok}", flush=True)
