import sys, json, glob
R70 = 0.70 * 0.767
def contig(lst, y=0.0, cap=None):
    ok = {round(o['x'], 3) for o in lst if o['y'] == y and o['cf'] and (cap is None or o['ext'] <= cap)}
    m = None; x = 0.05
    while round(x, 3) in ok: m = round(x, 3); x += 0.025
    return m
for f in sorted(glob.glob(sys.argv[1] + '/res_m5_*.json')):
    r = json.load(open(f)); print('==', f.split('/')[-1])
    print(' s    H   | floor y0 max(all/<=70%) | ctop | cside   [contiguous from 5cm, cm]')
    for row in r['rows']:
        g = lambda k: f"{(contig(row[k]) or 0)*100:4.1f}/{(contig(row[k], cap=R70) or 0)*100:4.1f}"
        print(f"{row['s']:.2f} {row['H']:.2f} | {g('floor')} | {g('ctop')} | {g('cside')}")
