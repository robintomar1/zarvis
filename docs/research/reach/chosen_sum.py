import json, os
R70 = 0.537
os.chdir('out_chosen')
def band(xs):
    if not xs: return None
    xs = sorted(xs); m = None; x = 0.05
    while any(abs(x - v) < 1e-6 for v in xs): m = round(x, 3); x += 0.025
    return (min(xs), max(xs), 'contig5->', m)
for s in ('0.0', '-0.05'):
    f = json.load(open(f'floor_s{s}.json'))
    for H in (0.2, 0.25):
        for y in (0, 0.1, -0.1, 0.2, -0.2):
            rs = [r for r in f['res'] if r['H'] == H and r['y'] == y]
            ok = [r['x'] for r in rs if r['st'] == 'ok']; c70 = [r['x'] for r in rs if r['st'] == 'ok' and r['ext'] <= R70]
            print('floor s', s, 'H', H, 'y', y, 'ok', band(ok), '<=70%', band(c70), 'fails', [(r['x'], r['st']) for r in rs if r['st'] != 'ok'])
    c = json.load(open(f'counter_s{s}.json'))
    for pl in ('flush', 'standoff'):
        for H in (0.85, 0.9, 0.92):
            for y in (0, 0.2, -0.2):
                rs = [r for r in c['res'] if r['place'] == pl and r['H'] == H and r['y'] == y]
                ok = [r['x'] for r in rs if r['st'] == 'ok']; c70 = [r['x'] for r in rs if r['st'] == 'ok' and r['ext'] <= R70]
                print('ctr s', s, pl, 'c=%.2f' % rs[0]['c'], 'H', H, 'y', y, 'ok', band(ok) if ok else rs[0]['st'], '<=70%', band(c70))
