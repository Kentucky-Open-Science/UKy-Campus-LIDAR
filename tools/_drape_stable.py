"""Verify the drape offset is STABLE (no LOD bobbing) under a stationary camera: poll the
applied overlay lift over ~18 s while Google tiles stream/refine, and report the time
series + how much it moves after it first settles.

    python tools/_drape_stable.py <port>
"""
import json, os, sys, time
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path = [p for p in sys.path if os.path.abspath(p or ".") != _HERE]
from playwright.sync_api import sync_playwright
PORT = sys.argv[1] if len(sys.argv) > 1 else "8134"
BASE = f"http://127.0.0.1:{PORT}"

FRAME = r"""() => { const v=window.__viewer, T=v.THREE;
  v.camera.position.set(420, 360, 420); v.controls.target.set(0, 290, 0); v.controls.update(); }"""
READ = r"""() => ({ off: window.__viewer.state.roadnet.group.position.y,
  loaded: (()=>{let n=0;try{window.__viewer.photoreal.tiles.forEachLoadedModel(()=>n++);}catch{}return n;})(),
  fps: (document.getElementById('fps')||{}).textContent })"""

with sync_playwright() as pw:
    b=pw.chromium.launch(args=['--ignore-gpu-blocklist','--use-gl=angle','--enable-unsafe-swiftshader'])
    pg=b.new_page(viewport={'width':1280,'height':900})
    pg.goto(f'{BASE}/?flat=0', wait_until='load')
    for _ in range(90):
        rs=pg.eval_on_selector('#road-status','e=>e.textContent') or ''
        ps=pg.eval_on_selector('#photoreal-status','e=>e.textContent') or ''
        if rs.startswith('roads:') and 'loading' not in rs and 'streaming' in ps: break
        time.sleep(0.5)
    pg.evaluate(FRAME)
    series=[]
    for i in range(36):                 # ~18 s, sampled every 0.5 s — camera held still
        r=pg.evaluate(READ)
        series.append((round(i*0.5,1), round(r['off'],2), r['loaded']))
        time.sleep(0.5)
    b.close()

print("t(s)  offset  tilesLoaded")
for t,o,l in series: print(f"{t:>4}  {o:>6}  {l}")
offs=[o for _,o,_ in series]
# stability after the first 6 s (allow initial settle)
tail=[o for t,o,_ in series if t>=6]
import statistics as st
print("\nfinal offset:", offs[-1])
print("tail (t>=6s) min/max/range:", min(tail), max(tail), round(max(tail)-min(tail),3))
print("tail stdev:", round(st.pstdev(tail),3))
print("VERDICT:", "STABLE (no bobbing)" if (max(tail)-min(tail))<0.6 else "STILL MOVING")
