"""Headless screenshots of the buildings layer in the web viewer (verification).
Writes extracted/view-buildings-oblique.png and view-buildings-topdown.png."""
import os
import socket
import subprocess
import sys
import time

from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, '..'))
WEB = os.path.join(ROOT, 'web')
OUT = os.path.join(ROOT, 'extracted')
PORT = 8137


def wait_port(p, t=10):
    for _ in range(t * 10):
        try:
            socket.create_connection(('127.0.0.1', p), 0.2).close()
            return True
        except OSError:
            time.sleep(0.1)
    return False


srv = subprocess.Popen([sys.executable, '-m', 'http.server', str(PORT)], cwd=WEB,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
try:
    wait_port(PORT)
    with sync_playwright() as pw:
        b = pw.chromium.launch(args=['--ignore-gpu-blocklist', '--use-gl=angle',
                                     '--enable-unsafe-swiftshader'])
        pg = b.new_page(viewport={'width': 1400, 'height': 1100})
        errors = []
        pg.on('console', lambda m: errors.append(m.text) if m.type == 'error' else None)
        pg.on('pageerror', lambda e: errors.append('PAGEERROR: ' + str(e)))
        pg.goto(f'http://127.0.0.1:{PORT}/', wait_until='load')

        import re
        ok = False
        for _ in range(600):                       # up to ~5 min
            bs = pg.eval_on_selector('#buildings-status', 'e=>e.textContent') or ''
            m = re.search(r'(\d+)/(\d+) loaded', bs)
            if m and m.group(1) == m.group(2) and int(m.group(2)) > 0:
                ok = True
                break
            time.sleep(0.5)
        time.sleep(1.5)
        print('loaded ok=', ok)
        print('terrain  :', pg.eval_on_selector('#terrain-status', 'e=>e.textContent'))
        print('buildings:', pg.eval_on_selector('#buildings-status', 'e=>e.textContent'))

        # dim the point cloud so buildings read clearly
        pg.evaluate('''() => {
          const v = window.__viewer;
          if (v.state.lidar.group) v.state.lidar.group.visible = false;
        }''')

        # oblique, everything on
        pg.evaluate('''() => {
          const v = window.__viewer; const T = v.THREE;
          v.state.buildings.group.visible = true;
          const box = new T.Box3().setFromObject(v.state.terrain.group);
          const c = box.getCenter(new T.Vector3());
          v.camera.position.set(c.x + 300, c.y + 240, c.z + 300);
          v.controls.target.set(c.x, c.y, c.z); v.controls.update();
        }''')
        time.sleep(1.0)
        pg.screenshot(path=os.path.join(OUT, 'view-buildings-oblique.png'))

        # top-down over the campus core
        pg.evaluate('''() => {
          const v = window.__viewer; const T = v.THREE;
          const box = new T.Box3().setFromObject(v.state.terrain.group);
          const c = box.getCenter(new T.Vector3());
          v.camera.position.set(c.x, c.y + 900, c.z + 1);
          v.controls.target.copy(c); v.controls.update();
        }''')
        time.sleep(1.0)
        pg.screenshot(path=os.path.join(OUT, 'view-buildings-topdown.png'))

        print('console errors:', len(errors))
        for e in errors[:10]:
            print('  !', e[:160])
        b.close()
finally:
    srv.terminate()
print('wrote extracted/view-buildings-oblique.png, view-buildings-topdown.png')
