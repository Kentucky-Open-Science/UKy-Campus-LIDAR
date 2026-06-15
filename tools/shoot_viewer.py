"""Headless screenshot of the web viewer for verification.
Starts a static server, loads the viewer, waits for terrain + roads, then writes
a top-down and an oblique screenshot to /tmp."""
import subprocess, sys, time, os, socket
from playwright.sync_api import sync_playwright

HERE = os.path.dirname(os.path.abspath(__file__))
WEB = os.path.normpath(os.path.join(HERE, '..', 'web'))
PORT = 8131


def wait_port(p, t=10):
    for _ in range(t * 10):
        try:
            socket.create_connection(('127.0.0.1', p), 0.2).close(); return True
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
        pg = b.new_page(viewport={'width': 1280, 'height': 1280})
        errors = []
        pg.on('console', lambda m: errors.append(m.text) if m.type == 'error' else None)
        pg.on('pageerror', lambda e: errors.append('PAGEERROR: ' + str(e)))
        pg.goto(f'http://127.0.0.1:{PORT}/', wait_until='load')

        # wait for terrain tiles + roads to load
        ok = False
        for _ in range(200):
            t = pg.eval_on_selector('#terrain-status', 'e=>e.textContent') or ''
            r = pg.eval_on_selector('#road-status', 'e=>e.textContent') or ''
            if 'tiles loaded' in t and r.startswith('roads:') and 'loading' not in r:
                ok = True; break
            time.sleep(0.5)
        time.sleep(2.0)
        print('loaded ok=', ok)
        print('terrain:', pg.eval_on_selector('#terrain-status', 'e=>e.textContent'))
        print('roads  :', pg.eval_on_selector('#road-status', 'e=>e.textContent'))

        # top-down, zoomed on a sub-area, BUILDINGS HIDDEN -> verify ribbons land
        # on the aerial streets
        pg.evaluate('''() => {
          const v = window.__viewer; const T = v.THREE;
          v.state.buildings.group.visible = false;
          const box = new T.Box3().setFromObject(v.state.terrain.group);
          const c = box.getCenter(new T.Vector3());
          v.camera.position.set(c.x, c.y + 320, c.z + 1);
          v.controls.target.copy(c); v.controls.update();
        }''')
        time.sleep(1.0)
        pg.screenshot(path='/tmp/view_topdown.png')

        # alignment proof: recolor ribbons bright red, hide props, top-down zoom
        pg.evaluate('''() => {
          const v = window.__viewer; const T = v.THREE;
          const rn = v.state.roadnet;
          rn.layers.trees.visible = false; rn.layers.cars.visible = false;
          rn.layers.signals.visible = false; rn.layers.markings.visible = false;
          const m = rn.layers.roads.getObjectByName('road-ribbons');
          if (m) { m.material.color.setHex(0xff2020); m.material.emissive = new T.Color(0x661010); }
          const box = new T.Box3().setFromObject(v.state.terrain.group);
          const c = box.getCenter(new T.Vector3());
          v.camera.position.set(c.x, c.y + 1400, c.z + 1);
          v.controls.target.copy(c); v.controls.update();
        }''')
        time.sleep(1.0)
        pg.screenshot(path='/tmp/view_redroads.png')
        # restore
        pg.evaluate('''() => {
          const v = window.__viewer; const rn = v.state.roadnet; const T = v.THREE;
          rn.layers.trees.visible = true; rn.layers.cars.visible = true;
          rn.layers.signals.visible = true; rn.layers.markings.visible = true;
          const m = rn.layers.roads.getObjectByName('road-ribbons');
          if (m) { m.material.color.setHex(0x2c2f34); m.material.emissive = new T.Color(0x000000); }
        }''')

        # nice oblique with everything on
        pg.evaluate('''() => {
          const v = window.__viewer; const T = v.THREE;
          v.state.buildings.group.visible = true;
          const box = new T.Box3().setFromObject(v.state.terrain.group);
          const c = box.getCenter(new T.Vector3());
          v.camera.position.set(c.x + 220, c.y + 180, c.z + 220);
          v.controls.target.set(c.x, c.y, c.z); v.controls.update();
        }''')
        time.sleep(1.0)
        pg.screenshot(path='/tmp/view_oblique.png')

        print('console errors:', len(errors))
        for e in errors[:10]:
            print('  !', e[:160])
        b.close()
finally:
    srv.terminate()
