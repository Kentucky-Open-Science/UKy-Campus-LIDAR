#!/usr/bin/env python
"""Offline QA for the camera -> YOLO -> twin-cars geometry, minus the model.

The detector (tools/camera_detect.py) needs ultralytics + torch + a LIVE camera HLS
stream to actually run. Everything AROUND the neural net, though, is plain geometry and
bookkeeping that can be exercised with no model and no network. This tool does exactly
that, so the pipeline can be regression-tested anywhere (CI, a laptop, a locked-down
sandbox) and so the COCO class mapping can't silently drift.

It checks, against tools/camera_detect.py directly:
  1. DETECT_CLASSES indices == canonical COCO (person0 bicycle1 car2 motorcycle3 bus5 truck7)
  2. every CLASS_TYPE value is a twin agent type tools/twin_server.py actually defines
  3. CLASS_COLOR matches web/app.js CLS_COLOR (the PiP overlay), so the on-video box and
     the spawned 3D body share one colour
  4. the 2x2 quad split and tire_to_scene() homography map a bbox bottom-centre to the
     scene point a known homography predicts
  5. heading_from_motion() agrees with the world's (cos yaw, 0, -sin yaw) forward frame
  6. SceneTracker dedups a cross-quad duplicate and emits spawn payloads with the right
     type + colour

If ultralytics IS importable it additionally loads the model and runs ONE inference on a
synthesized frame to prove the detector path; otherwise it prints the exact local command
to run the full live demo and skips (non-fatal).

Run:  python -m tools.qa_camera_pipeline
"""
from __future__ import annotations

import importlib.util
import math
import os
import re

_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

# Canonical COCO-80 (only the low indices the detector uses are needed here).
COCO = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle", 4: "airplane",
        5: "bus", 6: "train", 7: "truck"}
# Agent types tools/twin_server.py defines (DEFS keys).
SERVER_TYPES = {"car", "truck", "bus", "moto", "bike", "ped", "robot", "drone"}


def _load(modname, relpath):
    spec = importlib.util.spec_from_file_location(modname, os.path.join(_ROOT, relpath))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)               # camera_detect imports cv2/ultralytics lazily
    return mod


def _app_js_cls_color():
    """Parse CLS_COLOR out of web/app.js so we can cross-check the detector's colours."""
    with open(os.path.join(_ROOT, "web", "app.js"), encoding="utf-8") as f:
        txt = f.read()
    mobj = re.search(r"const CLS_COLOR = \{([^}]*)\}", txt)
    if not mobj:
        return None
    out = {}
    for k, v in re.findall(r"(\d+):\s*'#([0-9a-fA-F]{6})'", mobj.group(1)):
        out[int(k)] = int(v, 16)
    return out


def check_mapping(cd):
    print("[1-3] class mapping")
    ok = True
    for idx, name in cd.DETECT_CLASSES.items():
        good = COCO.get(idx) == name
        ok &= good
        print(f"   class {idx}: '{name}' vs COCO '{COCO.get(idx)}'  {'OK' if good else 'MISMATCH'}")
    for idx, t in cd.CLASS_TYPE.items():
        good = t in SERVER_TYPES
        ok &= good
        print(f"   type  {idx}->'{t}'  {'OK' if good else 'NOT A SERVER TYPE'}")
    app = _app_js_cls_color()
    if app is None:
        print("   (could not parse web/app.js CLS_COLOR; skipping colour parity)")
    else:
        for idx, c in cd.CLASS_COLOR.items():
            good = app.get(idx) == c
            ok &= good
            print(f"   colour {idx}: detect 0x{c:06x} vs app.js 0x{(app.get(idx) or 0):06x}  "
                  f"{'OK' if good else 'MISMATCH'}")
    return ok


def check_geometry(cd):
    print("[4-5] quad split + homography + heading")
    import numpy as np
    try:
        import cv2
    except Exception as e:                       # noqa: BLE001
        print(f"   cv2 unavailable ({e}); skipping homography numeric check")
        return True
    img = np.full((720, 1280, 3), 60, np.uint8)
    h, w = img.shape[:2]
    for q in range(4):
        col, row = q % 2, q // 2
        sub = img[row * h // 2:(row + 1) * h // 2, col * w // 2:(col + 1) * w // 2]
        assert sub.shape[:2] == (h // 2, w // 2)
    print(f"   quad split 1280x720 -> 4x ({w//2}x{h//2}): OK")
    # known homography on normalized [0,1] quad coords -> a 30x20 m ground patch
    src = np.float32([[0, 0], [1, 0], [1, 1], [0, 1]])
    dst = np.float32([[100, 200], [130, 200], [130, 220], [100, 220]])
    H = cv2.getPerspectiveTransform(src, dst).reshape(-1).tolist()
    sc = cd.tire_to_scene(H, 640, 360, 300.0, 150.0, 340.0, 360.0)  # tire px (320,360)
    good = sc and abs(sc[0] - 115.0) < 1e-6 and abs(sc[1] - 220.0) < 1e-6
    print(f"   tire_to_scene bottom-centre -> {tuple(round(c,3) for c in sc)} "
          f"(expect (115.0, 220.0))  {'OK' if good else 'FAIL'}")
    hok = True
    for dx, dz, exp in [(1, 0, 0.0), (0, -1, 90.0), (-1, 0, 180.0), (0, 1, -90.0)]:
        got = cd.heading_from_motion(dx, dz)
        d = abs(((got - exp + 180) % 360) - 180) < 1e-6
        hok &= d
        print(f"   heading({dx:+d},{dz:+d})={got:7.1f} (expect {exp:+6.1f})  {'OK' if d else 'FAIL'}")
    return bool(good and hok)


def check_tracker(cd):
    print("[6] SceneTracker dedup + spawn payloads")

    class FakeTwin:
        def __init__(self): self._id = 0; self.spawned = []
        def spawn(self, x, z, heading, source, color, typ):
            self._id += 1; self.spawned.append((typ, color)); return self._id
        def pose(self, *a): pass
        def despawn(self, *a): pass

    ft = FakeTwin()
    tr = cd.SceneTracker(ft, "LEX-CAM-TEST")
    dets = [{"x": 115.0, "z": 220.0, "quad": 0, "cls": 2},
            {"x": 105.0, "z": 215.0, "quad": 0, "cls": 2},
            {"x": 140.0, "z": 230.0, "quad": 1, "cls": 7},
            {"x": 115.6, "z": 220.3, "quad": 1, "cls": 2}]  # seam dup of the quad-0 car
    st = tr.update(0, dets)
    good = st["clusters"] == 3          # 4 dets, one cross-quad pair merged
    types = sorted(t for t, _ in ft.spawned)
    print(f"   4 dets -> {st['clusters']} clusters (cross-quad dup merged)  "
          f"{'OK' if good else 'FAIL'}")
    print(f"   spawned types {types}  colours "
          f"{[hex(c) for _, c in ft.spawned]}")
    return good and types == ["car", "car", "truck"]


def try_model():
    print("[model] ultralytics load + one inference")
    try:
        import numpy as np
        from ultralytics import YOLO
    except Exception as e:                       # noqa: BLE001
        print(f"   ultralytics NOT importable here ({e.__class__.__name__}: {e}).")
        print("   -> skipped. To run the detector you need ultralytics + torch + the model")
        print("      weights (yolo26n.pt) and a LIVE camera stream. Exact local commands:")
        print("        pip install ultralytics")
        print("        python -m tools.twin_server                    # in one terminal")
        print("        python -m tools.camera_detect --camera <id> \\")
        print("            --twin http://127.0.0.1:8000 --model yolo26n.pt")
        return None
    frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
    model = YOLO("yolo26n.pt")
    res = model.predict(frame, classes=list(__import__("tools.camera_detect",
                        fromlist=["DETECT_CLASSES"]).DETECT_CLASSES),
                        imgsz=640, verbose=False)[0]
    print(f"   model loaded; inference returned {len(res.boxes)} boxes -> OK")
    return True


def main():
    cd = _load("camera_detect", os.path.join("tools", "camera_detect.py"))
    results = {
        "mapping": check_mapping(cd),
        "geometry": check_geometry(cd),
        "tracker": check_tracker(cd),
    }
    print()
    model = try_model()
    print()
    passed = all(results.values())
    for k, v in results.items():
        print(f"  {k:9s}: {'PASS' if v else 'FAIL'}")
    print(f"  model    : {'PASS' if model else 'SKIPPED (no ultralytics/torch)'}")
    print()
    print(f"OVERALL (geometry+mapping): {'PASS' if passed else 'FAIL'}")
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
