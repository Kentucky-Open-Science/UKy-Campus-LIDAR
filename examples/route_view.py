"""Live top-down MJPEG video stream for the in-process gym demos.

The campus gym runs headless (no server, no renderer), so to *watch* an agent drive
this draws a following top-down view — roads, buildings, the planned route, the goal,
and the car — with PIL and serves it as an MJPEG stream you open in a browser:

    from route_view import StreamServer, TopDownRenderer
    srv = StreamServer(); srv.start("127.0.0.1", 8009)     # open http://127.0.0.1:8009/
    rend = TopDownRenderer(route_pts, buildings, goal, corridor)
    srv.publish(rend.render_jpeg(car={"x":x,"z":z,"yaw":yaw,"speed":s}, hud={...}))

Pure stdlib HTTP + Pillow (already a demo dependency). No display required, so it works
the same on a headless box or over SSH — just browse to the printed URL.
"""
import io
import json
import math
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from PIL import Image, ImageDraw, ImageFont

# import the sim's data dir to draw the surrounding street grid
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from tools.twin_server import DATA  # noqa: E402


# ----------------------------------------------------------------- streaming ---
_PAGE = (b"<!doctype html><html><head><meta charset=utf-8><title>campus car</title>"
         b"<style>html,body{margin:0;height:100%;background:#0c0e12;"
         b"display:flex;align-items:center;justify-content:center}"
         b"img{max-width:100vw;max-height:100vh;image-rendering:auto}</style></head>"
         b"<body><img src='/stream'></body></html>")


class StreamServer:
    """Holds the latest JPEG and fans it out to every connected browser as MJPEG."""

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None          # (frame_id, jpeg_bytes)
        self._id = 0
        self.closed = False
        self.httpd = None
        self.url = None

    def publish(self, jpeg):
        with self._cond:
            self._id += 1
            self._frame = (self._id, jpeg)
            self._cond.notify_all()

    def wait_frame(self, last_id, timeout):
        """Block until a frame newer than `last_id` exists (or timeout). -> (id, jpeg) | None."""
        with self._cond:
            if self._frame is not None and self._frame[0] != last_id:
                return self._frame
            self._cond.wait(timeout)
            if self.closed:
                return None
            if self._frame is not None and self._frame[0] != last_id:
                return self._frame
            return None

    def start(self, host="127.0.0.1", port=8009):
        handler = _make_handler(self)
        self.httpd = ThreadingHTTPServer((host, port), handler)
        self.httpd.daemon_threads = True
        shown = "127.0.0.1" if host in ("", "0.0.0.0") else host
        self.url = f"http://{shown}:{self.httpd.server_address[1]}/"
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        return self.url

    def close(self):
        self.closed = True
        with self._cond:
            self._cond.notify_all()
        if self.httpd is not None:
            threading.Thread(target=self.httpd.shutdown, daemon=True).start()


def _make_handler(server):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):          # keep the console clean
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(_PAGE)))
                self.end_headers()
                self.wfile.write(_PAGE)
                return
            if self.path.rstrip("/") == "/stream":
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                last = None
                try:
                    while not server.closed:
                        got = server.wait_frame(last, timeout=4.0)
                        if got is None:
                            continue
                        last, data = got[0], got[1]
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                        self.wfile.write(data)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass                    # browser tab closed — end this stream thread
                return
            self.send_error(404)
    return Handler


# ----------------------------------------------------------------- rendering ---
def _font(size):
    for name in ("arial.ttf", "DejaVuSans.ttf", "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _load_road_segments(bounds):
    """Road centreline segments [((x1,z1),(x2,z2)), ...] within `bounds` (x0,z0,x1,z1)."""
    x0, z0, x1, z1 = bounds
    path = os.path.join(DATA, "roads.json")
    segs = []
    try:
        roads = json.load(open(path)).get("roads", [])
    except Exception:  # noqa: BLE001 — drawing the grid is optional
        return segs
    for rd in roads:
        pts = rd.get("pts") or []
        for i in range(len(pts) - 1):
            ax, az = pts[i][0], pts[i][2]
            bx, bz = pts[i + 1][0], pts[i + 1][2]
            if (max(ax, bx) >= x0 and min(ax, bx) <= x1 and
                    max(az, bz) >= z0 and min(az, bz) <= z1):
                segs.append(((ax, az), (bx, bz)))
    return segs


class TopDownRenderer:
    """Renders a following top-down JPEG: world +x -> right, world +z -> down."""

    def __init__(self, route_pts, buildings, goal, corridor=8.0,
                 size=720, window_m=300.0):
        self.route = [(float(x), float(z)) for x, z in route_pts]
        self.buildings = buildings
        self.goal = (float(goal[0]), float(goal[1]))
        self.corridor = float(corridor)
        self.size = int(size)
        self.window_m = float(window_m)
        self.scale = self.size / self.window_m
        self._hud_font = _font(16)
        self._small_font = _font(13)
        # the whole-route street grid, loaded once (route bbox + a margin)
        xs = [p[0] for p in self.route] + [self.goal[0]]
        zs = [p[1] for p in self.route] + [self.goal[1]]
        m = 60.0
        self.roads = _load_road_segments((min(xs) - m, min(zs) - m,
                                          max(xs) + m, max(zs) + m))

    # world -> pixel, centred on (cx,cz)
    def _px(self, x, z, cx, cz):
        return (int((x - cx) * self.scale + self.size / 2),
                int((z - cz) * self.scale + self.size / 2))

    def _buildings_in_view(self, cx, cz, half):
        b = self.buildings
        cell = b.cell
        x0, x1, z0, z1 = cx - half, cx + half, cz - half, cz + half
        out = []
        for gx in range(int(x0 // cell) - 1, int(x1 // cell) + 2):
            for gz in range(int(z0 // cell) - 1, int(z1 // cell) + 2):
                for it in b.grid.get((gx, gz), ()):
                    mn, mx = it["min"], it["max"]
                    if mx[0] >= x0 and mn[0] <= x1 and mx[2] >= z0 and mn[2] <= z1:
                        out.append(it)
        return out

    def render_jpeg(self, car, hud=None, quality=72):
        cx, cz = float(car["x"]), float(car["z"])
        yaw = float(car.get("yaw", 0.0))
        half = self.window_m / 2.0
        img = Image.new("RGB", (self.size, self.size), (16, 18, 23))
        d = ImageDraw.Draw(img, "RGBA")

        # buildings in view
        for it in self._buildings_in_view(cx, cz, half + 40):
            mn, mx = it["min"], it["max"]
            p0 = self._px(mn[0], mn[2], cx, cz)
            p1 = self._px(mx[0], mx[2], cx, cz)
            d.rectangle([min(p0[0], p1[0]), min(p0[1], p1[1]),
                         max(p0[0], p1[0]), max(p0[1], p1[1])],
                        fill=(44, 48, 58), outline=(64, 70, 84))

        # surrounding street grid (faint)
        for (a, b) in self.roads:
            if (max(a[0], b[0]) < cx - half or min(a[0], b[0]) > cx + half or
                    max(a[1], b[1]) < cz - half or min(a[1], b[1]) > cz + half):
                continue
            d.line([self._px(*a, cx, cz), self._px(*b, cx, cz)], fill=(70, 76, 90), width=2)

        # the planned route: corridor band + bright centreline
        line = [self._px(x, z, cx, cz) for (x, z) in self.route]
        d.line(line, fill=(40, 90, 180, 90), width=max(2, int(2 * self.corridor * self.scale)))
        d.line(line, fill=(90, 170, 255), width=3, joint="curve")

        # goal marker
        gx, gy = self._px(self.goal[0], self.goal[1], cx, cz)
        d.ellipse([gx - 9, gy - 9, gx + 9, gy + 9], outline=(90, 230, 130), width=3)
        d.line([gx - 13, gy, gx + 13, gy], fill=(90, 230, 130), width=2)
        d.line([gx, gy - 13, gx, gy + 13], fill=(90, 230, 130), width=2)

        # the car: a heading-aligned triangle (forward = (cos yaw, -sin yaw))
        fx, fz = math.cos(yaw), -math.sin(yaw)
        rx, rz = -fz, fx                                   # right = forward rotated 90°
        L, Wc = 9.0, 5.0                                   # marker size in metres
        in_bldg = bool((hud or {}).get("in_bldg"))
        body = (240, 90, 80) if in_bldg else (250, 210, 70)
        tip = (cx + fx * L, cz + fz * L)
        bl = (cx - fx * L * 0.6 + rx * Wc, cz - fz * L * 0.6 + rz * Wc)
        br = (cx - fx * L * 0.6 - rx * Wc, cz - fz * L * 0.6 - rz * Wc)
        d.polygon([self._px(*tip, cx, cz), self._px(*bl, cx, cz), self._px(*br, cx, cz)],
                  fill=body, outline=(20, 20, 20))

        if hud:
            self._draw_hud(d, hud)
        buf = io.BytesIO()
        img.save(buf, "JPEG", quality=quality)
        return buf.getvalue()

    def _draw_hud(self, d, hud):
        lines = []
        if hud.get("title"):
            lines.append(hud["title"])
        if "progress" in hud:
            lines.append(f"progress {hud['progress']:.0%}   remaining {hud.get('remaining', 0):.0f} m")
        if "speed" in hud:
            lines.append(f"speed {hud['speed']:.1f} m/s   t {hud.get('sim_s', 0):.0f} s")
        flags = []
        if "on_road" in hud:
            flags.append("ON-ROAD" if hud["on_road"] else "OFF-ROAD")
        if hud.get("in_bldg"):
            flags.append("IN BUILDING")
        if hud.get("reached"):
            flags.append("ARRIVED")
        if flags:
            lines.append("  ".join(flags))
        if "ret" in hud:
            lines.append(f"return {hud['ret']:.0f}")
        pad, y = 10, 8
        w = 268
        d.rectangle([6, 4, 6 + w, 4 + 22 * len(lines) + 8], fill=(0, 0, 0, 130))
        for i, ln in enumerate(lines):
            font = self._hud_font if i == 0 else self._small_font
            d.text((pad + 6, y), ln, fill=(235, 238, 245), font=font)
            y += 22
