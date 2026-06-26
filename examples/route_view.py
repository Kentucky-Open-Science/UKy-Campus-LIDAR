"""Tiny MJPEG stream server: fan the latest JPEG out to every connected browser.

Used by car_rl_route.py --watch to re-serve the car's FIRST-PERSON camera — the frames
the mirrored car receives from the twin_server's renderer — as a video stream you open
in a browser. Pure stdlib (no rendering here; the frames come from the server).

    from route_view import StreamServer
    srv = StreamServer(); url = srv.start("127.0.0.1", 8009)   # open `url` in a browser
    srv.publish(jpeg_bytes)                                    # call repeatedly with fresh frames
    srv.close()
"""
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_PAGE = (b"<!doctype html><html><head><meta charset=utf-8><title>car camera</title>"
         b"<style>html,body{margin:0;height:100%;background:#0c0e12;"
         b"display:flex;align-items:center;justify-content:center}"
         b"img{max-width:100vw;max-height:100vh}</style></head>"
         b"<body><img src='/stream'></body></html>")


class StreamServer:
    """Holds the latest JPEG and streams it to every connected browser as MJPEG."""

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
        self.httpd = ThreadingHTTPServer((host, port), _make_handler(self))
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
            httpd = self.httpd

            def _stop():
                httpd.shutdown()
                httpd.server_close()       # release the listening socket / free the port

            threading.Thread(target=_stop, daemon=True).start()


def _make_handler(server):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):          # keep the console clean
            pass

        def do_HEAD(self):
            ok = self.path in ("/", "/index.html") or self.path.rstrip("/") == "/stream"
            self.send_response(200 if ok else 404)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()

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
