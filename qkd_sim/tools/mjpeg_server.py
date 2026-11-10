"""Tiny MJPEG-over-HTTP viewer, so the camera can be watched in a browser (also from Docker on Windows/macOS,
where no X11 window is available).

    server = MjpegServer(port=8080).start()
    server.update(bgr_image, ["line 1", "line 2"])     # call for every frame; encoding is rate-limited
    # open http://localhost:8080

Stdlib + OpenCV only. Raw frames and text overlays only: no detection.
"""
from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

PAGE = b"""<!doctype html><html><head><title>qkd_sim camera</title><style>
body{margin:0;background:#111;color:#ddd;font:14px system-ui,sans-serif;text-align:center}
img{max-width:100%;height:auto;display:block;margin:0 auto}p{margin:8px}</style></head>
<body><img src="/stream" alt="waiting for frames..."><p>Action 5 Pro (simulated), raw frames.
Refresh if the stream stops.</p></body></html>"""


class MjpegServer:
    def __init__(self, port: int = 8080, host: str = "0.0.0.0", *, max_width: int = 960, quality: int = 75,
                 max_fps: float = 15.0):
        self.addr = (host, int(port))
        self.max_width = max_width
        self.quality = quality
        self.min_dt = 1.0 / max_fps
        self._jpeg: bytes | None = None
        self._seq = 0
        self._cv = threading.Condition()
        self._last = 0.0
        self._httpd: ThreadingHTTPServer | None = None

    # ------------------------------------------------------------------------------------------ frames
    def update(self, bgr: np.ndarray, lines: list[str] | None = None) -> bool:
        """Offer a frame; returns True if it was encoded (False when rate-limited)."""
        import cv2
        now = time.monotonic()
        if now - self._last < self.min_dt:
            return False
        self._last = now
        h, w = bgr.shape[:2]
        if w > self.max_width:
            bgr = cv2.resize(bgr, (self.max_width, int(h * self.max_width / w)), interpolation=cv2.INTER_AREA)
        else:
            bgr = bgr.copy()
        if lines:
            font, scale = cv2.FONT_HERSHEY_SIMPLEX, 0.55
            width = max(cv2.getTextSize(t, font, scale, 1)[0][0] for t in lines)
            box = bgr[0:12 + 22 * len(lines), 0:20 + width]
            box[:] = (box * 0.35).astype(box.dtype)                  # darken behind the text
            for i, text in enumerate(lines):
                cv2.putText(bgr, text, (10, 20 + 22 * i), font, scale, (255, 255, 255), 1, cv2.LINE_AA)
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return False
        with self._cv:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._cv.notify_all()
        return True

    def latest(self) -> bytes | None:
        with self._cv:
            return self._jpeg

    # ------------------------------------------------------------------------------------------ http
    def start(self) -> "MjpegServer":
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):            # keep the console quiet
                pass

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(PAGE)))
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path.startswith("/frame.jpg"):
                    jpg = server.latest()
                    if jpg is None:
                        self.send_error(503, "no frame yet")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpg)))
                    self.end_headers()
                    self.wfile.write(jpg)
                elif self.path.startswith("/stream"):
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-cache")
                    self.end_headers()
                    seen = -1
                    try:
                        while True:
                            with server._cv:
                                server._cv.wait_for(lambda: server._seq != seen, timeout=5.0)
                                jpg, seen = server._jpeg, server._seq
                            if jpg is None:
                                continue
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                        return
                else:
                    self.send_error(404)

        self._httpd = ThreadingHTTPServer(self.addr, Handler)
        self._httpd.daemon_threads = True
        threading.Thread(target=self._httpd.serve_forever, name="mjpeg-http", daemon=True).start()
        return self

    @property
    def port(self) -> int:
        return self._httpd.server_address[1] if self._httpd else self.addr[1]

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
