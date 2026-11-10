"""Live dashboard: follows a run log while the demo is running and redraws every second.

    python src/demo_runner.py --source zmq --log run.jsonl        # the sandbox writes the log
    python tools/live_dashboard.py run.jsonl                      # then open http://127.0.0.1:8000

Shows the last 60 s of the same charts as tools/make_dashboard.py (QBER against the 11% gate,
the two error bars, drift and jitter, intensity, loss) plus the gate decision for each key
block. Standard library only; it serves on localhost.
"""
import argparse
import json
import os
import sys
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_dashboard as md  # noqa: E402

SCRIPT = """<script>
async function tick(){try{const r=await fetch('/body',{cache:'no-store'});
if(r.ok)document.getElementById('m').innerHTML=await r.text();}catch(e){}}
setInterval(tick,1000);tick();</script>"""
WAITING = "<h1>QKD Live Dashboard</h1><p>Waiting for the sandbox to write the run log...</p>"


def read_log(path):
    """Complete JSON lines of the log so far (a half-written last line is skipped). The secret
    key material is dropped: this page shows decisions, not keys."""
    rows = []
    try:
        with open(path) as f:
            for line in f:
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                r.pop("key", None)
                rows.append(r)
    except OSError:
        pass
    return rows


def make_handler(path, window_s):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, text, ctype="text/html; charset=utf-8", code=200):
            data = text.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            body = md.body_html(read_log(path), window_s, suffix=" (live)") or WAITING
            if self.path == "/body":
                self._send(body)
            elif self.path in ("/", "/index.html"):
                self._send(md.PAGE.replace("__BODY__", body).replace("__SCRIPT__", SCRIPT))
            else:
                self._send("not found", "text/plain", 404)

        def log_message(self, *args):
            pass

    return Handler


def main(argv=None):
    ap = argparse.ArgumentParser(description="Live view of a demo run log")
    ap.add_argument("log", help="JSONL run log written by demo_runner.py --log")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--window", type=float, default=60.0, help="seconds of history shown in the traces")
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args(argv)
    server = ThreadingHTTPServer(("127.0.0.1", a.port), make_handler(a.log, a.window))
    url = f"http://127.0.0.1:{a.port}"
    print(f"live dashboard on {url} (following {a.log}; Ctrl+C to stop)")
    if not a.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
