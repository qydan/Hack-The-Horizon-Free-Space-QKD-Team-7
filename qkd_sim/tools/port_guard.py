"""Runtime no-bypass guard (section 11.4, test B2).

Runs a Python module with every ZeroMQ bind/connect and every TCP socket connect checked against an allow-list
of ports. Disallowed attempts are logged to ``--log`` as failures and refused (the call raises PermissionError).

    python -m tools.port_guard --allow 5556 5557 5559 5560 --log runs/x/sandbox_ports.jsonl -- sandbox.main

tools/run_scenario.py starts the sandbox this way with ``bus.yaml: allow_list.sandbox``.
"""
from __future__ import annotations

import argparse
import json
import re
import runpy
import socket
import sys
import time
from pathlib import Path

_ENDPOINT = re.compile(r"^tcp://(?P<host>[^:]+):(?P<port>\d+)$")


class PortGuard:
    def __init__(self, allowed: set[int], log_path: Path | None):
        self.allowed = set(int(p) for p in allowed)
        self.log_path = log_path
        self.events: list[dict] = []

    def _log(self, kind: str, target: str, port: int | None, ok: bool) -> None:
        ev = {"t_wall": time.time(), "kind": kind, "target": target, "port": port, "allowed": ok}
        self.events.append(ev)
        if self.log_path:
            with open(self.log_path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps(ev) + "\n")
        if not ok:
            print(f"[port_guard] FAILURE: {kind} to {target} is not on the allow-list {sorted(self.allowed)}",
                  file=sys.stderr)

    def check_endpoint(self, kind: str, endpoint: str) -> None:
        m = _ENDPOINT.match(endpoint)
        port = int(m.group("port")) if m else None
        ok = port is not None and port in self.allowed
        self._log(kind, endpoint, port, ok)
        if not ok:
            raise PermissionError(f"port_guard: {kind} {endpoint} not allowed")

    def install(self) -> None:
        guard = self
        try:
            import zmq
            orig_bind, orig_connect = zmq.Socket.bind, zmq.Socket.connect

            def bind(self, addr, *a, **k):
                guard.check_endpoint("zmq.bind", addr)
                return orig_bind(self, addr, *a, **k)

            def connect(self, addr, *a, **k):
                guard.check_endpoint("zmq.connect", addr)
                return orig_connect(self, addr, *a, **k)

            zmq.Socket.bind, zmq.Socket.connect = bind, connect
        except ImportError:
            pass
        orig_sock_connect = socket.socket.connect

        def sock_connect(self, address, *a, **k):
            if isinstance(address, tuple) and len(address) >= 2 and self.family in (socket.AF_INET, socket.AF_INET6):
                port = int(address[1])
                ok = port in guard.allowed
                guard._log("socket.connect", f"{address[0]}:{port}", port, ok)
                if not ok:
                    raise PermissionError(f"port_guard: connect to {address} not allowed")
            return orig_sock_connect(self, address, *a, **k)

        socket.socket.connect = sock_connect


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--allow", type=int, nargs="+", required=True)
    ap.add_argument("--log", type=Path, default=None)
    ap.add_argument("module")
    ap.add_argument("args", nargs=argparse.REMAINDER)
    a = ap.parse_args()
    PortGuard(set(a.allow), a.log).install()
    sys.argv = [a.module] + [x for x in a.args if x != "--"]
    try:
        runpy.run_module(a.module, run_name="__main__", alter_sys=True)
    except NotImplementedError:
        print(f"[port_guard] {a.module} is an OPEN placeholder (NotImplementedError); exiting cleanly")


if __name__ == "__main__":
    main()
