"""Helper process that makes gz-transport service requests on behalf of a busy Python process.

Why: the gz-transport13 Python binding keeps the GIL while ``Node.request`` waits for the reply. In a process
that also receives frequent subscription callbacks (the 1 kHz sim clock, 30 Hz camera images) the callback
thread then blocks on the GIL, the reply is never processed and every request times out. A process with no
subscriptions does not have that problem (measured: ~0.4 ms per request, ~2,400 requests/s).

Protocol (ZeroMQ REQ/REP on 127.0.0.1, random port printed on the first stdout line as ``PORT <n>``):
  request:  [service, "module:Class" of request, "module:Class" of reply, serialized request, timeout_ms]
  reply:    [b"1" | b"0", serialized reply]
The proxy exits when its parent process goes away.
"""
from __future__ import annotations

import importlib
import os
import sys


def _cls(spec: str):
    mod, name = spec.split(":")
    return getattr(importlib.import_module(mod), name)


def main() -> None:
    import zmq
    from gz.transport13 import Node  # type: ignore

    parent = os.getppid()
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REP)
    port = sock.bind_to_random_port("tcp://127.0.0.1")
    print(f"PORT {port}", flush=True)
    node = Node()
    cache: dict[str, type] = {}
    while True:
        if not sock.poll(1000):
            if os.getppid() != parent:
                return
            continue
        service, req_t, rep_t, payload, timeout = sock.recv_multipart()
        try:
            req_cls = cache.setdefault(req_t.decode(), _cls(req_t.decode()))
            rep_cls = cache.setdefault(rep_t.decode(), _cls(rep_t.decode()))
            ok, rep = node.request(service.decode(), req_cls.FromString(payload), req_cls, rep_cls,
                                   int(timeout))
            sock.send_multipart([b"1" if ok else b"0", rep.SerializeToString() if ok else b""])
        except Exception as exc:  # noqa: BLE001
            print(f"proxy error: {exc!r}", file=sys.stderr, flush=True)
            sock.send_multipart([b"0", b""])


if __name__ == "__main__":
    main()
