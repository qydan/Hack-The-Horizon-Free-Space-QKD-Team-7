"""B1 (static no-bypass), B2 (runtime port allow-list) and C1 (OPEN blocks are stubs only)."""
from __future__ import annotations

import ast
import json
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
BUS = yaml.safe_load((ROOT / "config" / "bus.yaml").read_text())
TRUTH_PORT = BUS["ports"]["truth"]

FORBIDDEN_IMPORT_PREFIXES = ("sim_bridge.truth_publisher", "evaluation", "gz")
TRACKING_ALLOWED_LOCAL = ("sim_bridge.frame_source", "sim_bridge.gimbal_io", "telemetry")
LOCAL_PACKAGES = ("sim_bridge", "evaluation", "telemetry", "tools", "tracking", "calibration", "sandbox", "tests")
FORBIDDEN_LITERALS = (str(TRUTH_PORT), "camera_truth.yaml", "camera_truth", "allow_truth")


def py_files(pkg: str) -> list[Path]:
    return sorted((ROOT / pkg).rglob("*.py"))


def imported_modules(tree: ast.AST) -> list[str]:
    mods = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods += [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods.append(node.module)
            mods += [f"{node.module}.{a.name}" for a in node.names]
        elif isinstance(node, ast.Call) and getattr(node.func, "attr", getattr(node.func, "id", "")) in (
                "import_module", "__import__"):
            if node.args and isinstance(node.args[0], ast.Constant):
                mods.append(str(node.args[0].value))
    return mods


def violations(path: Path, *, tracking: bool) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    out = []
    for mod in imported_modules(tree):
        if any(mod == p or mod.startswith(p + ".") for p in FORBIDDEN_IMPORT_PREFIXES):
            out.append(f"imports {mod}")
        if tracking and mod.split(".")[0] in LOCAL_PACKAGES and mod.split(".")[0] != "tracking":
            if not any(mod == a or mod.startswith(a + ".") for a in TRACKING_ALLOWED_LOCAL):
                out.append(f"tracking may not import {mod}")
    for node in ast.walk(tree):
        if isinstance(node, ast.keyword) and node.arg == "allow_truth":
            out.append("uses allow_truth (truth port is evaluation-only)")
        if isinstance(node, ast.Constant) and isinstance(node.value, (str, int)):
            val = str(node.value)
            if (isinstance(node.value, int) and node.value == TRUTH_PORT) or any(
                    lit in val for lit in FORBIDDEN_LITERALS if not lit.isdigit()) or (
                    isinstance(node.value, str) and str(TRUTH_PORT) in val):
                out.append(f"literal {val!r}")
    return out


@pytest.mark.parametrize("pkg", ["tracking", "sandbox"])
def test_b1_static_no_bypass(pkg):
    files = py_files(pkg)
    assert files, f"{pkg}/ has no Python files"
    bad = {str(p.relative_to(ROOT)): v for p in files if (v := violations(p, tracking=(pkg == "tracking")))}
    assert not bad, f"no-bypass violations: {bad}"


def test_b1_checker_catches_violations(tmp_path):
    """The checker itself must flag the patterns it is meant to catch."""
    cases = {
        "a.py": "import sim_bridge.truth_publisher\n",
        "b.py": "from evaluation import score\n",
        "c.py": "from gz.transport13 import Node\n",
        "d.py": f"PORT = {TRUTH_PORT}\n",
        "e.py": "p = 'evaluation/camera_truth.yaml'\n",
        "f.py": f"ep = 'tcp://127.0.0.1:{TRUTH_PORT}'\n",
        "g.py": "from sim_bridge.fan_wind import FanJet\n",
        "h.py": "from telemetry.bus import Subscriber\ns = Subscriber('truth', allow_truth=True)\n",
    }
    for name, src in cases.items():
        p = tmp_path / name
        p.write_text(src)
        assert violations(p, tracking=True), f"checker missed {name}: {src!r}"
    ok = tmp_path / "ok.py"
    ok.write_text("from sim_bridge.frame_source import Frame\nfrom sim_bridge.gimbal_io import GimbalIO\n"
                  "from telemetry.bus import Publisher\n")
    assert not violations(ok, tracking=True)


def _run_guard(module_args: list[str], log: Path, allow: list[int]) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, "-m", "tools.port_guard", "--allow", *map(str, allow), "--log", str(log),
                           *module_args], cwd=ROOT, capture_output=True, text=True, timeout=60)


def test_b2_sandbox_uses_only_allowed_ports(tmp_path):
    """Run sandbox.main under the port guard (as run_scenario does). A placeholder exits at once; an implemented
    sandbox keeps running, so it is stopped after a few seconds. Every connection must be on the allow-list."""
    log = tmp_path / "ports.jsonl"
    proc = subprocess.Popen([sys.executable, "-m", "tools.port_guard", "--allow",
                             *map(str, BUS["allow_list"]["sandbox"]), "--log", str(log), "sandbox.main"],
                            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        _, err = proc.communicate(timeout=8)
        assert proc.returncode == 0, err
    except subprocess.TimeoutExpired:          # implemented sandbox: still listening, which is fine
        proc.kill()
        _, err = proc.communicate(timeout=10)
    events = [json.loads(x) for x in log.read_text().splitlines()] if log.exists() else []
    assert all(e["allowed"] for e in events), events
    assert "FAILURE" not in (err or "")


def test_b2_guard_flags_truth_port(tmp_path):
    """A sandbox that tries to reach the truth port is caught and logged as a failure."""
    mod = tmp_path / "evil_sandbox.py"
    mod.write_text(
        "import zmq\n"
        "s = zmq.Context.instance().socket(zmq.SUB)\n"
        "try:\n"
        f"    s.connect('tcp://127.0.0.1:{TRUTH_PORT}')\n"
        "except PermissionError:\n"
        "    pass\n"
        "s.connect('tcp://127.0.0.1:5556')\n")
    log = tmp_path / "ports.jsonl"
    res = subprocess.run([sys.executable, "-m", "tools.port_guard", "--allow",
                          *map(str, BUS["allow_list"]["sandbox"]), "--log", str(log), "evil_sandbox"],
                         cwd=ROOT, capture_output=True, text=True, timeout=60,
                         env={**__import__("os").environ, "PYTHONPATH": f"{tmp_path}{__import__('os').pathsep}{ROOT}"})
    assert res.returncode == 0, res.stderr
    events = [json.loads(x) for x in log.read_text().splitlines()]
    bad = [e for e in events if not e["allowed"]]
    assert len(bad) == 1 and bad[0]["port"] == TRUTH_PORT
    assert any(e["allowed"] and e["port"] == 5556 for e in events)
    assert "FAILURE" in res.stderr


# --------------------------------------------------------------------------------------------- C1
OPEN_PACKAGES = ["tracking", "calibration", "sandbox"]


def _implemented(pkg: str) -> bool:
    """An OPEN block counts as implemented once the team has written real code in it: then the C1 'still a stub'
    checks no longer apply (B1/B2 still do)."""
    from tools.run_scenario import is_open_stub
    main = ROOT / pkg / "main.py"
    if main.exists() and not is_open_stub(main, "run"):
        return True
    return any(stub_problems(f) for f in py_files(pkg))


def _is_docstring(node: ast.stmt) -> bool:
    return isinstance(node, ast.Expr) and isinstance(node.value, ast.Constant) and isinstance(node.value.value, str)


def _is_raise_nie(node: ast.stmt) -> bool:
    return isinstance(node, ast.Raise) and node.exc is not None and "NotImplementedError" in ast.unparse(node.exc)


def stub_problems(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    probs = []

    def check_func(fn: ast.FunctionDef, where: str):
        body = [n for n in fn.body if not _is_docstring(n)]
        if len(body) != 1 or not _is_raise_nie(body[0]):
            probs.append(f"{where}{fn.name}: body is not just a docstring + raise NotImplementedError")

    for node in tree.body:
        if _is_docstring(node) or isinstance(node, (ast.Import, ast.ImportFrom)):
            continue
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            check_func(node, "")
        elif isinstance(node, ast.ClassDef):
            for sub in node.body:
                if _is_docstring(sub) or (isinstance(sub, ast.AnnAssign) and sub.value is None):
                    continue
                if isinstance(sub, ast.Assign) and all(isinstance(t, ast.Name) for t in sub.targets) and \
                        isinstance(sub.value, ast.Constant) and any(isinstance(b, ast.Attribute) and b.attr == "Enum"
                                                                    or getattr(b, "id", "") == "Enum" for b in node.bases):
                    continue        # enum members (state names) are part of the interface
                if isinstance(sub, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    check_func(sub, f"{node.name}.")
                else:
                    probs.append(f"{node.name}: unexpected class-level statement {ast.unparse(sub)[:60]!r}")
        elif isinstance(node, ast.If) and "__name__" in ast.unparse(node.test):
            continue
        else:
            probs.append(f"unexpected module-level statement {ast.unparse(node)[:60]!r}")
    return probs


@pytest.mark.parametrize("pkg", OPEN_PACKAGES)
def test_c1_open_blocks_are_stubs(pkg):
    if _implemented(pkg):
        pytest.skip(f"{pkg}/ has been implemented by the team")
    files = py_files(pkg)
    assert files
    bad = {str(p.relative_to(ROOT)): pr for p in files if (pr := stub_problems(p))}
    assert not bad, bad


@pytest.mark.parametrize("pkg", OPEN_PACKAGES)
def test_c1_open_functions_raise(pkg):
    """Import every OPEN module and call each function / method with dummy arguments: must raise
    NotImplementedError."""
    if _implemented(pkg):
        pytest.skip(f"{pkg}/ has been implemented by the team")
    import dataclasses
    import importlib
    import inspect
    for path in py_files(pkg):
        mod = importlib.import_module(f"{pkg}." + path.stem) if path.stem != "__init__" else importlib.import_module(pkg)
        for name, obj in vars(mod).items():
            if getattr(obj, "__module__", None) != mod.__name__:
                continue
            if inspect.isfunction(obj):
                n = len(inspect.signature(obj).parameters)
                with pytest.raises(NotImplementedError):
                    obj(*([None] * n))
            elif inspect.isclass(obj) and "__init__" in vars(obj) and not dataclasses.is_dataclass(obj):
                n = len(inspect.signature(obj.__init__).parameters) - 1
                with pytest.raises(NotImplementedError):
                    obj(*([None] * n))


def test_c1_open_config_values_null():
    for name in ("tracking", "calibration", "sandbox"):
        if _implemented(name):
            continue                                   # the team fills in values once a block is written
        data = yaml.safe_load((ROOT / "config" / f"{name}.yaml").read_text())

        def leaves(d):
            if isinstance(d, dict):
                for v in d.values():
                    yield from leaves(v)
            else:
                yield d
        assert all(v is None for v in leaves(data)), f"config/{name}.yaml must have keys only (null values)"
    if _implemented("sandbox"):
        return
    sb = yaml.safe_load((ROOT / "config" / "sandbox.yaml").read_text())
    assert set(sb) == {"mu", "eta_d", "p_dark", "e_opt", "theta_fov_virtual_mrad", "eta_geo_mode", "block_s", "abort_qber"}
