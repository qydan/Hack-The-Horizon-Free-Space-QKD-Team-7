"""qkd: one command for the qkd_sim digital twin.

    qkd doctor                     check what is installed and say what to do next
    qkd setup                      install the Python packages for the offline tools and tests
    qkd setup --docker             also build the simulator image (Gazebo + PX4; ~30-60 min, once)
    qkd test                       offline tests (no simulator needed)
    qkd test --sim                 acceptance tests in Gazebo / PX4
    qkd list                       scenarios
    qkd demo [fixed|gimbal]        45 s hover with the fan on; live camera at http://localhost:8080
    qkd run t3 gimbal              run a scenario (any unique prefix; rig fixed|gimbal, or A|B)
    qkd run t3 gimbal --quick      20 s at fast fidelity
    qkd run t1 --no-view --duration 30 --fidelity full
    qkd score [run-dir]            score a run (default: the latest)
    qkd shell                      a shell inside the simulator image

On Windows use  .\\qkd ...  (qkd.cmd). Gazebo runs natively if it is installed, otherwise inside Docker
automatically; either way the live camera is served to your browser.
"""
from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
IMAGE = os.environ.get("QKD_IMAGE", "qkd_sim")
IS_LINUX = platform.system() == "Linux"
IS_WIN = platform.system() == "Windows"
VIEW_PORT = 8080
Q = r".\qkd" if IS_WIN else "./qkd"         # how to invoke this tool, for messages


# ------------------------------------------------------------------------------------------- environment
def venv_python() -> Path | None:
    for base in (ROOT / ".venv", ROOT.parent / ".venv"):
        exe = base / ("Scripts/python.exe" if IS_WIN else "bin/python")
        if exe.exists():
            return exe
    return None


def py() -> str:
    exe = venv_python()
    return str(exe) if exe else sys.executable


def missing_packages(python: str) -> list[str]:
    mods = {"numpy": "numpy", "yaml": "pyyaml", "cv2": "opencv", "zmq": "pyzmq", "jsonschema": "jsonschema",
            "pytest": "pytest", "scipy": "scipy"}
    code = ("import importlib.util,sys;"
            f"print(','.join(m for m in {list(mods)!r} if importlib.util.find_spec(m) is None))")
    try:
        out = subprocess.run([python, "-c", code], capture_output=True, text=True, timeout=60).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return list(mods.values())
    return [mods[m] for m in out.split(",") if m]


def gz_native() -> bool:
    if shutil.which("gz") is None:
        return False
    r = subprocess.run([py(), "-c", "import gz.transport13, gz.msgs10"], capture_output=True)
    return r.returncode == 0


def px4_native() -> bool:
    d = Path(os.environ.get("PX4_DIR", Path.home() / "PX4-Autopilot"))
    return (d / "build" / "px4_sitl_default" / "bin" / "px4").exists()


def docker_state() -> str:
    """'missing' | 'stopped' | 'no-image' | 'ready'."""
    if shutil.which("docker") is None:
        return "missing"
    if subprocess.run(["docker", "info"], capture_output=True).returncode != 0:
        return "stopped"
    if subprocess.run(["docker", "image", "inspect", IMAGE], capture_output=True).returncode != 0:
        return "no-image"
    return "ready"


def has_nvidia() -> bool:
    return shutil.which("nvidia-smi") is not None and \
        subprocess.run(["nvidia-smi", "-L"], capture_output=True).returncode == 0


def docker_run_args(inner: list[str], *, ports: tuple[int, ...] = (), gui: bool = False) -> list[str]:
    args = ["docker", "run", "--rm", "--init"]
    if sys.stdin.isatty() and sys.stdout.isatty():
        args.append("-it")
    if has_nvidia():
        args += ["--gpus", "all", "-e", "NVIDIA_DRIVER_CAPABILITIES=all"]
        if IS_WIN:
            # Docker Desktop on WSL 2: --gpus only gives CUDA. OpenGL (Gazebo's ogre2) reaches the GPU through
            # Mesa's d3d12 driver, which needs the WSL GPU libraries from the utility VM. Without this,
            # rendering silently falls back to llvmpipe on the CPU (~0.2x real time).
            args += ["-v", "/usr/lib/wsl:/usr/lib/wsl", "-e", "LD_LIBRARY_PATH=/usr/lib/wsl/lib",
                     "-e", "GALLIUM_DRIVER=d3d12", "-e", "MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA"]
    elif IS_LINUX and Path("/dev/dri").exists():
        args += ["--device", "/dev/dri"]
    else:
        args += ["-e", "LIBGL_ALWAYS_SOFTWARE=1"]
    if IS_LINUX:
        args += ["--net", "host", "--ipc", "host"]
        if gui and os.environ.get("DISPLAY"):
            args += ["-e", f"DISPLAY={os.environ['DISPLAY']}", "-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw"]
    else:
        for p in ports:
            args += ["-p", f"{p}:{p}"]
    args += ["-e", "QKD_RUN_SIM=1", "-v", f"{ROOT}:/qkd_sim", "-w", "/qkd_sim", IMAGE]
    return args + inner


def call(cmd: list[str]) -> int:
    print("$ " + " ".join(f'"{c}"' if " " in c else c for c in cmd), flush=True)
    try:
        return subprocess.call(cmd, cwd=ROOT)
    except KeyboardInterrupt:
        return 130


def need_simulator() -> str:
    """Return 'native' or 'docker', or exit with instructions."""
    if gz_native():
        return "native"
    state = docker_state()
    if state == "ready":
        return "docker"
    hint = {
        "missing": ("Gazebo is not installed here and Docker isn't either.\n"
                    "  Windows/macOS: install Docker Desktop (Windows: with the WSL 2 backend), start it, then run\n"
                    f"      {Q} setup --docker\n"
                    "  Linux: install Docker (and the NVIDIA Container Toolkit for an NVIDIA GPU), or Gazebo Harmonic natively."),
        "stopped": "Docker is installed but not running. Start Docker Desktop (or the docker service) and try again.",
        "no-image": (f"The simulator image isn't built yet. Run\n      {Q} setup --docker\n"
                     "  once (it builds Gazebo Harmonic + PX4 SITL; about 30-60 minutes)."),
    }[state]
    sys.exit(f"Can't start the simulator: {hint}")


# ------------------------------------------------------------------------------------------- commands
def cmd_doctor(_a) -> int:
    rows = []
    ok_py = sys.version_info >= (3, 10)
    rows.append(("Python >= 3.10", ok_py, sys.version.split()[0]))
    vp = venv_python()
    rows.append(("Virtual env", vp is not None, str(vp) if vp else f"none ({Q} setup creates .venv)"))
    miss = missing_packages(py())
    rows.append(("Offline packages", not miss, "ok" if not miss else "missing: " + ", ".join(miss)))
    native = gz_native()
    rows.append(("Gazebo Harmonic (native)", native, "found" if native else "not found (Docker is used instead)"))
    if native:
        rows.append(("PX4 SITL (native)", px4_native(), "PX4_DIR build found" if px4_native() else "set PX4_DIR"))
    ds = docker_state()
    rows.append(("Docker", ds in ("ready", "no-image"), {"missing": "not installed", "stopped": "installed, not running",
                                                         "no-image": "running", "ready": "running"}[ds]))
    rows.append(("Simulator image", ds == "ready", "built" if ds == "ready" else f"not built ({Q} setup --docker)"))
    gpu = has_nvidia()
    rows.append(("NVIDIA GPU", gpu, "found" if gpu else "not found: rendering falls back to software (slow)"))
    w = max(len(r[0]) for r in rows)
    print()
    for name, ok, info in rows:
        print(f"  {'OK ' if ok else '-- '} {name:<{w}}  {info}")
    print()
    if miss:
        nxt = f"{Q} setup"
    elif not native and ds in ("missing", "stopped"):
        nxt = (f"install Docker Desktop, start it, then run  {Q} setup --docker" if ds == "missing"
               else f"start Docker, then run  {Q} setup --docker")
    elif not native and ds == "no-image":
        nxt = f"{Q} setup --docker"
    else:
        nxt = f"{Q} demo"
    print(f"  Offline tools work: {'yes' if not miss else 'no'}.   Simulator: "
          f"{'native' if native else ('Docker' if ds == 'ready' else 'not available yet')}.")
    print(f"  Next step:  {nxt}\n")
    return 0


def cmd_setup(a) -> int:
    exe = venv_python()
    if exe is None:
        print("Creating .venv ...")
        rc = call([sys.executable, "-m", "venv", str(ROOT / ".venv")])
        if rc:
            return rc
        exe = venv_python()
    rc = call([str(exe), "-m", "pip", "install", "--upgrade", "pip", "-q"])
    rc = rc or call([str(exe), "-m", "pip", "install", "-q", "-r", str(ROOT / "requirements.txt")])
    if rc:
        return rc
    print(f"Offline tools ready. Try:  {Q} test")
    if a.docker:
        ds = docker_state()
        if ds in ("missing", "stopped"):
            sys.exit("Docker is " + ("not installed" if ds == "missing" else "not running") +
                     f". Install/start Docker Desktop (WSL 2 backend on Windows) and rerun: {Q} setup --docker")
        print("Building the simulator image (Gazebo Harmonic + PX4 SITL). This takes ~30-60 minutes the first time.")
        rc = call(["docker", "build", "-t", IMAGE, "-f", str(ROOT / "docker" / "Dockerfile"), str(ROOT)])
        if rc == 0:
            print(f"Simulator ready. Try:  {Q} demo")
    return rc


def cmd_test(a) -> int:
    if not a.sim:
        return call([py(), "-m", "pytest", "-q", *a.pytest_args])
    where = need_simulator()
    inner = ["python3", "-m", "pytest", "-m", "sim", "-s", *a.pytest_args]
    if where == "native":
        return subprocess.call([py(), "-m", "pytest", "-m", "sim", "-s", *a.pytest_args], cwd=ROOT,
                               env={**os.environ, "QKD_RUN_SIM": "1"})
    return call(docker_run_args(inner))


def cmd_list(_a) -> int:
    return call([py(), str(ROOT / "tools" / "run_scenario.py"), "--list"])


def _run(scenario: str, rig: str | None, a) -> int:
    opts = []
    if rig:
        opts += ["--rig", rig]
    if a.quick:
        opts.append("--quick")
    if a.duration:
        opts += ["--duration", str(a.duration)]
    if a.fidelity:
        opts += ["--fidelity", a.fidelity]
    if not a.no_view:
        opts += ["--view", str(VIEW_PORT)]
    if a.gui:
        if not IS_LINUX and not gz_native():
            print("Note: the Gazebo GUI can't be shown from Docker on this OS; use the browser view instead.")
        else:
            opts.append("--gui")
    where = need_simulator()
    if not a.no_view:
        print(f"\n  Live camera will be at  http://localhost:{VIEW_PORT}  once Gazebo is up.\n")
    if where == "native":
        return call([py(), str(ROOT / "tools" / "run_scenario.py"), scenario, *opts])
    return call(docker_run_args(["python3", "tools/run_scenario.py", scenario, *opts],
                                ports=(VIEW_PORT,) if not a.no_view else (), gui=a.gui))


def cmd_run(a) -> int:
    return _run(a.scenario, a.rig, a)


def cmd_demo(a) -> int:
    a.duration = a.duration or 45.0
    a.fidelity = a.fidelity or "fast"
    return _run("t5_latency", a.rig or "fixed", a)


def cmd_score(a) -> int:
    run = Path(a.run_dir) if a.run_dir else None
    if run is None:
        runs = sorted((ROOT / "runs").glob("*/summary.json"), key=lambda p: p.stat().st_mtime)
        if not runs:
            sys.exit(f"No runs yet. Try:  {Q} demo")
        run = runs[-1].parent
    return call([py(), "-m", "evaluation.score", str(run)])


def cmd_shell(_a) -> int:
    if gz_native():
        print("Gazebo is installed natively; no container needed.")
        return 0
    need_simulator()
    return call(docker_run_args(["bash"], ports=(VIEW_PORT,)))


def main() -> int:
    ap = argparse.ArgumentParser(prog="qkd", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("doctor", help="check the installation").set_defaults(fn=cmd_doctor)
    s = sub.add_parser("setup", help="install Python packages (and --docker: build the simulator image)")
    s.add_argument("--docker", action="store_true")
    s.set_defaults(fn=cmd_setup)
    t = sub.add_parser("test", help="run tests (--sim for the Gazebo acceptance tests)")
    t.add_argument("--sim", action="store_true")
    t.add_argument("pytest_args", nargs=argparse.REMAINDER)
    t.set_defaults(fn=cmd_test)
    sub.add_parser("list", help="list scenarios").set_defaults(fn=cmd_list)

    def run_opts(p):
        p.add_argument("--quick", action="store_true", help="20 s at fast fidelity")
        p.add_argument("--duration", type=float, help="sim seconds")
        p.add_argument("--fidelity", choices=["full", "fast"])
        p.add_argument("--no-view", action="store_true", help="don't serve the live camera")
        p.add_argument("--gui", action="store_true", help="Gazebo GUI (native or Linux only)")

    r = sub.add_parser("run", help="run a scenario: qkd run t3 gimbal")
    r.add_argument("scenario")
    r.add_argument("rig", nargs="?", help="fixed|gimbal (A|B); default: every rig the scenario lists")
    run_opts(r)
    r.set_defaults(fn=cmd_run)
    d = sub.add_parser("demo", help="45 s hover with the fan, live camera in the browser")
    d.add_argument("rig", nargs="?")
    run_opts(d)
    d.set_defaults(fn=cmd_demo)
    sc = sub.add_parser("score", help="score a run (default: latest)")
    sc.add_argument("run_dir", nargs="?")
    sc.set_defaults(fn=cmd_score)
    sub.add_parser("shell", help="shell inside the simulator image").set_defaults(fn=cmd_shell)
    a = ap.parse_args()
    if not getattr(a, "fn", None):
        ap.print_help()
        print(f"\nStart with:  {Q} doctor")
        return 0
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
