"""Install the STL2CAD add-in into Fusion's AddIns folder and set up its worker venv.

    python scripts/install_addin.py            # copy add-in, build the Rust worker (if cargo is
                                               # installed), else set up the Python worker
    python scripts/install_addin.py --with-python   # also set up the Python worker (AI snap step)
    python scripts/install_addin.py --dest DIR # install somewhere else (testing)
    python scripts/install_addin.py --no-venv  # copy only (and build Rust if possible)

Then in Fusion: Utilities > Add-Ins > Scripts and Add-Ins > Add-Ins > STL2CAD > Run.
Needs Python >= 3.10 on this machine (the one running this script).
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "fusion_addin" / "STL2CAD"


def default_dest() -> Path:
    if sys.platform == "darwin":
        base = Path.home() / "Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns"
    elif sys.platform.startswith("win"):
        base = Path(os.environ["APPDATA"]) / "Autodesk/Autodesk Fusion 360/API/AddIns"
    else:
        raise SystemExit("Fusion runs on Windows and macOS; pass --dest to install elsewhere.")
    return base / "STL2CAD"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dest", type=Path, default=None)
    ap.add_argument("--no-venv", action="store_true")
    ap.add_argument("--with-python", action="store_true", help="also create the Python worker venv")
    ap.add_argument("--no-rust", action="store_true", help="skip building the Rust worker")
    a = ap.parse_args()
    if sys.version_info < (3, 10):
        raise SystemExit("Python >= 3.10 required")
    dest = a.dest or default_dest()
    venv = dest / "venv"
    keep = venv if venv.exists() else None
    if dest.exists():
        for item in dest.iterdir():
            if item != keep:
                shutil.rmtree(item) if item.is_dir() else item.unlink()
    shutil.copytree(SRC, dest, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    print("add-in copied to", dest)
    have_rust = False
    if not a.no_rust and shutil.which("cargo"):
        subprocess.check_call(["cargo", "build", "--release"], cwd=ROOT / "rust")
        exe = "stl2cad-worker.exe" if os.name == "nt" else "stl2cad-worker"
        (dest / "bin").mkdir(exist_ok=True)
        shutil.copy2(ROOT / "rust" / "target" / "release" / exe, dest / "bin" / exe)
        have_rust = True
        print("Rust worker installed:", dest / "bin" / exe)
    elif not a.no_rust:
        print("cargo not found: skipping the Rust worker (install Rust from https://rustup.rs for the fast one)")
    if a.no_venv or (have_rust and not a.with_python):
        return 0
    if not venv.exists():
        subprocess.check_call([sys.executable, "-m", "venv", str(venv)])
    py = venv / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    subprocess.check_call([str(py), "-m", "pip", "install", "--upgrade", "pip"])
    subprocess.check_call([str(py), "-m", "pip", "install", str(ROOT)])
    print("worker ready:", py)
    print("In Fusion: Utilities > Add-Ins > Scripts and Add-Ins > Add-Ins > STL2CAD > Run")
    return 0


if __name__ == "__main__":
    sys.exit(main())
