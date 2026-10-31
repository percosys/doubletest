"""Worker CLI used by the Fusion add-in: STL in, build-plan JSON out.

    python -m stl2cad.worker part.stl --out plan.json [--tol 0.05] [--snap]

The LLM is never used unless --snap is given, and then only to name hole intent.
The API key comes from TYPESAFE_API_KEY or ~/.stl2cad/config.json ({"api_key": "..."}),
and is never printed or written to the plan.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from .classifier import JevError, JevTransport
from .convert import analyze


def _api_key() -> str | None:
    key = os.environ.get("TYPESAFE_API_KEY")
    if key:
        return key
    cfg = Path.home() / ".stl2cad" / "config.json"
    try:
        return json.loads(cfg.read_text()).get("api_key")
    except (OSError, ValueError):
        return None


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="stl2cad-worker")
    ap.add_argument("stl")
    ap.add_argument("--out", required=True)
    ap.add_argument("--tol", type=float, default=0.05, help="max deviation in mm")
    ap.add_argument("--max-edge", type=float, default=6.0, help="largest chamfer/fillet to look for, mm")
    ap.add_argument("--snap", action="store_true", help="snap holes to standard sizes (uses JEV)")
    ap.add_argument("--snap-tol", type=float, default=0.1, help="max radius change when snapping, mm")
    a = ap.parse_args(argv)

    transport = None
    note = None
    if a.snap:
        try:
            transport = JevTransport(api_key=_api_key())
        except JevError as e:
            note = f"snap requested but unavailable: {e}"
    plan = analyze(a.stl, a.tol, snap=a.snap, transport=transport, snap_tol=a.snap_tol,
                   max_edge=a.max_edge)
    if note:
        plan.review.append(note)
    Path(a.out).write_text(plan.to_json())
    print(json.dumps({"strategy": plan.strategy, "layers": len(plan.layers),
                      "deviation": plan.deviation, "llm_calls": plan.llm_calls}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
