"""stl2cad CLI: STL -> recipe.json + model.py (+ a review report)."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from . import mesh as meshmod
from .classifier import JevTransport, RuleTransport
from .emit_cadquery import emit
from .features import extract
from .questions import build_questions, build_state
from .recipe import build_recipe
from .segment import segment


def run(stl: str, out: Path, transport) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    m, _ = meshmod.align(meshmod.load_clean(stl))
    feats = extract(m, segment(m))
    state, questions = build_state(feats), build_questions(feats)
    # Keep the exact request inputs so any answer can be rerun.
    (out / "request_state.json").write_text(json.dumps(state, indent=2))
    answers = transport.ask(state, questions)
    recipe = build_recipe(feats, answers)
    (out / "recipe.json").write_text(recipe.to_json())
    (out / "model.py").write_text(emit(recipe))
    return {"recipe": recipe, "features": feats}


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="stl2cad")
    ap.add_argument("stl")
    ap.add_argument("--out", default="out")
    ap.add_argument("--transport", choices=["rules", "jev"], default="rules")
    a = ap.parse_args(argv)
    transport = JevTransport() if a.transport == "jev" else RuleTransport()
    res = run(a.stl, Path(a.out), transport)
    r = res["recipe"]
    print(f"{r.part_type}: stock {r.stock} mm, {len(r.holes)} holes")
    for item in r.review:
        print(f"  REVIEW: {item}")
    for n in r.notes:
        print(f"  note: {n}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
