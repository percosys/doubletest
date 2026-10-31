# stl2cad

Deterministic STL -> CAD recipe pipeline. Code measures and rebuilds geometry;
JEV (TypeSafe) supplies only small typed judgments, and code decides what to do with them.

```
STL -> mesh.py (clean, align) -> segment.py (planes, cylinders)
    -> features.py (holes, bosses, arcs, angled faces)
    -> questions.py (JEV questions + thresholds) -> classifier.py (Transport)
    -> decisions.py (truth tables) -> recipe.py -> emit_cadquery.py -> model.py -> STEP
```

## Use

```
pip install -e '.[dev]'
stl2cad part.stl --out out            # offline RuleTransport
stl2cad part.stl --out out --transport jev   # needs TYPESAFE_API_KEY + JevTransport wired up
pytest
```

Outputs: `request_state.json` (exact JEV input), `recipe.json`, `model.py` (CadQuery; run it with
`pip install cadquery` to get `model.step`).

## Status

Works (tested on synthetic parts): alignment, plane/cylinder segmentation, hole/boss detection,
hole-size snapping to ISO clearance and tap-drill sizes, review flags for unsettled judgments,
CadQuery emission of stock + holes.

Not done:
- **`JevTransport.ask` is not implemented.** The TypeSafe docs and API were unreachable when this
  was scaffolded, so the request/response mapping is unverified. Implement it against
  https://docs.typesafe.ai/go/api.md, raise `JevError` on any failure, and re-tune questions with
  real answers.
- Bosses, fillets/arcs, angled faces, pockets, patterns, revolve/shell strategies are detected
  (where noted) but not emitted.
- No geometric verification loop yet (rebuilt STEP vs. input STL).
- Question wording is a first draft and untested against JEV.
