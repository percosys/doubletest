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

Works: alignment, plane/cylinder segmentation, hole/boss detection, hole-size snapping to ISO
clearance and tap-drill sizes, review flags for unsettled judgments, CadQuery emission of stock + holes,
and the live `JevTransport` (`POST /v1/systemone`, default model `jev-1.14.0`, key from
`TYPESAFE_API_KEY`, request/response logged to `log_path`, any failure raises `JevError`).

Live probe (jev-1.14.0, 8 requests over two runs on synthetic plates and a sphere): standard clearance/tap holes
were judged intentional + fastener (0.84-0.95) with the right purpose; a sphere was `organic_freeform`
(0.99) and not a mechanical part (0.03). Ambiguous cases landed in the unsettled band and are
flagged for review: a 12 mm hole (`fastener` 0.32) and a 0.6 mm hole (`intentional` 0.41).
This is a small sample on synthetic parts, not a benchmark; run it on real parts before trusting the bands.

Design note: Jev is weak at numeric comparison, so code computes the nearest standard size and the
percent difference and passes them as words ("under 2 percent"); Jev only judges meaning.

Not done:
- Bosses, fillets/arcs, angled faces, pockets, patterns, revolve/shell strategies are detected
  (where noted) but not emitted.
- No geometric verification loop yet (rebuilt STEP vs. input STL).
- Untested on real STLs; only synthetic parts so far.
