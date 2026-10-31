# stl2cad

Convert an STL into a CAD model, deterministically. Built as a Fusion add-in: insert an STL,
select the mesh body, click **Convert Mesh to Model**.

## How it works

1. **Deterministic 2.5D rebuild (no LLM).** Find the main extrusion axis, cut the mesh into layers
   at every flat-face height, and take each layer's exact cross-section. Outlines become circles,
   exact lines and three-point arcs (corner-split, greedy longest-piece fitting), or a thinned
   polyline when the tessellation is too coarse to claim arcs. Plates, brackets, steps, pockets,
   bosses and through/blind holes come out as sketches + extrudes (join). Layers thinner than the
   tolerance are merged into their most similar neighbour, kept only if verification still passes.
2. **Edge treatments (no LLM).** The gap along each layer outline is measured at several depths,
   square to the edge, giving an offset-versus-depth profile that is fitted as a chamfer (straight)
   or fillet (quarter circle), convex or concave, per connected edge loop; identical loops are
   grouped. They are added as Fusion chamfer/fillet features on the matching edges.
3. **Verify.** The rebuild is compared with the STL (exact two-sided surface distance + volume).
   The rebuilt solid is a stack of sections with lofted walls (no boolean union). It is accepted
   only if max deviation + fitting error <= tolerance (default 0.05 mm).
4. **Turned parts.** If every section square to the axis is a set of concentric circles, the part is
   built as one revolve of its half-section (lines for cylinders/cones, arcs for tori/spheres).
5. **Fallback.** Otherwise the add-in falls back to Fusion's own mesh conversion.
6. **Optional AI (off by default).** "Snap holes to standard sizes" asks JEV (TypeSafe) whether
   each hole is a clearance or tap-drill screw hole (Python worker only). Geometry never depends
   on the AI, and a failed call is reported, not treated as an answer.

The methods are specified in [docs/methods.md](docs/methods.md), including a provenance statement.

```
Fusion mesh body -> STL (mm) -> stl2cad-worker -> plan.json -> sketches + extrudes in Fusion
                                (Rust binary: ~2.5 s on a 219k-face part; Python reference worker
                                 ~2 min, kept for the AI snap step and as a cross-check)
```

## Install the add-in (Windows/macOS)

```
python scripts/install_addin.py        # builds the Rust worker if cargo is installed
python scripts/install_addin.py --with-python   # also set up the Python worker (AI snap step)
```
Without `cargo` it falls back to the Python worker (needs Python >= 3.10; creates a venv inside the
add-in). Then in Fusion: Utilities > Add-Ins > Scripts and Add-Ins > Add-Ins > STL2CAD > Run. The
button is in Solid > Modify. For the optional snap step, set `TYPESAFE_API_KEY` or put
`{"api_key": "..."}` in `~/.stl2cad/config.json`.

## Use without Fusion

```
cd rust && cargo build --release && cargo test --release
./target/release/stl2cad-worker part.stl --out plan.json [--tol 0.05] [--max-edge 6]

pip install -e '.[dev]' && pytest     # Python reference worker and the add-in tests
python -m stl2cad.worker part.stl --out plan.json [--tol 0.05] [--snap]
```

## Status

Verified here: 26 Rust tests and 33 Python tests (generated parts, tilted/stepped/pocketed parts,
slivers, chamfers, fillets, countersinks, rounded and slotted outlines, turned parts (shaft, knob
with fillet and cone, tube, sphere), fallback decisions, plan contracts between the Rust worker and
the add-in builder for layers and for a revolve), plus one real STL.
- **Real STL (grinder disc holder, 219k faces, 85 mm tall):** `extruded_layers`, 3 layers + 6 edge
  treatments (0.26 mm bottom chamfer, 2 mm arm-top fillet, three 2 mm hole countersinks, 4 mm chamfer
  at the post base, 4 mm fillet on the post top), 0 AI calls. Max deviation 0.043 mm (tolerance
  0.05), mean 0.0004 mm. The main plate outline is 6 lines + 6 arcs. Rust: **2.5 s**; Python
  reference: ~2 min. Both give the same layers, treatments and volume error. The deviation is a
  sampled estimate (16,000 points), so the margin to 0.05 is thin.
- Live JEV calls (jev-1.14.0) for the hole-intent questions (Python worker).

**Not verified: the Fusion add-in itself.** `STL2CAD.py` and `plan_build.py` use the Fusion API
(mesh export, sketches, extrudes, fillet/chamfer features, Move feature, mesh conversion) as I
understand it, but I could not run Fusion or read the API reference from the build environment.
Expect to fix some API details on first run. The builder's logic (profile matching, layer order,
join vs new body, edge matching) is tested against a mock. The mesh-conversion fallback call is the
least certain; if it fails the add-in tells you to use Mesh > Modify > Convert Mesh manually.
The grinder plan needs ~15 edge matches for each of its two bottom chamfers and 12 for the arm fillet.

## Limits and not yet done

- Exact only for 2.5D parts (everything extruded along one axis), plus chamfers/fillets whose profile
  is uniform along each edge run, and for turned parts (everything coaxial, one revolve).
  **Not implemented yet** (specified in docs/methods.md): holes across the main axis (M8),
  design-value rounding without the LLM (M10), CAD-export-vs-scan detection (M11), and parts that mix
  a revolve with other features. Such parts fall back to mesh conversion.
- Edge treatments are searched on one axis only (the one with the most flat area, unless another is
  twice as accurate).
- Arcs are claimed only when the STL's tessellation sag is within 10 % of the tolerance; coarser
  tessellations stay polylines (still within tolerance).
- Chamfers/fillets that vary along one edge run, or stop part-way along it, are approximated or skipped.
- Results are plain sketches and extrudes, not named features; no dimensions/constraints yet.
- A fillet's exact tangent continuity at run ends is not enforced.
- The revolve profile is read from one cut through the axis; on a coarse tessellation that cut can sit
  up to the chord sag inside the true radius (verification measures it).
- Not watertight meshes skip the 2.5D path.
