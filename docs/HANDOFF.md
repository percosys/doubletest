# Handoff: stl2cad (resume here if the session dies)

Last updated 2026-11-01. Repo `percosys/doubletest`, branch **`claude/stl2cad-scaffold`**, last commit
`7aed3fd` ("Revolve for turned parts..."). Everything described here is committed and pushed. No PR has
been opened (the user has not asked for one).

## How to resume

1. Start a new Claude Code session on **repo `percosys/doubletest`, branch `claude/stl2cad-scaffold`**
   (check out that branch; do not start from `master`, which has no commits).
2. First message: *"Read docs/HANDOFF.md, then continue from section 'Next steps'."*
3. Environment settings the session needs (these are NOT stored in the repo):
   - **Network allow-list**: `github.com`, `index.crates.io`, `static.crates.io` (Rust crates),
     `pypi.org`, `files.pythonhosted.org`, and only if using the AI snap step `api.typesafe.ai` and
     `docs.typesafe.ai`. Keep "allow package managers" on.
   - **Secret** `TYPESAFE_API_KEY` (only for the optional AI snap step). The key used earlier was pasted
     into a chat transcript, so **rotate it** and set the new one as an environment secret. Never write
     it to a file or commit it.
   - Rust (`cargo`) and Python >= 3.10 must exist. The previous container had cargo 1.97, Python 3.13.
4. Files that live outside the repo and are gone with the container:
   - The real test part: `angle_grinder_disc_holder_vcd.stl` (the user uploaded it; 219k faces, 10 MB,
     85 mm tall). **Ask the user to upload it again** if you need the real-part benchmark.
   - Scratch renders and JSON plans (not needed).
   - The TypeSafe skill plugin (`claude plugin marketplace add typesafe-ai/skills` then
     `claude plugin install typesafe@typesafe-ai`). Reinstall it if working on the JEV part; its content
     is summarised in section 8.

## 1. What this project is

A **deterministic STL -> parametric CAD** converter, delivered as a **Fusion add-in**. The user inserts an
STL into Fusion, clicks a button, and a model is built "exactly to the STL". Design goals set by the user:
**limit LLM usage** (geometry must never depend on an LLM; an LLM/JEV is an optional extra), use an LLM
only as a cheap fallback/judgment, and be fast (hence the Rust rewrite).

Pipeline: `STL -> worker -> plan.json -> Fusion builds sketches/extrudes/revolve/fillets -> done`.
If the analysis cannot rebuild the part within tolerance (default **0.05 mm**), the plan says
`mesh_convert` and the add-in falls back to Fusion's own mesh conversion.

## 2. Repository map

| Path | What |
|---|---|
| `rust/` | **The main worker** (`stl2cad-worker`). Crate `stl2cad`. Deps: serde, serde_json, rayon, `i_overlay` 1.10 (polygon booleans), `earcutr` (triangulation). |
| `rust/src/analyze.rs` | Orchestration: revolve attempt, then layers (+sliver merge), then edge treatments, then the fallback decision; builds `plan.json`. |
| `rust/src/layers.rs`, `section.rs`, `planes.rs`, `poly.rs`, `fit.rs` | Layers along an axis, plane sections to loops, dominant axes + frame, 2D polygons/booleans, circle/line/arc fitting. |
| `rust/src/treatments.rs` | Chamfer/fillet recovery by offset-vs-depth profiles. |
| `rust/src/revolve.rs` | Turned parts: concentric-circle test, half-section profile, revolve verification. |
| `rust/src/verify.rs`, `bvh.rs` | Stack-of-sections surface (lofted walls, no boolean union) and exact two-sided distance via a bounding-box tree. |
| `rust/tests/pipeline.rs` | 26 tests on generated parts (helpers in `rust/src/testutil.rs`: `slab_mesh`, `loft`, `revolve_solid`, `uv_sphere`, `transform`, `write_stl`). |
| `src/stl2cad/` | **Python reference worker** (older, slower: ~2 min on the grinder part). Kept for the optional JEV/AI snap step and as a cross-check. `convert.py`, `layers.py`, `treatments.py`, `verify.py`, `segment.py`, `features.py`, `questions.py` (all JEV wording), `classifier.py` (JevTransport/RuleTransport), `decisions.py`, `standards.py`, `worker.py`. |
| `fusion_addin/STL2CAD/` | The add-in: `STL2CAD.py` (command, STL export, runs worker, builds), `plan_build.py` (all Fusion API calls), `stl_io.py` (pure-Python STL writer), `STL2CAD.manifest`. |
| `scripts/install_addin.py` | Copies the add-in to Fusion's AddIns folder, builds the Rust worker if `cargo` exists, optionally makes a Python venv (`--with-python`). |
| `tests/` | 33 Python tests (`test_addin.py` has the Fusion-builder mock tests and the **Rust-plan contract tests**; fixtures in `tests/fixtures/`). |
| `docs/methods.md` | **Clean-room method spec** (M1-M11) with a provenance statement. Read it before adding methods. |
| `README.md` | User-facing overview, install, status, limits. |

Plan schema (JSON, version 1, units mm): `strategy` = `extruded_layers` | `revolve` | `mesh_convert`;
`frame` (4x4 world-from-build-frame); `layers[]` of `{z0,z1,regions[{outer,holes,area,centroid}]}` where a
loop is `circle` | `polyline` | `path` (exact `line`/`arc` pieces, arcs are three-point `a,m,b` plus
`cx,cy,r`); `revolve.regions` (profile in sketch coords x=radius, y=axial); `treatments[]` with
`kind/family/z/size_xy/size_z/edges[]` (edge descriptors: `circle`, `segment`, `arc`); `deviation`,
`deviation_sharp`, `review[]`, `notes[]`, `llm_calls`, `stats`.

## 3. Methods implemented (details in `docs/methods.md`)

- **Layers (2.5D)**: levels at flat-face heights along the main axis; section at each layer's mid height;
  slivers thinner than the tolerance merged into the most similar neighbour **only if verification still
  passes**; the unmerged variant is used only if it is at least 2x better (otherwise sampling noise).
- **Exact outlines (M5)**: whole-circle test; else corner split + greedy longest-piece line/three-point-arc
  fitting. Arcs are claimed only when measured error <= 10% of tolerance (coarse STLs stay polylines).
- **Edge treatments (M6)**: for each exposed outline run at a layer interface, measure the real section's
  offset at 9 depths (nearest distance, median over the run), fit chamfer vs quarter-round fillet, fit
  **per loop**, then group identical loops. Convex (material removed) and concave (material added).
- **Revolve (M7)**: part is turned if sections square to the axis are concentric circles; profile is the
  half-section through the axis; fitted with M5; emitted as `revolve`.
- **Verification (M9)**: exterior of the rebuilt solid is built directly (walls + ledges, lofted walls in
  treated bands), volume exact (Simpson/Pappus), distances by BVH. Accept if max deviation + fitting error
  <= tolerance.
- Treatments are searched on **one axis only** (the one with most flat area unless another is 2x better).

## 4. Status and measured results

- Rust: **26 tests pass** (`cd rust && cargo test --release`). Python: **33 pass** (`python3 -m pytest -q`,
  takes ~2 min because the Python worker is slow).
- **Real part (grinder disc holder, 219k faces)**: `extruded_layers`, 3 layers + 6 edge treatments
  (two 0.26 mm bottom chamfers at z=0 and z=0.52, 2 mm arm-top fillet, three 2 mm hole countersinks, 4 mm chamfer at
  the post base, 4 mm fillet on the post top), 0 AI calls, max deviation **0.043 mm** (tolerance 0.05),
  mean ~0.0004 mm. Rust **~2.6 s**, ~1.2 GB; Python reference ~2 min with the same answer. The deviation is
  a sampled estimate (16,000 points); an independent 40,000-point check gave 0.045 mm, so the margin to
  0.05 is thin. The main plate outline is 6 lines + 6 arcs.
- Generated parts verified: plate with holes, tilted plate, stepped part with pocket, L-bracket, slivers,
  thin internal cavity (not merged), chamfered and filleted plate edges, countersunk holes + outline
  chamfer, rounded and slot outlines, stepped shaft, knob (fillet+cone), tilted tube, sphere (revolve),
  ellipsoid (fallback), open mesh (skipped).

## 5. NOT verified: the Fusion add-in itself  (highest risk)

Fusion was never available in the build environment and the Fusion API reference was unreachable.
`fusion_addin/STL2CAD/plan_build.py` and `STL2CAD.py` use these calls **from memory, never run in Fusion**:
mesh export (`MeshBody.mesh`/`displayMesh`, `nodeCoordinatesAsDouble`, `nodeIndices`), construction-plane
offset + sketches, `sketchLines.addByTwoPoints`, `sketchArcs.addByThreePoints`, `sketchCircles`, profile
`areaProperties()`, `extrudeFeatures`, `revolveFeatures.createInput(profile, yConstructionAxis, op)` +
`setAngleExtent`, `moveFeatures.createInput2/defineAsFreeMove`, `filletFeatures.addConstantRadiusEdgeSet`,
`chamferFeatures.createInput2` + `addEqualDistanceChamferEdgeSet` / `addTwoDistancesChamferEdgeSet`
(which distance lands on which face is a guess), `Matrix3D.setToAlignCoordinateSystems`, and the
mesh-convert fallback (`root.features.meshConvertFeatures` is a guess; on failure the add-in tells the user to
use Mesh > Modify > Convert Mesh). The builder **logic** is tested against a mock and against real plans
written by the Rust worker, which catches contract/logic bugs but not wrong API names.

**The user has a "built-in Autodesk Fusion MCP"** set up locally. Research result: it is a local server that
Fusion exposes (enable under Fusion Preferences > General > API > "Fusion MCP Server (Runs locally on this
device)", default port **27182**; unofficial sources give the endpoint `http://127.0.0.1:27182/mcp`, unverified).
It is documented only for Claude Desktop/Cursor/HTTP clients. A cloud session **cannot reach it**
(localhost). To test the add-in for real, the user must run **Claude Code on their own machine** with this repo
and the MCP added (`claude mcp add --transport http fusion http://127.0.0.1:27182/mcp`, verify with `/mcp`),
or run the add-in manually and paste back the errors. Whether that MCP can execute Python scripts is
unconfirmed (third-party notes mention an `execute` tool with `def run(_context):`).

## 6. Next steps (in priority order)

1. **Run the add-in in Fusion** (plate first, then the revolve case, then the grinder part) and fix API
   mistakes from the real tracebacks. Install: `python scripts/install_addin.py`, then Utilities > Add-Ins.
2. **Holes across the main axis** (method M8, specified in `docs/methods.md`, not built): parts with side
   holes currently fall back to mesh conversion.
3. **Mixed parts** (a revolve plus other features, e.g. the grinder's turned post on its plate): currently
   handled by the layer model; a revolve + features strategy is not built.
4. **LLM-free design-value rounding** (M10) and **CAD-vs-scan detection** (M11): not built. The current
   hole snapping uses JEV via the Python worker only.
5. Smaller: edge treatments only on one axis; partial/varying chamfers are approximated or skipped; fillet
   tangent continuity at run ends not enforced; plans are plain sketches+extrudes (no names, dimensions, or
   constraints); the worker blocks Fusion's UI while it runs (Rust: seconds; Python: minutes).

## 7. Decisions, lessons and pitfalls (read before changing things)

- **Clean room**: the user wants the *methods*, not another project's code. The methods came from reading
  ScanRuler's (CNC Kitchen, **AGPL-3.0**, commercial exception available) READMEs/docstrings/file headers
  only, never its implementations. Do **not** read or copy its code; extend `docs/methods.md` first and
  implement from it. Not a formal two-team clean room; counsel should review if it matters.
- **Never make geometry depend on the LLM**. A failed JEV call must raise (`JevError`), never become a "no".
- **Jev is bad at numbers** (per its jaggedness docs): code computes nearest standard sizes and passes
  words like "under 2 percent". Questions live only in `src/stl2cad/questions.py`.
- Bugs found the hard way (all fixed, keep the tests):
  - Boolean union of hundreds of thin slabs leaves stray internal shells (verification showed a fake 6.9 mm
    error). Verification builds the exterior directly instead.
  - `closest_point` on a 219k-face mesh needs gigabytes unless chunked (Python); the Rust BVH has no issue.
  - Plane regions grown across near-tangent fillet facets tilt the axis by ~0.25 deg (0.4 mm over 100 mm):
    use the shrinking-window mean-shift in `planes.rs::refine_normal`.
  - Choosing the axis by smallest sharp deviation is wrong (a chamfered plate is equally off on every axis).
  - A section plane through a mesh vertex line leaves ~0.0004 mm zigzag; corners must be measured over a
    baseline and arcs validated along straight edges between points (a long wall can otherwise pass as a
    huge-radius arc).
  - Generated test meshes must be outward-oriented and free of T-junctions; use `loft`/`slab_mesh`.
  - `trimesh` needs `networkx` for sections; `rtree` for distances; install via `pip install -e '.[dev]'`.
- Tolerance semantics: pass = `max_deviation + fitting_error <= tol` (default 0.05 mm). Arc/polyline budget is
  0.1 x tol.
- The Rust analysis is deterministic (rings are normalised to a fixed orientation and start).
- Set `STL2CAD_TRACE=1` for stage timings (Rust) / progress (Python).

## 8. TypeSafe / JEV (optional AI step)

JEV is TypeSafe's "System One" model. API: `POST https://api.typesafe.ai/v1/systemone` with
`{model:"jev-1.14.0", state, questions:{id:{type:"noul"|"choice", instructions, criteria}}}`, Bearer key;
answers are `noul` probability or `choice` + `probabilities`. Use thresholds 0.7/0.3 for yes/no, never gate on
`confidence`. Live probing worked (jev-1.14.0): screw holes judged clearly (0.84-0.95), ambiguous cases land in
the "unsettled" band and are flagged for review. Skill source: github.com/typesafe-ai/skills (SKILL.md).

## 9. Working agreements with this user

- A Stop hook (`~/.claude/stop-hook-git-check.sh`) demands that all changes are **committed and pushed**; do so after each
  meaningful step, to branch `claude/stl2cad-scaffold`, with the attribution trailer the harness specifies.
- **Do not open a pull request** unless asked.
- Report honestly what is verified vs only tested on a mock/generated part; flag thin margins.
- The user prefers a recommendation over a survey and short status lines while background jobs run.

## 10. Commands cheat sheet

```bash
# Rust worker
cd rust && cargo build --release && cargo test --release
./target/release/stl2cad-worker part.stl --out plan.json [--tol 0.05] [--max-edge 6]
STL2CAD_TRACE=1 ./target/release/stl2cad-worker part.stl --out plan.json

# Python reference + add-in tests
pip install -e '.[dev]' && python3 -m pytest -q          # ~2 min
python3 -m pytest -q tests/test_addin.py                 # ~2 s, builder + contract tests
python -m stl2cad.worker part.stl --out plan.json [--snap]

# Install into Fusion (Windows/macOS), builds the Rust worker if cargo exists
python scripts/install_addin.py [--with-python]
```

Regression check after any change to the analysis: re-run the real part if you have it (expect
`extruded_layers`, 3 layers, 6 treatments, max deviation about 0.043 mm, ~2.6 s) and keep both test suites
green. The contract fixtures `tests/fixtures/*.json` are real Rust-worker outputs; regenerate them only if the
plan schema changes on purpose.
