"""STL2CAD Fusion add-in: convert a mesh body into a parametric solid.

Flow: mesh body -> temp STL (mm) -> worker (external Python, installed by
scripts/install_addin.py) -> plan.json -> sketches + extrudes in the design.
If the plan says the 2.5D rebuild is not within tolerance, fall back to Fusion's
own mesh conversion. The LLM is used only if "Snap holes" is ticked.

UNTESTED INSIDE FUSION: written against the documented API; see the repo README.
"""

import json
import os
import subprocess
import sys
import tempfile
import traceback

import adsk.core
import adsk.fusion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import plan_build  # noqa: E402
import stl_io  # noqa: E402

CMD_ID = "stl2cad_convert"
PANEL_ID = "SolidModifyPanel"
_handlers = []  # keep references alive
_app = adsk.core.Application.get()
_ui = _app.userInterface


def _worker_python():
    env = os.environ.get("STL2CAD_PYTHON")
    if env and os.path.exists(env):
        return env
    here = os.path.dirname(os.path.abspath(__file__))
    for rel in (("venv", "Scripts", "python.exe"), ("venv", "bin", "python")):
        p = os.path.join(here, *rel)
        if os.path.exists(p):
            return p
    raise RuntimeError("STL2CAD worker Python not found. Run scripts/install_addin.py, or set "
                       "STL2CAD_PYTHON to a Python that has the stl2cad package installed.")


def _try_mesh_convert(root, mesh_body):
    """Fusion's own mesh -> solid conversion (faceted). API surface unverified."""
    try:
        coll = getattr(root.features, "meshConvertFeatures", None)
        if coll is None:
            return False
        inp = coll.createInput(mesh_body)
        coll.add(inp)
        return True
    except Exception:
        return False


def convert(mesh_body, tol_mm, snap, fallback):
    design = adsk.fusion.Design.cast(_app.activeProduct)
    root = design.rootComponent
    tm = getattr(mesh_body, "mesh", None) or mesh_body.displayMesh
    tmp = tempfile.mkdtemp(prefix="stl2cad_")
    stl, plan_path = os.path.join(tmp, "in.stl"), os.path.join(tmp, "plan.json")
    n = stl_io.write_binary_stl(stl, tm.nodeCoordinatesAsDouble, tm.nodeIndices, scale=10.0)

    cmd = [_worker_python(), "-m", "stl2cad.worker", stl, "--out", plan_path, "--tol", str(tol_mm)]
    if snap:
        cmd.append("--snap")
    _ui.messageBox("Analysing %d triangles. This can take a minute or two; click OK to start." % n,
                   "STL2CAD")
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=1800,
                          creationflags=0x08000000 if os.name == "nt" else 0)
    if proc.returncode != 0:
        raise RuntimeError("worker failed:\n" + (proc.stderr or proc.stdout)[-1500:])
    with open(plan_path) as fh:
        plan = json.load(fh)

    lines = []
    if plan["strategy"] == "extruded_layers":
        plan_build.build_plan(adsk.core, adsk.fusion, root, plan)
        mesh_body.isLightBulbOn = False
        d = plan["deviation"]
        lines.append("Built %d layer(s) as sketches + extrudes." % len(plan["layers"]))
        lines.append("Max deviation from the STL: %.4f mm (tolerance %.3f mm)." % (d["max_mm"], tol_mm))
        if plan["snapped"]:
            lines.append("Snapped %d hole size(s) to standard sizes." % len(plan["snapped"]))
    else:
        lines += plan["notes"][-2:]
        if fallback and _try_mesh_convert(root, mesh_body):
            lines.append("Converted the mesh with Fusion's mesh conversion instead.")
        else:
            lines.append("Use Mesh > Modify > Convert Mesh (Faceted) to get a solid that matches the STL.")
    for r in plan["review"]:
        lines.append("Review: " + r)
    lines.append("AI calls used: %d" % plan["llm_calls"])
    _ui.messageBox("\n".join(lines), "STL2CAD")


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        try:
            inputs = args.command.commandInputs
            sel = inputs.itemById("mesh")
            body = adsk.fusion.MeshBody.cast(sel.selection(0).entity)
            tol_mm = inputs.itemById("tol").value * 10.0  # value is in cm
            convert(body, tol_mm, inputs.itemById("snap").value, inputs.itemById("fallback").value)
        except Exception:
            _ui.messageBox("STL2CAD failed:\n" + traceback.format_exc(), "STL2CAD")


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        i = cmd.commandInputs
        s = i.addSelectionInput("mesh", "Mesh body", "Select the inserted STL mesh body")
        s.addSelectionFilter("MeshBodies")
        s.setSelectionLimits(1, 1)
        i.addValueInput("tol", "Tolerance", "mm", adsk.core.ValueInput.createByString("0.05 mm"))
        i.addBoolValueInput("fallback", "Convert mesh if rebuild is not accurate", True, "", True)
        i.addBoolValueInput("snap", "Snap holes to standard sizes (uses AI)", True, "", False)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(context):
    try:
        defs = _ui.commandDefinitions
        cd = defs.itemById(CMD_ID) or defs.addButtonDefinition(
            CMD_ID, "Convert Mesh to Model", "Rebuild an STL mesh body as a parametric solid", "")
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        panel = _ui.allToolbarPanels.itemById(PANEL_ID)
        if panel.controls.itemById(CMD_ID) is None:
            panel.controls.addCommand(cd)
    except Exception:
        _ui.messageBox("STL2CAD failed to start:\n" + traceback.format_exc())


def stop(context):
    try:
        panel = _ui.allToolbarPanels.itemById(PANEL_ID)
        ctrl = panel.controls.itemById(CMD_ID)
        if ctrl:
            ctrl.deleteMe()
        cd = _ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        _ui.messageBox("STL2CAD failed to stop:\n" + traceback.format_exc())
