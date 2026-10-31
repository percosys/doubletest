"""Build a stl2cad plan in Fusion. Logic is pure Python; Fusion modules are passed in.

NOTE: the Fusion API calls here were written from the API's documented shape but have
NOT been run inside Fusion. See the repo README ("Fusion add-in: untested").
"""

CM = 0.1  # plan is in millimetres, Fusion's API is in centimetres


def match_profiles(expected, actual, area_rtol=0.02, dist_tol=0.05):
    """Pick the sketch profiles that are material.

    A sketch with a hole yields two profiles (the ring and the hole's disc). Only the
    ring is material, so we match each expected region by area and centroid.
    expected/actual: lists of (area, cx, cy) in the same units. Returns indices into
    `actual`, one per expected region. Raises LookupError if any region is unmatched.
    """
    used, out = set(), []
    for ea, ex, ey in expected:
        best, best_d = None, None
        for i, (aa, ax, ay) in enumerate(actual):
            if i in used or abs(aa - ea) > area_rtol * ea:
                continue
            d = ((ax - ex) ** 2 + (ay - ey) ** 2) ** 0.5
            if d <= dist_tol and (best is None or d < best_d):
                best, best_d = i, d
        if best is None:
            raise LookupError("no sketch profile matches region area=%.5f at (%.4f, %.4f)" % (ea, ex, ey))
        used.add(best)
        out.append(best)
    return out


def _is_identity(m, tol=1e-9):
    return all(abs(m[r][c] - (1.0 if r == c else 0.0)) < tol for r in range(4) for c in range(4))


def _draw_loop(core, sketch, loop):
    P = lambda x, y: core.Point3D.create(x * CM, y * CM, 0)
    if loop["type"] == "circle":
        sketch.sketchCurves.sketchCircles.addByCenterRadius(P(loop["cx"], loop["cy"]), loop["r"] * CM)
        return
    if loop["type"] == "path":
        # exact lines and three-point arcs, joined end to end through shared sketch points
        lines, arcs = sketch.sketchCurves.sketchLines, sketch.sketchCurves.sketchArcs
        pieces = loop["pieces"]
        first_start, prev_end = None, None
        for i, pc in enumerate(pieces):
            start = prev_end if prev_end is not None else P(*pc["a"])
            end = first_start if (i == len(pieces) - 1 and first_start is not None) else P(*pc["b"])
            if pc["kind"] == "line":
                ent = lines.addByTwoPoints(start, end)
            else:
                ent = arcs.addByThreePoints(start, P(*pc["m"]), end)
            if first_start is None:
                first_start = ent.startSketchPoint
            prev_end = ent.endSketchPoint
        return
    pts = loop["pts"]
    lines = sketch.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P(*pts[0]), P(*pts[1]))
    prev = first
    for p in pts[2:]:
        prev = lines.addByTwoPoints(prev.endSketchPoint, P(*p))
    lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint)


def match_edge(desc, info, tol=1e-3):
    """Does a Fusion edge (info, in cm) match a plan edge descriptor (mm)?"""
    near = lambda a, b: abs(a - b) <= tol
    z = desc["z"] * CM
    if desc["type"] == "circle":
        if info["type"] != "circle":
            return False
        cx, cy, cz = info["c"]
        return (near(cx, desc["cx"] * CM) and near(cy, desc["cy"] * CM) and near(cz, z)
                and near(info["r"], desc["r"] * CM))
    if desc["type"] == "arc":
        if info["type"] != "arc":
            return False
        a = (desc["a"][0] * CM, desc["a"][1] * CM, z)
        b = (desc["b"][0] * CM, desc["b"][1] * CM, z)
        cx, cy, cz = info["c"]
        same = lambda p, q: all(near(p[k], q[k]) for k in range(3))
        ends = (same(a, info["p0"]) and same(b, info["p1"])) or (same(a, info["p1"]) and same(b, info["p0"]))
        return ends and near(cx, desc["cx"] * CM) and near(cy, desc["cy"] * CM) and near(cz, z) \
            and near(info["r"], desc["r"] * CM)
    if info["type"] != "segment":
        return False
    p0 = (desc["p0"][0] * CM, desc["p0"][1] * CM, z)
    p1 = (desc["p1"][0] * CM, desc["p1"][1] * CM, z)
    same = lambda a, b: all(near(a[k], b[k]) for k in range(3))
    return (same(p0, info["p0"]) and same(p1, info["p1"])) or (same(p0, info["p1"]) and same(p1, info["p0"]))


def _edge_info(core, edge):
    g = edge.geometry
    if g.curveType == core.Curve3DTypes.Circle3DCurveType:
        c = core.Circle3D.cast(g)
        return {"type": "circle", "c": (c.center.x, c.center.y, c.center.z), "r": c.radius}
    a, b = edge.startVertex.geometry, edge.endVertex.geometry
    if g.curveType == core.Curve3DTypes.Arc3DCurveType:
        arc = core.Arc3D.cast(g)
        return {"type": "arc", "c": (arc.center.x, arc.center.y, arc.center.z), "r": arc.radius,
                "p0": (a.x, a.y, a.z), "p1": (b.x, b.y, b.z)}
    return {"type": "segment", "p0": (a.x, a.y, a.z), "p1": (b.x, b.y, b.z)}


def apply_treatments(core, fusion, root, bodies, plan):
    """Fillets and chamfers recovered from the STL. Returns notes about anything skipped.

    Runs before the Move, so plan coordinates (aligned frame) are the body's coordinates.
    """
    notes = []
    if not plan.get("treatments") or not bodies:
        return notes
    body = bodies[0]
    V = core.ValueInput.createByReal
    for t in plan["treatments"]:
        infos = [(e, _edge_info(core, e)) for e in body.edges]  # edges change after each feature
        sel = core.ObjectCollection.create()
        for d in t["edges"]:
            hits = [e for e, i in infos if match_edge(d, i)]
            if not hits:
                notes.append("%s at z=%.2f: edge %s not found" % (t["kind"], t["z"], d["type"]))
            for e in hits:
                sel.add(e)
        if sel.count == 0:
            continue
        try:
            if t["kind"] == "fillet":
                fil = root.features.filletFeatures
                inp = fil.createInput()
                inp.addConstantRadiusEdgeSet(sel, V(t["size_xy"] * CM), True)
                fil.add(inp)
            else:
                cha = root.features.chamferFeatures
                inp = cha.createInput2()
                if abs(t["size_xy"] - t["size_z"]) <= 0.15 * max(t["size_xy"], t["size_z"]):
                    inp.chamferEdgeSets.addEqualDistanceChamferEdgeSet(sel, V(t["size_xy"] * CM), True)
                else:  # which distance lands on which face is unverified
                    inp.chamferEdgeSets.addTwoDistancesChamferEdgeSet(
                        sel, V(t["size_z"] * CM), V(t["size_xy"] * CM), True)
                    notes.append("asymmetric chamfer at z=%.2f: check its orientation" % t["z"])
                cha.add(inp)
        except Exception as ex:  # keep the sharp edges rather than fail the whole build
            notes.append("%s at z=%.2f skipped: %s" % (t["kind"], t["z"], ex))
    return notes


def build_plan(core, fusion, root, plan):
    """Create one sketch + extrude per layer (join), then the edge treatments, then move the
    result into the mesh pose. Returns (bodies, notes)."""
    planes, sketches = root.constructionPlanes, root.sketches
    extrudes = root.features.extrudeFeatures
    diag = max(plan["stats"]["extents_mm"]) * CM
    bodies = []
    for i, L in enumerate(plan["layers"]):
        pin = planes.createInput()
        pin.setByOffset(root.xYConstructionPlane, core.ValueInput.createByReal(L["z0"] * CM))
        sk = sketches.add(planes.add(pin))
        sk.name = "STL2CAD layer %d" % (i + 1)
        for reg in L["regions"]:
            for loop in [reg["outer"]] + reg["holes"]:
                _draw_loop(core, sk, loop)

        expected = [(r["area"] * CM * CM, r["centroid"][0] * CM, r["centroid"][1] * CM)
                    for r in L["regions"]]
        actual = []
        for k in range(sk.profiles.count):
            ap = sk.profiles.item(k).areaProperties()
            actual.append((ap.area, ap.centroid.x, ap.centroid.y))
        picks = match_profiles(expected, actual, dist_tol=0.005 * diag + 0.01)
        coll = core.ObjectCollection.create()
        for idx in picks:
            coll.add(sk.profiles.item(idx))

        op = (fusion.FeatureOperations.NewBodyFeatureOperation if i == 0
              else fusion.FeatureOperations.JoinFeatureOperation)
        inp = extrudes.createInput(coll, op)
        inp.setDistanceExtent(False, core.ValueInput.createByReal((L["z1"] - L["z0"]) * CM))
        ext = extrudes.add(inp)
        if i == 0:
            bodies = [ext.bodies.item(j) for j in range(ext.bodies.count)]

    notes = apply_treatments(core, fusion, root, bodies, plan)

    F = plan["frame"]
    if bodies and not _is_identity(F):
        V = core.Vector3D.create
        m = core.Matrix3D.create()
        m.setToAlignCoordinateSystems(
            core.Point3D.create(0, 0, 0), V(1, 0, 0), V(0, 1, 0), V(0, 0, 1),
            core.Point3D.create(F[0][3] * CM, F[1][3] * CM, F[2][3] * CM),
            V(F[0][0], F[1][0], F[2][0]), V(F[0][1], F[1][1], F[2][1]), V(F[0][2], F[1][2], F[2][2]))
        coll = core.ObjectCollection.create()
        for b in bodies:
            coll.add(b)
        moves = root.features.moveFeatures
        mi = moves.createInput2(coll)
        mi.defineAsFreeMove(m)
        moves.add(mi)
    return bodies, notes
