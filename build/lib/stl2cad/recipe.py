"""Feature recipe: the contract between analysis and CAD emission."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field

from .classifier import Answer
from .decisions import HoleDecision, decide_hole, verdict
from .features import Features


@dataclass
class Recipe:
    units: str
    part_type: str
    stock: list[float]  # x, y, z extents of the bounding box, min corner at origin
    holes: list[dict] = field(default_factory=list)
    review: list[str] = field(default_factory=list)  # items a person should look at
    notes: list[str] = field(default_factory=list)

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2)


def build_recipe(feats: Features, answers: dict[str, Answer]) -> Recipe:
    part_type = answers["part_type"].top
    r = Recipe("mm", part_type, feats.extents)

    if verdict(answers["part_type_fits"]) != "yes":
        r.review.append("part: type uncertain; recipe may not represent this part")
    if part_type != "flat_plate_with_holes":
        r.notes.append(f"part_type={part_type}: only stock + holes are emitted by this version")
    if feats.unexplained_area_fraction > 0.1:
        r.review.append(
            f"part: {100 * feats.unexplained_area_fraction:.0f}% of surface is not flat or cylindrical"
        )

    for h in feats.holes:
        d: HoleDecision = decide_hole(h, answers)
        if d.action == "drop":
            r.notes.append(f"{h.id} dropped: {d.reason}")
            continue
        r.holes.append({"id": h.id, "center": h.center, "axis": h.axis, "diameter": d.diameter,
                        "measured_diameter": d.measured_diameter, "depth": h.depth,
                        "standard": d.standard, "reason": d.reason})
        if d.action == "review":
            r.review.append(f"{h.id}: {d.reason}")

    if feats.bosses or feats.arcs or feats.facets:
        r.notes.append(
            f"not yet emitted: {len(feats.bosses)} bosses, {len(feats.arcs)} partial cylinders, "
            f"{len(feats.facets)} angled faces"
        )
    return r
