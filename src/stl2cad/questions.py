"""All JEV question wording, options, criteria and thresholds, in one place.

Design rules (from the TypeSafe skill):
  * JEV supplies judgments; ``decisions.py`` holds the truth tables that act on them.
  * One judgment per question, concrete and observable, phrased so true means yes.
  * Every Choice is paired with a Noul that says whether anything fits.
  * Questions are answered independently; the state carries all needed context.
  * Option names describe content (never A/B/1/2).

NOTE: these are transport-agnostic ``Question`` objects. Serialising them into a
TypeSafe request happens in ``classifier.JevTransport`` once the live request
schema has been verified against https://docs.typesafe.ai/go/api.md.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

from .features import Features


@dataclass(frozen=True)
class Thresholds:
    """Noul probability bands: >= yes -> yes, <= no -> no, between -> unsettled."""

    yes: float = 0.7
    no: float = 0.3


THRESHOLDS = Thresholds()


@dataclass(frozen=True)
class Question:
    id: str
    kind: Literal["choice", "noul"]
    instructions: str
    # choice: option name -> description of when it applies
    options: dict[str, str] = field(default_factory=dict)
    # noul: concrete situations for each side
    true_criterion: str = ""
    false_criterion: str = ""


PART_TYPES = {
    "flat_plate_with_holes": "A flat plate or slab, possibly with holes, pockets or notches.",
    "bent_or_l_bracket": "A bracket made of two or more flat flanges joined at angles.",
    "enclosure_shell": "A hollow box or housing with thin walls.",
    "turned_part": "A part that is round about one axis, like a shaft, bushing or knob.",
    "organic_freeform": "A sculpted, scanned or curved shape with few flat or cylindrical faces.",
}

HOLE_PURPOSES = {
    "screw_clearance": "A hole a screw passes through freely, e.g. 3.4 mm for an M3 screw.",
    "tap_pilot": "A smaller hole meant to be threaded, e.g. 2.5 mm for an M3 thread.",
    "dowel_or_press_fit": "A hole sized to hold a pin or bearing tightly.",
    "vent_or_lightening": "A hole for airflow, cable routing or weight reduction.",
}

ARC_ROLES = {
    "edge_fillet": "A rounded transition between two flat faces, about one quarter turn.",
    "slot_or_pocket_end": "The rounded end of a slot or the rounded corner of a pocket.",
    "rounded_outer_edge": "A rounded outside corner of the whole part.",
}


def part_questions() -> list[Question]:
    return [
        Question("part_type", "choice",
                 "Which kind of part is `part`?", options=PART_TYPES),
        Question("part_type_fits", "noul",
                 "Does one of the part kinds describe `part`?",
                 true_criterion="`part` is mostly flat faces and cylinders, as a machined or moulded part is.",
                 false_criterion="`part` is smooth and curved everywhere, like a scan of a statue."),
    ]


def hole_questions(hole_id: str) -> list[Question]:
    p = f"features.{hole_id}"
    return [
        Question(f"{hole_id}.intentional", "noul",
                 f"Is `{p}` a hole someone designed into the part?",
                 true_criterion="The diameter and depth look like a deliberate hole.",
                 false_criterion="It is a dent, a casting void or a scan artefact."),
        Question(f"{hole_id}.fastener", "noul",
                 f"Is `{p}` meant to take a screw or bolt?",
                 true_criterion="The diameter is close to a metric screw clearance or tap-drill size.",
                 false_criterion="The diameter is far from any screw size, or the hole is very large."),
        Question(f"{hole_id}.purpose", "choice",
                 f"What is `{p}` for?", options=HOLE_PURPOSES),
    ]


def arc_questions(arc_id: str) -> list[Question]:
    p = f"features.{arc_id}"
    return [
        Question(f"{arc_id}.designed", "noul",
                 f"Is `{p}` a rounded surface someone designed into the part?",
                 true_criterion="The radius is a clean value and the surface joins flat faces smoothly.",
                 false_criterion="The surface is a scan artefact or tessellation noise."),
        Question(f"{arc_id}.role", "choice",
                 f"What does `{p}` do on the part?", options=ARC_ROLES),
    ]


def build_state(feats: Features) -> dict:
    """Named-field state. Only facts a person would look at; nothing irrelevant."""
    return {
        "part": {
            "size_mm": dict(zip("xyz", feats.extents)),
            "flat_faces_parallel_to_axes": feats.axis_aligned_plane_count,
            "angled_flat_faces": len(feats.facets),
            "holes": len(feats.holes),
            "round_bosses": len(feats.bosses),
            "partial_cylinders": len(feats.arcs),
            "surface_not_flat_or_cylindrical_percent": round(100 * feats.unexplained_area_fraction, 1),
        },
        "features": {
            **{h.id: {"diameter_mm": h.diameter, "depth_mm": h.depth,
                      "part_thickness_along_axis_mm": None} for h in feats.holes},
            **{a.id: {"radius_mm": a.radius, "length_mm": a.length,
                      "angular_extent_deg": a.coverage_deg,
                      "surface_faces_inward": a.concave} for a in feats.arcs},
        },
    }


def build_questions(feats: Features) -> list[Question]:
    """One request's worth of questions (speculative fan-out: code reads what it needs)."""
    qs = part_questions()
    for h in feats.holes:
        qs += hole_questions(h.id)
    for a in feats.arcs:
        qs += arc_questions(a.id)
    return qs
