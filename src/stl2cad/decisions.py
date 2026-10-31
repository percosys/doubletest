"""Truth tables: JEV supplies judgments, code decides what to do with them."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

from .classifier import Answer
from .features import Hole
from .questions import THRESHOLDS, Thresholds

Verdict = Literal["yes", "no", "unsettled"]

# ISO 273 medium-fit clearance and common tap-drill diameters (mm).
CLEARANCE = {"M2": 2.4, "M2.5": 2.9, "M3": 3.4, "M4": 4.5, "M5": 5.5,
             "M6": 6.6, "M8": 9.0, "M10": 11.0, "M12": 13.5}
TAP_DRILL = {"M2": 1.6, "M2.5": 2.05, "M3": 2.5, "M4": 3.3, "M5": 4.2,
             "M6": 5.0, "M8": 6.8, "M10": 8.5, "M12": 10.2}


def verdict(a: Answer, th: Thresholds = THRESHOLDS) -> Verdict:
    p = a.p_true
    if p >= th.yes:
        return "yes"
    if p <= th.no:
        return "no"
    return "unsettled"


@dataclass
class HoleDecision:
    id: str
    action: Literal["include", "drop", "review"]
    measured_diameter: float
    diameter: float
    standard: str | None  # e.g. "M3 clearance"
    reason: str


def _snap(d: float, table: dict[str, float], rel: float = 0.04) -> tuple[str, float] | None:
    name, val = min(table.items(), key=lambda kv: abs(kv[1] - d))
    return (name, val) if abs(val - d) / val <= rel else None


def decide_hole(h: Hole, ans: dict[str, Answer]) -> HoleDecision:
    """Truth table over {intentional, fastener} x purpose. Every cell has an outcome."""
    intent = verdict(ans[f"{h.id}.intentional"])
    fast = verdict(ans[f"{h.id}.fastener"])
    purpose = ans[f"{h.id}.purpose"].top
    d = h.diameter

    if intent == "no":
        return HoleDecision(h.id, "drop", d, d, None, "judged not intentional")
    action = "include" if intent == "yes" else "review"
    why = "intentional" if intent == "yes" else "intent unsettled"

    snapped = None
    if fast == "yes" and purpose in ("screw_clearance", "tap_pilot"):
        table = CLEARANCE if purpose == "screw_clearance" else TAP_DRILL
        snapped = _snap(d, table)
    elif fast == "unsettled":
        action, why = "review", why + "; fastener unsettled"

    if snapped:
        name, val = snapped
        kind = "clearance" if purpose == "screw_clearance" else "tap drill"
        return HoleDecision(h.id, action, d, val, f"{name} {kind}", f"{why}; snapped to {name} {kind}")
    return HoleDecision(h.id, action, d, round(d, 2), None, why + "; kept measured size")
