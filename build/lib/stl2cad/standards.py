"""Standard hole sizes. Numeric comparison lives here, in code, not in JEV."""

from __future__ import annotations

# ISO 273 medium-fit clearance and common tap-drill diameters (mm).
CLEARANCE = {"M2": 2.4, "M2.5": 2.9, "M3": 3.4, "M4": 4.5, "M5": 5.5,
             "M6": 6.6, "M8": 9.0, "M10": 11.0, "M12": 13.5}
TAP_DRILL = {"M2": 1.6, "M2.5": 2.05, "M3": 2.5, "M4": 3.3, "M5": 4.2,
             "M6": 5.0, "M8": 6.8, "M10": 8.5, "M12": 10.2}


def nearest(d: float) -> tuple[str, float, float]:
    """Nearest standard hole: (label, size_mm, relative_deviation)."""
    best = min(
        ((f"{n} {kind}", v) for kind, t in (("clearance", CLEARANCE), ("tap drill", TAP_DRILL))
         for n, v in t.items()),
        key=lambda lv: abs(lv[1] - d),
    )
    return best[0], best[1], abs(best[1] - d) / best[1]


def deviation_words(rel: float) -> str:
    """Named bucket for the model; JEV reads words better than numbers."""
    if rel <= 0.02:
        return "under 2 percent"
    if rel <= 0.05:
        return "between 2 and 5 percent"
    return "more than 5 percent"
