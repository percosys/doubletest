"""Answering questions: a common interface with swappable transports.

``RuleTransport`` is an offline stand-in so the whole pipeline runs and is
testable without network access. ``JevTransport`` is the live adapter; its
request/response mapping is intentionally NOT implemented yet because the
TypeSafe docs were unreachable when this was scaffolded (see README).
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Protocol

from .questions import Question


@dataclass
class Answer:
    kind: str
    probs: dict[str, float]  # choice: option -> p ; noul: {"true": p, "false": 1-p}

    @property
    def p_true(self) -> float:
        return self.probs["true"]

    @property
    def top(self) -> str:
        return max(self.probs, key=self.probs.get)


class Transport(Protocol):
    def ask(self, state: dict, questions: list[Question]) -> dict[str, Answer]: ...


class JevError(RuntimeError):
    """Raised when a JEV call fails. A failed call is an error, never a 'no'."""


class JevTransport:
    """Live TypeSafe/JEV adapter (not wired up yet).

    To finish: read https://docs.typesafe.ai/go/api.md and the OpenAPI spec at
    https://api.typesafe.ai/openapi.json, then map ``Question`` -> request item
    and the response -> ``Answer``. Keep the API key server-side
    (``TYPESAFE_API_KEY``), log the exact payload, and raise ``JevError`` on any
    failure.
    """

    def __init__(self, api_key: str | None = None, base_url: str = "https://api.typesafe.ai/v1"):
        self.api_key = api_key or os.environ.get("TYPESAFE_API_KEY")
        self.base_url = base_url
        if not self.api_key:
            raise JevError("TYPESAFE_API_KEY is not set")

    def ask(self, state: dict, questions: list[Question]) -> dict[str, Answer]:
        raise NotImplementedError(
            "JEV request mapping is unverified; implement against the live docs/OpenAPI spec."
        )


class RuleTransport:
    """Offline heuristics answering the same questions (for tests and dry runs)."""

    def ask(self, state: dict, questions: list[Question]) -> dict[str, Answer]:
        part = state["part"]
        out: dict[str, Answer] = {}
        for q in questions:
            out[q.id] = self._answer(q, part, state["features"])
        return out

    @staticmethod
    def _noul(p: float) -> Answer:
        return Answer("noul", {"true": p, "false": 1 - p})

    def _answer(self, q: Question, part: dict, feats: dict) -> Answer:
        if q.id == "part_type":
            if part["surface_not_flat_or_cylindrical_percent"] > 40:
                top = "organic_freeform"
            elif part["flat_faces_parallel_to_axes"] >= 6 and part["holes"] >= 0:
                top = "flat_plate_with_holes"
            else:
                top = "turned_part"
            return Answer("choice", {k: (0.8 if k == top else 0.2 / (len(q.options) - 1))
                                     for k in q.options})
        if q.id == "part_type_fits":
            return self._noul(0.9 if part["surface_not_flat_or_cylindrical_percent"] < 40 else 0.1)
        fid, _, what = q.id.partition(".")
        f = feats[fid]
        if what == "intentional":
            return self._noul(0.95 if f["diameter_mm"] >= 1.0 else 0.2)
        if what == "fastener":
            return self._noul(0.85 if 1.5 <= f["diameter_mm"] <= 14 else 0.1)
        if what == "purpose":
            d = f["diameter_mm"]
            top = "screw_clearance" if 2.2 <= d <= 14 else "vent_or_lightening"
            return Answer("choice", {k: (0.7 if k == top else 0.3 / (len(q.options) - 1))
                                     for k in q.options})
        if what == "designed":
            return self._noul(0.9)
        if what == "role":
            top = "edge_fillet" if f["angular_extent_deg"] < 120 else "slot_or_pocket_end"
            return Answer("choice", {k: (0.7 if k == top else 0.3 / (len(q.options) - 1))
                                     for k in q.options})
        raise KeyError(q.id)
