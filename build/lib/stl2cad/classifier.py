"""Answering questions: a common interface with swappable transports.

``RuleTransport`` is an offline stand-in so the whole pipeline runs and is
testable without network access. ``JevTransport`` is the live adapter.
"""

from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
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
    """Live TypeSafe/JEV adapter: POST {base_url}/systemone (see docs.typesafe.ai/go/api.md).

    All questions of a request share one ``state`` and are answered independently.
    The exact request payload is kept in ``self.last_request`` (never the API key)
    and appended to ``log_path`` when given, so any answer can be rerun.
    """

    def __init__(self, api_key: str | None = None, model: str = "jev-1.14.0",
                 base_url: str = "https://api.typesafe.ai/v1", log_path: str | None = None,
                 timeout: float = 60.0, max_retries: int = 3):
        self.api_key = api_key or os.environ.get("TYPESAFE_API_KEY")
        if not self.api_key:
            raise JevError("TYPESAFE_API_KEY is not set")
        self.model, self.base_url, self.log_path = model, base_url.rstrip("/"), log_path
        self.timeout, self.max_retries = timeout, max_retries
        self.last_request: dict | None = None
        self.last_response: dict | None = None

    @staticmethod
    def payload_question(q: Question) -> dict:
        if q.kind == "noul":
            return {"type": "noul", "instructions": q.instructions,
                    "criteria": {"true": q.true_criterion, "false": q.false_criterion}}
        return {"type": "choice", "instructions": q.instructions, "criteria": dict(q.options)}

    def ask(self, state: dict, questions: list[Question]) -> dict[str, Answer]:
        body = {"model": self.model, "state": state,
                "questions": {q.id: self.payload_question(q) for q in questions}}
        self.last_request = body
        data = self._post(body)
        self.last_response = data
        if self.log_path:
            with open(self.log_path, "a") as fh:
                fh.write(json.dumps({"request": body, "response": data}) + "\n")
        return self._parse(questions, data)

    def _post(self, body: dict) -> dict:
        req = urllib.request.Request(
            f"{self.base_url}/systemone", data=json.dumps(body).encode(), method="POST",
            headers={"Authorization": f"Bearer {self.api_key}", "Content-Type": "application/json"})
        for attempt in range(self.max_retries + 1):
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as r:
                    return json.loads(r.read())
            except urllib.error.HTTPError as e:
                detail = e.read().decode(errors="replace")[:500]
                if e.code == 429 and attempt < self.max_retries:
                    time.sleep(float(e.headers.get("retry-after") or 2 ** attempt))
                    continue
                raise JevError(f"JEV HTTP {e.code}: {detail}") from e
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
                raise JevError(f"JEV request failed: {e}") from e
        raise JevError("JEV rate limit: retries exhausted")

    @staticmethod
    def _parse(questions: list[Question], data: dict) -> dict[str, Answer]:
        answers = data.get("answers")
        if not isinstance(answers, dict):
            raise JevError(f"malformed JEV response: {str(data)[:300]}")
        out: dict[str, Answer] = {}
        for q in questions:
            a = answers.get(q.id)
            if a is None or a.get("type") != q.kind:
                raise JevError(f"missing or mistyped answer for {q.id!r}: {a}")
            if q.kind == "noul":
                p = float(a["noul"])
                out[q.id] = Answer("noul", {"true": p, "false": 1 - p})
            else:
                probs = {k: float(v) for k, v in a["probabilities"].items()}
                if set(probs) != set(q.options):
                    raise JevError(f"unexpected options for {q.id!r}: {sorted(probs)}")
                out[q.id] = Answer("choice", probs)
        return out


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
