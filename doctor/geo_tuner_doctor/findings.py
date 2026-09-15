"""Finding: one check's conclusion with its evidence.

Stable ids (SETTLE_CAP_BOUND, ...) so reports can be diffed across sessions;
every finding names the numbers and where they came from, because a
recommendation without evidence is exactly the kind of hand-waving the
doctor exists to replace.
"""
from __future__ import annotations

import dataclasses

PASS = "PASS"
INFO = "INFO"
WARN = "WARN"
FAIL = "FAIL"
SKIP = "SKIP"

# FAIL first in the report; SKIP last (a skipped check is a statement about
# the inputs, not the session).
SEVERITY_ORDER = [FAIL, WARN, INFO, PASS, SKIP]


@dataclasses.dataclass
class Finding:
    id: str
    severity: str
    title: str
    axis: str | None = None
    evidence: dict = dataclasses.field(default_factory=dict)
    source: list[str] = dataclasses.field(default_factory=list)
    explanation: str = ""
    recommendation_ids: list[str] = dataclasses.field(default_factory=list)

    def to_dict(self) -> dict:
        d = dataclasses.asdict(self)
        return {k: v for k, v in d.items() if v not in (None, [], {}, "")}


def skip(check_id: str, title: str, reason: str) -> Finding:
    return Finding(id=check_id, severity=SKIP, title=title,
                   explanation=f"skipped: {reason}")


def worst_severity(findings) -> str:
    for sev in SEVERITY_ORDER:
        if any(f.severity == sev for f in findings):
            return sev
    return PASS


def banner(findings) -> tuple[str, str]:
    """(verdict, one sentence why) for the top of the report."""
    fails = [f for f in findings if f.severity == FAIL]
    warns = [f for f in findings if f.severity == WARN]
    if fails:
        return "FAILED", "; ".join(f.title for f in fails)
    if warns:
        return ("SUCCESS WITH WARNINGS",
                "; ".join(f.title for f in warns))
    return "SUCCESS", "all checks passed"
