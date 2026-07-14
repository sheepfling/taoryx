"""Evidence-linked grammar contracts for the supported TAOS 96 subset."""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict


class GrammarContract(BaseModel):
    """A narrow evidence-linked boundary for canonical parser claims."""

    model_config = ConfigDict(frozen=True)

    contract_id: str
    status: Literal["supported-subset"]
    evidence_ids: tuple[str, ...]
    description: str


TAOS96_FREE_FIELD_CONTRACT = GrammarContract(
    contract_id="taos96.free-field",
    status="supported-subset",
    evidence_ids=("taos96-scan-169-p4-4", "taos96-scan-170-p4-5", "taos96-scan-171-p4-6"),
    description="Line-oriented fields accept documented punctuation delimiters; # starts a comment.",
)
TAOS96_FRAMING_CONTRACT = GrammarContract(
    contract_id="taos96.problem-framing",
    status="supported-subset",
    evidence_ids=("taos96-scan-167-p4-2", "taos96-scan-168-p4-3"),
    description="Problems open with a parenthesized identifier, contain data blocks, and terminate with *end.",
)
TAOS96_HIERARCHY_CONTRACT = GrammarContract(
    contract_id="taos96.hierarchy",
    status="supported-subset",
    evidence_ids=("taos96-scan-169-p4-4", "taos96-scan-172-p4-7", "taos96-scan-173-p4-8"),
    description="Trajectory and segment scopes determine where child blocks are attached.",
)
TAOS96_PROBLEM_CATALOG_CONTRACT = GrammarContract(
    contract_id="taos96.problem-catalog",
    status="supported-subset",
    evidence_ids=("taos96-scan-215-p4-48",),
    description="The supported problem-level block catalog is explicit; unknown names are errors.",
)
TAORYX_MODE_CONTRACT = GrammarContract(
    contract_id="taoryx.dynamics-mode",
    status="supported-subset",
    evidence_ids=("taoryx-mode-extension",),
    description="taoryx problem files may select point-mass or explicitly named successor dynamics modes.",
)

SUPPORTED_PROBLEM_BLOCKS = frozenset(
    {"atmos", "define", "earth", "egs", "file", "optimize", "print", "radar", "search", "summarize", "survey", "title", "units/fmt", "wind"}
)
SUPPORTED_TAORYX_PROBLEM_BLOCKS = frozenset({"mode"})
SUPPORTED_TRAJECTORY_BLOCKS = frozenset({"define", "dwn/crs", "file", "iip", "initial", "print", "tangent"})
SUPPORTED_SEGMENT_BLOCKS = frozenset({"aero", "constants", "cg", "fly", "increment", "inertial", "integ", "limits", "prop", "rail", "reset", "when"})
SUPPORTED_FLY_GUIDANCE_RULES = frozenset(
    {
        "alt", "cl", "cs", "downria", "dynprs", "gamgc", "gamgd", "intercept", "l/d", "l/d-max",
        "mach", "nx", "ny", "nz", "propnav", "psigc", "psigd", "thrust", "upria", "vel",
    }
)

DOCUMENTED_STATE_VARIABLES = frozenset(
    {
        "alpha", "alphat", "alt", "altdt", "bankgc", "bankgd", "beta", "betae", "cg", "dynprs",
        "ep1", "ep2", "gamgc", "gamgd", "grmark", "grseg", "latgc", "latgcdt", "latgd", "latgddt",
        "long", "longdt", "mach", "mass", "nu", "plength", "plmark", "plseg", "phi", "pitchgc",
        "pitchgd", "pitchi", "power", "pres", "psigc", "psigd", "range", "rcm", "rcmdt", "reypft",
        "rho", "rollgc", "rollgd", "rolli", "segment", "sndspd", "sref", "temp", "thrust", "time",
        "tmark", "tseg", "vair", "vel", "vgr", "visc", "wt", "yawgc", "yawgd", "yawi",
    }
)

SUPPORTED_GRAMMAR_CONTRACTS = (
    TAOS96_FREE_FIELD_CONTRACT,
    TAOS96_FRAMING_CONTRACT,
    TAOS96_HIERARCHY_CONTRACT,
    TAOS96_PROBLEM_CATALOG_CONTRACT,
)
