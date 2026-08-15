"""Survey loose mapping boundaries and schema-shaped configuration projections.

The audit deliberately distinguishes numeric maps (for example, a named state
vector ``Mapping[str, float]``) from open payload bags such as
``dict[str, Any]``.  The former can be a valid mathematical representation;
the latter needs an explicit owner, validator, or typed envelope when another
module consumes it.
"""

from __future__ import annotations

import argparse
import ast
import json
from collections import Counter, defaultdict
from collections.abc import Iterable, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE_ROOTS = (ROOT / "src", ROOT / "packages")
AUDIT_SCHEMA = "taoryx.loose-contract-audit/v1"
BoundaryKind = Literal[
    "loose_parameter",
    "loose_return",
    "untyped_payload_parameter",
    "loose_model_field",
]
Priority = Literal["critical", "high", "medium", "low"]

_MAPPING_TOKENS = ("dict", "Dict", "Mapping", "MutableMapping")
_LOOSE_TOKENS = ("Any", "object")
_PAYLOAD_HINTS = (
    "arg",
    "config",
    "configuration",
    "data",
    "detail",
    "extension",
    "manifest",
    "metadata",
    "option",
    "payload",
    "plan",
    "projection",
    "request",
    "response",
    "value",
)
_CONFIGURATION_HINTS = (
    "author",
    "config",
    "configuration",
    "draft",
    "projection",
    "segment",
    "value",
)


@dataclass(frozen=True, slots=True)
class LooseContractFinding:
    """One public, loosely typed boundary discovered statically."""

    path: str
    line: int
    symbol: str
    kind: BoundaryKind
    annotation: str
    payload_role: str
    configuration_projection: bool
    consumer_modules: tuple[str, ...]
    priority: Priority
    recommendation: str

    def public_dict(self) -> dict[str, object]:
        """Return a JSON-safe stable record."""

        payload = asdict(self)
        payload["consumer_modules"] = list(self.consumer_modules)
        return payload
        ####

    ####


@dataclass(frozen=True, slots=True)
class _SourceModule:
    path: Path
    module: str
    tree: ast.Module

    ####


def audit(source_roots: Sequence[Path] = DEFAULT_SOURCE_ROOTS) -> dict[str, object]:
    """Return a deterministic inventory of public loose-contract boundaries."""

    modules = _load_modules(source_roots)
    consumers = _consumer_index(modules)
    findings: list[LooseContractFinding] = []
    for source in modules:
        findings.extend(_module_findings(source, consumers))
    findings.sort(key=lambda item: (_priority_rank(item.priority), item.path, item.line, item.symbol, item.kind))
    by_kind = Counter(item.kind for item in findings)
    by_priority = Counter(item.priority for item in findings)
    configuration_findings = [item for item in findings if item.configuration_projection]
    return {
        "schema": AUDIT_SCHEMA,
        "status": "survey_complete",
        "source_roots": [str(path) for path in source_roots],
        "summary": {
            "finding_count": len(findings),
            "configuration_projection_count": len(configuration_findings),
            "by_kind": dict(sorted(by_kind.items())),
            "by_priority": dict(sorted(by_priority.items())),
            "externally_consumed_count": sum(bool(item.consumer_modules) for item in findings),
        },
        "findings": [item.public_dict() for item in findings],
    }
    ####


def render_markdown(report: dict[str, object], *, limit: int | None = None) -> str:
    """Render the audit as a concise reviewer-facing Markdown survey."""

    summary = _mapping(report.get("summary"), path="summary")
    findings = _finding_records(report)
    if limit is not None:
        findings = findings[:limit]
    lines = [
        "# Loose contract boundary audit",
        "",
        "This is a static survey of public Python boundaries that carry an unbounded "
        "`dict`/`Mapping` or `Any` payload. It intentionally excludes typed numeric maps, "
        "which are often the correct representation for named state or actuator vectors.",
        "",
        "## Summary",
        "",
        f"- Findings: {summary['finding_count']}",
        f"- Configuration-projection candidates: {summary['configuration_projection_count']}",
        f"- Boundaries with detected cross-module consumers: {summary['externally_consumed_count']}",
        f"- By priority: {json.dumps(summary['by_priority'], sort_keys=True)}",
        "",
        "## Prioritized boundaries",
        "",
        "| Priority | Boundary | Kind | Consumers | Why it matters |",
        "| --- | --- | --- | ---: | --- |",
    ]
    for finding in findings:
        consumers = finding["consumer_modules"]
        consumer_count = len(consumers) if isinstance(consumers, list) else 0
        boundary = f"`{finding['path']}:{finding['line']}` — `{finding['symbol']}`"
        annotation = str(finding["annotation"]).replace("|", "\\|")
        role = str(finding["payload_role"]).replace("|", "\\|")
        lines.append(
            f"| {finding['priority']} | {boundary} | `{finding['kind']}` ({annotation}) | "
            f"{consumer_count} | {role} |"
        )
    if not findings:
        lines.append("| — | No loose public boundaries found | — | 0 | — |")
    lines.extend(
        [
            "",
            "## Normalization rule",
            "",
            "Treat a finding as a migration boundary, not an instruction to replace every map. "
            "Normalize configuration-shaped input at the owner’s ingress, validate it against "
            "the provider schema, and hand downstream consumers an immutable typed contract "
            "such as `PreparedTrajectoryConfiguration`. Keep intentionally open extensions "
            "behind an explicitly named extension field plus a validator and claim boundary.",
            "",
        ]
    )
    return "\n".join(lines)
    ####


def _load_modules(source_roots: Sequence[Path]) -> tuple[_SourceModule, ...]:
    """Parse eligible source files once and assign importable module names."""

    sources: list[_SourceModule] = []
    for source_root in source_roots:
        for path in sorted(source_root.rglob("*.py")):
            if _skip_path(path):
                continue
            try:
                tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            except SyntaxError as error:
                raise ValueError(f"could not parse {path}: {error}") from error
            sources.append(_SourceModule(path=path, module=_module_name(path, source_root), tree=tree))
    return tuple(sources)
    ####


def _skip_path(path: Path) -> bool:
    """Ignore generated, vendored, and fixture Python that is not product source."""

    ignored = {"__pycache__", ".git", ".venv", "build", "data", "dist", "tests"}
    return any(part in ignored for part in path.parts)
    ####


def _module_name(path: Path, source_root: Path) -> str:
    """Infer an import module name from core or package source layout."""

    parts = path.parts
    if "src" in parts:
        relative = path.relative_to(Path(*parts[: parts.index("src") + 1]))
    else:
        relative = path.relative_to(source_root)
    module_parts = list(relative.with_suffix("").parts)
    if module_parts[-1] == "__init__":
        module_parts.pop()
    return ".".join(module_parts)
    ####


def _consumer_index(modules: Iterable[_SourceModule]) -> dict[tuple[str, str], set[str]]:
    """Find direct cross-module calls to imported public functions.

    This is intentionally conservative: it records only imports that can be
    resolved statically, so a reported consumer is evidence rather than a
    speculative text match.
    """

    consumers: dict[tuple[str, str], set[str]] = defaultdict(set)
    for source in modules:
        imported: dict[str, tuple[str, str]] = {}
        for node in ast.walk(source.tree):
            if isinstance(node, ast.ImportFrom) and node.module is not None and node.level == 0:
                for imported_name in node.names:
                    if imported_name.name != "*":
                        imported[imported_name.asname or imported_name.name] = (node.module, imported_name.name)
        for node in ast.walk(source.tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in imported:
                consumers[imported[node.func.id]].add(source.module)
    return consumers
    ####


def _module_findings(
    source: _SourceModule,
    consumers: dict[tuple[str, str], set[str]],
) -> list[LooseContractFinding]:
    """Extract top-level public functions and Pydantic-like model fields."""

    findings: list[LooseContractFinding] = []
    for node in source.tree.body:
        if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef) and not node.name.startswith("_"):
            findings.extend(_function_findings(source, node, consumers))
        elif isinstance(node, ast.ClassDef) and not node.name.startswith("_"):
            findings.extend(_model_field_findings(source, node))
    return findings
    ####


def _function_findings(
    source: _SourceModule,
    node: ast.FunctionDef | ast.AsyncFunctionDef,
    consumers: dict[tuple[str, str], set[str]],
) -> list[LooseContractFinding]:
    """Find loose input and output annotations for one public function."""

    findings: list[LooseContractFinding] = []
    target_consumers = tuple(sorted(consumers.get((source.module, node.name), set()) - {source.module}))
    positional = (*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs)
    for argument in positional:
        if argument.arg in {"self", "cls"}:
            continue
        annotation = _annotation(argument.annotation)
        if _is_loose_mapping(annotation):
            findings.append(
                _finding(
                    source,
                    node.lineno,
                    f"{node.name}({argument.arg})",
                    "loose_parameter",
                    annotation,
                    target_consumers,
                )
            )
        elif argument.annotation is None and _is_payload_name(argument.arg):
            findings.append(
                _finding(
                    source,
                    node.lineno,
                    f"{node.name}({argument.arg})",
                    "untyped_payload_parameter",
                    "unannotated",
                    target_consumers,
                )
            )
    return_annotation = _annotation(node.returns)
    if _is_loose_mapping(return_annotation):
        findings.append(
            _finding(
                source,
                node.lineno,
                f"{node.name}()",
                "loose_return",
                return_annotation,
                target_consumers,
            )
        )
    return findings
    ####


def _model_field_findings(source: _SourceModule, node: ast.ClassDef) -> list[LooseContractFinding]:
    """Find public Pydantic/dataclass fields that act as unbounded bags."""

    findings: list[LooseContractFinding] = []
    for statement in node.body:
        if not isinstance(statement, ast.AnnAssign) or not isinstance(statement.target, ast.Name):
            continue
        annotation = _annotation(statement.annotation)
        field_name = statement.target.id
        if _is_loose_mapping(annotation) or (_is_payload_name(field_name) and _is_loose_any(annotation)):
            findings.append(
                _finding(
                    source,
                    statement.lineno,
                    f"{node.name}.{field_name}",
                    "loose_model_field",
                    annotation,
                    (),
                )
            )
    return findings
    ####


def _finding(
    source: _SourceModule,
    line: int,
    symbol: str,
    kind: BoundaryKind,
    annotation: str,
    consumers: tuple[str, ...],
) -> LooseContractFinding:
    """Classify one raw static signal into a stable, ranked audit finding."""

    lowered = symbol.casefold()
    configuration_projection = any(token in lowered for token in _CONFIGURATION_HINTS)
    payload_role = "configuration projection" if configuration_projection else "open payload"
    score = 1
    if _is_loose_any(annotation):
        score += 2
    if configuration_projection:
        score += 3
    if consumers:
        score += 2 + min(len(consumers), 2)
    if kind == "loose_return":
        score += 1
    priority: Priority = "critical" if score >= 8 else "high" if score >= 6 else "medium" if score >= 4 else "low"
    if configuration_projection:
        recommendation = (
            "Normalize at ingress into a structural configuration envelope, validate against the selected provider schema, "
            "and pass PreparedTrajectoryConfiguration downstream."
        )
    elif "extension" in lowered:
        recommendation = "Keep it only as an explicitly named extension bag with an owner, validator, and compatibility policy."
    else:
        recommendation = "Replace the open bag at the producer/consumer seam with a Pydantic model, TypedDict, or named protocol."
    return LooseContractFinding(
        path=_display_path(source.path),
        line=line,
        symbol=symbol,
        kind=kind,
        annotation=annotation,
        payload_role=payload_role,
        configuration_projection=configuration_projection,
        consumer_modules=consumers,
        priority=priority,
        recommendation=recommendation,
    )
    ####


def _annotation(node: ast.expr | None) -> str:
    """Render an annotation without depending on source formatting."""

    return ast.unparse(node) if node is not None else "unannotated"
    ####


def _is_loose_mapping(annotation: str) -> bool:
    """Return whether a mapping annotation accepts unconstrained values."""

    return any(token in annotation for token in _MAPPING_TOKENS) and (
        annotation in _MAPPING_TOKENS or _is_loose_any(annotation)
    )
    ####


def _is_loose_any(annotation: str) -> bool:
    """Return whether an annotation exposes Any/object as its value surface."""

    return any(token in annotation for token in _LOOSE_TOKENS)
    ####


def _is_payload_name(name: str) -> bool:
    """Return whether a name suggests a portable payload rather than a scalar."""

    lowered = name.casefold()
    return any(token in lowered for token in _PAYLOAD_HINTS)
    ####


def _priority_rank(priority: Priority) -> int:
    """Sort critical migration targets first."""

    return {"critical": 0, "high": 1, "medium": 2, "low": 3}[priority]
    ####


def _display_path(path: Path) -> str:
    """Render workspace-relative paths whenever possible."""

    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)
    ####


def _mapping(value: object, *, path: str) -> dict[str, object]:
    """Validate an internal JSON-object record defensively."""

    if not isinstance(value, dict):
        raise TypeError(f"{path} must be a dict")
    return value
    ####


def _finding_records(report: dict[str, object]) -> list[dict[str, object]]:
    """Validate the internal report payload before rendering it."""

    raw = report.get("findings")
    if not isinstance(raw, list):
        raise TypeError("findings must be a list")
    return [_mapping(item, path="findings[]") for item in raw]
    ####


def main(argv: Sequence[str] | None = None) -> int:
    """Write the static survey as JSON or Markdown."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        action="append",
        type=Path,
        dest="source_roots",
        help="source root to inspect; repeatable (defaults to src and packages)",
    )
    parser.add_argument("--format", choices=("markdown", "json"), default="markdown")
    parser.add_argument("--limit", type=int, default=None, help="maximum rendered Markdown findings")
    parser.add_argument("--output", type=Path, help="write the report here instead of stdout")
    arguments = parser.parse_args(argv)
    report = audit(tuple(arguments.source_roots) if arguments.source_roots else DEFAULT_SOURCE_ROOTS)
    text = (
        json.dumps(report, indent=2, sort_keys=True) + "\n"
        if arguments.format == "json"
        else render_markdown(report, limit=arguments.limit) + "\n"
    )
    if arguments.output is None:
        print(text, end="")
    else:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(text, encoding="utf-8")
        print(f"wrote {arguments.output}")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
