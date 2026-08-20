"""Validate the documentation ownership layout and its navigational links."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MARKDOWN_LINK = re.compile(r"(?<!!)\[[^\]]*\]\(([^)]+)\)")
FENCED_CODE_BLOCK = re.compile(r"(?ms)^ {0,3}(?:`{3,}|~{3,}).*?^ {0,3}(?:`{3,}|~{3,})[ \t]*$")

CANONICAL_DOCUMENTS = (
    "docs/README.md",
    "docs/api/README.md",
    "docs/api/trajectory-contracts.md",
    "docs/api/vehicle-composition-advertisement-api.md",
    "docs/api/mission-composition-provider-api.md",
    "docs/api/vehicle-interface-contract.md",
    "docs/api/vehicle-composition-registry.md",
    "docs/api/control-contracts.md",
    "docs/api/public-value-spaces.md",
    "docs/api/telemetry.md",
    "docs/api/route-tracking-contract.md",
    "docs/api/eom-timing-contract.md",
    "docs/api/simulation-runtime-run-manifest.md",
    "docs/api/sensor-plugin-api.md",
    "docs/developer/README.md",
    "docs/developer/interface-layers.md",
    "docs/developer/vehicle-plugin-authoring.md",
    "docs/developer/model-authoring-automation.md",
    "docs/plugins/README.md",
    "docs/verification/README.md",
    "docs/verification/airbreathing-racetrack-fidelity-ladder.md",
)

LEGACY_REDIRECTS = {
    "docs/architecture/trajectory-contracts.md": "docs/api/trajectory-contracts.md",
    "docs/architecture/mission-composition-provider-api.md": "docs/api/mission-composition-provider-api.md",
    "docs/architecture/vehicle-composition-advertisement-api.md": "docs/api/vehicle-composition-advertisement-api.md",
    "docs/architecture/vehicle-interface-contract.md": "docs/api/vehicle-interface-contract.md",
    "docs/architecture/vehicle-composition-registry.md": "docs/api/vehicle-composition-registry.md",
    "docs/architecture/control-contracts.md": "docs/api/control-contracts.md",
    "docs/architecture/public-value-spaces.md": "docs/api/public-value-spaces.md",
    "docs/architecture/eom-timing-contract.md": "docs/api/eom-timing-contract.md",
    "docs/architecture/route-tracking-contract.md": "docs/api/route-tracking-contract.md",
    "docs/architecture/telemetry.md": "docs/api/telemetry.md",
    "docs/architecture/simulation-runtime-run-manifest.md": "docs/api/simulation-runtime-run-manifest.md",
    "docs/architecture/sensor-plugin-api.md": "docs/api/sensor-plugin-api.md",
    "docs/architecture/developer-interface-layers.md": "docs/developer/interface-layers.md",
    "docs/architecture/vehicle-plugin-authoring.md": "docs/developer/vehicle-plugin-authoring.md",
    "docs/architecture/model-authoring-automation.md": "docs/developer/model-authoring-automation.md",
    "docs/architecture/parametric-interceptor-models.md": "packages/taoryx-parametric-interceptors/docs/model-architecture.md",
    "docs/airbreathing-racetrack-fidelity-ladder.md": "docs/verification/airbreathing-racetrack-fidelity-ladder.md",
    "docs/cadac-agm6.md": "packages/taoryx-cadac/docs/cadac-agm6.md",
    "docs/cadac-tables.md": "packages/taoryx-cadac/docs/cadac-tables.md",
    "docs/cadac-ads6-srbm.md": "packages/taoryx-cadac/docs/cadac-ads6-srbm.md",
    "docs/cadac-rocket6g.md": "packages/taoryx-cadac/docs/cadac-rocket6g.md",
    "docs/cadac-ads6-aircraft.md": "packages/taoryx-cadac/docs/cadac-ads6-aircraft.md",
    "docs/cadac-ghame6.md": "packages/taoryx-cadac/docs/cadac-ghame6.md",
    "docs/cadac-ads6-source-controller.md": "packages/taoryx-cadac/docs/cadac-ads6-source-controller.md",
    "docs/cadac-ads6-sam.md": "packages/taoryx-cadac/docs/cadac-ads6-sam.md",
    "docs/cadac-ads6-engagement.md": "packages/taoryx-cadac/docs/cadac-ads6-engagement.md",
    "docs/cadac-sraam6.md": "packages/taoryx-cadac/docs/cadac-sraam6.md",
    "docs/cadac-falcon6.md": "packages/taoryx-cadac/docs/cadac-falcon6.md",
    "docs/cadac-missile-tuning.md": "packages/taoryx-cadac/docs/cadac-missile-tuning.md",
    "docs/cadac-porting.md": "packages/taoryx-cadac/docs/cadac-porting.md",
}


def _local_markdown_destination(raw_target: str) -> str | None:
    """Return a local Markdown link's path portion, or ``None`` for external links."""

    target = raw_target.strip().strip("<>")
    path, _separator, _fragment = target.partition("#")
    if not path or re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", path):
        return None
    return path
    ####


def _validate_local_links(root: Path, source: Path) -> list[str]:
    """Return broken local Markdown targets from one navigational document."""

    errors: list[str] = []
    content = FENCED_CODE_BLOCK.sub("", source.read_text(encoding="utf-8"))
    for match in MARKDOWN_LINK.finditer(content):
        destination = _local_markdown_destination(match.group(1))
        if destination is None:
            continue
        target = (source.parent / destination).resolve()
        if not target.exists():
            errors.append(
                f"{source.relative_to(root)} links to missing {destination!r}"
            )
    return errors
    ####


def _validate_canonical_links(root: Path, source: Path, legacy_paths: set[Path]) -> list[str]:
    """Reject new documentation links that point at a compatibility redirect."""

    errors: list[str] = []
    content = FENCED_CODE_BLOCK.sub("", source.read_text(encoding="utf-8"))
    for match in MARKDOWN_LINK.finditer(content):
        destination = _local_markdown_destination(match.group(1))
        if destination is None:
            continue
        target = (source.parent / destination).resolve()
        if target in legacy_paths:
            errors.append(
                f"{source.relative_to(root)} links to migration pointer {target.relative_to(root)}"
            )
    return errors
    ####


def _navigational_documents(root: Path) -> tuple[Path, ...]:
    """Return every user-facing Markdown page governed by the documentation map."""

    documents = {root / relative for relative in CANONICAL_DOCUMENTS}
    documents.update(root / relative for relative in LEGACY_REDIRECTS)
    documents.update((root / "docs").rglob("*.md"))
    documents.update((root / "packages").rglob("*.md"))
    return tuple(sorted(documents))
    ####


def validate_documentation_layout(root: Path = ROOT) -> list[str]:
    """Check canonical locations, redirect pointers, package coverage, and local links."""

    root = root.resolve()
    errors: list[str] = []
    for relative in CANONICAL_DOCUMENTS:
        if not (root / relative).is_file():
            errors.append(f"missing canonical documentation page: {relative}")

    for legacy_relative, canonical_relative in LEGACY_REDIRECTS.items():
        legacy = root / legacy_relative
        canonical = root / canonical_relative
        if not canonical.is_file():
            errors.append(f"redirect target is missing: {canonical_relative}")
            continue
        if not legacy.is_file():
            errors.append(f"missing compatibility redirect: {legacy_relative}")
            continue
        legacy_text = legacy.read_text(encoding="utf-8")
        if not legacy_text.startswith("# Moved:"):
            errors.append(f"legacy page must be a short moved pointer: {legacy_relative}")

    plugin_index = (root / "docs/plugins/README.md")
    if plugin_index.is_file():
        index_text = plugin_index.read_text(encoding="utf-8")
        for project in sorted((root / "packages").glob("taoryx-*/pyproject.toml")):
            package_root = project.parent
            if not (package_root / "README.md").is_file():
                errors.append(f"plug-in package has no package README: {package_root.relative_to(root)}")
            if f"`{package_root.name}`" not in index_text:
                errors.append(f"plug-in directory omits {package_root.name}")
            expected_link = f"../../packages/{package_root.name}/README.md"
            if expected_link not in index_text:
                errors.append(f"plug-in directory has no canonical README link for {package_root.name}")

    legacy_paths = {(root / relative).resolve() for relative in LEGACY_REDIRECTS}
    for document in _navigational_documents(root):
        if document.is_file():
            errors.extend(_validate_local_links(root, document))
            errors.extend(_validate_canonical_links(root, document, legacy_paths))
    return errors
    ####


def main() -> int:
    """Run the documentation-layout validation as a repository command."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--quiet", action="store_true", help="Suppress the success summary.")
    arguments = parser.parse_args()
    errors = validate_documentation_layout()
    if errors:
        print("documentation layout validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1
    if not arguments.quiet:
        print("documentation layout and navigational links validated")
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
