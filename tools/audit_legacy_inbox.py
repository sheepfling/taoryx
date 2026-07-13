"""Inventory and classify the quarantined legacy TAOS toolkit."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INBOX = ROOT / "INBOX" / "taos_legacy_preservation_intial_work_maybe_faulty"
DEFAULT_OUTPUT = ROOT / "metadata" / "legacy_inbox_manifest.json"
GRAMMAR_REGISTRY = ROOT / "metadata" / "legacy_grammar_registry.json"


@dataclass(frozen=True)
class InventoryEntry:
    path: str
    size: int
    sha256: str
    disposition: str
    rationale: str


def disposition_for(path: Path) -> tuple[str, str]:
    parts = path.parts
    name = path.name
    if name == ".DS_Store" or "__pycache__" in parts or ".pytest_cache" in parts:
        return "discard-generated", "Local filesystem or test cache artifact."
    if "dist" in parts or name in {"SHA256SUMS.txt"}:
        return "discard-generated", "Release output or transient verification artifact."
    if "manual_evidence" in parts:
        return "archive-research", "Evidence ledger or page note retained through the hashed inbox archive."
    if "grammar" in parts and path.suffix == ".ebnf":
        return "archive-research", "Candidate grammar fragment retained as research; canonical claims live in taoryx contracts."
    if "schemas" in parts:
        return "archive-research", "Candidate structural schema retained as research; not a canonical taoryx model."
    if "examples" in parts:
        return "archive-research", "Candidate fixture retained as research; not a verified TAOS fixture."
    if "tests" in parts:
        return "archive-research", "Legacy regression behavior retained as research; canonical tests are ported selectively."
    if "src" in parts and path.suffix == ".py":
        if name in {"lexical.py", "models.py", "parser.py", "writer.py", "profiles.py", "evidence.py"}:
            return "archive-research", "Useful preservation or evidence concept has been selectively ported."
        return "archive-research", "Legacy analyzer or model retained as research; not canonical runtime code."
    if path.suffix in {".md", ".toml", ".json"}:
        return "archive-research", "Project metadata or notes retained as research provenance."
    return "archive-research", "Unclassified legacy artifact retained in the hashed research archive."


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def inventory(inbox: Path) -> list[InventoryEntry]:
    if not inbox.is_dir():
        raise FileNotFoundError(f"legacy inbox does not exist: {inbox}")
    entries: list[InventoryEntry] = []
    for path in sorted(p for p in inbox.rglob("*") if p.is_file()):
        relative = path.relative_to(inbox)
        disposition, rationale = disposition_for(relative)
        entries.append(
            InventoryEntry(
                path=relative.as_posix(),
                size=path.stat().st_size,
                sha256=sha256(path),
                disposition=disposition,
                rationale=rationale,
            )
        )
    return entries


def verify_fixture_round_trips(inbox: Path) -> int:
    legacy_src = inbox / "toolkit" / "src"
    if not legacy_src.is_dir():
        raise FileNotFoundError(f"legacy package source does not exist: {legacy_src}")
    sys.path.insert(0, str(legacy_src))
    Dialect = importlib.import_module("taos_legacy.models").Dialect
    parse_text = importlib.import_module("taos_legacy.parser").parse_text
    render_lossless = importlib.import_module("taos_legacy.writer").render_lossless
    ingest_file = importlib.import_module("taoryx.language").ingest_file

    fixture_paths = sorted(
        path
        for directory in (ROOT / "examples" / "chapter03", ROOT / "examples" / "chapter04")
        for path in directory.iterdir()
        if path.suffix in {".tbl", ".prb"}
    )
    for path in fixture_paths:
        data = path.read_bytes()
        text = data.decode("utf-8")
        newline = "crlf" if "\r\n" in text else "lf" if "\n" in text else "none"
        legacy_document = parse_text(
            text.replace("\r\n", "\n"),
            source_path=str(path),
            dialect=Dialect.TAOS_96,
            newline=newline,
            final_newline=data.endswith((b"\n", b"\r")),
        )
        if render_lossless(legacy_document).encode("utf-8") != data:
            raise ValueError(f"legacy round-trip changed fixture: {path}")
        canonical_document = ingest_file(path)
        if not canonical_document.valid:
            raise ValueError(f"canonical ingestion rejected baseline fixture: {path}")
    print(f"Legacy differential fixture check passed: {len(fixture_paths)} fixtures.")
    ####
    return len(fixture_paths)
####


def verify_grammar_registry(inbox: Path) -> None:
    registry = json.loads(GRAMMAR_REGISTRY.read_text(encoding="utf-8"))
    entries = registry["entries"]
    registered = {entry["file"] for entry in entries}
    grammar_dir = inbox / "toolkit" / "grammar"
    actual = {path.name for path in grammar_dir.glob("*.ebnf")}
    missing = sorted(actual - registered)
    extra = sorted(registered - actual)
    if missing or extra:
        raise ValueError(f"grammar registry mismatch: missing={missing}, extra={extra}")
    forbidden = [entry["file"] for entry in entries if entry["status"] == "verified"]
    if forbidden:
        raise ValueError(f"legacy grammar registry cannot promote verified entries: {forbidden}")
    print(f"Legacy grammar registry reconciles {len(entries)} candidate files.")
####


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inbox", type=Path, default=DEFAULT_INBOX)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--verify-fixtures", action="store_true")
    args = parser.parse_args()
    entries = inventory(args.inbox)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(
            {
                "format": "taoryx-legacy-inbox-manifest",
                "schema_version": "1",
                "source": str(args.inbox.relative_to(ROOT)),
                "entries": [asdict(entry) for entry in entries],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    counts: dict[str, int] = {}
    for entry in entries:
        counts[entry.disposition] = counts.get(entry.disposition, 0) + 1
    print(f"Legacy inbox inventory written: {args.output.relative_to(ROOT)}")
    print(f"  {len(entries)} files hashed")
    for disposition, count in sorted(counts.items()):
        print(f"  {disposition}: {count}")
    if args.verify_fixtures:
        verify_grammar_registry(args.inbox)
        verify_fixture_round_trips(args.inbox)
    ####
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
