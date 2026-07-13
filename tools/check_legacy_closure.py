"""Fail until the legacy inbox migration has no provisional decisions left."""

from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "metadata" / "legacy_inbox_manifest.json"
GRAMMAR_REGISTRY = ROOT / "metadata" / "legacy_grammar_registry.json"

FINAL_DISPOSITIONS = {
    "adapted-canonical",
    "archive-research",
    "discard-generated",
    "preserve-canonical",
}
FINAL_GRAMMAR_STATUSES = {"accepted", "archived", "rejected"}


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    entries = manifest["entries"]
    disposition_counts = Counter(entry["disposition"] for entry in entries)
    unresolved_files = [entry for entry in entries if entry["disposition"] not in FINAL_DISPOSITIONS]

    grammar = json.loads(GRAMMAR_REGISTRY.read_text(encoding="utf-8"))
    unresolved_grammar = [entry for entry in grammar["entries"] if entry["status"] not in FINAL_GRAMMAR_STATUSES]

    if unresolved_files or unresolved_grammar:
        print("Legacy inbox migration is not closed.")
        print(f"  unresolved files: {len(unresolved_files)} of {len(entries)}")
        for disposition, count in sorted(disposition_counts.items()):
            if disposition not in FINAL_DISPOSITIONS:
                print(f"    {disposition}: {count}")
        print(f"  unresolved grammar entries: {len(unresolved_grammar)} of {len(grammar['entries'])}")
        print("  final file dispositions: " + ", ".join(sorted(FINAL_DISPOSITIONS)))
        print("  final grammar statuses: " + ", ".join(sorted(FINAL_GRAMMAR_STATUSES)))
        return 1

    print(f"Legacy inbox migration closed: {len(entries)} files and {len(grammar['entries'])} grammar entries resolved.")
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
