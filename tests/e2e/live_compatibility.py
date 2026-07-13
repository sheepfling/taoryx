"""Explicit compatibility boundaries for the imported v23 corpus.

The registry is intentionally empty after the manual-backed v23 grammar fixes.
Future exceptions must be recorded here with a source-preserving rationale
instead of being silently ignored by the positive-case tests.
"""

from __future__ import annotations

LIVE_POSITIVE_DIAGNOSTICS: dict[str, frozenset[str]] = {
}

MISMATCH_RATIONALE: dict[str, str] = {}
####
