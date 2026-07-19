"""Deterministic binding of source table files into the runtime table namespace."""

from __future__ import annotations

import re
from collections import defaultdict
from dataclasses import replace
from pathlib import Path
from typing import Mapping

from taoryx.language.models import TableDefinition, TableDocument

from .lowering import RuntimeTable, lower_tables

_FAMILY_MARKERS = ("static", "collective", "differential", "rate", "control", "combined")


def bind_runtime_tables(
    documents: list[TableDocument] | tuple[TableDocument, ...],
    unit_settings: Mapping[str, str | None],
) -> tuple[dict[str, str], dict[str, RuntimeTable]]:
    """Return the available-table catalog and lowered tables for source documents.

    TAOS table names are local to a source file, while a vehicle family may
    legitimately publish several ``cx``/``cy``/``cm`` families.  Keep the
    unqualified name when it is unique.  When it is not, expose deterministic
    source-qualified aliases such as ``cx-static`` and ``cx-collective``.
    Problem files continue to use ordinary table references and expressions;
    qualification is a runtime binding concern rather than a grammar feature.
    """

    definitions: list[tuple[TableDocument, TableDefinition]] = [
        (document, definition)
        for document in documents
        for definition in document.tables
    ]
    grouped: dict[str, list[tuple[TableDocument, TableDefinition]]] = defaultdict(list)
    for document, definition in definitions:
        grouped[definition.name.casefold()].append((document, definition))

    lowered_documents = {id(document): lower_tables(document, unit_settings) for document in documents}
    available: dict[str, str] = {}
    lowered: dict[str, RuntimeTable] = {}
    used: set[str] = set()
    for document, definition in definitions:
        raw_name = definition.name.casefold()
        if len(grouped[raw_name]) == 1:
            alias = raw_name
        else:
            alias = _qualified_name(raw_name, definition.location.path)
            if alias in used:
                alias = _qualified_name(raw_name, definition.location.path, include_stem=True)
            if alias in used:
                raise ValueError(f"cannot create a unique runtime alias for table {definition.name!r}")
        used.add(alias)
        available[alias] = definition.table_type

        lowered_document = lowered_documents[id(document)]
        runtime_table = lowered_document.get(raw_name)
        if runtime_table is None:
            raise ValueError(f"table {definition.name!r} could not be lowered")
        lowered[alias] = replace(runtime_table, name=alias)
    return available, lowered
####


def _qualified_name(name: str, source: str, *, include_stem: bool = False) -> str:
    stem = Path(source).stem.casefold()
    marker = next((item for item in _FAMILY_MARKERS if item in stem), None)
    suffix = marker or stem
    if include_stem and marker is not None:
        suffix = f"{marker}-{stem}"
    suffix = re.sub(r"[^a-z0-9]+", "-", suffix).strip("-") or "source"
    return f"{name}-{suffix}"
####
