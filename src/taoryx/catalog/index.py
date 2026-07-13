"""Load and index the generated TAOS algorithm catalog."""

from __future__ import annotations

import csv
import json
from collections import defaultdict, deque
from pathlib import Path
from typing import Any

from .models import AlgorithmRecord


class AlgorithmCatalog:
    """Read-only catalog index with dependency and equation lookups."""

    def __init__(self, payload: dict[str, Any], equation_map: dict[str, tuple[str, ...]] | None = None) -> None:
        self.catalog_version = str(payload.get("catalog_version", ""))
        self._records = tuple(AlgorithmRecord.model_validate(item) for item in payload.get("algorithms", []))
        self._by_id = {record.id: record for record in self._records}
        if len(self._by_id) != len(self._records):
            raise ValueError("algorithm IDs are not unique")
        missing = sorted({dependency for record in self._records for dependency in record.dependencies} - self._by_id.keys())
        if missing:
            raise ValueError(f"catalog has missing dependencies: {missing}")
        self._equation_map = equation_map or {}
        self._validate_acyclic()
    ####

    def all(self) -> tuple[AlgorithmRecord, ...]:
        """Return records in catalog order."""

        return self._records
    ####

    def get(self, algorithm_id: str) -> AlgorithmRecord:
        """Return an algorithm by its stable catalog identifier."""

        try:
            return self._by_id[algorithm_id]
        except KeyError as error:
            raise KeyError(f"unknown algorithm: {algorithm_id}") from error
        ####
    ####

    def by_phase(self, phase: str) -> tuple[AlgorithmRecord, ...]:
        """Return algorithms assigned to one implementation phase."""

        return tuple(record for record in self._records if record.phase == phase)
    ####

    def by_domain(self, domain: str) -> tuple[AlgorithmRecord, ...]:
        """Return algorithms assigned to one domain."""

        return tuple(record for record in self._records if record.domain == domain)
    ####

    def for_equation(self, equation: str) -> tuple[AlgorithmRecord, ...]:
        """Return algorithms mapped to a canonical manual equation."""

        return tuple(self._by_id[algorithm_id] for algorithm_id in self._equation_map.get(equation, ()))
    ####

    def dependency_order(self) -> tuple[AlgorithmRecord, ...]:
        """Return records in a dependency-first topological order."""

        indegree = {record.id: len(record.dependencies) for record in self._records}
        successors: dict[str, list[str]] = defaultdict(list)
        for record in self._records:
            for dependency in record.dependencies:
                successors[dependency].append(record.id)
        queue = deque(sorted(identifier for identifier, degree in indegree.items() if degree == 0))
        ordered: list[AlgorithmRecord] = []
        while queue:
            identifier = queue.popleft()
            ordered.append(self._by_id[identifier])
            for successor in sorted(successors[identifier]):
                indegree[successor] -= 1
                if indegree[successor] == 0:
                    queue.append(successor)
                ####
            ####
        ####
        if len(ordered) != len(self._records):
            raise ValueError("catalog dependency graph contains a cycle")
        return tuple(ordered)
    ####

    def _validate_acyclic(self) -> None:
        self.dependency_order()
    ####
####


def load_catalog(catalog_path: Path, equation_map_path: Path | None = None) -> AlgorithmCatalog:
    """Load a generated catalog and optionally its equation mapping CSV."""

    payload = json.loads(catalog_path.read_text(encoding="utf-8"))
    equation_map: dict[str, tuple[str, ...]] = {}
    if equation_map_path is not None:
        with equation_map_path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                ids = json.loads(row["algorithm_ids"])
                equation_map[row["equation"]] = tuple(ids)
            ####
        ####
    ####
    return AlgorithmCatalog(payload, equation_map)
####
