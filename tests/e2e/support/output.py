from __future__ import annotations

import math
import re
from pathlib import Path

import numpy as np


def parse_column_file(path: Path) -> dict[str, np.ndarray]:
    lines = [line.strip() for line in path.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip()]
    if not lines:
        raise ValueError(f"Output file is empty: {path}")
    ####
    header_index = next((index for index, line in enumerate(lines) if re.search(r"[A-Za-z]", line)), None)
    if header_index is None:
        raise ValueError(f"No column header found in {path}")
    ####
    headers = lines[header_index].split()
    rows: list[list[float]] = []
    for line in lines[header_index + 1:]:
        parts = line.split()
        if len(parts) != len(headers):
            continue
        ####
        try:
            rows.append([float(part.replace("D", "E").replace("d", "e")) for part in parts])
        except ValueError:
            continue
        ####
    ####
    if not rows:
        raise ValueError(f"No numeric rows found in {path}")
    ####
    matrix = np.asarray(rows, dtype=float)
    return {name: matrix[:, index] for index, name in enumerate(headers)}
####


def has_nan(columns: dict[str, np.ndarray]) -> bool:
    return any(any(math.isnan(float(value)) for value in series) for series in columns.values())
####
