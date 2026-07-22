from __future__ import annotations

from pathlib import Path

import yaml

from tools.generate_problem_files import CATALOG, check_catalog, render_catalog


def test_problem_catalog_is_current() -> None:
    """All tracked generated problem files are reproducible from metadata."""

    generated = check_catalog(CATALOG)
    catalog = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    assert len(generated) == len(catalog["scenarios"])
    assert all(path.exists() for path in generated)
    assert all(path.suffix == ".prb" for path in generated)
####


def test_problem_generation_is_deterministic(tmp_path: Path) -> None:
    """Rendering the catalog does not depend on process state or timestamps."""

    first = tuple(path.read_text(encoding="utf-8") for path in check_catalog(CATALOG))
    rendered_paths = render_catalog(CATALOG)
    second = tuple(path.read_text(encoding="utf-8") for path in rendered_paths)
    assert first == second
    assert all(path.is_file() for path in rendered_paths)
####
