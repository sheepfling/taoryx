from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATACLASS_ROOTS = (ROOT / "src", ROOT / "scripts", ROOT / "tools")


def _is_dataclass_decorator(node: ast.expr) -> bool:
    if isinstance(node, ast.Name):
        return node.id == "dataclass"
    return isinstance(node, ast.Attribute) and node.attr == "dataclass"


def _has_slots_keyword(node: ast.Call) -> bool:
    return any(keyword.arg == "slots" and isinstance(keyword.value, ast.Constant) and keyword.value.value is True for keyword in node.keywords)


def test_all_repository_dataclasses_are_slotted() -> None:
    unslotted: list[str] = []
    for root in DATACLASS_ROOTS:
        for path in root.rglob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if not isinstance(node, ast.ClassDef):
                    continue
                decorators = [decorator for decorator in node.decorator_list if _is_dataclass_decorator(decorator)]
                for decorator in decorators:
                    if isinstance(decorator, ast.Name) or not _has_slots_keyword(decorator):
                        unslotted.append(f"{path.relative_to(ROOT)}:{node.lineno}:{node.name}")

    assert unslotted == [], "unslotted dataclasses: " + ", ".join(unslotted)
