"""Compose ordinary TAOS/TAORYX problem text from external fragments."""

from __future__ import annotations

import argparse
from pathlib import Path


def compose(template: str, fragments: dict[str, str], values: dict[str, str]) -> str:
    """Expand external fragment and scalar placeholders in a problem template."""

    result = template
    for name, fragment in fragments.items():
        result = result.replace(f"{{{{FRAGMENT:{name}}}}}", fragment.rstrip())
    for name, value in values.items():
        result = result.replace(f"{{{{{name}}}}}", value)
    unresolved = [token for token in ("{{FRAGMENT:", "{{") if token in result]
    if unresolved:
        raise ValueError(f"unresolved composition placeholder: {unresolved[0]}")
    return result


def main() -> int:
    """Compose one problem template from ``name=path`` fragments."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("template", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--fragment", action="append", default=[], metavar="NAME=PATH")
    parser.add_argument("--set", action="append", default=[], metavar="NAME=VALUE")
    args = parser.parse_args()

    def pairs(items: list[str]) -> dict[str, str]:
        result: dict[str, str] = {}
        for item in items:
            name, separator, value = item.partition("=")
            if not separator or not name:
                raise ValueError(f"expected NAME=VALUE, got {item!r}")
            result[name] = value
        return result
    ####

    fragment_paths = pairs(args.fragment)
    fragments = {name: Path(path).read_text(encoding="utf-8") for name, path in fragment_paths.items()}
    output = compose(args.template.read_text(encoding="utf-8"), fragments, pairs(args.set))
    args.output.write_text(output, encoding="utf-8")
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
