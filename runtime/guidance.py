"""Resolution of parser *fly rules into guidance records."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class FlyRule:
    source: str
    target: str
    values: tuple[float, ...]
    indirect: bool = False


def resolve_fly_rules(rules: Iterable[FlyRule], *, trajectories: Iterable[str] = ()) -> dict[str, FlyRule]:
    """Resolve direct, indirect, and wildcard fly rules deterministically."""

    known = tuple(trajectories)
    resolved: dict[str, FlyRule] = {}
    for rule in rules:
        if rule.target not in known and rule.target != "*" and known:
            raise KeyError(f"unknown target trajectory: {rule.target}")
        if len(rule.values) not in {1, 2, 3}:
            raise ValueError(f"invalid angle set for {rule.source}")
        targets = known if rule.target == "*" else (rule.target,)
        for target in targets:
            key = f"{rule.source}->{target}"
            if key in resolved:
                raise ValueError(f"duplicate fly rule: {key}")
            resolved[key] = FlyRule(rule.source, target, rule.values, rule.indirect)
    return resolved
####
