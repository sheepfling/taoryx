"""CADAC text-output intake and deterministic channel-parity metrics."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Mapping, Sequence

from pydantic import Field, model_validator

from .input_ast import CadacModel, source_name_for


class CadacPlotParseError(ValueError):
    """Malformed legacy CADAC ``plot*.asc`` output."""


####


class CadacPlotSeries(CadacModel):
    """One tokenized CADAC plot stream with stable channel ordering."""

    source_name: str = Field(min_length=1)
    banner: str = Field(min_length=1)
    channels: tuple[str, ...] = Field(min_length=1)
    rows: tuple[tuple[float, ...], ...]

    @model_validator(mode="after")
    def validate_shape(self) -> "CadacPlotSeries":
        if len(self.channels) != len(set(name.casefold() for name in self.channels)):
            raise ValueError("CADAC plot channels must be unique")
        ####
        width = len(self.channels)
        if any(len(row) != width for row in self.rows):
            raise ValueError("CADAC plot row width must match channel count")
        ####
        return self

    ####

    def column(self, name: str) -> tuple[float, ...]:
        """Return one case-insensitive plot column."""

        key = name.casefold()
        for index, channel in enumerate(self.channels):
            if channel.casefold() == key:
                return tuple(row[index] for row in self.rows)
            ####
        ####
        raise KeyError(name)

    ####


####


class CadacParityChannel(CadacModel):
    """Error metrics for one aligned source/Python output channel."""

    channel: str = Field(min_length=1)
    sample_count: int = Field(ge=0)
    max_absolute_error: float = Field(ge=0.0)
    max_relative_error: float = Field(ge=0.0)
    absolute_tolerance: float = Field(ge=0.0)
    relative_tolerance: float = Field(ge=0.0)
    passed: bool


####


class CadacParityReport(CadacModel):
    """Multi-channel parity report at already-aligned output epochs."""

    schema_id: str = "taoryx.cadac.parity-report/v0alpha1"
    source_name: str = Field(min_length=1)
    channels: tuple[CadacParityChannel, ...] = Field(min_length=1)

    @property
    def passed(self) -> bool:
        return all(channel.passed for channel in self.channels)

    ####


####


def parse_cadac_plot(text: str, *, source_name: str = "<memory>") -> CadacPlotSeries:
    """Parse the whitespace-oriented legacy CADAC plot format."""

    lines = [line.strip() for line in text.splitlines() if line.strip()]
    if len(lines) < 2:
        raise CadacPlotParseError(f"{source_name}: plot output requires banner and count lines")
    ####
    count_tokens = lines[1].split()
    if len(count_tokens) < 3:
        raise CadacPlotParseError(f"{source_name}: malformed plot count line")
    ####
    try:
        channel_count = int(count_tokens[-1])
    except ValueError as error:
        raise CadacPlotParseError(f"{source_name}: plot channel count must be an integer") from error
    ####
    if channel_count <= 0:
        raise CadacPlotParseError(f"{source_name}: plot channel count must be positive")
    ####
    tokens = " ".join(lines[2:]).split()
    if len(tokens) < channel_count:
        raise CadacPlotParseError(f"{source_name}: plot channel banner is incomplete")
    ####
    channels = tuple(tokens[:channel_count])
    data_tokens = tokens[channel_count:]
    if len(data_tokens) % channel_count:
        raise CadacPlotParseError(f"{source_name}: plot data contains {len(data_tokens)} fields for rows of width {channel_count}")
    ####
    values: list[float] = []
    for token in data_tokens:
        try:
            value = float(token)
        except ValueError as error:
            raise CadacPlotParseError(f"{source_name}: non-numeric plot datum {token!r}") from error
        ####
        if not math.isfinite(value):
            raise CadacPlotParseError(f"{source_name}: non-finite plot datum {token!r}")
        ####
        values.append(value)
    ####
    rows = tuple(tuple(values[offset : offset + channel_count]) for offset in range(0, len(values), channel_count))
    return CadacPlotSeries(source_name=source_name, banner=lines[0], channels=channels, rows=rows)


####


def parse_cadac_plot_file(path: str | Path) -> CadacPlotSeries:
    source_path = Path(path)
    return parse_cadac_plot(source_path.read_text(encoding="utf-8"), source_name=source_name_for(source_path))


####


def compare_cadac_plot_channels(
    source: CadacPlotSeries,
    expected: Mapping[str, Sequence[float]],
    *,
    absolute_tolerance: float = 1.0e-9,
    relative_tolerance: float = 1.0e-9,
) -> CadacParityReport:
    """Compare named Python channels against an equally sampled CADAC plot stream."""

    if absolute_tolerance < 0.0 or relative_tolerance < 0.0:
        raise ValueError("parity tolerances must be nonnegative")
    ####
    reports: list[CadacParityChannel] = []
    for channel, expected_values in expected.items():
        actual_values = source.column(channel)
        normalized_expected = tuple(float(value) for value in expected_values)
        if len(actual_values) != len(normalized_expected):
            raise ValueError(f"channel {channel!r} sample count differs: source={len(actual_values)}, expected={len(normalized_expected)}")
        ####
        max_absolute = 0.0
        max_relative = 0.0
        passed = True
        for actual, wanted in zip(actual_values, normalized_expected, strict=True):
            absolute_error = abs(actual - wanted)
            scale = max(abs(actual), abs(wanted))
            relative_error = 0.0 if scale == 0.0 else absolute_error / scale
            max_absolute = max(max_absolute, absolute_error)
            max_relative = max(max_relative, relative_error)
            if absolute_error > absolute_tolerance + relative_tolerance * scale:
                passed = False
            ####
        ####
        reports.append(
            CadacParityChannel(
                channel=channel,
                sample_count=len(actual_values),
                max_absolute_error=max_absolute,
                max_relative_error=max_relative,
                absolute_tolerance=absolute_tolerance,
                relative_tolerance=relative_tolerance,
                passed=passed,
            )
        )
    ####
    if not reports:
        raise ValueError("at least one parity channel must be requested")
    ####
    return CadacParityReport(source_name=source.source_name, channels=tuple(reports))


####


__all__ = [
    "CadacParityChannel",
    "CadacParityReport",
    "CadacPlotParseError",
    "CadacPlotSeries",
    "compare_cadac_plot_channels",
    "parse_cadac_plot",
    "parse_cadac_plot_file",
]
