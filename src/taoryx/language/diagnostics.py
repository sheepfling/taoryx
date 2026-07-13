from __future__ import annotations

from enum import StrEnum

from pydantic import BaseModel


class Severity(StrEnum):
    ERROR = "error"
    WARNING = "warning"
    NOTE = "note"
####


class SourceLocation(BaseModel):
    path: str
    line: int
    column: int = 1
####


class Diagnostic(BaseModel):
    severity: Severity
    code: str
    message: str
    location: SourceLocation | None = None
####
