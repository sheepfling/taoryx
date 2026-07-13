from __future__ import annotations

import re
from pathlib import Path

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.models import RecoveredRecord, TableAssignment, TableCall, TableDefinition, TableDocument, TableOperation


def _source_line(source_lines: list[str], line: int) -> str | None:
    return source_lines[line - 1] if 0 < line <= len(source_lines) else None
####


def _convert_operation(item, path: str, source_lines: list[str]) -> TableOperation:
    operand = item.operand
    if hasattr(operand, "name") and hasattr(operand, "arguments"):
        operand = TableCall(name=operand.name, arguments=list(operand.arguments))
    ####
    return TableOperation(
        operator=item.operator,
        operand=operand,
        label=item.label,
        condition=item.condition,
        extrapolation=item.extrapolation,
        location=SourceLocation(path=path, line=item.line),
        source_text=_source_line(source_lines, item.line),
        assignments=[
            TableAssignment(
                name=value.name,
                values=value.values,
                location=SourceLocation(path=path, line=value.line),
                source_text=_source_line(source_lines, value.line),
            )
            for value in item.assignments
        ],
        nested=_convert_operation(item.nested, path, source_lines) if item.nested is not None else None,
    )
####


def parse_table_text(text: str, path: str = "<memory>") -> TableDocument:
    # Reuse the mature fixture parser while exposing stable canonical models.
    from taoryx.language._legacy_table import (
        TableParseResult,
        _semantic_validate_table,
        _TableTokenParser,
        _tokenize_table,
    )

    parser = _TableTokenParser(_tokenize_table(text), Path(path))
    tables = parser.parse_all()
    issues = list(parser.issues)
    for table in tables:
        _semantic_validate_table(table, issues)
    ####
    issues.sort(
        key=lambda issue: (
            issue.line if issue.line is not None else len(text.splitlines()) + 1,
            issue.column if issue.column is not None else 1,
        )
    )
    executable_complete = bool(tables) and not any(table.omissions for table in tables) and not any(issue.severity == "error" for issue in issues)
    result = TableParseResult(path=path, tables=tables, issues=issues, executable_complete=executable_complete)
    document = TableDocument(executable_complete=result.executable_complete)
    source_lines = text.splitlines()
    for issue in result.issues:
        line = issue.line or 1
        document.diagnostics.append(Diagnostic(severity=Severity(issue.severity), code=issue.code, message=issue.message, location=SourceLocation(path=path, line=line, column=issue.column or 1)))
        recovered_text = source_lines[line - 1] if 0 < line <= len(source_lines) else ""
        document.recovered_records.append(RecoveredRecord(text=recovered_text, code=issue.code, location=SourceLocation(path=path, line=line, column=issue.column or 1)))
    ####
    for table in result.tables:
        for omission_line in table.omissions:
            document.recovered_records.append(
                RecoveredRecord(
                    text=_source_line(source_lines, omission_line) or "",
                    code="documentation-excerpt",
                    location=SourceLocation(path=path, line=omission_line),
                )
            )
        ####
        location_line = table.header_line
        definition = TableDefinition(
            name=table.name,
            table_type=table.table_type,
            format=table.format,
            location=SourceLocation(path=path, line=location_line),
            source_text=_source_line(source_lines, location_line),
            independent_variables=table.independent_variables,
            options=table.options,
            omissions=table.omissions,
        )
        definition.assignments = [
            TableAssignment(
                name=item.name,
                values=item.values,
                location=SourceLocation(path=path, line=item.line),
                source_text=_source_line(source_lines, item.line),
            )
            for item in table.assignments
        ]
        definition.operations = [_convert_operation(item, path, source_lines) for item in table.operations]
        document.tables.append(definition)
    ####
    return document
####


def parse_table_file(path: str | Path) -> TableDocument:
    source = Path(path)
    text = source.read_bytes().decode("utf-8", errors="surrogateescape")
    return parse_table_text(text, str(source))
####


def table_type_catalog(document: TableDocument) -> dict[str, str]:
    """Return unambiguous, case-folded table names and their declared types.

    Duplicate names are deliberately omitted. A caller must not infer a table
    type from an ambiguous table file; normal ingestion diagnostics still report
    the duplicate declarations separately.
    """
    grouped: dict[str, list[TableDefinition]] = {}
    for table in document.tables:
        grouped.setdefault(table.name.casefold(), []).append(table)
    ####
    return {name: tables[0].table_type for name, tables in grouped.items() if len(tables) == 1}
####


def table_variable_catalog(document: TableDocument) -> dict[str, set[str]]:
    """Return variables explicitly used by each unambiguous table definition."""
    grouped: dict[str, list[TableDefinition]] = {}
    for table in document.tables:
        grouped.setdefault(table.name.casefold(), []).append(table)
    ####
    catalog: dict[str, set[str]] = {}
    for name, tables in grouped.items():
        if len(tables) != 1:
            continue
        ####
        table = tables[0]
        variables = {variable.casefold() for variable in table.independent_variables}
        variables.update(assignment.name.casefold() for assignment in table.assignments)

        def collect_operation(operation: TableOperation) -> None:
            operand = operation.operand
            if isinstance(operand, TableCall):
                variables.update(argument.casefold() for argument in operand.arguments)
            elif isinstance(operand, str) and operation.operator not in {"csto", "goto"}:
                variables.add(operand.casefold())
            ####
            if operation.condition:
                variables.update(re.findall(r"[A-Za-z_][A-Za-z0-9_.-]*", operation.condition.casefold()))
            ####
            if operation.nested is not None:
                collect_operation(operation.nested)
            ####

        for operation in table.operations:
            collect_operation(operation)
        catalog[name] = variables
    ####
    return catalog
####
