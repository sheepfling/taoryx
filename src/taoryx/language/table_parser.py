from __future__ import annotations

from pathlib import Path

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.models import TableAssignment, TableDefinition, TableDocument, TableOperation


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
    executable_complete = bool(tables) and not any(table.omissions for table in tables)
    result = TableParseResult(path=path, tables=tables, issues=issues, executable_complete=executable_complete)
    document = TableDocument(executable_complete=result.executable_complete)
    for issue in result.issues:
        document.diagnostics.append(Diagnostic(severity=Severity(issue.severity), code=issue.code, message=issue.message, location=SourceLocation(path=path, line=issue.line or 1)))
    ####
    for table in result.tables:
        location_line = table.assignments[0].line if table.assignments else table.operations[0].line if table.operations else 1
        definition = TableDefinition(name=table.name, table_type=table.table_type, format=table.format, location=SourceLocation(path=path, line=location_line), independent_variables=table.independent_variables, options=table.options, omissions=table.omissions)
        definition.assignments = [TableAssignment(name=item.name, values=item.values, location=SourceLocation(path=path, line=item.line)) for item in table.assignments]
        definition.operations = [TableOperation(operator=item.operator, operand=item.operand.name if hasattr(item.operand, "name") else item.operand, label=item.label, condition=item.condition, extrapolation=item.extrapolation, location=SourceLocation(path=path, line=item.line), assignments=[TableAssignment(name=value.name, values=value.values, location=SourceLocation(path=path, line=value.line)) for value in item.assignments]) for item in table.operations]
        document.tables.append(definition)
    ####
    return document
####


def parse_table_file(path: str | Path) -> TableDocument:
    source = Path(path)
    return parse_table_text(source.read_text(encoding="utf-8"), str(source))
####
