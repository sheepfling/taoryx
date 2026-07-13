from __future__ import annotations

import re
from pathlib import Path

from taoryx.language.diagnostics import Diagnostic, Severity, SourceLocation
from taoryx.language.models import (
    RecoveredRecord,
    TableAssignment,
    TableAssignmentFragment,
    TableCall,
    TableDefinition,
    TableDocument,
    TableHeaderFragment,
    TableOperation,
    TableOperationFragment,
    TableSimpleBodyFragment,
    TableSkewedFragment,
)


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


def _fragment_diagnostics(parser, text: str, path: str) -> tuple[list[Diagnostic], list[RecoveredRecord]]:
    source_lines = text.splitlines()
    diagnostics: list[Diagnostic] = []
    recovered_records: list[RecoveredRecord] = []
    for issue in sorted(parser.issues, key=lambda item: (item.line or len(source_lines) + 1, item.column or 1)):
        line = issue.line or 1
        location = SourceLocation(path=path, line=line, column=issue.column or 1)
        diagnostics.append(Diagnostic(severity=Severity(issue.severity), code=issue.code, message=issue.message, location=location))
        recovered_records.append(RecoveredRecord(text=_source_line(source_lines, line) or "", code=issue.code, location=location))
    ####
    return diagnostics, recovered_records
####


def _parse_table_operation_fragment(text: str, path: str, *, allow_body_markers: bool) -> TableOperationFragment:
    """Parse a sequence of full-table operations without inventing a table.

    This route is for manual displays whose manifest identifies them as an
    ``operation_fragment``.  It retains syntax diagnostics and source lines,
    but deliberately does not run table-level semantic validation such as
    interpolation-data completeness.
    """

    from taoryx.language._legacy_table import ParseIssue, _TableTokenParser, _tokenize_table

    parser = _TableTokenParser(_tokenize_table(text), Path(path))
    source_lines = text.splitlines()
    operations: list[TableOperation] = []
    while parser.current() is not None:
        parser.skip_newlines()
        token = parser.current()
        if token is None:
            break
        ####
        if allow_body_markers and token.value.lower() == "start":
            parser.advance()
            continue
        ####
        if token.value.lower() == "end":
            parser.advance()
            operations.append(
                TableOperation(
                    operator="end",
                    location=SourceLocation(path=path, line=token.line, column=token.column),
                    source_text=_source_line(source_lines, token.line),
                )
            )
            continue
        ####
        operation = parser.parse_operation()
        if operation is not None:
            operations.append(_convert_operation(operation, path, source_lines))
            continue
        ####
        parser.advance()
        parser.issues.append(
            ParseIssue(
                severity="error",
                code="unparsed-operation-fragment-token",
                message=f"Unparsed operation-fragment token {token.value!r}.",
                line=token.line,
                column=token.column,
            )
        )
    ####
    diagnostics, recovered_records = _fragment_diagnostics(parser, text, path)
    return TableOperationFragment(source_text=text, operations=operations, diagnostics=diagnostics, recovered_records=recovered_records)
####


def parse_table_operation_fragment(text: str, path: str = "<memory>") -> TableOperationFragment:
    """Parse a sequence of full-table operations without inventing a table."""

    return _parse_table_operation_fragment(text, path, allow_body_markers=False)
####


def parse_table_body_fragment(text: str, path: str = "<memory>") -> TableOperationFragment:
    """Parse a documented ``start``/operation/``end`` full-table body excerpt."""

    return _parse_table_operation_fragment(text, path, allow_body_markers=True)
####


def parse_table_assignment_fragment(text: str, path: str = "<memory>") -> TableAssignmentFragment:
    """Parse table numeric assignments without requiring a surrounding table."""

    from taoryx.language._legacy_table import ParseIssue, _TableTokenParser, _tokenize_table

    parser = _TableTokenParser(_tokenize_table(text), Path(path))
    source_lines = text.splitlines()
    assignments: list[TableAssignment] = []
    while parser.current() is not None:
        parser.skip_newlines()
        token = parser.current()
        if token is None:
            break
        ####
        assignment = parser.parse_numeric_assignment()
        if assignment is not None:
            assignments.append(
                TableAssignment(
                    name=assignment.name,
                    values=assignment.values,
                    location=SourceLocation(path=path, line=assignment.line),
                    source_text=_source_line(source_lines, assignment.line),
                )
            )
            continue
        ####
        parser.advance()
        parser.issues.append(
            ParseIssue(
                severity="error",
                code="unparsed-assignment-fragment-token",
                message=f"Unparsed table-assignment token {token.value!r}.",
                line=token.line,
                column=token.column,
            )
        )
    ####
    diagnostics, recovered_records = _fragment_diagnostics(parser, text, path)
    return TableAssignmentFragment(source_text=text, assignments=assignments, diagnostics=diagnostics, recovered_records=recovered_records)
####


def parse_table_header_fragment(text: str, path: str = "<memory>") -> TableHeaderFragment:
    """Parse a ``table type(axis,...) options`` header excerpt."""

    from taoryx.language._legacy_table import (
        IDENTIFIER_RE,
        OPTION_ATOM_RE,
        TABLE_TYPES,
        ParseIssue,
        _TableTokenParser,
        _tokenize_table,
        _validate_table_options,
    )

    parser = _TableTokenParser(_tokenize_table(text), Path(path))
    parser.skip_newlines()
    table_type: str | None = None
    independent_variables: list[str] = []
    options: dict[str, str | float] = {}
    table_token = parser.accept("table")
    if table_token is None:
        current = parser.current()
        parser.issues.append(
            ParseIssue(
                severity="error",
                code="missing-table-header-keyword",
                message="Expected 'table' at the start of a table-header fragment.",
                line=current.line if current else 1,
                column=current.column if current else 1,
            )
        )
    ####
    type_token = parser.advance()
    if type_token is not None and type_token.value != "\n":
        table_type = type_token.value.lower()
        if table_type not in TABLE_TYPES:
            parser.issues.append(
                ParseIssue(
                    severity="error",
                    code="unknown-table-type",
                    message=f"Unknown table type {table_type!r}.",
                    line=type_token.line,
                    column=type_token.column,
                )
            )
    else:
        parser.issues.append(
            ParseIssue(
                severity="error",
                code="missing-table-type",
                message="A table-header fragment requires a table type.",
                line=type_token.line if type_token else 1,
                column=type_token.column if type_token else 1,
            )
        )
    ####
    if parser.accept("(") is None:
        current = parser.current()
        parser.issues.append(
            ParseIssue(
                severity="error",
                code="missing-independent-variable-list",
                message="A table-header fragment requires a parenthesized independent-variable list.",
                line=current.line if current else type_token.line if type_token else 1,
                column=current.column if current else type_token.column if type_token else 1,
            )
        )
    else:
        while parser.current() is not None and parser.current().value not in {")", "\n"}:
            variable = parser.advance()
            if variable is None:
                break
            ####
            independent_variables.append(variable.value.lower())
            if IDENTIFIER_RE.fullmatch(variable.value) is None:
                parser.issues.append(
                    ParseIssue(
                        severity="error",
                        code="invalid-independent-variable",
                        message=f"Independent variable {variable.value!r} must be an identifier.",
                        line=variable.line,
                        column=variable.column,
                    )
                )
        ####
        if parser.accept(")") is None:
            current = parser.current()
            parser.issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-independent-variable-list-end",
                    message="Table-header fragment is missing ')' after its independent variables.",
                    line=current.line if current else table_token.line if table_token else 1,
                    column=current.column if current else table_token.column if table_token else 1,
                )
            )
    ####
    while parser.current() is not None and parser.current().value != "\n":
        token = parser.current()
        if token is None:
            break
        ####
        if token.value.lower() in {"extrap", "no-extrap"}:
            option = parser.advance()
            if option is not None:
                previous = options.get("extrapolation")
                if previous is not None and previous != option.value.lower():
                    parser.issues.append(
                        ParseIssue(
                            severity="error",
                            code="conflicting-table-extrapolation",
                            message="A table header may specify either 'extrap' or 'no-extrap', not both.",
                            line=option.line,
                            column=option.column,
                        )
                    )
                options["extrapolation"] = option.value.lower()
            ####
            continue
        ####
        key = parser.advance()
        if key is None:
            break
        ####
        if parser.accept("=") is None:
            parser.issues.append(
                ParseIssue(
                    severity="error",
                    code="invalid-table-header-option",
                    message=f"Table header option {key.value!r} requires '=' and a value.",
                    line=key.line,
                    column=key.column,
                )
            )
            continue
        ####
        value = parser.advance()
        if value is None or value.value == "\n":
            parser.issues.append(
                ParseIssue(
                    severity="error",
                    code="missing-table-header-option-value",
                    message=f"Table header option {key.value!r} is missing its value.",
                    line=key.line,
                    column=key.column,
                )
            )
            continue
        ####
        if re.fullmatch(r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eEdD][+-]?\d+)?", value.value):
            options[key.value.lower()] = float(value.value.replace("d", "e").replace("D", "E"))
        elif OPTION_ATOM_RE.fullmatch(value.value) is not None:
            options[key.value.lower()] = value.value
        else:
            parser.issues.append(
                ParseIssue(
                    severity="error",
                    code="invalid-table-header-option-value",
                    message=f"Table header option {key.value!r} has invalid value {value.value!r}.",
                    line=value.line,
                    column=value.column,
                )
            )
    ####
    if table_type is not None:
        _validate_table_options(table_type, options, table_token.line if table_token else 1, parser.issues)
    ####
    diagnostics, recovered_records = _fragment_diagnostics(parser, text, path)
    return TableHeaderFragment(
        source_text=text,
        table_type=table_type,
        independent_variables=independent_variables,
        options=options,
        diagnostics=diagnostics,
        recovered_records=recovered_records,
    )
####


def parse_simple_table_body_fragment(text: str, path: str = "<memory>") -> TableSimpleBodyFragment:
    """Parse a ``table ...`` header followed by simple assignments.

    This fragment route intentionally does not require dependent values or
    enforce independent-axis ordering; those checks need a complete table
    context and belong to :func:`parse_table_text`.
    """

    lines = text.splitlines(keepends=True)
    first_content = next((index for index, line in enumerate(lines) if line.strip()), None)
    if first_content is None:
        header_text = text
        body_text = ""
    else:
        header_text = lines[first_content]
        body_text = "".join(lines[first_content + 1:])
    ####
    header = parse_table_header_fragment(header_text, path)
    padded_body = "\n" * (first_content + 1 if first_content is not None else 0) + body_text
    assignment_fragment = parse_table_assignment_fragment(padded_body, path) if body_text else TableAssignmentFragment(source_text=padded_body)
    diagnostics = [*header.diagnostics, *assignment_fragment.diagnostics]
    recovered_records = [*header.recovered_records, *assignment_fragment.recovered_records]
    diagnostics.sort(key=lambda item: (item.location.line if item.location else 0, item.location.column if item.location else 0))
    recovered_records.sort(key=lambda item: (item.location.line, item.location.column))
    return TableSimpleBodyFragment(
        source_text=text,
        header=header,
        assignments=assignment_fragment.assignments,
        diagnostics=diagnostics,
        recovered_records=recovered_records,
    )
####


def parse_skewed_assignment_groups_fragment(text: str, path: str = "<memory>") -> TableSkewedFragment:
    """Parse a skewed interpolation operation and preserve assignment groups."""

    lines = text.splitlines(keepends=True)
    operation_line = next((index for index, line in enumerate(lines) if line.strip()), None)
    if operation_line is None:
        operation_fragment = parse_table_operation_fragment(text, path)
        return TableSkewedFragment(
            source_text=text,
            diagnostics=operation_fragment.diagnostics,
            recovered_records=operation_fragment.recovered_records,
        )
    ####
    operation_source = "\n" * operation_line + lines[operation_line]
    operation_fragment = parse_table_operation_fragment(operation_source, path)
    operation = operation_fragment.operations[0] if operation_fragment.operations else None
    groups: list[list[TableAssignment]] = []
    group_start: int | None = None
    ranges: list[tuple[int, int]] = []
    for index in range(operation_line + 1, len(lines)):
        if lines[index].strip():
            if group_start is None:
                group_start = index
            continue
        ####
        if group_start is not None:
            ranges.append((group_start, index))
            group_start = None
    ####
    if group_start is not None:
        ranges.append((group_start, len(lines)))
    ####
    diagnostics = list(operation_fragment.diagnostics)
    recovered_records = list(operation_fragment.recovered_records)
    for start, end in ranges:
        group_text = "\n" * start + "".join(lines[start:end])
        assignment_fragment = parse_table_assignment_fragment(group_text, path)
        groups.append(assignment_fragment.assignments)
        diagnostics.extend(assignment_fragment.diagnostics)
        recovered_records.extend(assignment_fragment.recovered_records)
    ####
    diagnostics.sort(key=lambda item: (item.location.line if item.location else 0, item.location.column if item.location else 0))
    recovered_records.sort(key=lambda item: (item.location.line, item.location.column))
    return TableSkewedFragment(
        source_text=text,
        operation=operation,
        assignment_groups=groups,
        diagnostics=diagnostics,
        recovered_records=recovered_records,
    )
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
