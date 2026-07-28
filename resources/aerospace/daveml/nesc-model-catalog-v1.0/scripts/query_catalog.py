from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("models")
    scenario_parser = subparsers.add_parser("scenarios")
    scenario_parser.add_argument("--model")
    document_parser = subparsers.add_parser("documents")
    document_parser.add_argument("--model")
    document_parser.add_argument("--compile-status")
    subparsers.add_parser("capabilities")
    args = parser.parse_args()
    connection = sqlite3.connect(args.root / "catalog.sqlite")
    connection.row_factory = sqlite3.Row
    try:
        if args.command == "models":
            sql = "SELECT * FROM model_families ORDER BY priority, model_id"
            parameters: list[str] = []
        elif args.command == "scenarios":
            sql = "SELECT * FROM scenarios"
            parameters = []
            if args.model:
                sql += " WHERE model_id = ?"
                parameters.append(args.model)
            sql += " ORDER BY scenario_id"
        elif args.command == "documents":
            clauses = []
            parameters = []
            if args.model:
                clauses.append("model_id = ?")
                parameters.append(args.model)
            if args.compile_status:
                clauses.append("compile_status = ?")
                parameters.append(args.compile_status)
            sql = "SELECT * FROM source_documents"
            if clauses:
                sql += " WHERE " + " AND ".join(clauses)
            sql += " ORDER BY source_set, source_path"
        else:
            sql = "SELECT * FROM host_capabilities ORDER BY priority, capability_id"
            parameters = []
        rows = [dict(row) for row in connection.execute(sql, parameters)]
        print(json.dumps(rows, indent=2, sort_keys=True))
    finally:
        connection.close()
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
