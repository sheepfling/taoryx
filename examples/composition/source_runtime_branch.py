"""Execute a source problem, then branch its runtime model at flight time."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.language.grammar_contracts import GrammarProfile  # noqa: E402
from taoryx.language.ingest import ingest_file  # noqa: E402
from taoryx.language.models import ProblemDocument  # noqa: E402
from taoryx.outputs import build_run_artifact  # noqa: E402
from taoryx.runtime.engine import compute_trajectories  # noqa: E402
from taoryx.runtime.lowering import lower_problem_document  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("build/composition/source-runtime-branch"))
    arguments = parser.parse_args()
    source = Path(__file__).with_name("source_runtime_branch.prb")
    ingested = ingest_file(source, profile=GrammarProfile.TAORYX)
    if not isinstance(ingested.document, ProblemDocument) or any(item.severity.value == "error" for item in ingested.diagnostics):
        raise SystemExit("source problem did not ingest cleanly")
    lowered = lower_problem_document(ingested.document)
    original = lowered.cases[0].problem
    original_result = compute_trajectories(original, max_steps=20)
    branch = original.clone_at(1.0)
    branch_result = compute_trajectories(branch, max_steps=20)

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    build_run_artifact(source.name, original, original_result).write_json(arguments.output_dir / "original.json")
    build_run_artifact(source.name, branch, branch_result).write_json(arguments.output_dir / "branch.json")
    print(f"source={source}")
    print(f"original_final_time={original_result.states['1'][-1].time}")
    print(f"branch_start_time={branch.metadata['cloned_at_time']}")
    print(f"artifacts={arguments.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
