from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))
####

from ballistic_simulation import LaunchCase, load_cd_table, simulate_and_render


def main() -> None:
    root = Path(__file__).resolve().parent
    generated = root / "generated"
    artifacts = root / "artifacts"
    cone_table = load_cd_table(generated / "cone_cd.tbl")

    case = LaunchCase(
        name="reference-cone-launch",
        initial_speed_mps=600.0,
        flight_path_angle_deg=20.0,
    )

    simulate_and_render(
        case,
        cone_table,
        history_path=generated / "ballistic_cone_history.tbl",
        plot_path=artifacts / "ballistic_cone_profiles.png",
    )
    print(generated / "ballistic_cone_history.tbl")
    print(artifacts / "ballistic_cone_profiles.png")
####


if __name__ == "__main__":
    main()
####
