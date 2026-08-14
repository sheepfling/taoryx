"""Regression tests for package-based developer tool entrypoints."""

from __future__ import annotations

import json

import pytest

import tools.dev as dev
from scripts.bootstrap import editable_install_command
from taoryx.builtin_plugins import source_plugin_entry_points
from taoryx.plugins import VehicleCatalogFragment, discover_plugins
from taoryx.trajectory.catalog_mission_composition import CatalogMissionCompositionProvider
from taoryx.trajectory.registry_mission_composition import RegistryMissionCompositionProvider
from taoryx.vehicle_composition_registry import load_vehicle_composition_registry
from tools.dev import QUICK_TEST_PATHS, _changed_test_paths, tool_script
from tools.solve_vehicle_trim_evidence import main as solve_trim_evidence_main
from tools.validate_plugin_developer_route import validate as validate_plugin_developer_route
from tools.verify_plugin_wheels import (
    _SMOKE_SCRIPT,
    PLUGIN_WHEEL_SPECS,
    command_display,
    requested_selectors,
    resolve_python,
    selected_specs,
    smoke_expectations,
)


def test_tool_script_builds_module_invocation() -> None:
    command = tool_script("validate_hl20_qualification.py", "--validate-only")

    assert command[1:4] == ["-m", "tools.validate_hl20_qualification", "--validate-only"]


def test_quality_gate_previews_ruff_fixes_then_runs_pyright(monkeypatch: pytest.MonkeyPatch) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.quality()

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "ruff",
            "check",
            "--no-cache",
            "--fix",
            "--diff",
            *dev.RUFF_QUALITY_PATHS,
        ],
        [dev.project_python(), "-m", "pyright", *dev.PYRIGHT_QUALITY_PATHS],
        tool_script("validate_plugin_developer_route.py"),
    ]
    ####


def test_ruff_fix_is_local_and_uses_the_same_quality_surface(monkeypatch: pytest.MonkeyPatch) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.ruff_fix()

    assert commands == [
        [dev.project_python(), "-m", "ruff", "check", "--no-cache", "--fix", *dev.RUFF_QUALITY_PATHS]
    ]
    ####


def test_quality_scope_contains_every_declared_plugin_entry_point() -> None:
    assert len(dev.RUFF_QUALITY_PATHS) == len(set(dev.RUFF_QUALITY_PATHS))
    assert set(dev.PLUGIN_ENTRY_POINT_PATHS) <= set(dev.RUFF_QUALITY_PATHS)
    assert "packages/taoryx-cadac/src/taoryx_cadac/plugin.py" in dev.RUFF_QUALITY_PATHS
    assert "packages/taoryx-f16/src/taoryx_f16/plugin.py" in dev.RUFF_QUALITY_PATHS
    assert "packages/taoryx-simple-aero/src/taoryx_simple_aero/plugin.py" in dev.RUFF_QUALITY_PATHS
    ####


def test_core_quality_workflow_has_static_quality_and_a_deterministic_pytest_temp_root() -> None:
    workflow = (dev.ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")

    assert "python -m tools.dev quality" in workflow
    assert "--basetemp build/ci-quality-pytest" in workflow
    assert "--strict-markers" in workflow
    assert 'python-version: "3.12"' in workflow
    assert "python -m pip install -e \".[dev]\"" in workflow
    ####


def test_trim_evidence_tool_accepts_a_focused_plugin_scope(tmp_path) -> None:
    output = tmp_path / "hl20-trim.json"

    assert (
        solve_trim_evidence_main(
            [
                "--family",
                "reference_hl20_mod_k",
                "--plugin",
                "taoryx.hl20",
                "--json",
                str(output),
            ]
        )
        == 0
    )

    payload = json.loads(output.read_text(encoding="utf-8"))
    assert [report["family_id"] for report in payload["reports"]] == ["reference_hl20_mod_k"]
    assert payload["reports"][0]["status"] == "verified"
    ####


def test_changed_test_selection_maps_changed_source_to_importing_tests() -> None:
    selected = _changed_test_paths(("src/taoryx/simulation_runtime_quality.py",))

    assert "tests/unit/test_simulation_runtime_quality.py" in selected


def test_changed_test_selection_falls_back_to_quick_suite_without_test_mapping() -> None:
    assert _changed_test_paths(("docs/BUILDING_TESTS.md",)) == QUICK_TEST_PATHS


def test_declared_composition_families_have_one_focused_versioned_plugin_boundary() -> None:
    """Every standard vehicle family has one package owner and focused proof path.

    This is intentionally metadata-only. It protects the plug-in boundary
    without turning a new family registration into a catalogue-wide trajectory
    execution test.
    """

    plugins = discover_plugins(include_external=False)
    owner_by_family: dict[str, str] = {}
    package_by_plugin: dict[str, str] = {}
    for contribution in plugins.records("vehicle_catalog_fragment"):
        fragment = contribution.value
        assert isinstance(fragment, VehicleCatalogFragment)
        package_by_plugin[contribution.plugin.id] = contribution.plugin.package
        for family_id in fragment.family_ids:
            assert family_id not in owner_by_family, f"multiple vehicle catalog fragments own {family_id!r}"
            owner_by_family[family_id] = contribution.plugin.id

    declared_families = {
        declaration.family_id
        for declaration in load_vehicle_composition_registry(plugins=plugins).vehicles
    }
    assert set(owner_by_family) == declared_families
    assert set(dev.FOCUSED_VEHICLE_PLUGIN_IDS) >= declared_families
    assert set(dev.VEHICLE_VERTICAL_TEST_PATHS) >= declared_families
    assert set(dev.FOCUSED_VEHICLE_ASSET_CHECKS) >= declared_families
    assert dev.VEHICLE_INTERFACE_VERTICAL_FAMILIES >= declared_families
    assert set(dev.FOCUSED_VEHICLE_INTERFACE_FIDELITIES) >= declared_families
    assert set(dev.FOCUSED_VEHICLE_WITNESS_IDS) >= declared_families

    wheel_plugin_ids = {spec.plugin_id for spec in PLUGIN_WHEEL_SPECS}
    for family_id, plugin_id in owner_by_family.items():
        assert dev.FOCUSED_VEHICLE_PLUGIN_IDS[family_id] == (plugin_id,)
        assert plugin_id in wheel_plugin_ids
        assert (dev.ROOT / "tools" / dev.FOCUSED_VEHICLE_ASSET_CHECKS[family_id]).is_file()
        assert (dev.ROOT / "packages" / package_by_plugin[plugin_id] / "README.md").is_file()
        for path in dev.VEHICLE_VERTICAL_TEST_PATHS[family_id]:
            assert (dev.ROOT / path.split("::", maxsplit=1)[0]).is_file()
    ####


def test_isolated_vehicle_modules_do_not_return_to_host_or_compatibility_package() -> None:
    """Keep moved family implementations out of the host and legacy aggregate."""

    family_module_tokens = (
        "a320",
        "b747",
        "cadac",
        "dual_launch",
        "f16",
        "hl20",
        "hummingbird",
        "nesc",
        "passive_tumbling",
        "reduced_fixed_wing",
        "simple_aero",
        "skywalker",
        "source_f16",
        "source_table",
        "tumbling",
        "x15",
    )
    legacy_roots = (
        dev.ROOT / "src" / "taoryx",
        dev.ROOT / "packages" / "taoryx-reference-models" / "src" / "taoryx",
    )

    unexpected = sorted(
        source.relative_to(dev.ROOT).as_posix()
        for root in legacy_roots
        for source in root.rglob("*.py")
        if any(token in source.name.casefold() for token in family_module_tokens)
    )

    assert unexpected == []
    ####


def test_vehicle_slice_selects_only_the_declared_f16_vertical_contract(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_vehicle_vertical("f16_s119")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/f16/test_f16_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-f16_s119",
        ]
    ]
    ####


def test_cadac_gate_runs_only_its_selected_source_bound_actor_slices(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """CADAC onboarding adds one model gate without retesting the full catalog."""

    families: list[str] = []
    boundaries: list[str] = []
    monkeypatch.setattr(dev, "test_vehicle_vertical", families.append)
    monkeypatch.setattr(dev, "test_cadac_plugin_boundary", lambda: boundaries.append("cadac"))

    dev.test_cadac()

    assert boundaries == ["cadac"]
    assert families == [
        "cadac_aim5",
        "cadac_cruise5",
        "cadac_magsix",
        "cadac_ghame3",
        "cadac_ghame6",
        "cadac_rocket6g",
        "cadac_ads6_srbm",
        "cadac_agm6",
        "cadac_ads6_engagement",
        "cadac_sraam6",
        "cadac_ads6_sam",
        "cadac_ads6_aircraft",
        "cadac_falcon6",
    ]
    ####


def test_cadac_discovery_gate_stays_at_its_deferred_entry_point_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """CADAC discovery work does not need to execute an actor slice."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_cadac_plugin_boundary()

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/cadac/test_taoryx_plugin.py",
            "-m",
            "not slow and not artifact",
            "-o",
            "addopts=",
        ]
    ]
    ####


def test_debug_models_gate_checks_its_exact_package_data_before_its_vertical_slice(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The low-fidelity debug loop never accepts a stale package-data mirror."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_debug_models()

    assert commands[0] == [
        dev.project_python(),
        "-m",
        "tools.extract_debug_models_plugin_assets",
        "--check",
    ]
    assert commands[1] == [
        dev.project_python(),
        "-m",
        "pytest",
        "tests/families/debug_models/test_debug_models_vertical.py",
        "-m",
        "not slow and not artifact",
        "-o",
        "addopts=",
    ]
    ####


def test_daveml_gate_runs_only_its_discovery_and_lazy_import_slice(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """DAVE-ML maintenance does not pull a vehicle family into the inner loop."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_daveml_plugin()

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/daveml/test_daveml_plugin_vertical.py",
            "-m",
            "not slow and not artifact",
            "-o",
            "addopts=",
        ]
    ]
    ####


def test_daveml_check_adds_only_its_installed_wheel_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The package gate composes the focused source test with its own wheel."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_daveml_plugin()

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/daveml/test_daveml_plugin_vertical.py",
            "-m",
            "not slow and not artifact",
            "-o",
            "addopts=",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.verify_plugin_wheels",
            "--plugin",
            "daveml",
            "--python",
            dev.project_python(),
        ],
    ]
    ####


def test_daveml_wheel_spec_forces_its_lazy_model_format_import() -> None:
    spec = next(item for item in PLUGIN_WHEEL_SPECS if item.selector == "daveml")

    assert spec.owned_modules == (
        "taoryx.trajectory.daveml_evaluator",
        "taoryx.trajectory.daveml_import",
        "taoryx.trajectory.daveml_semantic",
        "taoryx_daveml.plugin",
    )
    assert spec.model_format_imports == (("daveml", "taoryx.trajectory.daveml_import"),)
    assert spec.exercise_provider_catalog is False
    ####


def test_reachability_gate_checks_its_exact_assets_before_its_vertical_slice(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Reachability maintenance does not trigger an X-15 or HL-20 study."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_reachability_plugin()

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_reachability_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/reachability/test_reachability_plugin_vertical.py",
            "-m",
            "not slow and not artifact",
            "-o",
            "addopts=",
        ],
    ]
    ####


def test_reachability_check_adds_only_its_installed_wheel_boundary(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The overlay package gate composes its source slice and dedicated wheel."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_reachability_plugin()

    assert commands[-1] == [
        dev.project_python(),
        "-m",
        "tools.verify_plugin_wheels",
        "--plugin",
        "reachability",
        "--python",
        dev.project_python(),
    ]
    assert commands[:-1] == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_reachability_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/reachability/test_reachability_plugin_vertical.py",
            "-m",
            "not slow and not artifact",
            "-o",
            "addopts=",
        ],
    ]
    ####


def test_reachability_wheel_spec_excludes_legacy_passive_modules_and_requires_owned_assets() -> None:
    spec = next(item for item in PLUGIN_WHEEL_SPECS if item.selector == "reachability")

    assert spec.wheel_dependency_selectors == ("daveml", "x15", "hl20")
    assert spec.forbidden_wheel_prefixes == (
        "taoryx/passive_tumbling_composition_execution.py",
        "taoryx/passive_tumbling_mission_translation.py",
    )
    assert spec.required_wheel_paths == (
        "taoryx/reachability_catalog.py",
        "taoryx/reachability_envelope.py",
        "taoryx_reachability/capabilities.py",
        "taoryx_reachability/preflight.py",
        "taoryx_reachability/provider.py",
        "taoryx_reachability/data/package_data_provenance.json",
        "taoryx_reachability/data/verification/reachability_profile_catalog.yaml",
        "taoryx_reachability/data/x15_overlay/verification/vehicle_composition_registry.yaml",
        "taoryx_reachability/data/x15_overlay/verification/vehicle_execution_bindings.yaml",
        "taoryx_reachability/data/x15_overlay/verification/vehicle_execution_witnesses.yaml",
        "taoryx_reachability/data/x15_overlay/examples/vehicle_composition/x15_staged_booster_reachability_3dof_compose.yaml",
        "taoryx_reachability/data/hl20_overlay/verification/vehicle_composition_registry.yaml",
        "taoryx_reachability/data/hl20_overlay/verification/vehicle_execution_bindings.yaml",
        "taoryx_reachability/data/hl20_overlay/verification/vehicle_execution_witnesses.yaml",
        "taoryx_reachability/data/hl20_overlay/examples/vehicle_composition/hl20_source_booster_release_replay_3dof_compose.yaml",
        "taoryx_reachability/data/hl20_overlay/examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml",
        "taoryx_reachability/data/examples/showcases/hl20_california_to_hawaii/quality_gates.yaml",
    )
    ####


def test_f16_vehicle_check_stays_at_reduced_tiers_and_its_own_plugin(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("f16_s119")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_f16_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "f16_s119",
            "--fidelity",
            "point_mass_3dof",
            "--fidelity",
            "pseudo_6dof",
            "--plugin",
            "taoryx.f16",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "f16-3dof-batch",
            "--witness",
            "f16-pseudo6dof-batch",
            "--witness",
            "f16-3dof-episode",
            "--witness",
            "f16-pseudo6dof-episode",
            "--summary",
            "--plugin",
            "taoryx.f16",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--family",
            "f16_s119",
            "--parity-only",
            "--summary",
            "--plugin",
            "taoryx.f16",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/f16/test_f16_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-f16_s119",
        ],
    ]
    ####


def test_a320_vehicle_check_stays_at_reduced_tiers_and_its_own_plugin(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("a320_openap_3dof")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_a320_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "a320_openap_3dof",
            "--fidelity",
            "point_mass_3dof",
            "--fidelity",
            "pseudo_6dof",
            "--plugin",
            "taoryx.a320",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "a320-3dof-batch",
            "--witness",
            "a320-pseudo6dof-batch",
            "--witness",
            "a320-3dof-episode",
            "--witness",
            "a320-pseudo6dof-episode",
            "--summary",
            "--plugin",
            "taoryx.a320",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--family",
            "a320_openap_3dof",
            "--parity-only",
            "--summary",
            "--plugin",
            "taoryx.a320",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/a320/test_a320_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-a320_openap_3dof",
        ],
    ]
    ####


def test_x8_vehicle_check_stays_at_reduced_tiers_and_its_own_shared_plugin(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("skywalker_x8")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_source_table_fixed_wing_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "skywalker_x8",
            "--fidelity",
            "point_mass_3dof",
            "--fidelity",
            "pseudo_6dof",
            "--plugin",
            "taoryx.source-table-fixed-wing",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "x8-3dof-batch",
            "--witness",
            "x8-pseudo6dof-batch",
            "--witness",
            "x8-3dof-episode",
            "--witness",
            "x8-pseudo6dof-episode",
            "--summary",
            "--plugin",
            "taoryx.source-table-fixed-wing",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--family",
            "skywalker_x8",
            "--parity-only",
            "--summary",
            "--plugin",
            "taoryx.source-table-fixed-wing",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/source_table_fixed_wing/test_source_table_fixed_wing_plugin_boundary.py",
            "tests/families/source_table_fixed_wing/test_x8_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-skywalker_x8",
        ],
    ]
    ####


def test_b747_vehicle_check_stays_at_reduced_tiers_and_its_own_shared_plugin(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The transport gate excludes its separate direct-wrench/controller lane."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("b747")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_source_table_fixed_wing_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "b747",
            "--fidelity",
            "point_mass_3dof",
            "--fidelity",
            "pseudo_6dof",
            "--plugin",
            "taoryx.source-table-fixed-wing",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "b747-3dof-batch",
            "--witness",
            "b747-pseudo6dof-batch",
            "--witness",
            "b747-3dof-episode",
            "--witness",
            "b747-pseudo6dof-episode",
            "--summary",
            "--plugin",
            "taoryx.source-table-fixed-wing",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--family",
            "b747",
            "--parity-only",
            "--summary",
            "--plugin",
            "taoryx.source-table-fixed-wing",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/source_table_fixed_wing/test_source_table_fixed_wing_plugin_boundary.py",
            "tests/families/source_table_fixed_wing/test_b747_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-b747",
        ],
    ]
    ####


def test_x15_vehicle_check_stays_at_its_local_direct_wrench_contract(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The normal X-15 plug-in gate does not pull in reachability or surfaces."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("x15")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_x15_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "x15",
            "--fidelity",
            "rigid_body_6dof_direct_wrench",
            "--plugin",
            "taoryx.x15",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "x15-local-direct-wrench-screen-batch",
            "--witness",
            "x15-local-direct-wrench-screen-episode",
            "--summary",
            "--plugin",
            "taoryx.x15",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--family",
            "x15",
            "--parity-only",
            "--summary",
            "--plugin",
            "taoryx.x15",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/x15/test_x15_plugin_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-x15",
        ],
    ]
    ####


def test_hl20_vehicle_check_stays_at_its_local_direct_wrench_contract(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The normal HL-20 plug-in gate does not pull in reachability or surfaces."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("hl20_mod_k")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_hl20_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "hl20_mod_k",
            "--fidelity",
            "rigid_body_6dof_direct_wrench",
            "--plugin",
            "taoryx.hl20",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "hl20-local-direct-wrench-screen-batch",
            "--witness",
            "hl20-local-direct-wrench-screen-episode",
            "--summary",
            "--plugin",
            "taoryx.hl20",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--family",
            "hl20_mod_k",
            "--parity-only",
            "--summary",
            "--plugin",
            "taoryx.hl20",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/hl20/test_hl20_plugin_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-hl20_mod_k",
        ],
    ]
    ####


def test_nesc_vehicle_check_stays_at_its_source_replay_contract(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """The NESC gate excludes its separately installed passive child runtime."""

    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("reference_nesc_two_stage_rocket")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_nesc_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "reference_nesc_two_stage_rocket",
            "--fidelity",
            "point_mass_3dof",
            "--fidelity",
            "pseudo_6dof",
            "--plugin",
            "taoryx.nesc",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "nesc-3dof-batch",
            "--witness",
            "nesc-pseudo6dof-batch",
            "--summary",
            "--plugin",
            "taoryx.nesc",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/nesc/test_nesc_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-reference_nesc_two_stage_rocket",
        ],
    ]
    ####


def test_tumbling_body_vehicle_check_stays_batch_only_and_uses_its_own_plugin(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("tumbling_body")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_passive_bodies_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "tumbling_body",
            "--fidelity",
            "point_mass_3dof",
            "--fidelity",
            "pseudo_6dof",
            "--plugin",
            "taoryx.passive-bodies",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "tumbling-3dof-batch",
            "--witness",
            "tumbling-pseudo6dof-batch",
            "--summary",
            "--plugin",
            "taoryx.passive-bodies",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/passive_bodies/test_tumbling_body_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-tumbling_body",
        ],
    ]
    ####


def test_vehicle_slice_selects_the_simple_aero_workflow_contract(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_vehicle_vertical("simple_aero")

    assert commands[0] == [
        dev.project_python(),
        "-m",
        "tools.extract_simple_aero_plugin_assets",
        "--check",
    ]
    assert commands[1][3] == "tests/families/simple_aero/test_simple_aero_workflow_vertical.py"
    assert commands[1][-1] == ".pytest-vehicle-simple_aero"
    ####


def test_vehicle_slice_selects_the_dual_launch_workflow_contract(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.test_vehicle_vertical("dual_launch_glider")

    assert commands[0] == [
        dev.project_python(),
        "-m",
        "tools.extract_dual_launch_plugin_assets",
        "--check",
    ]
    assert commands[1][3] == "tests/families/dual_launch/test_dual_launch_workflow_vertical.py"
    assert commands[1][-1] == ".pytest-vehicle-dual_launch_glider"
    ####


def test_hummingbird_vehicle_check_stays_at_its_reduced_contract_and_own_plugin(
    monkeypatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_vehicle_vertical("hummingbird")

    assert commands == [
        [
            dev.project_python(),
            "-m",
            "tools.extract_hummingbird_plugin_assets",
            "--check",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_interface_catalog",
            "--check",
            "--family",
            "hummingbird",
            "--fidelity",
            "pseudo_6dof",
            "--plugin",
            "taoryx.hummingbird",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--witness",
            "hummingbird-pseudo6dof-batch",
            "--witness",
            "hummingbird-pseudo6dof-episode",
            "--summary",
            "--plugin",
            "taoryx.hummingbird",
        ],
        [
            dev.project_python(),
            "-m",
            "tools.validate_vehicle_execution_witnesses",
            "--family",
            "hummingbird",
            "--parity-only",
            "--summary",
            "--plugin",
            "taoryx.hummingbird",
        ],
        [
            dev.project_python(),
            "-m",
            "pytest",
            "tests/families/hummingbird/test_hummingbird_vehicle_vertical.py",
            "-q",
            "-x",
            "-o",
            "addopts=",
            "--strict-markers",
            "-m",
            "not slow and not artifact and not simple_aero",
            "--basetemp",
            ".pytest-vehicle-hummingbird",
        ],
    ]
    ####


def test_vehicle_catalogue_gate_runs_only_catalogue_contract_checks(
    monkeypatch,
) -> None:
    calls: list[str] = []
    for name in (
        "test_vehicle_catalogue",
        "check_vehicle_models",
        "check_vehicle_interfaces",
        "onboard_vehicles",
    ):
        monkeypatch.setattr(dev, name, lambda name=name: calls.append(name))

    dev.vehicle_catalogue()

    assert calls == [
        "test_vehicle_catalogue",
        "check_vehicle_models",
        "check_vehicle_interfaces",
        "onboard_vehicles",
    ]
    ####


def test_bootstrap_full_profile_installs_every_editable_distribution() -> None:
    command = editable_install_command(
        "python",
        profile="full",
        with_dependencies=True,
        with_sensors=False,
    )

    assert command == [
        "python",
        "-m",
        "pip",
        "install",
        "-e",
        ".[dev]",
        "-e",
        "packages/taoryx-daveml",
        "-e",
        "packages/taoryx-debug-models",
        "-e",
        "packages/taoryx-a320",
        "-e",
        "packages/taoryx-f16",
        "-e",
        "packages/taoryx-hummingbird",
        "-e",
        "packages/taoryx-nesc",
        "-e",
        "packages/taoryx-passive-bodies",
        "-e",
        "packages/taoryx-simple-aero",
        "-e",
        "packages/taoryx-dual-launch",
        "-e",
        "packages/taoryx-x15",
        "-e",
        "packages/taoryx-hl20",
        "-e",
        "packages/taoryx-source-table-fixed-wing",
        "-e",
        "packages/taoryx-reference-models",
        "-e",
        "packages/taoryx-reachability",
    ]
    ####


def test_bootstrap_developer_profile_installs_direct_plug_ins_without_compatibility_aggregate() -> None:
    command = editable_install_command(
        "python",
        profile="developer",
        with_dependencies=True,
        with_sensors=False,
    )

    assert command.count("-e") == 14
    assert "packages/taoryx-reference-models" not in command
    assert command[-1] == "packages/taoryx-reachability"
    ####


def test_bootstrap_compatibility_profile_installs_only_aggregate_dependencies() -> None:
    command = editable_install_command(
        "python",
        profile="compatibility",
        with_dependencies=True,
        with_sensors=False,
    )

    assert command.count("-e") == 12
    assert "packages/taoryx-reference-models" in command
    assert "packages/taoryx-debug-models" not in command
    assert "packages/taoryx-passive-bodies" not in command
    assert "packages/taoryx-reachability" not in command
    ####


def test_bootstrap_offline_sensor_fallback_keeps_full_local_distribution_set() -> None:
    command = editable_install_command(
        "python",
        profile="full",
        with_dependencies=False,
        with_sensors=True,
    )

    assert command[4:6] == ["--no-build-isolation", "--no-deps"]
    assert command.count("-e") == 15
    assert command[-1] == "packages/taoryx-reachability"
    ####


def test_developer_source_roots_follow_declared_entry_points() -> None:
    """The task runner must not retain a manually ordered sibling package list."""

    expected_roots = {entry_point.project.parent / "src" for entry_point in source_plugin_entry_points()}

    assert dev.SOURCE_ROOTS[0] == dev.ROOT / "src"
    assert set(dev.SOURCE_ROOTS[1:]) == expected_roots
    ####


def test_direct_plugin_developer_route_is_self_contained() -> None:
    """Direct packages must not silently regain the legacy aggregate."""

    validate_plugin_developer_route()
    ####


def test_catalog_provider_is_the_canonical_direct_plugin_host_name() -> None:
    """Direct plug-ins have a catalog-scoped host name, not an aggregate one."""

    assert issubclass(CatalogMissionCompositionProvider, RegistryMissionCompositionProvider)
    with pytest.raises(TypeError, match="explicit resolved vehicle catalog"):
        CatalogMissionCompositionProvider()
    ####


def test_developer_plugin_gate_checks_metadata_before_installed_compliance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commands: list[list[str]] = []
    installed_checks: list[str] = []
    monkeypatch.setattr(dev, "run", commands.append)
    monkeypatch.setattr(dev, "installation_check", lambda: installed_checks.append("installed"))

    dev.check_developer_plugins()

    assert commands == [tool_script("validate_plugin_developer_route.py")]
    assert installed_checks == ["installed"]
    ####


def test_focused_plugin_gate_checks_metadata_then_one_fresh_wheel(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.check_plugin_install("f16")

    assert commands == [
        tool_script("validate_plugin_developer_route.py"),
        tool_script("verify_plugin_wheels.py", "--plugin", "f16", "--python", dev.project_python()),
    ]
    ####


def test_handoff_builds_from_an_sdist_before_the_release_wheel(monkeypatch) -> None:
    calls: list[list[str]] = []
    monkeypatch.setattr(dev, "equation_audit", lambda: None)
    monkeypatch.setattr(dev, "check", lambda: None)
    monkeypatch.setattr(dev, "run", calls.append)

    dev.handoff()

    assert calls[0] == [dev.project_python(), "-m", "build", "--no-isolation", "--outdir", "dist"]
    assert "--wheel" not in calls[0]
    ####


def test_plugin_wheel_smoke_defaults_to_each_currently_isolated_distribution() -> None:
    assert tuple(item.selector for item in selected_specs(None)) == (
        "daveml",
        "cadac",
        "a320",
        "f16",
        "hummingbird",
        "source-table-fixed-wing",
        "x15",
        "hl20",
        "debug-models",
        "simple-aero",
        "dual-launch",
        "nesc",
        "passive-bodies",
    )
    assert tuple(item.plugin_id for item in selected_specs(("hummingbird",))) == ("taoryx.hummingbird",)
    assert tuple(item.selector for item in selected_specs(("reachability",))) == (
        "daveml",
        "x15",
        "hl20",
        "reachability",
    )
    reachability_expectations = smoke_expectations(
        selected_specs(("reachability",)),
        exercised_selectors=requested_selectors(("reachability",)),
    )
    assert [item["exercise_boundary"] for item in reachability_expectations] == [False, False, False, True]
    assert reachability_expectations[-1]["deferred_module_names"] == (
        "taoryx_reachability.capabilities",
        "taoryx_reachability.preflight",
        "taoryx.hl20_source_aerodynamics",
        "taoryx.hl20_source_model",
        "taoryx.hl20_source_release_composition_execution",
        "taoryx.x15_staged_composition_execution",
    )
    hl20 = next(item for item in selected_specs(("hl20",)) if item.selector == "hl20")
    assert hl20.exercise_composition_batch is False
    passive_bodies = selected_specs(("passive-bodies",))[0]
    assert passive_bodies.deferred_module_names == (
        "numpy",
        "scipy",
        "taoryx.family_adapter",
        "taoryx.family_adapter_registry",
        "taoryx_passive_bodies.capabilities",
        "taoryx_passive_bodies.preflight",
        "taoryx_passive_bodies.deployment_runtime",
        "taoryx.passive_body_dynamics",
        "taoryx.passive_body_interface",
        "taoryx.passive_tumbling_composition_execution",
        "taoryx.passive_tumbling_mission_translation",
    )
    f16 = selected_specs(("f16",))[1]
    assert f16.deferred_module_names == (
        "numpy",
        "scipy",
        "taoryx.f16_composition_episode",
        "taoryx.f16_local_physical_control_screen",
        "taoryx.f16_local_physical_lqi_screen",
        "taoryx.f16_mission_capability",
        "taoryx.f16_mission_translation",
        "taoryx.f16_physical_schedule_interior_screen",
        "taoryx.f16_physical_schedule_transition_screen",
        "taoryx.f16_reduced_batch_episode_parity",
        "taoryx.f16_reduced_execution",
        "taoryx.f16_reduced_interface",
        "taoryx.f16_trim_evidence",
        "taoryx.source_f16",
        "taoryx.trajectory.f16_reduced_adapter",
        "taoryx.trajectory.f16_reductions",
        "taoryx.trajectory.f16_reference",
    )
    hummingbird = selected_specs(("hummingbird",))[0]
    assert hummingbird.deferred_module_names == (
        "numpy",
        "scipy",
        "taoryx.aero_drag_tables",
        "taoryx.composition_batch_episode_parity",
        "taoryx.hummingbird_composition_episode",
        "taoryx.hummingbird_composition_execution",
        "taoryx.hummingbird_local_physical_control_screen",
        "taoryx.hummingbird_mission_capability",
        "taoryx.hummingbird_mission_translation",
        "taoryx.hummingbird_preflight",
        "taoryx.hummingbird_reduced_interface",
        "taoryx.local_direct_wrench",
        "taoryx.local_direct_wrench_mission_translation",
        "taoryx.source_table_multirotor",
        "taoryx.trajectory.hummingbird_3dof",
        "taoryx.trajectory.hummingbird_adapter",
        "taoryx.trajectory.hummingbird_pseudo6dof",
    )
    assert tuple(item.selector for item in selected_specs(("cross-plugin-deployment",))) == (
        "daveml",
        "passive-bodies",
        "cross-plugin-deployment",
    )
    assert hummingbird.resource_module == "taoryx_hummingbird.resources"
    assert hummingbird.composition_witness == "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml"
    assert hummingbird.batch_only_witnesses == ("examples/vehicle_composition/hummingbird_local_direct_wrench_screen_compose.yaml",)
    a320 = selected_specs(("a320",))[0]
    assert a320.deferred_module_names == (
        "numpy",
        "scipy",
        "taoryx.aero_drag_tables",
        "taoryx.a320_composition_episode",
        "taoryx.a320_mission_capability",
        "taoryx.a320_reduced_batch_episode_parity",
        "taoryx.a320_reduced_execution",
        "taoryx.control_allocation",
        "taoryx.family_adapter",
        "taoryx.family_adapter_registry",
        "taoryx.local_native_coordinate_lqi",
        "taoryx.local_native_coordinate_lqi_composition_execution",
        "taoryx.trajectory.a320_adapter",
        "taoryx.trajectory.a320_openap",
        "taoryx.trajectory.a320_pseudo6dof",
        "taoryx.trajectory.a320_racetrack",
    )
    assert a320.resource_module == "taoryx_a320.resources"
    assert a320.composition_witness == "examples/vehicle_composition/a320_racetrack_capability_pseudo6dof_compose.yaml"
    assert a320.batch_only_witnesses == ("examples/vehicle_composition/a320_local_native_coordinate_lqi_screen_compose.yaml",)
    assert a320.forbidden_wheel_prefixes == (
        "taoryx_a320/data/resources/aerospace/daveml/taoryx-corpus-v1.1/qualified-models/",
        "taoryx_a320/data/resources/aerospace/daveml/taoryx-corpus-v1.1/source-corpora/",
    )
    x15 = selected_specs(("x15",))[0]
    assert x15.deferred_module_names == (
        "numpy",
        "scipy",
        "taoryx.aero_drag_tables",
        "taoryx.control_allocation",
        "taoryx.direct_wrench",
        "taoryx.family_adapter",
        "taoryx.family_adapter_registry",
        "taoryx.local_direct_wrench",
        "taoryx.local_direct_wrench_batch_episode_parity",
        "taoryx.local_direct_wrench_composition_execution",
        "taoryx.local_direct_wrench_mission_translation",
        "taoryx.physical_lqr",
        "taoryx.x15_adapter",
        "taoryx.x15_local_physical_surface_lqi_screen",
        "taoryx.x15_maneuvers",
        "taoryx.x15_surface_authority_screen",
    )
    f16 = selected_specs(("f16",))[1]
    assert f16.resource_module == "taoryx_f16.resources"
    assert f16.composition_witness == "examples/vehicle_composition/f16_racetrack_capability_pseudo6dof_compose.yaml"
    assert f16.trim_evidence_family_id == "reference_f16_s119"
    hl20 = selected_specs(("hl20",))[1]
    assert hl20.resource_module == "taoryx_hl20.resources"
    assert hl20.trim_evidence_family_id == "reference_hl20_mod_k"
    dual_launch = selected_specs(("dual-launch",))[0]
    assert dual_launch.resource_module is None
    assert dual_launch.composition_witness is None
    assert dual_launch.workflow_endpoint_ids == ("dual-launch-attached-booster-batch",)
    reference_models = selected_specs(("reference-models",))[0]
    assert not reference_models.exercise_provider_catalog
    assert reference_models.forbidden_wheel_prefixes == ("taoryx/",)
    assert reference_models.forbidden_top_level_packages == ("taoryx",)
    cross_plugin_deployment = selected_specs(("cross-plugin-deployment",))[-1]
    assert cross_plugin_deployment.focused_provider_id == "taoryx.nesc.mission-composition"
    ####


def test_plugin_wheel_smoke_stabilizes_a_relative_interpreter_path() -> None:
    assert resolve_python("python") == "python"
    assert resolve_python(".venv/bin/python").endswith("/.venv/bin/python")
    ####


def test_plugin_wheel_smoke_does_not_echo_its_inline_child_program() -> None:
    assert command_display(["python", "-c", _SMOKE_SCRIPT]) == "python -c <installed-wheel smoke>"
    ####


def test_plugin_wheel_smoke_compiles_compositions_in_its_selected_catalog() -> None:
    assert _SMOKE_SCRIPT.count("compile_vehicle_composition(source_request, plugins=catalog)") == 1
    assert _SMOKE_SCRIPT.count("compile_vehicle_composition(additional_request, plugins=catalog)") == 1
    assert _SMOKE_SCRIPT.count("compile_vehicle_composition(batch_only_request, plugins=catalog)") == 1
    ####


def test_cross_plugin_wheel_smoke_uses_its_selected_provider() -> None:
    assert "api_provider = providers.provider(provider_id)" in _SMOKE_SCRIPT
    assert "api_provider = RegistryMissionCompositionProvider()" not in _SMOKE_SCRIPT
    ####


def test_plugin_wheel_smoke_uses_the_project_interpreter(monkeypatch) -> None:
    commands: list[list[str]] = []
    monkeypatch.setattr(dev, "run", commands.append)

    dev.plugin_wheel_smoke()

    assert commands == [tool_script("verify_plugin_wheels.py", "--python", dev.project_python())]
    ####


def test_installation_check_does_not_overlay_developer_source_roots(monkeypatch) -> None:
    captured: dict[str, object] = {}
    monkeypatch.setenv("PYTHONPATH", "/source-tree-fallback")
    monkeypatch.setattr(
        dev.subprocess,
        "run",
        lambda command, **kwargs: captured.update(command=command, **kwargs),
    )

    dev.installation_check()

    assert captured["command"][-3:] == ["plugins", "check", "--profile", "developer"][-3:]
    assert "PYTHONPATH" not in captured["env"]
    ####
