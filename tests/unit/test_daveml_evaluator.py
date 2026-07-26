from __future__ import annotations

from pathlib import Path

import pytest

from taoryx.trajectory.daveml_atmosphere import load_daveml_atmosphere
from taoryx.trajectory.daveml_evaluator import evaluate_daveml_checkdata, load_daveml_graph


def test_official_atmosphere_binding_preserves_hash_and_normalizes_si() -> None:
    binding = load_daveml_atmosphere(
        Path("resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
    )
    values = binding.evaluate(0.0)
    assert binding.source_sha256 == "b0803b550447f2fa10c81bf7e703449a1944fb633955990a1d01c2dea71c29ca"
    assert values["temperature_k"] == pytest.approx(288.15)
    assert values["pressure_pa"] == pytest.approx(2116.22 * 47.88025898033584)
    assert values["density_ratio"] == pytest.approx(1.0)
    assert values["speed_of_sound_m_s"] == pytest.approx(1116.3975415003385 * 0.3048)


def test_direct_point_function_checkdata_is_evaluated() -> None:
    payload = b"""
    <DAVEfunc>
      <function>
        <independentVarPts varID="x">0 10</independentVarPts>
        <dependentVarPts varID="y">0 20</dependentVarPts>
      </function>
      <checkData><staticShot name="mid">
        <checkInputs><signal><signalID>x</signalID><signalValue>2.5</signalValue></signal></checkInputs>
        <checkOutputs><signal><signalID>y</signalID><signalValue>5</signalValue></signal></checkOutputs>
      </staticShot></checkData>
    </DAVEfunc>
    """
    results = evaluate_daveml_checkdata(payload)
    assert len(results) == 1
    assert results[0].status == "passed"
    assert results[0].actual == 5.0


def test_function_source_precedes_placeholder_initial_value() -> None:
    payload = b"""
    <DAVEfunc>
      <variableDef varID="x" name="x" initialValue="0"/>
      <variableDef varID="y" name="y"/>
      <function>
        <independentVarRef varID="x"/><dependentVarRef varID="y"/>
        <independentVarPts varID="x">0 1</independentVarPts>
        <dependentVarPts varID="y">10 20</dependentVarPts>
      </function>
      <checkData><staticShot name="derived">
        <checkInputs><signal><signalID>x</signalID><signalValue>0.5</signalValue></signal></checkInputs>
        <checkOutputs><signal><signalID>y</signalID><signalValue>15</signalValue></signal></checkOutputs>
      </staticShot></checkData>
    </DAVEfunc>
    """
    results = evaluate_daveml_checkdata(payload)
    assert len(results) == 1
    assert results[0].status == "passed"
    assert results[0].actual == 15.0


def test_typed_graph_can_be_reused_for_named_outputs() -> None:
    payload = b"""
    <DAVEfunc>
      <variableDef varID="x" name="x" initialValue="0"/>
      <variableDef varID="y" name="y"/>
      <function>
        <independentVarRef varID="x"/><dependentVarRef varID="y"/>
        <independentVarPts varID="x">0 1</independentVarPts>
        <dependentVarPts varID="y">10 20</dependentVarPts>
      </function>
    </DAVEfunc>
    """
    graph = load_daveml_graph(payload, document_id="fixture")
    values = graph.evaluate({"x": 0.5}, ("y",))
    assert graph.document_id == "fixture"
    assert graph.unit_for("y") is None
    assert graph.dimension_for("y") is None
    assert values == {"y": 15.0}


def test_typed_graph_exposes_source_dimension() -> None:
    graph = load_daveml_graph(b'<DAVEfunc><variableDef varID="x" units="deg"/></DAVEfunc>')
    assert graph.unit_for("x") == "deg"
    assert graph.dimension_for("x") == "angle"
    area = load_daveml_graph(b'<DAVEfunc><variableDef varID="s" units="f2"/></DAVEfunc>')
    assert area.dimension_for("s") == "length^2"


def test_typed_graph_evaluates_vector_constant_and_input() -> None:
    payload = b'<DAVEfunc><variableDef varID="v" initialValue="1 2 3"/><variableDef varID="w"/></DAVEfunc>'
    graph = load_daveml_graph(payload, document_id="vector")
    assert graph.evaluate_vectors({}, ("v",))["v"] == (1.0, 2.0, 3.0)
    assert graph.evaluate_vectors({"w": (4.0, 5.0)}, ("w",))["w"] == (4.0, 5.0)
    with pytest.raises(ValueError, match="no supported vector source"):
        graph.evaluate_vectors({}, ("w",))


def test_regular_gridded_table_checkdata_is_evaluated() -> None:
    payload = b"""
    <DAVEfunc>
      <breakpointDef bpID="x_axis"><bpVals>0 10</bpVals></breakpointDef>
      <griddedTableDef gtID="table">
        <breakpointRefs><bpRef bpID="x_axis"/></breakpointRefs>
        <dataTable>0 20</dataTable>
      </griddedTableDef>
      <function>
        <independentVarRef varID="x"/><dependentVarRef varID="y"/>
        <functionDefn><griddedTableRef gtID="table"/></functionDefn>
      </function>
      <checkData><staticShot name="mid">
        <checkInputs><signal><signalID>x</signalID><signalValue>2.5</signalValue></signal></checkInputs>
        <checkOutputs><signal><signalID>y</signalID><signalValue>5</signalValue></signal></checkOutputs>
      </staticShot></checkData>
    </DAVEfunc>
    """
    results = evaluate_daveml_checkdata(payload)
    assert results[0].status == "passed"
    assert results[0].actual == 5.0


def test_regular_gridded_table_checkdata_clamps_like_davetools() -> None:
    payload = b"""
    <DAVEfunc>
      <breakpointDef bpID="x_axis"><bpVals>0 10</bpVals></breakpointDef>
      <griddedTableDef gtID="table">
        <breakpointRefs><bpRef bpID="x_axis"/></breakpointRefs>
        <dataTable>0 20</dataTable>
      </griddedTableDef>
      <function>
        <independentVarRef varID="x"/><dependentVarRef varID="y"/>
        <functionDefn><griddedTableRef gtID="table"/></functionDefn>
      </function>
      <checkData><staticShot name="upper-clamp">
        <checkInputs><signal><signalID>x</signalID><signalValue>20</signalValue></signal></checkInputs>
        <checkOutputs><signal><signalID>y</signalID><signalValue>20</signalValue></signal></checkOutputs>
      </staticShot></checkData>
    </DAVEfunc>
    """
    results = evaluate_daveml_checkdata(payload)
    assert results[0].status == "passed"
    assert results[0].actual == 20.0


def test_ungridded_official_table_mismatch_is_explicitly_reported() -> None:
    source = Path("resources/aerospace/daveml/official-conformance-v1/twoD_ungridded.dml")
    results = evaluate_daveml_checkdata(source.read_bytes())
    assert results
    assert [result.status for result in results] == ["unsupported"] * 4
    assert all(result.actual is None for result in results)
    assert all("griddedTableRef" in (result.reason or "") for result in results)
    assert results[0].absolute_tolerance == 0.0005


def test_official_atmosphere_checkdata_uses_named_variables_and_tolerances() -> None:
    source = Path("resources/aerospace/daveml/official-conformance-v1/atmos_76.dml")
    results = evaluate_daveml_checkdata(source.read_bytes())
    assert len(results) == 126
    assert all(result.status == "passed" for result in results)


def test_official_f16_checkdata_resolves_nested_calculations() -> None:
    source = Path("resources/aerospace/daveml/official-conformance-v1/F16_aero.dml")
    results = evaluate_daveml_checkdata(source.read_bytes())
    assert len(results) == 102
    assert all(result.status == "passed" for result in results)
