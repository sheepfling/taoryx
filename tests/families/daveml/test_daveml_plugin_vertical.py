"""Focused DAVE-ML plug-in discovery and lazy-import contract."""

from __future__ import annotations

from taoryx_daveml.plugin import DAVEMLFormatHandler

from taoryx.plugins import discover_plugins


def test_daveml_format_handler_is_selected_and_imports_only_on_use() -> None:
    """The shared format package has a vertical boundary independent of vehicles."""

    catalog = discover_plugins(include_external=False, selected=("taoryx.daveml",))

    assert tuple(plugin.id for plugin in catalog.plugins) == ("taoryx.daveml",)
    revision = catalog.plugin_revision("taoryx.daveml")
    assert revision.package == "taoryx-daveml"
    assert revision.version == "0.1.0a0"
    assert len(revision.fingerprint) == 64

    contribution = catalog.contribution("model_format", "daveml")
    assert contribution.plugin.id == "taoryx.daveml"
    assert isinstance(contribution.value, DAVEMLFormatHandler)
    assert contribution.value.format_id == "daveml"

    module = contribution.value.import_module()
    assert module.__name__ == "taoryx.trajectory.daveml_import"
    assert callable(module.load_daveml_family_import)
    ####
