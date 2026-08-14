# taoryx-daveml

Optional DAVE-ML support for Taoryx. This distribution owns the DAVE-ML
import, semantic IR, evaluator, compatibility overlay, collection, and replay
modules. Installing it publishes the `daveml` model-format contribution
through the versioned `taoryx.plugins` entry-point group.

The implementation occupies the shared `taoryx.trajectory` namespace so
historical imports remain valid without placing DAVE-ML code in the core
Taoryx wheel.

## Focused plug-in boundary

The DAVE-ML package is a shared model-format dependency, not a vehicle family.
Its focused maintenance gate proves only the selected discovery path and the
lazy format-handler import; it does not construct or rerun F-16, HL-20, NESC,
or another consumer:

```bash
python tools/dev.py test-daveml
python tools/dev.py check-daveml
```

The installed-wheel step builds fresh source distributions, installs only core
and `taoryx-daveml`, verifies the package-managed version and revision
fingerprint, and invokes the registered `daveml` handler. The handler must load
`taoryx.trajectory.daveml_import` from that installed wheel rather than an
editable source tree. Local wheel builds also clear their staging namespace
before copying declared sources, so removed DAVE-ML modules cannot survive in a
reused `build/` directory.

## Install from this checkout

Run from the repository root so the core dependency is supplied locally:

```bash
python -m pip install -e . -e packages/taoryx-daveml
taoryx plugins inspect taoryx.daveml --no-builtin
```

See the [installation guide](../../docs/INSTALLATION.md) for the complete model
suite, release wheels, and contributor setup.
