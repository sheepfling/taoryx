# taoryx-daveml

Optional DAVE-ML support for Taoryx. This distribution owns the DAVE-ML
import, semantic IR, evaluator, compatibility overlay, collection, and replay
modules. Installing it publishes the `daveml` model-format contribution
through the versioned `taoryx.plugins` entry-point group.

The implementation occupies the shared `taoryx.trajectory` namespace so
historical imports remain valid without placing DAVE-ML code in the core
Taoryx wheel.

## Install from this checkout

Run from the repository root so the core dependency is supplied locally:

```bash
python -m pip install -e . -e packages/taoryx-daveml
taoryx plugins inspect taoryx.daveml --no-builtin
```

See the [installation guide](../../docs/INSTALLATION.md) for the complete model
suite, release wheels, and contributor setup.
