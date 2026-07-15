# Runtime inputs

This fixture demonstrates the three problem-file input surfaces:

* `parameter`: setup-time values such as initial mass or controller gains;
* `control`: values supplied by a player, controller, or time-stepper consumer;
* `status`: values selected for inspection and telemetry.

`moving-target-x` is a mutable runtime parameter representing an externally
updated target input. It is intentionally marked `mutable=true`; ordinary
setup parameters remain immutable after load.

```python
from taoryx.runtime import LoadedProgram

program = LoadedProgram.load(
    "mission.prb",
    parameter_overrides={"initial-mass": 125.0, "guidance-gain": 2.5},
)
program.set_control("throttle", 0.8, vehicle="1")
program.set_parameter("moving-target-x", 1500.0)
print(program.inspect()["parameters"])
print(program.observe(vehicle="1").as_dict())
```

The runtime does not prescribe how a model consumes these values. Lowered
derivatives receive controls through the named state view and parameters
through the model parameter mapping.
