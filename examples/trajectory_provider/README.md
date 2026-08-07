# Product 3 provider example

`product_three_reference.py` is a complete provider plug-in walkthrough:

1. publish provider metadata and two vehicle models;
2. inspect initialization, capability, channel, and segment contracts;
3. prepare a typed request with canonical units and a variable-length segment sequence;
4. execute the selected model; and
5. emit the standard `taoryx.product-three-trajectory/v1` result envelope.

Run it from the repository root after installing the package, or with
`PYTHONPATH=src` in a source checkout:

```bash
PYTHONPATH=src python3 examples/trajectory_provider/product_three_reference.py \
  --output /tmp/product-three-trajectory.json
```

The two advertised models are intentionally analytical fixtures. They show
the shape of a source-owned provider without claiming historical TAOS
compatibility, vehicle fidelity, or qualification evidence.
