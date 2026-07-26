# Incorporation architecture

```text
Exact DAVE-ML / NESC source
        ↓
Loss-preserving source IR
        ↓
Taoryx model collection and authority bindings
        ↓
Canonical SI / frame adapters
        ↓
3-DOF, pseudo-6DOF, or 6-DOF runtime compiler
        ↓
Scenario sidecar: environment, events, controls, applied loads
        ↓
Source checks + dimensional checks + NESC trajectory comparisons
        ↓
Qualified .txair package
```

A model family may produce several fidelity tiers from the same source authority. Reduced models must preserve their parent's accepted translational behavior rather than invent a second unrelated performance model.
