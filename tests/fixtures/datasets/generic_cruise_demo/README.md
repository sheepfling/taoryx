# Generic Cruise Demo Dataset

This is a synthetic, non-flight-qualified source pack for exercising the
TAORYX dataset compiler and current 3-DOF runtime. Source files are long-form
CSV. The manifest is authoritative for axes, dependent columns, conventions,
and provenance; generated `.tbl` files are build products.

The pack intentionally retains force-coefficient effects for flaps and a
speed brake. It does not emit moments or actuator dynamics because the current
runtime is point-mass 3-DOF.
