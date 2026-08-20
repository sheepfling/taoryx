# CADAC package documentation

These notes are owned by `taoryx-cadac`. They describe source-bound CADAC
family behavior, conversion, controls, and explicit nonclaims; they do not
define a reusable TAORYX API.

Start with the [package README](../README.md) for installation and the focused
developer loop. The shared provider/control/result contracts remain in the
repository [API reference](../../../docs/api/README.md).

## Package guides

- [Source-backed family integration](cadac-porting.md)
- [Table normalization](cadac-tables.md)
- [Missile tuning and reduced-order authoring](cadac-missile-tuning.md)
- [ADS6 source-controller path](cadac-ads6-source-controller.md)

## Model and composition notes

- [AGM6](cadac-agm6.md), [FALCON6](cadac-falcon6.md),
  [GHAME6](cadac-ghame6.md), [ROCKET6G](cadac-rocket6g.md), and
  [SRAAM6](cadac-sraam6.md)
- [ADS6 AIRCRAFT3](cadac-ads6-aircraft.md),
  [ADS6 SAM](cadac-ads6-sam.md), [ADS6 SRBM](cadac-ads6-srbm.md), and
  [ADS6 engagement](cadac-ads6-engagement.md)

Run `python tools/dev.py plugin-focus cadac` to print the package-local
contract, source-discovery, and selected vehicle gates.
