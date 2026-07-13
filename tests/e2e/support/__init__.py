"""Support for the evidence-bounded TAOS v23 end-to-end corpus.

The package deliberately keeps runtime execution opt-in. Static corpus checks
use the live :mod:`taoryx.language` parser; historical execution requires an
external ``TAOS_EXE`` supplied by the caller.
"""
