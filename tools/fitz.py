from __future__ import annotations

import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("_taoryx_fitz", ROOT / "fitz.py")
if SPEC is None or SPEC.loader is None:
    raise ImportError("Could not load local fitz compatibility shim.")
####
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

Matrix = MODULE.Matrix
Pixmap = MODULE.Pixmap
Page = MODULE.Page
Document = MODULE.Document
open = MODULE.open
####
