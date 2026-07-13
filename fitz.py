from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Any

from pypdf import PdfReader


class Matrix:
    def __init__(self, x: float = 1.0, y: float | None = None) -> None:
        self.x = float(x)
        self.y = float(x if y is None else y)
    ####


class Pixmap:
    def __init__(self, pdf_path: Path, page_number: int, matrix: Matrix | None) -> None:
        self._pdf_path = pdf_path
        self._page_number = page_number
        self._matrix = matrix or Matrix()
    ####

    def save(self, path: str | Path) -> None:
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        scale = max(self._matrix.x, self._matrix.y, 1.0)
        dpi = max(int(round(72.0 * scale)), 1)
        with tempfile.TemporaryDirectory(prefix="taoryx-pdftoppm-") as tmpdir:
            prefix = Path(tmpdir) / "page"
            command = [
                "pdftoppm",
                "-f",
                str(self._page_number),
                "-l",
                str(self._page_number),
                "-singlefile",
                "-png",
                "-r",
                str(dpi),
                str(self._pdf_path),
                str(prefix),
            ]
            subprocess.run(command, check=True, capture_output=True)
            generated = prefix.with_suffix(".png")
            shutil.move(str(generated), str(output_path))
        ####
    ####


class Page:
    def __init__(self, reader: PdfReader, index: int, pdf_path: Path) -> None:
        self._reader = reader
        self._index = index
        self._pdf_path = pdf_path
    ####

    def get_text(self, mode: str = "text", **_: Any) -> str:
        if mode not in {"text", "plain"}:
            raise ValueError(f"Unsupported text extraction mode: {mode!r}")
        ####
        return self._reader.pages[self._index].extract_text() or ""
    ####

    def get_pixmap(self, matrix: Matrix | None = None, alpha: bool = False, **_: Any) -> Pixmap:
        if alpha:
            raise ValueError("Alpha channels are not supported by the local fitz shim.")
        ####
        return Pixmap(self._pdf_path, self._index + 1, matrix)
    ####


class Document:
    def __init__(self, pdf_path: str | Path) -> None:
        self._path = Path(pdf_path)
        self._reader = PdfReader(str(self._path))
    ####

    def __enter__(self) -> "Document":
        return self
    ####

    def __exit__(self, exc_type, exc, traceback) -> None:
        return None
    ####

    def close(self) -> None:
        return None
    ####

    def __len__(self) -> int:
        return len(self._reader.pages)
    ####

    @property
    def page_count(self) -> int:
        return len(self)
    ####

    def __getitem__(self, index: int) -> Page:
        if index < 0:
            index += len(self)
        ####
        if index < 0 or index >= len(self):
            raise IndexError(index)
        ####
        return Page(self._reader, index, self._path)
    ####

    def get_toc(self, simple: bool = False) -> list[Any]:
        outline = getattr(self._reader, "outline", None)
        if outline is None:
            outline = getattr(self._reader, "outlines", [])
        ####
        rows: list[Any] = []

        def walk(items: list[Any], level: int) -> None:
            for item in items:
                if isinstance(item, list):
                    walk(item, level + 1)
                    continue
                ####
                title = getattr(item, "title", None)
                if title is None:
                    continue
                ####
                page_number = self._reader.get_destination_page_number(item) + 1
                rows.append([level, title, page_number] if simple else [level, title, page_number, item])
            ####
        ####

        walk(list(outline), 1)
        return rows
    ####


def open(path: str | Path, *_, **__) -> Document:
    return Document(path)
####
