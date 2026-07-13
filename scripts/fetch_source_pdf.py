"""Fetch the TAOS 1995 source PDF into a local cache.

The scan is intentionally stored outside version control. By default the
script downloads to `.cache/taoryx/TAOS_manual_1995.pdf`. Pass `--link-root`
to create the ignored `TAOS_manual_1995.pdf` symlink in the repository root.
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URLS = [
    "https://digital.library.unt.edu/ark:/67531/metadc623738/m2/1/high_res_d/162896.pdf",
    "https://archive.org/download/taos-users-manual-1995/taos-users-manual-1995.pdf",
]
CACHE_PATH = ROOT / ".cache" / "taoryx" / "TAOS_manual_1995.pdf"
ROOT_LINK = ROOT / "TAOS_manual_1995.pdf"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
        ####
    ####
    return digest.hexdigest()
####


def download(url: str, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".part")
    if temporary.exists():
        temporary.unlink()
    ####
    print(f"Downloading {url}")
    with urllib.request.urlopen(url) as response, temporary.open("wb") as handle:
        shutil.copyfileobj(response, handle)
    ####
    temporary.replace(destination)
####


def ensure_link(target: Path) -> None:
    if target.exists() or target.is_symlink():
        target.unlink()
    ####
    target.symlink_to(CACHE_PATH)
####


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--url",
        action="append",
        help="source URL to try before the defaults",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=CACHE_PATH,
        help="destination file path for the cached PDF",
    )
    parser.add_argument(
        "--link-root",
        action="store_true",
        help="create the repository-root TAOS_manual_1995.pdf symlink after download",
    )
    return parser.parse_args()
####


def main() -> int:
    args = parse_args()
    urls = [*(args.url or []), *DEFAULT_URLS]
    destination = args.output
    if destination.exists():
        print(f"Using existing {destination}")
    else:
        last_error: Exception | None = None
        for url in urls:
            try:
                download(url, destination)
                last_error = None
                break
            except Exception as error:  # pragma: no cover - network dependent
                last_error = error
                print(f"Failed to fetch from {url}: {error}")
            ####
        ####
        if last_error is not None:
            raise SystemExit(f"Unable to fetch source PDF: {last_error}")
        ####
    ####
    size_mb = destination.stat().st_size / (1024 * 1024)
    print(f"Cached {destination} ({size_mb:.1f} MiB, sha256 {sha256_file(destination)})")
    if args.link_root:
        ensure_link(ROOT_LINK)
        print(f"Linked {ROOT_LINK} -> {destination.relative_to(ROOT)}")
    ####
    return 0
####


if __name__ == "__main__":
    raise SystemExit(main())
####
