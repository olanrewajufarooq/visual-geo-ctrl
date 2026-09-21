#!/usr/bin/env python3
"""Download and extract RATM drone racing trajectory telemetry CSV files.

Downloads the split zip release assets from Drone-Racing/drone-racing-dataset (v3.0.0),
reassembles the archives, and extracts only the *_cam_ts_sync.csv and
*_500hz_freq_sync.csv trajectory files. Cross-platform Python implementation
replacing the legacy Windows CMD script.
"""

import argparse
import os
from pathlib import Path
import shutil
import sys
import tempfile
import urllib.request
import zipfile
from typing import List


RELEASE_TAG = "v3.0.0"
REPO_BASE = f"https://github.com/Drone-Racing/drone-racing-dataset/releases/download/{RELEASE_TAG}"

CHUNKS = {
    "autonomous": [f"autonomous_zipchunk0{i}" for i in range(1, 4)],
    "piloted": [f"piloted_zipchunk0{i}" for i in range(1, 8)],
}


def download_chunk(url: str, dest_path: Path) -> None:
    """Download a single chunk with progress display."""
    if dest_path.is_file() and dest_path.stat().st_size > 0:
        print(f"  Using cached chunk: {dest_path.name}")
        return

    dest_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = dest_path.with_suffix(".tmp")
    print(f"  Downloading: {url} -> {dest_path.name}...")

    def progress_callback(count, block_size, total_size):
        if total_size > 0:
            pct = count * block_size * 100 / total_size
            mb_downloaded = count * block_size / (1024 * 1024)
            mb_total = total_size / (1024 * 1024)
            sys.stdout.write(f"\r    {pct:.1f}% ({mb_downloaded:.1f} MB / {mb_total:.1f} MB)")
            sys.stdout.flush()

    try:
        urllib.request.urlretrieve(url, temp_path, reporthook=progress_callback)
        sys.stdout.write("\n")
        temp_path.replace(dest_path)
    except Exception as exc:
        if temp_path.is_file():
            temp_path.unlink()
        raise RuntimeError(f"Failed downloading {url}: {exc}") from exc


def reassemble_zip(chunk_paths: List[Path], out_zip: Path) -> None:
    """Binary concatenate split zip chunks into a valid zip archive."""
    print(f"  Reassembling {len(chunk_paths)} chunks -> {out_zip.name}...")
    out_zip.parent.mkdir(parents=True, exist_ok=True)
    with open(out_zip, "wb") as out_f:
        for chunk in chunk_paths:
            with open(chunk, "rb") as in_f:
                shutil.copyfileobj(in_f, out_f)


def extract_csvs(zip_path: Path, dest_dir: Path, group: str) -> int:
    """Extract only trajectory CSV files from the zip archive into dest_dir."""
    print(f"  Extracting telemetry CSVs to {dest_dir}...")
    dest_dir.mkdir(parents=True, exist_ok=True)
    count = 0
    with zipfile.ZipFile(zip_path, "r") as zf:
        for member in zf.namelist():
            if member.endswith("_cam_ts_sync.csv") or member.endswith("_500hz_freq_sync.csv"):
                # member path is typically "group/flight-xxx/file.csv"
                parts = Path(member).parts
                if len(parts) >= 2 and parts[0] == group:
                    rel_path = Path(*parts[1:])
                else:
                    rel_path = Path(*parts)
                target = dest_dir / rel_path
                target.parent.mkdir(parents=True, exist_ok=True)
                with zf.open(member) as src, open(target, "wb") as dst:
                    shutil.copyfileobj(src, dst)
                count += 1
    print(f"  Extracted {count} CSV files for {group}.")
    return count


def process_group(
    group: str,
    dest_root: Path,
    cache_root: Path,
    force_clean: bool = False,
) -> None:
    target_dir = dest_root / group
    if target_dir.is_dir() and any(target_dir.iterdir()):
        if not force_clean:
            print(f"Destination already contains data in '{target_dir}'. Re-run with --force-clean to replace it.")
            return
        print(f"Removing existing target: {target_dir}")
        shutil.rmtree(target_dir)

    group_cache = cache_root / group
    group_cache.mkdir(parents=True, exist_ok=True)

    chunk_files = []
    for chunk_name in CHUNKS[group]:
        chunk_url = f"{REPO_BASE}/{chunk_name}"
        chunk_path = group_cache / chunk_name
        download_chunk(chunk_url, chunk_path)
        chunk_files.append(chunk_path)

    zip_path = cache_root / f"{group}.zip"
    reassemble_zip(chunk_files, zip_path)
    extract_csvs(zip_path, target_dir, group)


def main():
    parser = argparse.ArgumentParser(
        description="Download and extract RATM drone racing trajectory CSV files."
    )
    parser.add_argument(
        "mode",
        nargs="?",
        choices=["all", "autonomous", "piloted"],
        default="all",
        help="Trajectory group to download: all (default), autonomous, or piloted.",
    )
    parser.add_argument(
        "--clean-mode",
        choices=["safe", "force-clean"],
        default="safe",
        help="Whether to fail on existing data ('safe', default) or replace it ('force-clean').",
    )
    parser.add_argument(
        "--force-clean",
        action="store_true",
        help="Shorthand for --clean-mode force-clean.",
    )
    args = parser.parse_args()

    force_clean = args.force_clean or (args.clean_mode == "force-clean")
    dest_root = Path(__file__).resolve().parent
    cache_root = dest_root / "archive"

    print(
        f"[ratm] Downloading RATM flight datasets (mode={args.mode}, "
        f"clean_mode={'force-clean' if force_clean else 'safe'})"
    )

    groups = ["autonomous", "piloted"] if args.mode == "all" else [args.mode]
    for grp in groups:
        print(f"\n--- Processing {grp} ---")
        process_group(grp, dest_root, cache_root, force_clean=force_clean)

    print("\nTrajectory CSV download complete.")


if __name__ == "__main__":
    main()
