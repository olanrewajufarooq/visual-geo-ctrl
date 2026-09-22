#!/usr/bin/env python3
"""Build canonical replay artifacts for recorded flight trajectories.

Replaces the legacy process_trajectories.m with native Python .npz processing.
"""

import argparse
import sys
from pathlib import Path

# Add repo root to sys.path so trajectories package is discoverable
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from trajectories.replay_scripts.replay_processor import ReplayProcessor


def main():
    parser = argparse.ArgumentParser(
        description="Build canonical SE(3) replay artifacts for recorded flight trajectories."
    )
    parser.add_argument(
        "--clear-cache",
        action="store_true",
        help="Rebuild artifacts even if cached files are up-to-date.",
    )
    parser.add_argument(
        "--format",
        type=str,
        choices=["npz", "mat"],
        default="npz",
        help="Output artifact format: 'npz' (default, pure Python) or 'mat' (legacy MATLAB).",
    )
    parser.add_argument(
        "--ids",
        nargs="*",
        default=[
            "lemniscate_01_auto",
            "lemniscate_02_auto",
            "lemniscate_03_auto",
            "lemniscate_04_auto",
        ],
        help="Specific trajectory IDs to process (default: lemniscate_01_auto through lemniscate_04_auto).",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Process all trajectory IDs defined in manifest.json.",
    )
    parser.add_argument(
        "--full-precision",
        action="store_true",
        help="Use the legacy 10 ms, 200-iteration WNOJ configuration; substantially slower.",
    )
    args = parser.parse_args()

    traj_ids = None if args.all else args.ids
    postprocessing_options = None if args.full_precision else {
        "knotIntervalSeconds": 0.25,
        "maxIterations": 10,
        "maxDampingTrials": 4,
    }

    print("[replay] Starting trajectory preprocessing.")
    summary = ReplayProcessor.process_all(
        clear_cache=args.clear_cache,
        trajectory_ids=traj_ids,
        output_format=args.format,
        postprocessing_options=postprocessing_options,
    )
    print(f"Processed {summary['processedCount']} replay trajectories ({summary['cachedCount']} cached).")


if __name__ == "__main__":
    main()
