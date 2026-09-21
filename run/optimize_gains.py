"""Run staged block-coordinate gain optimization for AGC UAV controllers in PyBullet."""

import sys
import argparse
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.opt.staged_optimizer import run_staged_optimization


def main():
    parser = argparse.ArgumentParser(description="Staged gain optimization for AGC UAV controller.")
    parser.add_argument(
        "--mode",
        type=str,
        default="all",
        choices=["nominal", "euclidean", "bregman", "all"],
        help="Controller mode to optimize",
    )
    parser.add_argument(
        "--coriolis",
        type=str,
        default="all",
        choices=["c1", "c2", "all"],
        help="Coriolis factorization form",
    )
    parser.add_argument(
        "--schedule",
        type=str,
        default="hierarchical",
        choices=["hierarchical", "classic"],
        help="Optimization schedule: hierarchical (7-stage) or classic (4-stage)",
    )
    parser.add_argument(
        "--method",
        type=str,
        default="pso",
        choices=["pso", "de"],
        help="Global search method: 'pso' (Particle Swarm) or 'de' (Differential Evolution)",
    )
    parser.add_argument(
        "--polish",
        action="store_true",
        help="Run local Nelder-Mead simplex polish on the final 'all' stage",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Simulation duration for candidate scoring (seconds)",
    )
    parser.add_argument(
        "--replay-id",
        type=str,
        default="lemniscate_01_auto",
        help="Trajectory benchmark ID",
    )
    parser.add_argument(
        "--swarm-size",
        type=int,
        default=50,
        help="Swarm size for PSO / population multiplier for DE",
    )
    parser.add_argument(
        "--max-iter",
        type=int,
        default=50,
        help="Maximum iterations per stage",
    )
    parser.add_argument(
        "--max-stall",
        type=int,
        default=15,
        help="Maximum iterations without improvement before early stopping",
    )
    parser.add_argument(
        "--no-parallel",
        action="store_true",
        help="Disable multi-core parallel particle evaluations",
    )
    parser.add_argument(
        "--no-promote",
        action="store_true",
        help="Do not update the gain registries if incumbent improves",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducible optimization search",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Custom output directory for optimization results and checkpoints",
    )
    parser.add_argument(
        "--inplace-save",
        dest="inplace_save",
        action="store_true",
        default=True,
        help="Save optimization checkpoints under results/inplace (default)",
    )
    parser.add_argument(
        "--timestamped-save",
        dest="inplace_save",
        action="store_false",
        help="Save optimization checkpoints under results/timestamped/<timestamp>",
    )
    args = parser.parse_args()

    mode_arg = None if args.mode == "all" else args.mode
    coriolis_arg = None if args.coriolis == "all" else args.coriolis

    run_staged_optimization(
        mode=mode_arg,
        coriolis=coriolis_arg,
        schedule=args.schedule,
        method=args.method,
        polish=args.polish,
        duration=args.duration,
        replay_id=args.replay_id,
        swarm_size=args.swarm_size,
        max_iter=args.max_iter,
        max_stall=args.max_stall,
        parallel=not args.no_parallel,
        promote=not args.no_promote,
        output_dir=args.output_dir,
        inplace_save=args.inplace_save,
        seed=args.seed,
    )


if __name__ == "__main__":
    main()
