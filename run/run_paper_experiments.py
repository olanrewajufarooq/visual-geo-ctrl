"""Run the reproducible paper experiments and export tracked artifacts."""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))

from agc.io.persistence import save_run
from agc.paper.diagnostics import connection_identity
from agc.sim.default_scenario import default_scenario
from agc.sim.metrics import compute_metrics
from agc.sim.paper_metrics import (
    compute_nominal_reaching_bound,
    compute_reaching_time,
    compute_recovery_time,
    compute_window_metrics,
)
from agc.sim.run_scenario import run_scenario
from agc.viz.diagnostics import derive_diagnostics
from agc.viz.paper_experiment_figures import export_adaptive_figures, export_nominal_figures


def _paper_root(path: str | None) -> Path:
    return Path(path) if path else REPO_ROOT / "results" / "papers"


def _raw_root(path: str | None) -> Path:
    return Path(path) if path else REPO_ROOT / "results" / "paper-runs"


def _run_variant(mode: str, coriolis: str, duration: float, payload_enabled: bool, raw_root: Path):
    scenario = default_scenario(
        mode=mode,
        coriolis=coriolis,
        duration=duration,
        payload_enabled=payload_enabled,
        enable_pacing=False,
        gui=False,
    )
    run, failure = run_scenario(scenario)
    if failure is not None:
        raise RuntimeError(f"{mode}_{coriolis} failed at {failure['time']}: {failure['message']}")
    label = f"{mode}_{coriolis}"
    metrics = compute_metrics(run)
    save_run(str(raw_root / label), run, metrics, scenario)
    return run, scenario, metrics


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    keys = list(rows[0])
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def _write_manifest(root: Path, command: str, duration: float) -> None:
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = "unknown"
    root.mkdir(parents=True, exist_ok=True)
    with (root / "manifest.json").open("w", encoding="utf-8") as stream:
        json.dump({"experiment": command, "duration": duration, "commit": commit, "environment": "agc", "factorizations": ["lc", "rb"]}, stream, indent=2)


def adaptive_drop(duration: float, paper_root: Path, raw_root: Path) -> None:
    labels = [("nominal", "lc"), ("euclidean", "lc"), ("bregman", "lc")]
    runs: Dict[str, dict] = {}; scenarios: Dict[str, dict] = {}; rows = []
    for mode, form in labels:
        run, scenario, _ = _run_variant(mode, form, duration, True, raw_root)
        label = f"{mode}_{form}"; runs[label] = run; scenarios[label] = scenario
        pre = compute_window_metrics(run, 0.0, min(10.0, duration))
        post_start = min(12.0, duration)
        post = compute_window_metrics(run, post_start, duration)
        recovery = compute_recovery_time(run, 10.0) if duration > 10.0 else None
        rows.append({"controller": label, "positionRMSE": post["positionRMSE"], "attitudeRMSE": post["attitudeRMSE"], "peakReleasePosition": pre["maxPositionError"], "recoveryTime": recovery, "wrenchEffort": post["integratedSquaredWrench"], "minimumPseudoEigenvalue": compute_metrics(run)["minimumPseudoEigenvalue"]})
    export_adaptive_figures(runs, scenarios, str(paper_root / "figures"))
    _write_csv(paper_root / "tables" / "adaptive_tracking.csv", rows)
    _write_csv(paper_root / "tables" / "baseline_comparison.csv", rows)
    _write_manifest(paper_root, "adaptive-drop", duration)


def nominal_reaching(duration: float, paper_root: Path, raw_root: Path) -> None:
    run, scenario, _ = _run_variant("nominal", "lc", duration, False, raw_root)
    pi = scenario["plantPi"]
    inertia = np.asarray(__import__("agc.math.inertia", fromlist=["inertia_from_pi"]).inertia_from_pi(pi))
    lambda_s = np.linalg.inv(scenario["controller"]["Lambda"])
    bound = compute_nominal_reaching_bound(run, inertia, lambda_s, scenario["controller"]["ks"], scenario["controller"]["alpha"])
    observed = compute_reaching_time(run, threshold=1e-3, dwell_time=0.5)
    export_nominal_figures(run, scenario, str(paper_root / "figures"), bound)
    _write_csv(paper_root / "tables" / "nominal_theory.csv", [{"observedReachingTime": observed, "theoreticalBound": bound}])
    _write_manifest(paper_root, "nominal-reaching", duration)


def nominal_connection(duration: float, paper_root: Path, raw_root: Path) -> None:
    run, scenario, _ = _run_variant("nominal", "lc", duration, False, raw_root)
    residuals = []
    for index in range(0, len(run["t"]), max(1, len(run["t"]) // 200)):
        state = {"H": run["H"][index], "V": run["V"][index]}
        desired = {"H": run["Hdesired"][index], "V": run["Vdesired"][index], "Vdot": np.zeros(6)}
        residuals.append(connection_identity(state, desired, scenario["controller"], scenario["plantPi"])["residualNorm"])
    export_nominal_figures(run, scenario, str(paper_root / "figures"), connection_residual=residuals)
    _write_csv(paper_root / "tables" / "nominal_connection.csv", [{"maxConnectionResidual": max(residuals, default=float("nan"))}])
    _write_manifest(paper_root, "nominal-connection", duration)


def repeatability(duration: float, paper_root: Path, raw_root: Path) -> None:
    rows = []
    for trial in range(5):
        run, _, metrics = _run_variant("bregman", "lc", duration, True, raw_root / f"trial_{trial + 1}")
        rows.append({"trial": trial + 1, "positionRMSE": metrics["positionRMSE"], "attitudeRMSE": metrics["attitudeRMSE"], "wrenchRMS": metrics["wrenchRMS"]})
    _write_csv(paper_root / "tables" / "repeatability.csv", rows)
    _write_manifest(paper_root, "repeatability", duration)


def run_all(duration: float, paper_root: Path, raw_root: Path) -> None:
    """Run the complete paper experiment suite in a deterministic order."""
    adaptive_drop(duration, paper_root, raw_root)
    nominal_connection(duration, paper_root, raw_root)
    nominal_reaching(duration, paper_root, raw_root)
    repeatability(duration, paper_root, raw_root)
    _write_manifest(paper_root, "all", duration)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("command", nargs="?", default="all", choices=["all", "adaptive-drop", "nominal-connection", "nominal-reaching", "repeatability"])
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--raw-output-dir", default=None)
    args = parser.parse_args()
    paper_root = _paper_root(args.output_dir); raw_root = _raw_root(args.raw_output_dir)
    if args.command == "all": run_all(args.duration, paper_root, raw_root)
    elif args.command == "adaptive-drop": adaptive_drop(args.duration, paper_root, raw_root)
    elif args.command == "nominal-connection": nominal_connection(args.duration, paper_root, raw_root)
    elif args.command == "nominal-reaching": nominal_reaching(args.duration, paper_root, raw_root)
    else: repeatability(args.duration, paper_root, raw_root)


if __name__ == "__main__":
    main()
