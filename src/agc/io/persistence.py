"""Persistence utilities for saving and loading AGC simulation runs and batch suites."""

import os
import json
from datetime import datetime
from pathlib import Path
from typing import Dict, Any, List, Optional
import numpy as np


def default_results_root(
    repository_root: Optional[str] = None,
    inplace_save: bool = False,
    timestamp: Optional[str] = None,
) -> Path:
    """Return the default result root for an in-place or timestamped run."""
    root = Path(repository_root) if repository_root is not None else Path(__file__).resolve().parent.parent.parent.parent
    if inplace_save:
        return root / "results" / "inplace"
    stamp = timestamp or datetime.now().strftime("%Y%m%d_%H%M%S")
    return root / "results" / "timestamped" / stamp


def _to_json_serializable(obj: Any) -> Any:
    """Convert nested structures with numpy arrays/scalars to native JSON types."""
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating, np.integer)):
        return obj.item()
    if isinstance(obj, dict):
        return {str(k): _to_json_serializable(v) for k, v in obj.items() if not callable(v)}
    if isinstance(obj, (list, tuple)):
        return [_to_json_serializable(x) for x in obj]
    if callable(obj):
        return None
    return obj


def save_run(
    save_dir: str,
    run: Dict[str, Any],
    metrics: Optional[Dict[str, float]],
    scenario: dict,
    failure: Optional[Dict[str, Any]] = None,
) -> None:
    """Save simulation run arrays to run.npz and structured run metadata to metadata.json."""
    os.makedirs(save_dir, exist_ok=True)
    npz_path = os.path.join(save_dir, "run.npz")

    # 1. Filter arrays and scalars for compressed npz
    arrays = {}
    for key, val in run.items():
        if isinstance(val, np.ndarray):
            arrays[key] = val
        elif isinstance(val, (int, float, str, bool)):
            arrays[key] = np.array(val)

    np.savez_compressed(npz_path, **arrays)

    # 2. Build structured metadata
    mode = str(scenario["controller"]["mode"]).lower()
    coriolis = str(scenario["controller"]["coriolis"]).lower()
    variant_name = f"{mode}_{coriolis}"

    payload_drop = scenario.get("payloadDrop")
    serializable_payload = _to_json_serializable(payload_drop) if payload_drop else None

    metadata = {
        "schemaVersion": "1.0",
        "variant": variant_name,
        "mode": mode,
        "coriolis": coriolis,
        "replayId": scenario.get("replayId", "unknown"),
        "timing": {
            "duration": float(scenario["duration"]),
            "dtPlant": float(scenario["dtPlant"]),
            "dtControl": float(scenario["dtControl"]),
            "dtAdaptation": float(scenario["dtAdaptation"]),
        },
        "controller": _to_json_serializable(scenario["controller"]),
        "plantPi": _to_json_serializable(scenario["plantPi"]),
        "plantGravity": _to_json_serializable(scenario["plantGravity"]),
        "payloadDrop": serializable_payload,
        "metrics": _to_json_serializable(metrics) if metrics else None,
        "failure": _to_json_serializable(failure) if failure else None,
        "completionStatus": "completed" if failure is None else "failed",
    }

    metadata_path = os.path.join(save_dir, "metadata.json")
    with open(metadata_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    # 3. Save backward-compatible metrics.txt if completed
    if metrics:
        metrics_path = os.path.join(save_dir, "metrics.txt")
        with open(metrics_path, "w", encoding="utf-8") as f:
            for k, v in metrics.items():
                f.write(f"{k}: {v}\n")


def load_run(save_dir: str) -> Dict[str, Any]:
    """Load simulation run data and metadata from directory."""
    npz_path = os.path.join(save_dir, "run.npz")
    if not os.path.isfile(npz_path):
        raise FileNotFoundError(f"Run file not found: {npz_path}")

    with np.load(npz_path, allow_pickle=True) as data:
        run = {key: data[key] for key in data.files}

    # Load metadata if present
    metadata_path = os.path.join(save_dir, "metadata.json")
    if os.path.isfile(metadata_path):
        with open(metadata_path, "r", encoding="utf-8") as f:
            metadata = json.load(f)
        run["metadata"] = metadata
        run["metrics"] = metadata.get("metrics")
        run["failure"] = metadata.get("failure")

    return run


def save_batch_suite(
    suite_dir: str,
    scenarios: List[dict],
    batch: Dict[str, Any],
) -> List[Dict[str, Any]]:
    """Save complete comparison batch suite with per-variant directories and manifest.json."""
    suite_path = Path(suite_dir)
    os.makedirs(suite_path, exist_ok=True)

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    variants_summary = []

    for i, scen in enumerate(scenarios):
        mode = scen["controller"]["mode"].lower()
        coriolis = scen["controller"]["coriolis"].lower()
        variant_name = f"{mode}_{coriolis}"

        run = batch["runs"][i]
        metrics = batch["metrics"][i]
        failure = batch["failures"][i]

        variant_dir = suite_path / variant_name
        save_run(str(variant_dir), run, metrics, scen, failure=failure)

        summary_entry = {
            "name": variant_name,
            "mode": mode,
            "coriolis": coriolis,
            "status": "completed" if failure is None else "failed",
            "positionRMSE": float(metrics["positionRMSE"]) if metrics else None,
            "attitudeRMSE": float(metrics["attitudeRMSE"]) if metrics else None,
            "finalSlidingNorm": float(metrics["finalSlidingNorm"]) if metrics else None,
            "failure": failure,
        }
        variants_summary.append(summary_entry)

    manifest = {
        "schemaVersion": "1.0",
        "timestamp": timestamp,
        "suiteDirectory": str(suite_path),
        "totalScenarios": len(scenarios),
        "successCount": sum(1 for v in variants_summary if v["status"] == "completed"),
        "variants": variants_summary,
    }

    manifest_path = suite_path / "manifest.json"
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    return variants_summary


def resolve_result_suite(path: Optional[str] = None) -> Path:
    """Resolve a results directory or latest suite directory containing manifest.json or run.npz."""
    root = Path(__file__).resolve().parent.parent.parent.parent
    timestamped_results = root / "results" / "timestamped"
    inplace_results = root / "results" / "inplace"

    if path:
        p = Path(path)
        if p.is_file():
            p = p.parent
        if (p / "manifest.json").is_file():
            return p
        if (p / "run.npz").is_file():
            return p.parent
        if p.is_dir():
            # Check subdirectories for manifest.json
            for sub in sorted(p.iterdir(), reverse=True):
                if sub.is_dir() and (sub / "manifest.json").is_file():
                    return sub
            return p
        raise FileNotFoundError(f"Specified result path does not exist: {path}")

    # Prefer the newest timestamped suite, then the in-place suite.
    if timestamped_results.is_dir():
        candidate_suites = [
            d for d in timestamped_results.iterdir()
            if d.is_dir() and (d / "manifest.json").is_file()
        ]
        if candidate_suites:
            candidate_suites.sort(key=lambda d: d.stat().st_mtime, reverse=True)
            return candidate_suites[0]

    if inplace_results.is_dir():
        if (inplace_results / "manifest.json").is_file():
            return inplace_results
        variants = [d for d in inplace_results.iterdir() if d.is_dir() and (d / "run.npz").is_file()]
        if variants:
            return inplace_results

    raise FileNotFoundError(
        f"No result suites found under {timestamped_results} or {inplace_results}"
    )
