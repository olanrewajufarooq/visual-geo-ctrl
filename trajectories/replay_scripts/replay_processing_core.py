"""Core preprocessing pipeline for canonical replay trajectory artifacts."""

from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import os
from pathlib import Path
import time
from typing import Dict, Any, List, Optional, Union
import numpy as np

from .replay_kinematics import ReplayKinematics
from .write_replay_artifact import write_replay_artifact


class ReplayProcessingCore:
    """Build and load canonical replay trajectory artifacts."""

    @staticmethod
    def default_root_dir() -> Path:
        """Return the default processed trajectory root directory."""
        this_dir = Path(__file__).resolve().parent
        return this_dir.parent / "processed"

    @staticmethod
    def load_manifest(manifest_path: Union[str, Path]) -> Dict[str, Any]:
        manifest_path = Path(manifest_path)
        if not manifest_path.is_file():
            raise FileNotFoundError(f"Replay manifest '{manifest_path}' does not exist.")
        with open(manifest_path, "r", encoding="utf-8") as f:
            return json.load(f)

    @staticmethod
    def resolve_source_path(root_dir: Path, source_file: str) -> Path:
        """Resolve source file relative to processed/ first, then trajectories/."""
        source_path = root_dir / source_file
        if source_path.is_file():
            return source_path
        trajectories_root = root_dir.parent
        source_path = trajectories_root / source_file
        return source_path

    @staticmethod
    def is_cache_valid(artifact_path: Path, raw_path: Path, entry: Dict[str, Any]) -> bool:
        if not artifact_path.is_file():
            return False
        if raw_path.is_file():
            if raw_path.stat().st_mtime > artifact_path.stat().st_mtime:
                return False
        return True

    @staticmethod
    def process_all(
        root_dir: Optional[Union[str, Path]] = None,
        manifest_path: Optional[Union[str, Path]] = None,
        use_parallel: bool = True,
        clear_cache: bool = False,
        trajectory_ids: Optional[List[str]] = None,
        output_format: str = "npz",
        postprocessing_options: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Convert manifest-listed raw CSV files into .npz (or .mat) artifacts."""
        root_dir = Path(root_dir) if root_dir else ReplayProcessingCore.default_root_dir()
        root_dir.mkdir(parents=True, exist_ok=True)

        if manifest_path is None:
            manifest_path = root_dir / "manifest.json"
        manifest_path = Path(manifest_path)

        manifest = ReplayProcessingCore.load_manifest(manifest_path)
        all_ids = list(manifest.keys())
        ids = ReplayProcessingCore.select_manifest_ids(all_ids, trajectory_ids)

        n = len(ids)
        raw_paths = []
        out_paths = []
        entries = []

        for tid in ids:
            entry = dict(manifest[tid])
            if postprocessing_options is not None:
                postprocessing = dict(entry.get("postprocessing", {}))
                wnoj_options = dict(postprocessing.get("wnoj", {}))
                wnoj_options.update(postprocessing_options)
                postprocessing["wnoj"] = wnoj_options
                entry["postprocessing"] = postprocessing
            entries.append(entry)
            raw_path = ReplayProcessingCore.resolve_source_path(root_dir, entry["source_file"])
            raw_paths.append(raw_path)

            ext = f".{output_format.lower().lstrip('.')}"
            artifact_file = entry.get("artifact_file", f"{tid}{ext}")
            out_file = Path(artifact_file).with_suffix(ext)
            out_path = root_dir / out_file
            out_paths.append(out_path)

        print(f"[replay] Processing {n} trajectories from {manifest_path}")

        cached = [False] * n
        if not clear_cache:
            for i in range(n):
                cached[i] = ReplayProcessingCore.is_cache_valid(out_paths[i], raw_paths[i], entries[i])

        pending = [i for i in range(n) if not cached[i]]
        elapsed = [0.0] * n
        errors = [None] * n

        if pending:
            print(f"[replay] Preprocessing {len(pending)} pending trajectories...")
            for idx in pending:
                print(f"[replay {idx+1}/{n}] Starting {ids[idx]}...", flush=True)
                t0 = time.perf_counter()
                try:
                    traj = ReplayKinematics.process_single(raw_paths[idx], ids[idx], entries[idx])
                    write_replay_artifact(out_paths[idx], traj, output_format=output_format)
                except Exception as exc:
                    errors[idx] = str(exc)
                elapsed[idx] = time.perf_counter() - t0
                if errors[idx] is None:
                    print(f"[replay {idx+1}/{n}] Finished {ids[idx]} in {elapsed[idx]:.1f} s", flush=True)
        else:
            print("[replay] Cache hit for all trajectories.")

        failed = []
        for i in range(n):
            print(f"[replay {i+1}/{n}] {ids[i]}")
            print(f"  source: {raw_paths[i]}")
            if cached[i]:
                print(f"  cache: reused {out_paths[i]}")
            elif errors[i] is None:
                print(f"  artifact: {out_paths[i]} ({elapsed[i]:.1f} s)")
            else:
                failed.append(ids[i])
                print(f"  FAILED after {elapsed[i]:.1f} s: {errors[i]}")

        if failed:
            raise RuntimeError(f"Replay preprocessing failed for: {', '.join(failed)}")

        summary = {
            "processedCount": sum(1 for c in cached if not c),
            "cachedCount": sum(1 for c in cached if c),
            "totalCount": n,
            "trajectoryIds": ids,
            "manifestPath": str(manifest_path),
            "outputFormat": output_format,
            "clearCache": clear_cache,
        }
        print(f"[replay] Completed {summary['totalCount']} trajectories "
              f"({summary['processedCount']} processed, {summary['cachedCount']} cached).")
        return summary

    @staticmethod
    def select_manifest_ids(all_ids: List[str], requested_ids: Optional[Union[str, List[str]]]) -> List[str]:
        if not requested_ids:
            return all_ids
        if isinstance(requested_ids, str):
            requested_ids = [requested_ids]
        unknown = [tid for tid in requested_ids if tid not in all_ids]
        if unknown:
            raise KeyError(f"Unknown replay trajectory IDs: {', '.join(unknown)}")
        return [tid for tid in requested_ids if tid in all_ids]

    @staticmethod
    def resolve_replay_config(replay_cfg: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        if isinstance(replay_cfg, str):
            replay_id = replay_cfg
            root_dir = ReplayProcessingCore.default_root_dir()
            manifest_path = root_dir / "manifest.json"
        elif isinstance(replay_cfg, dict):
            if "id" not in replay_cfg:
                raise KeyError("cfg.traj.replay must contain 'id'.")
            replay_id = str(replay_cfg["id"])
            root_dir = Path(replay_cfg.get("rootDir", ReplayProcessingCore.default_root_dir()))
            manifest_path = Path(replay_cfg.get("manifestFile", root_dir / "manifest.json"))
        else:
            raise TypeError("replay_cfg must be a string ID or dictionary.")

        return {
            "id": replay_id,
            "rootDir": root_dir,
            "manifestPath": manifest_path,
        }

    @staticmethod
    def load_entry(replay_cfg: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        resolved = ReplayProcessingCore.resolve_replay_config(replay_cfg)
        manifest = ReplayProcessingCore.load_manifest(resolved["manifestPath"])
        replay_id = resolved["id"]
        if replay_id not in manifest:
            raise KeyError(f"Replay id '{replay_id}' not found in manifest '{resolved['manifestPath']}'.")

        entry = dict(manifest[replay_id])
        entry["id"] = replay_id
        entry["rootDir"] = str(resolved["rootDir"])
        entry["manifestPath"] = str(resolved["manifestPath"])

        # Check .npz then .mat
        npz_candidate = resolved["rootDir"] / f"{replay_id}.npz"
        mat_candidate = resolved["rootDir"] / f"{replay_id}.mat"
        if npz_candidate.is_file():
            entry["artifact_path"] = str(npz_candidate)
        elif mat_candidate.is_file():
            entry["artifact_path"] = str(mat_candidate)
        else:
            entry["artifact_path"] = str(resolved["rootDir"] / entry.get("artifact_file", f"{replay_id}.npz"))
        return entry

    @staticmethod
    def load_artifact(replay_cfg: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        from vgc.sim.replay_trajectory import ReplayTrajectory

        entry = ReplayProcessingCore.load_entry(replay_cfg)
        sampler = ReplayTrajectory(entry["artifact_path"])
        return sampler.traj

    @staticmethod
    def default_axis_limits(cfg: Any, pad: float = 2.0) -> List[float]:
        try:
            traj = ReplayProcessingCore.load_artifact(cfg)
            p = np.asarray(traj["p"], dtype=float)
            mins = np.min(p, axis=0) - pad
            maxs = np.max(p, axis=0) + pad
            z_min = min(float(mins[2]), 0.0)
            z_max = max(float(maxs[2]), 0.0)
            return [float(mins[0]), float(maxs[0]), float(mins[1]), float(maxs[1]), z_min, z_max]
        except Exception:
            return [-5.0, 5.0, -5.0, 5.0, 0.0, 5.0]
