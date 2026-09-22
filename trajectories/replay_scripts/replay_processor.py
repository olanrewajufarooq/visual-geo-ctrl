"""Public compatibility facade for replay trajectory processing."""

from pathlib import Path
from typing import Dict, Any, List, Optional, Union

from .replay_processing_core import ReplayProcessingCore


class ReplayProcessor:
    """Public facade for replay trajectory processing and artifact loading."""

    @staticmethod
    def process_all(
        clear_cache: bool = False,
        trajectory_ids: Optional[List[str]] = None,
        root_dir: Optional[Union[str, Path]] = None,
        manifest_path: Optional[Union[str, Path]] = None,
        use_parallel: bool = True,
        output_format: str = "npz",
        postprocessing_options: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """Convert manifest-listed raw CSV flight files into canonical artifacts."""
        return ReplayProcessingCore.process_all(
            root_dir=root_dir,
            manifest_path=manifest_path,
            use_parallel=use_parallel,
            clear_cache=clear_cache,
            trajectory_ids=trajectory_ids,
            output_format=output_format,
            postprocessing_options=postprocessing_options,
        )

    @staticmethod
    def load_entry(replay_cfg: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """Resolve one replay manifest entry from configuration or trajectory ID."""
        return ReplayProcessingCore.load_entry(replay_cfg)

    @staticmethod
    def load_artifact(replay_cfg: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
        """Load a processed replay artifact dictionary from configuration or trajectory ID."""
        return ReplayProcessingCore.load_artifact(replay_cfg)

    @staticmethod
    def default_axis_limits(cfg: Any, pad: float = 2.0) -> List[float]:
        """Compute spatial visualization limits for replay trajectories."""
        return ReplayProcessingCore.default_axis_limits(cfg, pad=pad)

    @staticmethod
    def default_root_dir() -> Path:
        """Return the default processed trajectory root directory."""
        return ReplayProcessingCore.default_root_dir()
