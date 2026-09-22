"""Asset resolution helper for simulation models and environment scenes.

Resolves static assets across repository checkout (`assets/...`).
"""

from pathlib import Path


def get_asset_path(*subpaths: str) -> Path:
    """Resolve an asset file path from the repository root assets directory."""
    rel = Path(*subpaths)

    # 1. Check repository root assets
    repo_root = Path(__file__).resolve().parents[3]
    candidate = repo_root / "assets" / rel
    if candidate.exists():
        return candidate

    # 2. Check packaged assets inside agc.assets if installed
    pkg_assets_dir = Path(__file__).resolve().parent.parent / "assets"
    candidate = pkg_assets_dir / rel
    if candidate.exists():
        return candidate

    # 3. Fallback to repo root assets location
    return repo_root / "assets" / rel
