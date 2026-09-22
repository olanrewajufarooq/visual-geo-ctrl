"""Asset resolution helper for simulation models and environment scenes.

Resolves static assets across:
1. Installed package data (`src/agc/assets/...`)
2. Local repository checkout (`assets/...`)
"""

from pathlib import Path


def get_asset_path(*subpaths: str) -> Path:
    """Resolve an asset file path, searching package data first, then repository root."""
    rel = Path(*subpaths)

    # 1. Check packaged assets inside agc.assets
    pkg_assets_dir = Path(__file__).resolve().parent.parent / "assets"
    candidate = pkg_assets_dir / rel
    if candidate.exists():
        return candidate

    # 2. Check repository root assets
    repo_root = Path(__file__).resolve().parents[3]
    candidate = repo_root / "assets" / rel
    if candidate.exists():
        return candidate

    # 3. Fallback to package location
    return pkg_assets_dir / rel
