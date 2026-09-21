"""Pytest test configuration and environment fixtures."""

import os
import shutil
import tempfile
from pathlib import Path
import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
LOCAL_TMP = REPO_ROOT / ".pytest_tmp"
LOCAL_TMP.mkdir(parents=True, exist_ok=True)

# Point standard tempfile to local temp directory to avoid Windows system temp permissions
os.environ["TMPDIR"] = str(LOCAL_TMP)
os.environ["TEMP"] = str(LOCAL_TMP)
os.environ["TMP"] = str(LOCAL_TMP)
tempfile.tempdir = str(LOCAL_TMP)


@pytest.fixture
def clean_tmp_dir(tmp_path):
    """Provide a verified writable temporary directory."""
    test_dir = tmp_path / "test_workspace"
    test_dir.mkdir(parents=True, exist_ok=True)
    yield test_dir
    try:
        shutil.rmtree(test_dir, ignore_errors=True)
    except Exception:
        pass
