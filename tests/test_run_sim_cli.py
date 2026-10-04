"""CLI defaults for standalone simulation payload behavior."""

import importlib.util
from pathlib import Path

import pytest


RUN_SIM = Path(__file__).resolve().parents[1] / "run" / "run_sim.py"
SPEC = importlib.util.spec_from_file_location("agc_run_sim_cli", RUN_SIM)
run_sim = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(run_sim)


@pytest.mark.parametrize(
    ("mode", "expected"),
    [("nominal", False), ("euclidean", True), ("bregman", True)],
)
def test_payload_defaults_follow_controller_mode(mode, expected):
    args = run_sim.build_parser().parse_args(["--mode", mode])
    assert run_sim.resolve_payload_enabled(mode, args.payload_enabled) is expected


@pytest.mark.parametrize(
    ("mode", "flag", "expected"),
    [
        ("nominal", "--payload", True),
        ("euclidean", "--no-payload", False),
        ("bregman", "--no-payload", False),
    ],
)
def test_payload_flag_overrides_mode_default(mode, flag, expected):
    args = run_sim.build_parser().parse_args(["--mode", mode, flag])
    assert run_sim.resolve_payload_enabled(mode, args.payload_enabled) is expected


def test_payload_override_flags_are_mutually_exclusive():
    with pytest.raises(SystemExit):
        run_sim.build_parser().parse_args(["--payload", "--no-payload"])
