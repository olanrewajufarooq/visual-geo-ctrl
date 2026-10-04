# Troubleshooting

Install the project with `pip install -e .[dev]` and verify imports with:

```bash
python -c "import vgc; from vgc.sim import default_scenario; print('Imports OK')"
```

If a run fails, try a short duration and `--no-pacing` first. The plant and controller rates must remain integer multiples of the plant step.
