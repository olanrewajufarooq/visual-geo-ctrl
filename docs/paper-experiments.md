# Paper experiments

The paper experiment runner uses the `agc` Conda environment and writes
publication artifacts to `results/papers/`:

```text
conda activate agc
python run/run_paper_experiments.py
python run/run_paper_experiments.py adaptive-drop
python run/run_paper_experiments.py nominal-connection
python run/run_paper_experiments.py nominal-reaching
python run/run_paper_experiments.py repeatability
```

The adaptive release benchmark uses a 10 s release, a 5 cm position
threshold, a 5 degree geodesic attitude threshold, and a 1 s recovery
dwell. The commanded wrench is reported; the current plant does not expose
rotor allocation or actuator saturation.
