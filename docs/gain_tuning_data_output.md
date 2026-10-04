# Gain-Tuning Data Output

The optimizer saves under `results/optimization/best-gain/` by default. Pass `--timestamped-save` to isolate artifacts under `results/optimization/timestamped/<timestamp>/`, or `--output-dir` to select a custom artifact root.

## Output layout

Each selected mode/factorization pair has an independent artifact directory and incumbent file:

```text
results/optimization/best-gain/
  nominal_lc.json
  nominal_rb.json
  euclidean_lc.json
  euclidean_rb.json
  bregman_lc.json
  bregman_rb.json
  nominal_lc/optimization.json
  nominal_rb/optimization.json
  ...
```

`<mode>_<coriolis>/optimization.json` records the selected mode, factorization, schedule, method, training conditions, incumbent, and stage history. It is rewritten as stages complete. The root-level `<mode>_<coriolis>.json` stores the incumbent candidate used for future comparisons. Optimization and promotion operate only on that matching pair; an LC result never replaces or changes the RB entry, and changing one controller mode does not propagate tracking gains to another.

The optimizer re-evaluates the saved candidate under the current run’s selected conditions before comparing it with a new candidate. The paper runner reads `src/agc/config/optimized_gains.py` and does not run optimization.

## Gain fields

| Gain field | Dimension | Meaning |
| --- | --- | --- |
| `KRdiag` | $3 \times 1$ | Diagonal attitude tracking gains |
| `Kxidiag` | $3 \times 1$ | Diagonal position tracking gains |
| `LambdaDiag` | $6 \times 1$ | Diagonal sliding metric gains |
| `kd` | Scalar | Linear transverse damping gain |
| `ks` | Scalar | Fractional transverse dissipation gain |
| `alpha` | Scalar | Fractional reaching exponent |
| `gammaE` | $10 \times 1$ | Euclidean adaptation gains |
| `gammaB` | Scalar | Bregman adaptation gain |
