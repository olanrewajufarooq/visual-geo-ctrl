# Gain Optimizer (`agc.opt`)

The package implements the staged gain-search workflow exposed by `run/optimize_gains.py`. Its behavior and configured gain seeds are restored from `ad38b3d`.

## Modules

| Module | Responsibility |
| --- | --- |
| `bounds.py` | Search bounds, mode/factorization expansion, coordinate blocks, and schedules. |
| `encoding.py` | Maps between controller gain dictionaries and optimizer vectors. |
| `objective.py` | Scores one condition and computes the arithmetic-mean score across conditions. |
| `pso.py` | Particle Swarm Optimization. |
| `de.py` | Differential Evolution and optional Nelder–Mead polishing. |
| `bregman_profile.py` | Logarithmic scalar profile search for the Bregman adaptation gain. |
| `staged_optimizer.py` | Builds training scenarios, evaluates candidates, resumes stage artifacts, and promotes gains. |

## Independent optimization targets

The three modes are `nominal`, `euclidean`, and `bregman`. Each can be optimized with the Levi–Civita (`lc`) or rigid-body (`rb`) factorization. The default `all` selection expands to six independent runs and six independently promoted registry entries. No mode shares tracking gains with another mode, and an LC result is never compared with or promoted over an RB result.

Hierarchical schedules use six stages for nominal and seven for adaptive modes. Classic schedules use one `all` stage for nominal and four stages (`all`, `nonadaptive`, `adaptive`, `all`) for adaptive modes. Bregman adaptation profiles its scalar learning gain on a logarithmic grid during the adaptive stage.

## Objective and default training set

Each mode/factorization pair uses the arithmetic mean of six closed-loop evaluations: three replay IDs (`lemniscate_02_auto`, `lemniscate_03_auto`, `lemniscate_04_auto`) crossed with two payload profiles (`flat_light`, `tall_heavy`). Nominal tuning disables the payload for these replay conditions; adaptive tuning includes the payload-release cases. If any condition fails, that candidate is marked failed.

The objective includes normalized tracking, parameter-estimation, and separate force/torque effort terms. Force RMS is normalized by 50 N and torque RMS by 1 N m, each with weight 0.25. See `objective.py` for exact weights and scales.

## Running the optimizer

```powershell
python run/optimize_gains.py --mode all --coriolis all --schedule hierarchical
python run/optimize_gains.py --mode bregman --coriolis rb --method de --polish
```

Defaults are PSO, swarm size 20, 50 iterations per stage, 10 stall iterations, 30-second evaluation runs, parallel candidate evaluations, and in-place saving. The CLI supports hierarchical/classic schedules, PSO/DE, optional final-stage polishing, replay/profile overrides, and disabling registry promotion. See `docs/gain-tuning.md` for usage and `docs/gain_tuning_data_output.md` for artifact paths.
