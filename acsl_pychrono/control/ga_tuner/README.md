# GA Tuner Module

Genetic algorithm-based parameter tuning for UAV control systems (PID and MRAC).

## Installation

```bash
pip install numpy scipy deap pymoo
```

Also requires `pychrono` - see project installation guide.

## Quick Start

```bash
python acsl_pychrono/control/ga_tuner/run_tuner.py
```

Or in Python:

```python
from acsl_pychrono.control.ga_tuner import create_uav_ga_tuner, summarize_tuner_result, GAConfig

tuner = create_uav_ga_tuner(ga_config=GAConfig())
result = tuner.optimize()
summarize_tuner_result(tuner, result)
```

## Configuration

All user-defined knobs live in `ga_config/`. Change these to customize the tuner with the controller, parameters, and simulation you want:

| File | Purpose |
|------|---------|
| `ga_config.py` | Algorithm (DEAP/PYMOO), population size, generations, controller type |
| `param_config.py` | Which parameters to tune (PID or MRAC selections) |
| `evaluator_config.py` | Evaluator type (inner/outer), parallel workers, log directory |
| `metrics_config.py` | Single vs multi-objective, metric weights, normalization |
| `mission_config.py` | Trajectory, duration, visualization flags |

### Pick a controller
- Set `controller_type` in `ga_config.py` **and** `CONTROLLER_TYPE` in `param_config.py` to the same value (`"PID"` or `"MRAC"`).
- Choose `EVALUATOR_TYPE` in `evaluator_config.py` — `"inner"` (attitude) or `"outer"` (position).

### Choose which parameters to tune
`param_config.py` contains selectors that are expanded by `param_utils.py`:
- PID: set `PID_TUNING_SELECTION` to `"all"`, `"translational"`, `"rotational"`, `"proportional"`, `"integral"`, `"derivative"`, a list of parameter names, or `{"groups": [...]}` combining groups from `PID_PARAMETER_GROUPS`.
- MRAC: set `MRAC_TUNING_SELECTION` per Gamma matrix (`gamma_x_tran`, `gamma_r_tran`, `gamma_theta_tran`, `gamma_x_rot`, `gamma_r_rot`, `gamma_theta_rot`) using `"full"`, `"diagonal"`, `{"diagonal": [mask...]}`, or `{"selection_matrix": [...]}` lower-triangular masks.
- The computed `TUNED_PARAMETERS` is used automatically by `TUNING_CONFIG`.

### Optimization mode
- Set `multi_objective` in `metrics_config.py`:
  - `True` → returns a Pareto front (multiple trade-off solutions)
  - `False` → weighted sum single objective, you can also customize the weights accordingly

## Output

Multi-objective mode returns a Pareto front:
```
Pareto front (5 solutions):
  Solution 1: [attitude_error, angular_rate_error, control_effort] = [0.024, 0.343, 0.106]
  Solution 2: ...
```

Single-objective mode returns one best solution with a scalar fitness value.

## Troubleshooting

**RuntimeWarning: overflow/invalid value** - Expected during exploration. Bad parameters get penalty values (999.0).

**Large control effort values** - Numerical instability from unstable parameters. Automatically penalized.
