# Recursive State Estimation of a Robot from Total-Station Observations

Course project for **Recursive State Estimation for Dynamic Systems (SoSe 2026)**, Geodätisches Institut Hannover — Group 10.

**Team:** Arjun Mohandas · Aswin Sam · Edvin Tony Thampy

This repository implements and compares six recursive Bayesian filters that estimate the **3D position and velocity** of a robot driving laps on a race track, using noisy **total-station observations** (horizontal angle *H*, vertical angle *V*, slope distance *D*) and a laser-tracker ground truth for evaluation. Every filter is implemented from scratch in NumPy, following the notation of the lecture, and each notebook is fully self-contained and documented with the underlying equations.

---

## Problem setup

| | |
|---|---|
| **State vector** | `x = [X, Y, Z, vX, vY, vZ]ᵀ` — Cartesian position (m) and velocity (m/s) |
| **Motion model** | Constant velocity with white-acceleration process noise, `σ_a = 2.5 m/s²` (common setting for all filters) |
| **Observations** | Polar `[H, V, D]ᵀ` from a total station, `Δt ≈ 0.01 s` |
| **Ground truth** | Laser-tracker positions `[X, Y, Z]` (reference velocity derived by differentiation) |
| **Metrics** | Component-wise absolute deviation, cumulative RMSE (position & velocity), improvement over raw measurements, computation time |

For the linear filter the polar observations are first converted to Cartesian coordinates and the measurement covariance is propagated with the Jacobian; all nonlinear filters work directly with the polar observation function.

---

## Repository structure

```
.
├── 01_classic_kalman_filter.ipynb            Linear time-discrete Kalman filter (LKF / LDKF)
├── 02_extended_kalman_filter.ipynb           EKF — unconstrained, projection-constrained, perfect-measurement-constrained
├── 03_iterated_extended_kalman_filter.ipynb  IEKF — unconstrained, projection-constrained, perfect-measurement-constrained
├── 04_unscented_kalman_filter.ipynb          UKF (sigma points, unscented transform)
├── 05_ensemble_kalman_filter.ipynb           EnKF (stochastic ensemble, N_e study)
├── 06_particle_filter.ipynb                  PF (SIR with residual resampling, N_p study)
├── 07_filter_comparison.ipynb                Side-by-side comparison of ALL filters (figures, tables, conclusions)
├── Data/
│   ├── polar0011_slower_0.csv                Total-station observations: Time [HH:MM:SS], H [deg], V [deg], D [mm]
│   └── tracker0011_slower_0.csv              Ground truth: X [m], Y [m], Z [m]
├── Figures/                                  All plots exported by notebooks 01–06 (read by notebook 07)
├── requirements.txt
└── README.md
```

### Notebook guide

| # | Notebook | What it does |
|---|---|---|
| 01 | Classic Kalman Filter | Polar→Cartesian conversion, Jacobian-based covariance propagation, constant-velocity model, derivation of `Q_ww` from `σ_a`, full predict/update loop, evaluation plots. |
| 02 | Extended Kalman Filter | Nonlinear observation function `h(x)` and its Jacobian. Three variants: **unconstrained EKF**, **projection-based constrained EKF** (the track's two circular arcs are fitted with geometric least squares and used as a hard constraint via state projection), and **perfect-measurement constrained EKF** (constraint added as a pseudo-measurement). |
| 03 | Iterated EKF | Same three variants as 02, with an iterated measurement update (converges in ≤3 iterations). |
| 04 | Unscented Kalman Filter | Sigma-point generation (Cholesky), UT weights, prediction/measurement sigma propagation, cross-covariance and gain — no Jacobians needed. |
| 05 | Ensemble Kalman Filter | Ensemble generation, perturbed observations, ensemble update; study of the influence of the ensemble size `N_e`. |
| 06 | Particle Filter | Particle propagation, Gaussian likelihood weighting, effective sample size, residual resampling; study of particle count `N_p`. |
| 07 | **Filter comparison** | Collects the figures and numbers from 01–06 and compares all filters: trajectories, deviations, cumulative RMSE, improvement over raw measurements, computation time, convergence behaviour, effect of the geometric constraints, and a final assessment. **Start here if you only want the results.** |

Each implementation notebook follows the same layout: *Aim → Pre-processing → Procedure (equations + code) → Observations (plots) → Discussion → Conclusion*, so the filters are easy to compare cell-by-cell.

---

## Getting started

### 1. Clone and install

```bash
git clone <repo-url>
cd group_10_03
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Requires Python ≥ 3.9 with `numpy`, `pandas`, `matplotlib`, `scipy` and `jupyter`.

### 2. Run the notebooks

```bash
jupyter lab        # or: jupyter notebook
```

Open the notebooks **from the repository root** — they load data with relative paths (`Data/...`) and save plots to `Figures/...`.

* Run **01–06** to reproduce the individual filters. Each one regenerates its own figures in `Figures/`.
* Run **07** to regenerate the comparison. It only reads the PNGs in `Figures/` and the result tables, so it runs in seconds.

The notebooks are committed **with their outputs**, so you can read all results on GitHub without executing anything.

> **Runtime note:** the LKF, EKF, IEKF and UKF finish in a few seconds. The EnKF (`N_e = 500`) and the Particle Filter (`N_p = 1500`) each take roughly **15–20 minutes** on a laptop because they propagate hundreds of samples per epoch. Reduce `N_e` / `N_p` for quick experiments.

---

## Key results (`σ_a = 2.5 m/s²`)

Overall RMSE against the laser-tracker ground truth:

| Filter | Configuration | Position RMSE (m) | Velocity RMSE (m/s) |
|---|---|---:|---:|
| Linear Kalman Filter | — | 0.00340 | 0.0494 |
| Extended Kalman Filter | unconstrained | 0.00341 | 0.0506 |
| EKF, projection constraint | circular arcs | 0.00341 | 0.0506 |
| EKF, perfect measurement | circular arcs | 0.00324 | 0.0486 |
| **Iterated EKF** | unconstrained | **0.00339** | **0.0494** |
| IEKF, projection constraint | circular arcs | 0.00341 | 0.0506 |
| **IEKF, perfect measurement** | circular arcs | **0.00324** | **0.0486** |
| Unscented Kalman Filter | — | 0.00340 | 0.0494 |
| Ensemble Kalman Filter | `N_e = 500` | 0.00576 | 0.0587 |
| Particle Filter | `N_p = 1500` | 0.00886 | 0.0693 |

Take-aways (see notebook 07 for the full discussion):

* All filters converge and track the robot without divergence; the constant-velocity model represents the dynamics well.
* The **Iterated EKF** is the best unconstrained filter, and the **perfect-measurement IEKF** is the best overall once the known circular track geometry is exploited.
* Projection-based constraints keep the estimate on the track but slightly increase RMSE; perfect-measurement constraints improve accuracy *and* geometric consistency.
* The UKF matches the LKF/IEKF accuracy without Jacobians at ~3× the cost; EnKF and PF are stable but less accurate and orders of magnitude slower.

---

## Extending the work

* **Try another dataset:** drop new `polar*.csv` / `tracker*.csv` files into `Data/` and change the two `pd.read_csv(...)` lines at the top of each notebook. Column names must match the ones listed above.
* **Tune the process noise:** `sigma_a` is defined once near the top of each notebook's *Procedure* section.
* **Change sample sizes:** `N_e` (EnKF) and `N_p` (PF) are single variables in notebooks 05 and 06.
* **Add a new filter:** copy the structure of the closest existing notebook, save its plots to `Figures/` using the same naming scheme (`<FILTER>_2D_Trajectory.png`, `<FILTER>_Position_Absolute_Deviation.png`, …) and add it to the comparison grids in notebook 07.

---

## Use of AI

ChatGPT was used for code review, debugging, summarising background material and typesetting lengthy LaTeX equations in the markdown cells. All implementation decisions and final results were verified by the team (see the statement at the top of notebook 01).
