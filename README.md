# **Koopman-EDMD-MPC-Lateral-Control**

## Koopman/EDMD + Model Predictive Control for Data-Driven Lateral Vehicle Control

> **Author:** Sanskar Sunil Patil  
> **Course:** EEE 598: Data-Driven Dynamical Systems  
> **Institution:** Arizona State University

---

## Abstract

**Koopman-EDMD-MPC-Lateral-Control** studies whether a data-driven Koopman/EDMD lifted linear predictor, embedded inside a constrained Model Predictive Controller, can outperform a classical feedforward plus LQR baseline for lateral vehicle tracking.

The central motivation is practical: Koopman-based Extended Dynamic Mode Decomposition (EDMD) promises a bridge between nonlinear system behavior and convex optimization-based control design. By lifting the physical state into a higher-dimensional observable space where the dynamics evolve approximately linearly, the learned model can be embedded directly into a constrained quadratic MPC formulation. That structural advantage is real and is one of the main reasons Koopman methods are appealing in data-driven control.

This project is not a demonstration that Koopman-MPC works in isolation. Instead, it asks a more disciplined question:

> Does a learned Koopman/EDMD predictor, when placed inside a closed-loop MPC controller, outperform a strong classical baseline on the same lateral tracking problem?

The work develops a fully reproducible end-to-end pipeline that:

- defines a ground-truth dynamic bicycle model in road-aligned tracking-error coordinates,
- generates informative training, validation, and test trajectories across multiple maneuvers,
- learns a Koopman/EDMD lifted linear predictor in standardized coordinates,
- embeds that predictor into a constrained MPC framework for steering control,
- evaluates prediction quality, closed-loop tracking, and robustness across maneuvers and operating conditions,
- and compares the learned-model approach against a classical feedforward plus LQR baseline and a SINDy predictor.

The learned Koopman/EDMD model achieves extremely accurate one-step and finite-horizon open-loop prediction in the nominal operating regime. However, closed-loop evaluation shows that Koopman-MPC does **not** outperform the classical feedforward plus LQR baseline, which achieves lower tracking error and lower steering effort. Robustness studies across speed further show that the fixed-speed Koopman controller degrades severely at high speed, while a speed-aware EDMD extension still fails to surpass the baseline. In parallel, SINDy emerges as the strongest learned predictor in open-loop accuracy.

That balance is the central result:

- prediction accuracy, controller-friendly model structure, and closed-loop robustness are fundamentally different design objectives,
- the best predictor (SINDy) is not the best controller,
- the most controller-friendly learned model (Koopman/EDMD) does not outperform the classical baseline,
- and these conditions are explicitly characterized rather than treated as failures.

This positions the project as a controlled, comparative investigation into the relationship between learned prediction and closed-loop control performance.

---

## Project Idea in One Paragraph

Lateral vehicle control requires the controller to reduce lateral and heading error while respecting actuator limits and remaining robust across maneuvers and operating conditions. Data-driven methods such as Koopman/EDMD are attractive because they learn a lifted linear predictor that fits naturally inside convex MPC formulations. This work investigates whether that structural advantage translates into superior closed-loop control. The approach is deliberately comparative: instead of demonstrating Koopman-MPC in isolation, it evaluates the full chain from simulated dynamics and dataset generation to prediction quality, controller synthesis, closed-loop evaluation, and robustness analysis, all benchmarked against a strong classical controller and a competing learned predictor.

### Inspiration from Prior Work

The baseline vehicle model implementation is adapted from the [Vehicle Lateral Stability (Python Implementation)](https://github.com/aymisxx/VehicleLateralStability-Py) by A. Mishra. That repository provides a dynamic bicycle model simulator in road-aligned tracking-error coordinates together with a feedforward plus LQR baseline controller.

This project draws on that implementation as the ground-truth plant and classical control reference. However, instead of focusing on classical stability analysis, this work extends the problem into data-driven modeling and control: learning Koopman/EDMD predictors from simulated trajectories, embedding them into constrained MPC, and studying the trade-off between prediction accuracy and closed-loop performance.

The connection is therefore foundational rather than methodological, using the established vehicle model as a controlled benchmark for data-driven control evaluation.

## Why This Work Matters

This work addresses a practical gap between data-driven prediction and data-driven control.

It develops a complete pipeline that:

- learns a Koopman/EDMD lifted linear model suitable for constrained MPC,
- evaluates that controller against a strong classical baseline under identical conditions,
- tests robustness across operating speeds and maneuver types,
- and compares multiple learned predictors to separate prediction quality from control quality.

Rather than aiming for state-of-the-art tracking performance, the focus is on **understanding when and why learned controllers succeed or fail relative to classical baselines**, making the approach suitable for informing practical design decisions in data-driven vehicle control systems.

---

## Mathematical Modeling

### Ground-Truth Lateral Vehicle Dynamics

The ground-truth plant is a planar dynamic bicycle model written in road-aligned tracking-error coordinates. The state is defined as:

`x = [ey, epsi, vy, r]^T`

where:

- `ey` is lateral tracking error relative to the reference path,
- `epsi` is heading error relative to the reference heading,
- `vy` is lateral velocity,
- `r` is yaw rate.

The physical control input is the front steering angle `delta`. The longitudinal speed `Vx` and reference curvature `kappa_ref(t)` are treated as known exogenous signals.

### Tire Force Model

The continuous-time vehicle dynamics are based on the standard small-slip dynamic bicycle approximation with linear tire forces:

`Fyf = Caf * alpha_f,    Fyr = Car * alpha_r`

where `Caf` and `Car` are front and rear cornering stiffnesses, and `alpha_f`, `alpha_r` are the slip angles under small-angle approximation:

`alpha_f ≈ delta - (vy + lf * r) / Vx,    alpha_r ≈ -(vy - lr * r) / Vx`

### Road-Aligned Error Dynamics

With these assumptions, the full state-space form is:

```
e_y_dot   = vy + Vx * epsi
epsi_dot  = r - Vx * kappa_ref
vy_dot    = -(Caf + Car)/(m*Vx) * vy - ((Caf*lf - Car*lr)/(m*Vx) + Vx) * r + (Caf/m) * delta
r_dot     = -(Caf*lf - Car*lr)/(Iz*Vx) * vy - (Caf*lf^2 + Car*lr^2)/(Iz*Vx) * r + (Caf*lf/Iz) * delta
```

This model serves as the reference plant for all simulations and evaluations.

### Discrete-Time Simulation

All simulation and data generation use a forward-Euler update:

`x_{k+1} = x_k + dt * x_dot_k`

implemented as `bike_step(x, delta, kappa_ref, Vx, cfg)` with a numerical safeguard:

`Vx_eff = max(0.1, Vx)`

to prevent division-by-zero in slip-angle formulas.

### Koopman/EDMD Lifted Modeling

Instead of evolving the physical state directly, the Koopman approach defines a lifted observable vector:

`z_k = psi(x_k) in R^nz`

and approximates the lifted dynamics by a linear system:

`z_{k+1} ≈ A * z_k + B * u_k`

If the observable dictionary is sufficiently expressive, nonlinear dynamics can be approximated as linear evolution in lifted coordinates. This is especially attractive for controller design because linear prediction models can be embedded directly into convex quadratic MPC formulations.

### Important Clarification

The Koopman/EDMD lifted linear model is an approximation, not an exact representation. It is a deliberately structured, data-driven transformation designed to produce a controller-friendly linear predictor, without claiming to capture the full nonlinear dynamics exactly.

---

## Scope and Boundaries

This work is intentionally scoped as a **comparative study of data-driven modeling and control**, not a demonstration that Koopman-MPC is superior to classical methods.

The focus is on:

- evaluating whether a learned Koopman/EDMD predictor improves closed-loop tracking over a classical baseline,
- quantifying robustness across operating speeds and maneuver types,
- comparing multiple learned predictors (EDMD vs. SINDy) to separate prediction quality from control quality,
- and identifying the conditions under which learned controllers succeed or fail.

This scope is deliberate. The entire project is simulation-based and deterministic, which isolates the central question without introducing complications such as measurement noise, sensor dropouts, or experimental inconsistencies.

Accordingly, this work does not claim real-world deployment readiness, noise-robust performance, or superiority of data-driven methods over classical control. It also does not treat strong open-loop prediction as evidence of strong closed-loop control.

Instead, the work concentrates on controlled, reproducible evaluation of the full chain from data generation to prediction to closed-loop control, while explicitly characterizing strengths and limitations.

---

## Nominal Parameters and Assumptions

The notebook uses a fixed configuration object storing all nominal parameters:

### Vehicle Parameters

| Parameter | Value |
|---|---:|
| Mass `m` | 1500 kg |
| Yaw inertia `Iz` | 3000 kg m^2 |
| Front axle distance `lf` | 1.2 m |
| Rear axle distance `lr` | 1.6 m |
| Front cornering stiffness `Caf` | 80,000 N/rad |
| Rear cornering stiffness `Car` | 80,000 N/rad |
| Wheelbase `L` | 2.8 m |

### Operating Conditions

| Parameter | Value |
|---|---:|
| Nominal speed `Vx` | 15 m/s |
| Simulation start `t0` | 0 s |
| Simulation end `tf` | 25 s |
| Time step `dt` | 0.01 s |

### Actuator Constraints

| Parameter | Value |
|---|---:|
| Steering bound `\|delta\|` | 0.5 rad |
| Steering-rate bound `\|delta_dot\|` | 0.8 rad/s |

### MPC Settings

| Parameter | Value |
|---|---:|
| Prediction horizon | 2.0 s |
| Prediction steps `N` | 200 |

The project assumes a deterministic, noise-free simulation environment. No measurement noise, sensor corruption, or external disturbances are added. This is important because it partly explains why the learned predictors can achieve extremely small one-step prediction errors in the nominal operating regime.

---

## Repository Structure

```text
.
├── notebooks
│   └── Final_00_koopman_edmd_mpc_lateral_control.ipynb
├── results
│   ├── figures/
│   ├── tables/
│   ├── models/
│   │   └── edmd_model.npz
│   ├── data/
│   └── logs/
│       └── run_manifest.json
├── LICENSE
├── README.md
└── requirements.txt
```

### Folder Meanings

#### `notebooks/`

The main notebook implementing the complete end-to-end pipeline. This is a single, self-contained, reproducible notebook that performs all stages from data generation through final comparison.

#### `results/figures/`

All saved plots and visualizations generated during the pipeline run, including tracking comparisons, prediction rollouts, robustness sweeps, and the final predictor-vs-controller comparison figure.

#### `results/tables/`

Metrics tables in CSV format, including nominal tracking metrics, speed-sweep results, predictor comparison tables, and runtime benchmarks.

#### `results/models/`

Learned model artifacts, including `edmd_model.npz` containing the lifted transition matrix `A`, input matrix `B`, scalers, and the selector matrix `C_sel`.

#### `results/data/`

Intermediate data artifacts produced during the pipeline.

#### `results/logs/`

Run manifests and reproducibility records, including `run_manifest.json` storing configuration, timestamps, and artifact paths.

---

## Dataset

This work uses **entirely simulated data** generated from the ground-truth dynamic bicycle model. This choice makes the study fully controlled and reproducible.

### Data Generation

All trajectories are produced by numerically simulating the truth-model bicycle dynamics with forward-Euler integration at:

| Parameter | Value |
|---|---:|
| Time step `dt` | 0.01 s |
| Simulation horizon | 0 to 25 s |
| Nominal speed `Vx` | 15 m/s |

### Persistently Exciting Inputs

The notebook generates informative rollouts using:

- multi-sine steering inputs with randomized frequencies, phases, and weights,
- bounded steering amplitude and steering-rate profiles,
- and maneuver types alternating among straight driving, lane-change, and slalom.

Curvature amplitude and steering excitation amplitude are randomized from rollout to rollout to prevent the dataset from collapsing onto one narrow operating condition.

### Fixed-Speed Dataset

For the nominal EDMD study:

- **Total trajectories:** 30
- **Training:** 21 trajectories
- **Validation:** 4 trajectories
- **Test:** 5 trajectories

The dataset is split by **entire trajectory** rather than by individual time sample to avoid information leakage.

### Speed-Aware Dataset

For the multi-speed extension:

- **Speeds used:** {10, 15, 20, 25} m/s
- **Total trajectories:** 40
- **Training:** 28 trajectories
- **Validation:** 6 trajectories
- **Test:** 6 trajectories

### Reference Maneuvers

Three maneuver classes are used throughout the project:

1. **Straight driving:** `kappa_ref(t) = 0`
2. **Lane change:** two smooth curvature pulses of opposite sign, producing an S-shaped path
3. **Slalom:** sinusoidal curvature signal

### Data Coverage Limitation

A learned model is reliable only within the operating regime covered by the training data. Performance can degrade when the controller is evaluated at speeds not represented during training, under more aggressive curvature profiles, or in operating regions requiring larger steering actions. This motivates the later robustness sweeps over speed.

---

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/sanskarpatil/koopman-mpc-lateral-control
cd koopman-mpc-lateral-control
```

### 2. Create a Python environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Run the notebook

Open and run the main notebook:

```bash
jupyter notebook notebooks/Final_00_koopman_edmd_mpc_lateral_control.ipynb
```

Use **Run All** to execute the complete pipeline. The notebook is config-driven and artifact-producing: learned models, figures, tables, and run manifests are automatically saved to `results/`.

---

## **How the Work Was Done (The Pipeline)**

The full pipeline is implemented in a single end-to-end notebook and proceeds through the following stages.

## Stage 0: Reproducibility, Configuration, and Results Folder

Purpose:

> Set up the notebook backbone so that the full pipeline is reproducible, config-driven, and artifact-producing.

This stage creates:

- a `CFG` dataclass storing all simulation, vehicle, and control parameters,
- a `results/` folder tree for figures, tables, models, data, and logs,
- helper functions for saving artifacts (`save_fig`, `save_csv`, `save_npz`, `save_json`),
- and an initial run manifest for reproducibility tracking.

## Stage 1: Reference Generation (Curvature-Based Maneuvers)

Purpose:

> Construct a reference path generator that produces smooth maneuver trajectories from a prescribed curvature profile.

Given longitudinal speed `Vx` and reference curvature `kappa_ref(t)`, the notebook computes:

- reference yaw rate: `r_ref(t) = Vx * kappa_ref(t)`
- reference heading: `psi_ref(t) = integral(r_ref(t) dt)`
- reference path: `x_ref(t)`, `y_ref(t)` by integrating heading

Three maneuver presets are generated: straight, lane-change (S-shaped with `kappa_max = 0.002 m^-1`), and slalom (sinusoidal curvature).

## Stage 2: Truth-Model Simulator

Purpose:

> Implement the ground-truth dynamic bicycle model as a deterministic discrete-time simulator.

The simulator implements `bike_step(x, delta, kappa_ref, Vx, cfg)` using forward-Euler integration and serves as the reference plant for all data generation, model learning, and controller evaluation.

## Stage 3: Baseline Controller (Feedforward + LQR)

Purpose:

> Build a strong classical control benchmark that the learned controller must beat to be considered useful.

The baseline combines:

- a feedforward term from a curvature-to-steering relation with understeer-gradient correction,
- a discrete-time LQR feedback law designed from the linearized bicycle model,
- the same steering saturation and rate constraints used in the learned-controller experiments.

This baseline is intentionally strong rather than trivial.

## Stage 4: Data Generation for Koopman/EDMD

Purpose:

> Generate informative open-loop rollouts using persistently exciting steering inputs and multiple maneuver families.

The notebook generates 30 trajectories with alternating maneuver types, randomized curvature amplitudes, and randomized steering excitation. The dataset is split 21/4/5 by entire trajectory for training, validation, and test.

## Stage 5: Koopman/EDMD Lifted Model Learning

Purpose:

> Learn a lifted linear predictor in standardized coordinates using Extended Dynamic Mode Decomposition.

### Lifting Dictionary

```
psi(x) = [1, ey, epsi, vy, r, ey^2, epsi^2, vy^2, r^2, ey*epsi,
           ey*vy, ey*r, epsi*vy, epsi*r, vy*r, sin(epsi), cos(epsi)]^T
```

This gives a lifted dimension of `nz = 17`.

### Augmented Input

```
u_k = [delta_k, kappa_ref_k]^T
```

Including curvature as part of the input is essential because the heading-error dynamics contain the exogenous term `-Vx * kappa_ref(t)`.

### EDMD Regression

The regression is solved in standardized coordinates as a ridge-regularized least-squares problem with `lambda = 1e-6`. Physical state reconstruction is exact through a fixed selector matrix `C_sel`.

## Stage 6: Multi-Step Open-Loop Prediction

Purpose:

> Evaluate whether the learned EDMD model can maintain accurate predictions over a finite horizon.

The model is iterated forward over a 200-step horizon on held-out test trajectories. This evaluation is especially important because very small one-step error can still hide accumulated drift over longer horizons.

### Result

The fixed-speed EDMD model achieved extremely accurate open-loop prediction in the nominal regime:

- One-step validation RMSE: `[1.59e-9, 7.37e-10, 2.74e-12, 2.40e-11]` for `[ey, epsi, vy, r]`
- Multi-step rollout RMSE: `[5.75e-8, 5.47e-9, 4.94e-10, 3.09e-10]`

## Stage 7: Koopman-MPC Controller Design

Purpose:

> Embed the learned lifted model into a constrained finite-horizon MPC controller.

The controller formulation includes:

- quadratic cost penalizing state deviation, steering magnitude, corrective steering, and steering-rate variation,
- steering magnitude and rate constraints matching the actuator limits,
- a feedforward-centered formulation where MPC optimizes a corrective steering term around a nominal feedforward signal,
- and a minimal spectral shrink for numerical robustness.

The optimization is a convex quadratic program solved with CVXPY/OSQP, with SCS as fallback.

## Stage 8: Nominal Closed-Loop Evaluation (Lane-Change + Slalom)

Purpose:

> Determine whether the learned Koopman-MPC improves closed-loop tracking relative to the classical baseline.

### Nominal Tracking Results

| Metric | Lane-BL | Lane-KMPC | Slalom-BL | Slalom-KMPC |
|---|---:|---:|---:|---:|
| RMS ey [m] | 0.009534 | 0.037620 | 0.012183 | 0.037619 |
| Max \|ey\| [m] | 0.018609 | 0.039390 | 0.017238 | 0.039362 |
| RMS epsi [rad] | 0.000813 | 0.001108 | 0.000842 | 0.001098 |
| Max \|epsi\| [rad] | 0.00332 | 0.01168 | 0.00120 | 0.01139 |

### Interpretation

The baseline feedforward plus LQR controller achieved smaller RMS lateral error and smaller RMS heading error than Koopman-MPC for both maneuvers. Koopman-MPC remained feasible and stable but exhibited a persistent lateral offset and less smooth steering behavior. This is already an important result: a controller-friendly learned model can be viable without being the best controller in practice.

## Stage 9: Robustness Sweep (Multiple Speeds)

Purpose:

> Evaluate generalization beyond the nominal operating point.

The speed sweep tests both controllers at:

`Vx in {10, 15, 20, 25} m/s`

### Fixed-Speed Koopman-MPC Results

RMS lateral errors for lane change:

- `0.0371, 0.0356, 0.0349, 82.75 m` at Vx = 10, 15, 20, 25 m/s

The corresponding baseline RMS lateral errors:

- `0.00158, 0.01108, 0.03311, 0.07343 m`

### Failure Rule

The notebook defines failure as `RMS(ey) > 2.0 m` and reports Koopman-MPC failure at 25 m/s for both lane-change and slalom. The fixed-speed learned controller has limited operating-point robustness.

## Stage 10: Robustness Sweep Summary + Failure Analysis

Purpose:

> Systematically characterize when and why the learned controller fails.

The failure mechanism is distribution shift: the EDMD model is trained at `Vx = 15 m/s`, while the robustness evaluation includes speeds up to 25 m/s. The learned model extrapolates beyond its training regime, producing incorrect steering corrections that accumulate over time. The lateral error does not increase gradually but diverges rapidly after an initial period of tracking.

## Stage 11: Speed-Aware Koopman (LPV-EDMD Extension)

Purpose:

> Test whether augmenting the EDMD input with longitudinal speed can recover robust closed-loop performance.

A multi-speed dataset is constructed using `Vx in {10, 15, 20, 25} m/s` with 40 total trajectories. The augmented input becomes:

`u_k = [delta_k, kappa_k, Vx]^T`

### One-Step Validation RMSE (Speed-Aware)

`[0.01153, 0.00012, 0.00989, 0.00441]` for `[ey, epsi, vy, r]`

This is notably worse than the fixed-speed EDMD, which is expected because a single affine lifted model must represent dynamics across a broader operating envelope.

## Stage 12: Speed-Aware Koopman-MPC Results

Purpose:

> Evaluate whether the speed-aware predictor improves closed-loop control.

### Result

The speed-aware Koopman-MPC produced **substantially worse** RMS lateral error than the baseline at every tested speed:

- Lane change: `5.36, 25.84, 58.17, 117.84 m` at Vx = 10, 15, 20, 25 m/s
- Slalom: `5.37, 29.79, 66.30, 129.51 m`

### Interpretation

This negative result is informative: simply adding speed as an extra input is not sufficient to obtain a robust speed-aware Koopman-MPC controller. Achieving robustness across operating conditions likely requires more structured model representations such as explicitly parameterized LPV models or gain-scheduled control designs.

## Stage 15: Predictor Comparison (Koopman/EDMD vs. SINDy)

Purpose:

> Compare the two learned predictors to determine which achieves stronger open-loop accuracy.

SINDy is fit using a structured candidate library of 30 terms with sequential thresholded least squares, converging to only 3 active terms per state equation.

### One-Step Validation RMSE

| Predictor | ey | epsi | vy | r |
|---|---:|---:|---:|---:|
| Koopman/EDMD | 1.915e-3 | 1.419e-5 | 3.015e-8 | 1.452e-5 |
| SINDy | 3.13e-10 | 1.46e-8 | 1.67e-8 | 4.82e-9 |

### Multi-Step Rollout RMSE

| Predictor | ey | epsi | vy | r |
|---|---:|---:|---:|---:|
| Koopman/EDMD | 2.415e-1 | 1.825e-3 | 3.01e-4 | 1.66e-4 |
| SINDy | 7.88e-6 | 8.77e-7 | 1.56e-7 | 7.30e-8 |

### Main Conclusion

SINDy outperforms Koopman/EDMD for all four states in both metrics. Koopman/EDMD remains valuable because it yields a lifted linear representation directly compatible with MPC. That distinction between **best predictor** and **most controller-friendly representation** is the main technical message of the project.

## Stage 16: Final Comparison Plot

Purpose:

> Visualize the central lesson: prediction accuracy vs. control performance.

The final comparison figure shows that on the prediction side, SINDy achieves lower RMSE than Koopman/EDMD, while on the control side, the baseline LQR achieves substantially lower closed-loop tracking error than Koopman-MPC. This summarizes the central lesson: the best predictor is not necessarily the best controller.

## Stage 17: Final Results and Discussion

Purpose:

> Consolidate the full experimental narrative into final conclusions.

## Stage 20: Runtime and Solver-Time Comparison

Purpose:

> Compare the computational cost of the main closed-loop controllers.

This section adds an implementation-oriented perspective, comparing mean per-step computation time between baseline LQR and Koopman-MPC. The baseline LQR is substantially faster, reinforcing the practical trade-off: the baseline may remain preferable when simplicity and real-time efficiency matter most.

---

## Comparison and Ablation Summary

The project contains two comparison axes: **controller comparison** and **predictor comparison**. Together, they form an ablation-style analysis separating three related questions: whether the limitation comes from the controller architecture, the learned prediction model, or the operating-region generalization.

### Controller Comparison

| Controller | Main Observation |
|---|---|
| Baseline feedforward + LQR | Best overall closed-loop tracking and robustness |
| Fixed-speed Koopman-MPC | Feasible and stable nominally, degrades severely at high speed |
| Speed-aware Koopman-MPC | Worse than baseline across the full speed sweep |

### Predictor Comparison

| Predictor | Main Observation |
|---|---|
| Koopman/EDMD | Controller-compatible lifted linear model for MPC |
| SINDy | Strongest one-step and multi-step prediction accuracy |

### Ablation Variants

| Variant | Purpose | Main Observation |
|---|---|---|
| EDMD without explicit kappa_ref input | Reduced-input ablation | Less suitable for changing-path prediction |
| EDMD with delta and kappa_ref inputs | Main Koopman-MPC predictor | Controller-compatible lifted linear model for constrained MPC |
| Speed-aware EDMD with Vx augmentation | Robustness extension | Does not outperform the baseline controller |
| SINDy predictor | Open-loop prediction benchmark | Achieves the strongest prediction accuracy |
| Feedforward plus LQR | Classical control baseline | Achieves the best closed-loop tracking and robustness |

---

## Results Summary

| Metric | Value |
|---|---|
| Baseline RMS ey (lane change, nominal) | 0.009534 m |
| Koopman-MPC RMS ey (lane change, nominal) | 0.037620 m |
| Baseline RMS ey (slalom, nominal) | 0.012183 m |
| Koopman-MPC RMS ey (slalom, nominal) | 0.037619 m |
| EDMD one-step RMSE ey (nominal) | ~1.59e-9 |
| SINDy one-step RMSE ey | ~3.13e-10 |
| Koopman-MPC failure speed | 25 m/s |

### Key Observation

The results show that the learned Koopman/EDMD model achieves extremely strong open-loop prediction but does not translate that accuracy into superior closed-loop control. The classical feedforward plus LQR baseline remains the best tested controller across all maneuvers and speeds.

This confirms that prediction accuracy and closed-loop control quality are **related but fundamentally different objectives**, and successful engineering requires evaluating all three (prediction accuracy, controller-compatible structure, and closed-loop robustness) rather than assuming that one implies the others.

---

## Deliverables

The notebook produces the following artifacts automatically:

| Artifact | Location |
|---|---|
| Learned EDMD model | `results/models/edmd_model.npz` |
| All figures | `results/figures/` |
| Metrics tables | `results/tables/` |
| Run manifest | `results/logs/run_manifest.json` |

### Definition of Done

The notebook is considered complete if **Run All** successfully:

- generates or loads the dataset,
- fits the EDMD model,
- validates prediction quality,
- builds the Koopman-MPC controller,
- runs closed-loop simulations,
- produces comparison figures and metrics,
- saves artifacts to `results/`,
- and writes a run manifest for reproducibility.

---

## Limitations and Scope Constraints

This study isolates the relationship between learned prediction and closed-loop control, without extending into real-world deployment or noise-robust evaluation.

### Scientific Scope Constraints

- The entire project is simulation-based and deterministic.
- The ground-truth plant is an idealized dynamic bicycle model with linear tire forces under a small-slip approximation.
- Learned models were trained on data generated by the same model used for evaluation.
- The Koopman lifting dictionary is fixed and relatively simple.
- No measurement noise, sensor corruption, or external disturbances are modeled.
- The study does not claim real-world deployment readiness.

### Observed Performance Limitations

- The fixed-speed Koopman-MPC does not outperform the classical baseline in nominal tracking.
- The fixed-speed Koopman controller degrades severely at speeds above the nominal training condition.
- The speed-aware EDMD extension does not recover robust closed-loop performance.
- Simply adding speed as an extra EDMD input is not sufficient for robust speed-aware control.

### Ablation Limitations

- The lifting dictionary, regularization strength, and MPC prediction horizon were fixed throughout the study.
- A systematic ablation over these design choices was not performed.
- The observed performance reflects a single configuration rather than the full range of possible design trade-offs.

### Interpretation Constraint

Strong open-loop prediction should not be interpreted as evidence of strong closed-loop control. The project explicitly demonstrates that these are different objectives.

---

## Final Conclusion

**Koopman-EDMD-MPC-Lateral-Control** establishes that data-driven vehicle lateral modeling and control is feasible and informative, but the best method depends on the engineering objective.

The key outcomes are:

- Koopman/EDMD successfully produced a learned lifted linear model for MPC formulation, but the resulting Koopman-MPC did not outperform the classical baseline,
- the classical feedforward plus LQR baseline emerged as the strongest tested controller across all maneuvers and speeds,
- the fixed-speed Koopman controller lacked robustness at higher speeds, and the speed-aware extension did not solve the problem,
- SINDy was the strongest learned predictor in both one-step and multi-step open-loop evaluation,
- and explicit failure analysis clarified when and why the learned controllers degrade.

Taken together, the project demonstrates a complete progression from data generation to model learning, controller synthesis, closed-loop evaluation, robustness analysis, and predictor comparison.

> The best predictor does not necessarily mean the best controller. Prediction accuracy, controller-friendly model structure, and closed-loop robustness are fundamentally different design objectives, and successful engineering requires evaluating all three.

---

## **Academic Context and Acknowledgment**

This project was completed as part of **EEE 598: Data-Driven Dynamical Systems** at **Arizona State University**, under the guidance of **Prof. Amarsagar Reddy Ramapuram Matavalam**.

The course framework and feedback played a significant role in shaping this project.

The baseline vehicle model implementation is adapted from the [Vehicle Lateral Stability (Python Implementation)](https://github.com/aymisxx/VehicleLateralStability-Py) by A. Mishra.

---

## Statement on Use of Generative AI

Generative AI tools (including ChatGPT by OpenAI and Claude by Anthropic) were used during this work to assist with drafting, editing, and improving the clarity and structure of written content, as well as for code organization and debugging support.

All technical decisions, implementations, and results were critically reviewed, validated, and integrated by the author, who assumes full responsibility for the final work.

---

## Future Work

Promising next steps include:

- richer Koopman lifting dictionaries,
- improved regularization and stabilization of learned lifted models,
- larger multi-speed training datasets,
- structured LPV or gain-scheduled Koopman formulations,
- hybrid physics-informed plus data-driven learning approaches,
- lifting-dictionary ablation studies comparing reduced and expanded variants,
- joint sensitivity studies over regularization parameter and MPC horizon length,
- noisy simulation studies and disturbance injection,
- and evaluation on real or experimentally collected vehicle datasets.

---

## References

### [1] Vehicle Lateral Stability (Python Implementation)

A. Mishra (aymisxx),  
**"Vehicle Lateral Stability (Python Implementation),"**  
GitHub repository, 2026.  
Repository: <https://github.com/aymisxx/VehicleLateralStability-Py>

### [2] Koopman Spectral Properties

I. Mezic,  
**"Spectral properties of dynamical systems, model reduction and decompositions,"**  
Nonlinear Dynamics, vol. 41, no. 1-3, pp. 309-325, 2005.

### [3] Data-Driven Approximation of the Koopman Operator

M. O. Williams, I. G. Kevrekidis, and C. W. Rowley,  
**"A data-driven approximation of the Koopman operator: Extending dynamic mode decomposition,"**  
Journal of Nonlinear Science, vol. 25, no. 6, pp. 1307-1346, 2015.

### [4] SINDy

S. L. Brunton, J. L. Proctor, and J. N. Kutz,  
**"Discovering governing equations from data by sparse identification of nonlinear dynamical systems,"**  
Proceedings of the National Academy of Sciences, vol. 113, no. 15, pp. 3932-3937, 2016.

### [5] Data-Driven Science and Engineering

S. L. Brunton and J. N. Kutz,  
**Data-Driven Science and Engineering: Machine Learning, Dynamical Systems, and Control.**  
Cambridge, U.K.: Cambridge University Press, 2019.

### [6] Model Predictive Control

J. B. Rawlings, D. Q. Mayne, and M. M. Diehl,  
**Model Predictive Control: Theory, Computation, and Design,** 2nd ed.  
Nob Hill Publishing, 2017.

---
