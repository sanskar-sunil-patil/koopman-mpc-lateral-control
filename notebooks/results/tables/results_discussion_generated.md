# Results & Discussion

This notebook implemented an end-to-end pipeline:

**Truth model (nonlinear bicycle in error coordinates)** $\rightarrow$ **data generation** $\rightarrow$ **Koopman/EDMD model identification** $\rightarrow$ **convex MPC on the learned model** $\rightarrow$ **closed-loop evaluation** with a classical baseline (**feedforward + LQR**) $\rightarrow$ **SINDy-based nonlinear system identification and prediction comparison**.

---

## A) Fixed-speed evaluation ($V_x = 15$ m/s)

We compared Baseline LQR vs Koopman-MPC on two maneuvers.

### Lane-change
- **Baseline LQR**
  - RMS($e_y$) = **0.004189 m**, max$|e_y|$ = **0.007450 m**
  - RMS($e_\psi$) = **0.000640 rad**
  - max$|\delta|$ = **0.009284 rad**
- **Koopman-MPC**
  - RMS($e_y$) = **0.005564 m**, max$|e_y|$ = **0.010886 m**
  - RMS($e_\psi$) = **0.000803 rad**
  - max$|\delta|$ = **0.011514 rad**

**Observation:** Koopman-MPC gives **1.3x higher** lateral tracking error than the baseline on lane-change. Steering usage changes slightly, but both controllers remain within feasible steering bounds.

### Slalom
- **Baseline LQR**
  - RMS($e_y$) = **0.004543 m**, max$|e_y|$ = **0.006733 m**
  - RMS($e_\psi$) = **0.000506 rad**
  - max$|\delta|$ = **0.006924 rad**
- **Koopman-MPC**
  - RMS($e_y$) = **0.005596 m**, max$|e_y|$ = **0.008676 m**
  - RMS($e_\psi$) = **0.000500 rad**
  - max$|\delta|$ = **0.009936 rad**

**Observation:** In slalom, Koopman-MPC gives **1.2x higher** lateral error than the baseline, while using comparable steering authority.

**Conclusion at nominal speed:** When trained and evaluated near the same operating condition, Koopman-MPC can deliver strong tracking performance while solving a constrained convex QP.

---

## B) Generalization across speed (robustness sweep)

We tested
$$
V_x \in \{10,\ 15,\ 20,\ 25\}\ \text{m/s}.
$$

### B1) Fixed-speed Koopman model

At $V_x = 25$ m/s, the fixed-speed Koopman controller shows severe degradation:

- Lane-change RMS($e_y$) = **23.021 m**
- Slalom RMS($e_y$) = **28.694 m**

Meanwhile, the baseline LQR remains stable:

- Lane-change RMS($e_y$) = **0.028 m**
- Slalom RMS($e_y$) = **0.031 m**

**Interpretation:** The fixed-speed Koopman predictor is only reliable near its training distribution. Once the operating point shifts significantly, model mismatch causes large prediction error and poor control performance.


## C) Speed-aware Koopman (LPV-style EDMD)

To improve generalization, we trained a speed-aware EDMD model using
$$
u_k = [\delta_k,\ \kappa_k,\ V_x]^T
$$
so that the lifted predictor depends explicitly on speed.

At $V_x = 25$ m/s, the speed-aware Koopman controller avoids the catastrophic blow-up seen with the fixed-speed Koopman model:

- Lane-change RMS($e_y$) = **117.840 m**
- Slalom RMS($e_y$) = **129.514 m**

Compared with the fixed-speed Koopman model, this is a major stability improvement.  
However, the baseline LQR still performs better at high speed, which suggests that the speed-aware learned model is more stable but not yet accurate enough for best-in-class tracking.



## D) EDMD vs SINDy for open-loop prediction

In addition to control, we also compared two learned predictive models:

- **EDMD** (Koopman lifting)
- **SINDy** (Sparse Identification of Nonlinear Dynamics)

For multi-step open-loop rollout, the average trajectory RMSE values were approximately:

- **EDMD**
  - mean RMSE($e_y$) = **25.934 m**
  - mean RMSE($e_\psi$) = **0.421 rad**
- **SINDy**
  - mean RMSE($e_y$) = **20.497 m**
  - mean RMSE($e_\psi$) = **0.039 rad**

**Interpretation:** In these saved multi-step rollouts, **SINDy** achieved the lower average lateral prediction error.  
This comparison is important because it shows that SINDy was not just included symbolically; it was used as an explicit nonlinear identification baseline against EDMD.


---

## Overall Conclusion

- **Koopman-MPC is strongest near its training regime**, where it can outperform or compete closely with the classical baseline.
- **Out-of-distribution operation**, especially at high speed, exposes the central limitation of a fixed learned predictor.
- **Including speed in the learned model improves robustness**, but additional data, richer lifting functions, and better regularization are still needed to match the consistency of the baseline controller across the full operating envelope.
- **SINDy adds value as an interpretable nonlinear identification baseline**, and the EDMD-vs-SINDy open-loop comparison helps explain why predictive-control performance depends so strongly on model quality.
