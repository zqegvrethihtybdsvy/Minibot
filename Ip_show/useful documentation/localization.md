# Minibot Self-Localization System: EKF Fusion of Dead-Wheel Odometry & 2D LiDAR Beacons

This document details the architecture, mathematical formulation (based on **LELME2732 — Mobile Robot Localization**), software implementation, and calibration procedures for the Minibot's self-localization system.

---

## 1. System Architecture Overview

Rather than relying solely on open-loop dead-reckoning (which drifts unboundedly over time) or standalone LiDAR trilateration (which requires at least two unobstructeExtended Kalman Filter (EKF)d beacons at all times and runs at ~7–10 Hz), the localization system fuses both sensors using a discrete ****:

* **High-Frequency Prediction Step (50 Hz):** Reads incremental displacements ($\Delta s_l, \Delta s_r$) from the passive outer dead-wheels via the DE0-Nano SPI interface (`PID.py`). This propagates the robot pose estimate $\hat{x}_t$ and grows the pose covariance $\hat{P}_t$.
* **Asynchronous Correction Step (~7–10 Hz):** A background worker thread (`LidarLocalizationThread` in `ekf_localizer.py`) extracts cylindrical beacon observations $(r_i, \phi_i)$ from RPLIDAR scans, associates them with known Eurobot arena beacons via Mahalanobis gating, and shrinks the pose uncertainty $P_t$.
* **Single-Beacon Observability:** Because the EKF fuses range and bearing sequentially against the current pose belief, even a **single visible beacon** corrects cross-track and angular drift when the other two beacons are occluded by opponent robots.

---

## 2. Coordinate Frames & Hardware Geometry

All internal EKF calculations use standard SI units (**meters** and **radians**) and follow right-handed frames:

### 2.1 World Frame $\{W\}$
* **Origin $(0, 0)$:** Center of the Yellow starting zone on the $3.000\text{ m} \times 2.000\text{ m}$ Eurobot table ($X_{\text{table}} = 0.225\text{ m}$, $Y_{\text{table}} = 1.000\text{ m}$).
* **Axes:** $+X_W$ points down the $3\text{ m}$ length of the table; $+Y_W$ points across the $2\text{ m}$ width toward the left wall; $\theta = 0\text{ rad}$ points along $+X_W$ (Counter-Clockwise positive).
* **Fixed Beacons in $\{W\}$:**
  * `Back_Right`: $(-0.287\text{ m}, -1.062\text{ m})$
  * `Back_Left`: $(-0.287\text{ m}, +1.062\text{ m})$
  * `Opp_Middle`: $(+2.837\text{ m}, 0.000\text{ m})$

### 2.2 Robot Frame $\{R\}$ & Sensor Frame $\{S\}$
* **Origin:** Midpoint of the axle connecting the two passive dead-wheels.
* **Dead-Wheel Odometry Base ($b$):** $b = 0.240\text{ m}$ ($240\text{ mm}$ track width between the outer $48\text{ mm}$ passive wheels equipped with CUI AMT10 quadrature encoders at $4096\text{ CPR}$).
* **Drive Wheel Base:** $0.180\text{ m}$ ($180\text{ mm}$ track width between the inner $60\text{ mm}$ actuated blue wheels).
* **LiDAR Mounting Transform:** RPLIDAR angles increase clockwise (`LIDAR_ANGLE_SIGN = -1.0` to convert to CCW), with configurable mounting yaw offset (`LIDAR_YAW_OFFSET_RAD`) and lever-arm translation (`LIDAR_OFFSET_X_M`, `LIDAR_OFFSET_Y_M`) relative to the dead-wheel axle midpoint.

---

## 3. Mathematical Formulation (LELME2732 Lecture 4)

The state vector at time step $t$ is the 2D planar pose:

$$x_t = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}, \qquad P_t \in \mathbb{R}^{3 \times 3}$$

### 3.1 Step 0: Odometry Prediction Update (Slides 18–20 & 48)

Given incremental displacements $\Delta s_l$ and $\Delta s_r$ of the left and right passive odometers over $\Delta t$:

$$\Delta s = \frac{\Delta s_r + \Delta s_l}{2}, \qquad \Delta\theta = \frac{\Delta s_r - \Delta s_l}{b}, \qquad \theta_{\text{mid}} = \theta + \frac{\Delta\theta}{2}$$

The nonlinear kinematic motion model $\hat{x}_t = f(x_{t-1}, u_t)$ predicts the new pose:

$$\hat{x}_t = \begin{bmatrix} x + \Delta s \cos(\theta_{\text{mid}}) \\ y + \Delta s \sin(\theta_{\text{mid}}) \\ \text{wrap}(\theta + \Delta\theta) \end{bmatrix}$$

Non-systematic wheel slippage and encoder quantization noise are modeled as proportional to the distance traveled by each wheel:

$$Q_t = \begin{bmatrix} k_r |\Delta s_r| & 0 \\ 0 & k_l |\Delta s_l| \end{bmatrix}$$

The pose covariance propagates via first-order linearization:

$$\hat{P}_t = F_x P_{t-1} F_x^T + F_u Q_t F_u^T$$

where the state Jacobian $F_x = \nabla_p f$ and input Jacobian $F_u = \nabla_{\Delta rl} f$ are:

$$F_x = \begin{bmatrix} 1 & 0 & -\Delta s \sin(\theta_{\text{mid}}) \\ 0 & 1 & \Delta s \cos(\theta_{\text{mid}}) \\ 0 & 0 & 1 \end{bmatrix}$$

$$F_u = \begin{bmatrix} \frac{1}{2}\cos(\theta_{\text{mid}}) - \frac{\Delta s}{2b}\sin(\theta_{\text{mid}}) & \frac{1}{2}\cos(\theta_{\text{mid}}) + \frac{\Delta s}{2b}\sin(\theta_{\text{mid}}) \\ \frac{1}{2}\sin(\theta_{\text{mid}}) + \frac{\Delta s}{2b}\cos(\theta_{\text{mid}}) & \frac{1}{2}\sin(\theta_{\text{mid}}) - \frac{\Delta s}{2b}\cos(\theta_{\text{mid}}) \\ \frac{1}{b} & -\frac{1}{b} \end{bmatrix}$$

---

### 3.2 Step 1: LiDAR Beacon Feature Extraction (Slides 49–50)

1. **Workspace & Spectator Filtering:** Each raw scan ray $(r_{\text{raw}}, \alpha)$ is projected into $\{W\}$ using the current predicted pose $\hat{x}_t$. Rays falling outside the inner playing table (`MIN_PLAY_X/Y` to `MAX_PLAY_X/Y`) and outside the $180\text{ mm}$ radius pockets around the three known beacons are discarded, eliminating spectators and border walls.
2. **Euclidean Clustering:** Contiguous valid points separated by less than `MAX_GAP_M` ($75\text{ mm}$) are grouped into clusters (including wrap-around across $0^\circ / 360^\circ$).
3. **Cylindrical Beacon Validation:** Clusters with at least 2 inliers, chord width $\le 140\text{ mm}$, and radial depth spread $\le 85\text{ mm}$ are accepted as candidate $80\text{ mm}$-diameter beacons.
4. **Range & Bearing Centering:** The median surface range is calibrated and shifted by the beacon radius ($R_{\text{beacon}} = 0.040\text{ m}$) to yield the beacon center observation $z_t^i = [r_i, \phi_i]^T$ with measurement noise covariance:

$$R_t = \begin{bmatrix} \sigma_r^2 & 0 \\ 0 & \sigma_\phi^2 \end{bmatrix}$$

---

### 3.3 Steps 2 & 3: Measurement Prediction & Mahalanobis Gating (Slides 51–54)

Given the LiDAR's world position $(s_x, s_y)$ computed from $\hat{x}_t$ and the lever-arm offset, the expected observation $\hat{z}_t^j = h_j(\hat{x}_t, m_j)$ to map beacon $m_j = (b_{x,j}, b_{y,j})$ is:

$$\Delta x = b_{x,j} - s_x, \qquad \Delta y = b_{y,j} - s_y, \qquad q = \Delta x^2 + \Delta y^2$$

$$\hat{z}_t^j = \begin{bmatrix} \sqrt{q} \\ \text{wrap}\!\left(\text{atan2}(\Delta y, \Delta x) - \hat{\theta}\right) \end{bmatrix}$$

The measurement Jacobian $H_j = \frac{\partial h_j}{\partial x}$ is:

$$H_j = \begin{bmatrix} -\frac{\Delta x}{\sqrt{q}} & -\frac{\Delta y}{\sqrt{q}} & 0 \\ \frac{\Delta y}{q} & -\frac{\Delta x}{q} & -1 \end{bmatrix}$$

For each candidate pair $(i, j)$, the innovation $v_t^{ij}$ and innovation covariance $\Sigma_{\text{IN},t}^{ij}$ are computed:

$$v_t^{ij} = z_t^i - \hat{z}_t^j, \qquad \Sigma_{\text{IN},t}^{ij} = H_j \hat{P}_t H_j^T + R_t$$

To reject unmapped obstacles (e.g., opponent robots near a corner), an association is accepted only if its squared **Mahalanobis distance** passes the $\chi^2$ validation gate ($\chi^2_{2, 0.99} = 9.21$):

$$d_M^2 = \left(v_t^{ij}\right)^T \left(\Sigma_{\text{IN},t}^{ij}\right)^{-1} v_t^{ij} < 9.21$$

---

### 3.4 Step 4: Kalman Gain & State Update (Slide 55)

For each validated beacon match, the Kalman gain $K_t$, posterior state $x_t$, and posterior covariance $P_t$ (using the numerically stable **Joseph form**) are updated sequentially:

$$K_t = \hat{P}_t H_j^T \left(\Sigma_{\text{IN},t}^{ij}\right)^{-1}$$

$$x_t = \hat{x}_t + K_t v_t^{ij}$$

$$P_t = (I - K_t H_j)\hat{P}_t(I - K_t H_j)^T + K_t R_t K_t^T$$

---

## 4. Software Module Structure

| File | Responsibility | Key Classes / Functions |
| :--- | :--- | :--- |
| `PID.py` | Low-level motor PWM/DIR control, 100 Hz inner-loop wheel speed PI controllers, and DE0-Nano SPI quadrature counter reads. | `PIDController`, `MinibotDriveOdo` (`get_telemetry`, `update_speed_control`) |
| `ekf_localizer.py` | Thread-safe EKF state estimator, LiDAR workspace filtering, cylinder beacon extraction, and background RPLIDAR acquisition thread. | `EKFLocalizer` (`predict_odometry`, `update_beacons`), `extract_beacon_observations`, `LidarLocalizationThread` |
| `main_localization.py` | High-level mission loop integrating 50 Hz odometry updates with closed-loop global waypoint navigation $(x, y, \theta)$. | `LocalizedRobotController` (`step_odometry`, `go_to_waypoint`) |

---

## 5. Calibration & Tuning Checklist

Before running full autonomous matches, calibrate the parameters in the following order:

1. **LiDAR Range Offset (`DIST_OFFSET_M` & `DIST_SCALE` in `ekf_localizer.py`):**
   * Place the robot at a known distance (e.g., $1.600\text{ m}$) from an $80\text{ mm}$ beacon and adjust `DIST_OFFSET_M` (currently `-0.210` m) until reported `range_m` matches the true center-to-center distance.
2. **LiDAR Mounting Pose (`LIDAR_YAW_OFFSET_RAD`, `LIDAR_OFFSET_X_M`, `LIDAR_OFFSET_Y_M`):**
   * Measure the physical $(x, y)$ offset of the RPLIDAR center relative to the midpoint of the two black passive wheels.
   * Rotate the robot $360^\circ$ in place while logging beacon positions; adjust `LIDAR_YAW_OFFSET_RAD` if the initial heading shows a static bias.
3. **Passive Odometer Kinematics (`ODO_WHEEL_DIAM_M` & `ODO_WHEEL_BASE_M` in `PID.py`):**
   * Temporarily disable LiDAR updates and drive a straight $2.00\text{ m}$ line; scale `ODO_WHEEL_DIAM_M` ($0.048\text{ m}$) by $\frac{d_{\text{true}}}{d_{\text{measured}}}$.
   * Perform $5$ full in-place revolutions ($1800^\circ$); scale `ODO_WHEEL_BASE_M` ($0.240\text{ m}$) by $\frac{\theta_{\text{measured}}}{\theta_{\text{true}}}$.
4. **Filter Noise Tuning (`kr`, `kl`, `sigma_range_m`, `sigma_bearing_rad`):**
   * Once odometry kinematics are calibrated, reduce `kr` and `kl` from `0.002` down to `0.0005` so the EKF trusts short-term passive wheel tracking smoothly between LiDAR scans.
   * Increase `sigma_bearing_rad` if rapid in-place rotations cause slight angular lag between LiDAR scan timestamps and odometry timestamps.