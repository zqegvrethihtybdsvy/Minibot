# Minibot PI Speed & Trajectory Controller Guide (ELME2313)

This guide covers the mathematical modeling of the Minibot drive motors, analytical PI gain calculation via pole cancellation (Option B), controller parameter selection, and a complete breakdown of both the open-loop identification script (`identify_motors_imc.py`) and the closed-loop trajectory controller (`PID.py`).

---

## 1. Open-Loop Plant Model of the Drive Wheels

With only applied motor voltage $u(t)$ ($\text{V}$) as the control input and wheel linear speed $v(t)$ ($\text{m/s}$) as the measured feedback (no armature current sensor), the electromechanical dynamics of each **Pololu 37D #4752** gearmotor driving the Minibot chassis are modeled as a **first-order linear time-invariant (LTI) system** preceded by a static Coulomb friction deadzone $V_{\text{dead}}$:

$$G(s) = \frac{V(s)}{U_{\text{eff}}(s)} = \frac{K}{\tau s + 1}$$

* **Negligible Electrical Pole:** The motor's electrical time constant ($\tau_e = L_a / R_a < 1\text{ ms}$) is two orders of magnitude faster than the robot's mechanical inertia, leaving a single dominant mechanical pole at $s = -1/\tau$.
* **Static Gain ($K$ in $\text{(m/s)/V}$):** Ratio of steady-state wheel speed $v_{\infty}$ to applied voltage:
  * **Lumped static gain (for standalone PI):**
    $$K_{\text{raw}} = \frac{v_{\infty}}{U_0}$$
  * **Effective slope gain (when explicitly compensating for deadzone $V_{\text{dead}}$):**
    $$K_{\text{eff}} = \frac{v_{\infty}}{U_0 - V_{\text{dead}}}$$
* **Mechanical Time Constant ($\tau$ in $\text{s}$):** Time elapsed from the step command ($t = 0$) until the wheel speed reaches $63.2\%$ ($1 - e^{-1}$) of its steady-state value $v_{\infty}$:
  $$v(t) = v_{\infty}\left(1 - e^{-t/\tau}\right)$$

---

## 2. Analytical PI Gain Design (Option B: Pole Cancellation / IMC)

Because the plant is first-order and velocity measurements have discrete encoder tick quantization ($\Delta v \approx 0.0196\text{ m/s}$ per tick at $100\text{ Hz}$), derivative action ($K_d$) would amplify high-frequency quantization spikes into H-bridge chatter. A **PI controller** ($K_d = 0$) is therefore optimal:

$$C(s) = K_p + \frac{K_i}{s} = K_p \left(\frac{s + K_i / K_p}{s}\right)$$

### Pole-Cancellation Derivation
1. Place the PI controller's zero ($s = -K_i / K_p$) directly on top of the motor's mechanical pole ($s = -1/\tau$):
   $$\frac{K_i}{K_p} = \frac{1}{\tau} \quad \Longrightarrow \quad C(s)G(s) = \frac{K K_p}{\tau s}$$
2. Closing the unity-feedback loop yields a pure first-order closed-loop transfer function with desired time constant $\tau_{\text{cl}}$:
   $$T(s) = \frac{C(s)G(s)}{1 + C(s)G(s)} = \frac{1}{\tau_{\text{cl}} s + 1}, \qquad \tau_{\text{cl}} = \frac{\tau}{K K_p}$$
3. Solving directly for the analytical PI gains:
   $$K_p = \frac{\tau}{K \, \tau_{\text{cl}}} \quad \left[\frac{\text{V}}{\text{m/s}}\right], \qquad K_i = \frac{1}{K \, \tau_{\text{cl}}} \quad \left[\frac{\text{V}}{\text{m}}\right]$$

### Standalone PI vs. PI + Feedforward + Deadzone Compensation

| Architecture | Control Law | Gains Used | Best Application |
| :--- | :--- | :--- | :--- |
| **Standalone PI** | $u(t) = K_p e(t) + K_i \int_0^t e(\tau)\,d\tau$ | `kp_pi`, `ki_pi` (using $K_{\text{raw}}$) | **ELME2313 lab validation:** pure LTI feedback loop matching $T(s) = \frac{1}{\tau_{\text{cl}}s + 1}$ without nonlinear jumps |
| **PI + Feedforward** | $u(t) = \frac{v_{\text{ref}}}{K_{\text{eff}}} + V_{\text{dead}}\operatorname{sign}(v_{\text{ref}}) + u_{\text{PI}}(t)$ | `kp_ff`, `ki_ff` (using $K_{\text{eff}}$) | **Competition navigation:** eliminates startup stiction lag and tracks trapezoidal speed profiles with near-zero delay |

---

## 3. Summary of Controller & Hardware Parameters

### 3.1 Inner Velocity Loop & Identification Parameters

| Parameter | Symbol / Variable | Value | Engineering Rationale |
| :--- | :--- | :--- | :--- |
| **Supply Voltage Rail** | `SUPPLY_VOLTAGE` | $12.0\text{ V}$ | Clamps controller output to $[-12.0\text{ V}, +12.0\text{ V}]$ |
| **Step Test Amplitude** | `STEP_VOLTAGE_V` | $6.0\text{ V}$ | Drives at $\sim 0.30\text{–}0.35\text{ m/s}$—well above stiction without risking wheel slip |
| **Step Duration** | `STEP_DURATION_S` | $1.5\text{ s}$ | $> 5\tau$ so the wheels reach $> 99\%$ steady-state velocity |
| **Identification Sample Rate** | `SAMPLE_RATE_HZ` | $100.0\text{ Hz}$ | Matches the $10\text{ ms}$ velocity register update rate |
| **Closed-Loop Time Constant** | `DESIRED_TAU_CL_S` | $0.10\text{–}0.12\text{ s}$ | Gives $95\%$ settling in $3\tau_{\text{cl}} \approx 0.30\text{–}0.36\text{ s}$, avoids $12\text{ V}$ saturation, and keeps $\sim 80^\circ$ phase margin |
| **Integral Anti-Windup Limit** | `max_integral` | $\frac{0.5 \, V_{\text{sup}}}{K_i}$ | Caps integrator contribution at $\pm 6.0\text{ V}$ to prevent overshoot after saturation |
| **Encoder CPR Note** | `MOTOR_ENCODER_CPR` | `32` (`960 ticks/rev`) | Use `32` (`16 PPR × 2×` edge decoding) in both scripts so `identify_motors_imc.py` and `PID.py` share the exact same $\text{m/s}$ scale |

### 3.2 Outer Cascade Position & Heading Loop Parameters (`PID.py`)

`PID.py` uses a **cascaded control structure**: outer proportional loops convert position/angle error into target wheel velocities ($\text{m/s}$), and the inner `PIDController` instances regulate motor voltage ($\text{V}$):

| Loop / Parameter | Variable | Value | Role in Trajectory Execution |
| :--- | :--- | :--- | :--- |
| **Outer Distance Gain** | `kp_dist` | $2.0\text{ s}^{-1}$ | Decelerates smoothly as remaining distance $\Delta d \to 0$ |
| **Straight-Line Heading Trim** | `kp_heading` | $1.5\text{ s}^{-1}$ | Applies differential velocity trim $\pm k_{p,\text{head}}(d_L - d_R)$ to keep straight lines drift-free |
| **Minimum Creep Speed** | `0.08 m/s` / `0.07 m/s` | $0.07\text{–}0.08\text{ m/s}$ | Prevents stalling in static friction during the final millimeters of a move |
| **Outer Yaw Rate Gain** | `kp_omega` | $2.5\text{ s}^{-1}$ | Converts heading error $\Delta\theta$ ($\text{rad}$) into desired angular velocity $\omega$ ($\text{rad/s}$) |
| **Stopping Tolerances** | `5 mm` / `1.0 deg` | $0.005\text{ m}$, $1.0^\circ$ | Terminates each segment cleanly once target pose is reached |

---

## 4. Explanation of the Two Scripts

### 4.1 Open-Loop Identification Script (`identify_motors_imc.py`)
1. **Hardware Initialization (`MotorIdentifier.__init__`):** Configures `10 kHz` PWM and `DIR` pins on BCM `12, 5, 13, 6`, opens `SPI0 CE1` at `1 MHz` (Mode 0), and resets the counters (`0xFF`).
2. **Phase 1 — Static Friction Measurement (`measure_deadzone`):** Ramps voltage from $0.0\text{ V}$ up to $3.0\text{ V}$ in $0.05\text{ V}$ increments every $60\text{ ms}$. Once a wheel registers $> 0.015\text{ m/s}$ for two consecutive samples, it records $V_{\text{dead}}$ independently for the left and right motors, then stops for $0.8\text{ s}$.
3. **Phase 2 — Step Response Logging (`measure_step_response`):** Applies `STEP_VOLTAGE_V = 6.0 V` to both motors and logs `(time_s, left_mps, right_mps)` at a strict `100 Hz` cadence using `time.perf_counter()` drift compensation.
4. **Analytical Parameter Extraction (`analyze_channel`):**
   * Averages the final $30\%$ of samples to compute $v_{\infty}$.
   * Computes both $K_{\text{eff}} = v_{\infty}/(U_0 - V_{\text{dead}})$ and $K_{\text{raw}} = v_{\infty}/U_0$.
   * Finds the exact crossing time of $0.6321 \, v_{\infty}$ via linear interpolation between adjacent samples to determine $\tau$.
   * Evaluates $K_p = \frac{\tau}{K \tau_{\text{cl}}}$ and $K_i = \frac{1}{K \tau_{\text{cl}}}$, saves `step_identification.csv` for plotting, and prints ready-to-paste constants.

### 4.2 Closed-Loop Square-Path Controller (`PID.py`)
1. **`PIDController` Class:** Implements a discrete-time PID regulator with anti-windup clamping (`self.integral` bounded so $K_i \int e\,dt \le \pm 6.0\text{ V}$) and output saturation at $\pm 12.0\text{ V}$.
2. **Telemetry & Dual-Track Scaling (`MinibotDriveOdo.get_telemetry`):** Reads drive wheel speeds (`0x10`, `0x11`) using the inner wheel diameter (`0.060 m`) and passive odometer distances (`0x13`, `0x14`) using the outer wheel diameter (`0.048 m`), applying sign inversion (`RIGHT_MOTOR_INVERTED = True`) for the mirrored right side.
3. **Closed-Loop Straight Driving (`drive_straight`):** Computes remaining distance from the average of the passive odometers, shapes the velocity reference with `kp_dist`, and injects a differential steering correction `trim = kp_heading * (l_m - r_m)` into `update_speed_control()`.
4. **Closed-Loop In-Place Turning (`rotate_in_place`):** Measures true robot yaw angle $\theta = (d_R - d_L) / 0.240\text{ m}$ from the outer passive wheels, computes target angular velocity $\omega = k_{p,\omega}(\theta_{\text{target}} - \theta)$, and commands opposite linear velocities $\mp \omega (0.180 / 2)$ on the inner drive wheels.
5. **Fail-Safe Shutdown (`emergency_stop` + `os._exit(0)`):** Drives PWM/DIR low and locks BCM `12, 13, 5, 6` via `pinctrl` so PCB pull-up resistors cannot latch the motors on after exit.

---

## 5. Complete Source Codes

### 5.1 Motor Identification & Option B Gain Calculator (`identify_motors_imc.py`)

> **Important Note on `MOTOR_ENCODER_CPR`:** Ensure `MOTOR_ENCODER_CPR = 32` is used in `identify_motors_imc.py` (matching `PID.py`) so that $K$ and the resulting $K_p, K_i$ gains use the exact same $\text{m/s}$ unit scaling as your controller.

```python
import spidev
from gpiozero import PWMOutputDevice, OutputDevice
import time
import math
import struct
import csv
import os

# GPIO Pin Definitions (Raspberry Pi -> Pololu Dual MC33926)
LEFT_PWM_PIN  = 12
LEFT_DIR_PIN  = 5
RIGHT_PWM_PIN = 13
RIGHT_DIR_PIN = 6

# Motor Mounting Polarity
LEFT_MOTOR_INVERTED  = False
RIGHT_MOTOR_INVERTED = True

# True Physical Constants (Pololu #4752 30:1 + 60 mm diameter wheels)
SUPPLY_VOLTAGE        = 12.0      # V
DRIVE_WHEEL_DIAM_M    = 0.060     # 60 mm diameter
GEAR_RATIO            = 30.0
MOTOR_ENCODER_CPR     = 32        # 16 PPR * 2x FPGA edge decoding (960 ticks/drive rev)
FPGA_SAMPLE_FREQ_HZ   = 100.0     # Speed update rate (100 Hz)

TICKS_PER_DRIVE_REV   = MOTOR_ENCODER_CPR * GEAR_RATIO
METERS_PER_MOTOR_TICK = (math.pi * DRIVE_WHEEL_DIAM_M) / TICKS_PER_DRIVE_REV

# SPI Registers
REG_LEFT_SPEED  = 0x10
REG_RIGHT_SPEED = 0x11
REG_RESET       = 0x7F

# Identification Experiment Settings
STEP_VOLTAGE_V        = 6.0       # Safe step amplitude (~0.30-0.35 m/s)
STEP_DURATION_S       = 1.5       # Long enough for >99% steady-state settling
SAMPLE_RATE_HZ        = 100.0     # Match 100 Hz update rate
DESIRED_TAU_CL_S      = 0.1       # Desired closed-loop time constant tau_cl (s)


class MotorIdentifier:
    def __init__(self):
        self.m1_pwm = PWMOutputDevice(pin=LEFT_PWM_PIN, frequency=10000, initial_value=0)
        self.m1_dir = OutputDevice(pin=LEFT_DIR_PIN, initial_value=False)
        self.m2_pwm = PWMOutputDevice(pin=RIGHT_PWM_PIN, frequency=10000, initial_value=0)
        self.m2_dir = OutputDevice(pin=RIGHT_DIR_PIN, initial_value=False)

        self.spi = spidev.SpiDev()
        self.spi.open(0, 1)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0
        self.spi.xfer2([0xFF, 0x00, 0x00, 0x00, 0x00])

    def set_voltage(self, left_v: float, right_v: float):
        left_v = max(min(left_v, SUPPLY_VOLTAGE), -SUPPLY_VOLTAGE)
        right_v = max(min(right_v, SUPPLY_VOLTAGE), -SUPPLY_VOLTAGE)

        eff_l = -left_v if LEFT_MOTOR_INVERTED else left_v
        eff_r = -right_v if RIGHT_MOTOR_INVERTED else right_v

        self.m1_dir.value = eff_l < 0
        self.m1_pwm.value = abs(eff_l) / SUPPLY_VOLTAGE

        self.m2_dir.value = eff_r < 0
        self.m2_pwm.value = abs(eff_r) / SUPPLY_VOLTAGE

    def read_speeds_mps(self):
        rx_l = self.spi.xfer2([REG_LEFT_SPEED, 0, 0, 0, 0])
        rx_r = self.spi.xfer2([REG_RIGHT_SPEED, 0, 0, 0, 0])

        raw_l = struct.unpack(">i", bytes(rx_l[1:5]))[0]
        raw_r = struct.unpack(">i", bytes(rx_r[1:5]))[0]

        if LEFT_MOTOR_INVERTED:
            raw_l = -raw_l
        if RIGHT_MOTOR_INVERTED:
            raw_r = -raw_r

        v_l = raw_l * FPGA_SAMPLE_FREQ_HZ * METERS_PER_MOTOR_TICK
        v_r = raw_r * FPGA_SAMPLE_FREQ_HZ * METERS_PER_MOTOR_TICK
        return v_l, v_r

    def stop(self):
        try:
            self.set_voltage(0.0, 0.0)
        except Exception:
            pass
        os.system("pinctrl set 12,13,5,6 op dl > /dev/null 2>&1 || raspi-gpio set 12,13,5,6 op dl > /dev/null 2>&1")

    def measure_deadzone(self):
        """Ramps voltage slowly from 0V to 3.0V to find breakaway voltage V_dead."""
        print("[Phase 1] Measuring static friction deadzone (V_dead)...")
        v_dead_l = None
        v_dead_r = None
        streak_l = 0
        streak_r = 0

        v = 0.0
        while v <= 3.0 and (v_dead_l is None or v_dead_r is None):
            self.set_voltage(v, v)
            time.sleep(0.06)
            vl, vr = self.read_speeds_mps()

            if v_dead_l is None:
                streak_l = streak_l + 1 if vl > 0.015 else 0
                if streak_l >= 2:
                    v_dead_l = v - 0.05

            if v_dead_r is None:
                streak_r = streak_r + 1 if vr > 0.015 else 0
                if streak_r >= 2:
                    v_dead_r = v - 0.05

            v += 0.05

        self.set_voltage(0.0, 0.0)
        time.sleep(0.8)

        v_dead_l = v_dead_l if v_dead_l is not None else 0.0
        v_dead_r = v_dead_r if v_dead_r is not None else 0.0
        print(f"  -> Left V_dead  = {v_dead_l:.2f} V")
        print(f"  -> Right V_dead = {v_dead_r:.2f} V\n")
        return v_dead_l, v_dead_r

    def measure_step_response(self):
        """Applies a clean step voltage U0 and logs speed at 100 Hz."""
        print(f"[Phase 2] Applying {STEP_VOLTAGE_V:.1f} V step for {STEP_DURATION_S:.1f} s...")
        dt = 1.0 / SAMPLE_RATE_HZ
        samples = []

        t0 = time.perf_counter()
        self.set_voltage(STEP_VOLTAGE_V, STEP_VOLTAGE_V)

        while True:
            now = time.perf_counter()
            t = now - t0
            if t > STEP_DURATION_S:
                break

            vl, vr = self.read_speeds_mps()
            samples.append((t, vl, vr))

            next_tick = t0 + len(samples) * dt
            sleep_time = next_tick - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

        self.set_voltage(0.0, 0.0)
        print("  -> Step complete.\n")
        return samples


def analyze_channel(times, speeds, u_step, v_dead, label):
    """Extracts K and tau for G(s) = K / (tau*s + 1) and computes Option B PI gains."""
    # 1. Steady-state speed v_inf: average over last 30% of the step
    steady_idx = int(0.7 * len(speeds))
    v_inf = sum(speeds[steady_idx:]) / len(speeds[steady_idx:])

    if v_inf <= 0.02:
        raise RuntimeError(f"[{label}] Motor did not spin during step test (v_inf={v_inf:.4f} m/s).")

    # 2. Static Gain K [(m/s) / V]
    k_eff = v_inf / max(u_step - v_dead, 0.1)   # With deadzone compensation
    k_raw = v_inf / u_step                        # Without deadzone compensation

    # 3. Mechanical time constant tau: time from t = 0 to reach 63.2% of v_inf
    v_632 = 0.6321 * v_inf
    tau = times[-1]
    for i in range(1, len(speeds)):
        if speeds[i] >= v_632:
            v0, v1 = speeds[i - 1], speeds[i]
            t0, t1 = times[i - 1], times[i]
            frac = (v_632 - v0) / (v1 - v0) if (v1 > v0) else 0.0
            tau = t0 + frac * (t1 - t0)
            break

    tau = max(tau, 0.02)
    tau_cl = DESIRED_TAU_CL_S

    # 4. Option B (Pole Cancellation): Kp = tau / (K * tau_cl), Ki = 1 / (K * tau_cl)
    kp_ff = tau / (k_eff * tau_cl)
    ki_ff = 1.0 / (k_eff * tau_cl)

    kp_pi = tau / (k_raw * tau_cl)
    ki_pi = 1.0 / (k_raw * tau_cl)

    print(f"=== {label.upper()} MOTOR IDENTIFICATION & OPTION B GAINS ===")
    print(f"  Steady-state speed v_inf : {v_inf:.4f} m/s  (at {u_step:.1f} V)")
    print(f"  Deadzone V_dead          : {v_dead:.2f} V")
    print(f"  Static gain K (w/ V_dead): {k_eff:.4f} (m/s)/V  [raw K = {k_raw:.4f}]")
    print(f"  Time constant tau        : {tau:.4f} s")
    print(f"  Closed-loop tau_cl       : {tau_cl:.4f} s")
    print(f"  ----------------------------------------------------------")
    print(f"  [Option B - Standalone PI]      Kp = {kp_pi:.2f} V/(m/s),  Ki = {ki_pi:.2f} V/m")
    print(f"  [Option B - PI + Feedforward]   Kp = {kp_ff:.2f} V/(m/s),  Ki = {ki_ff:.2f} V/m\n")

    return {
        "v_inf": v_inf, "v_dead": v_dead, "K_eff": k_eff, "K_raw": k_raw,
        "tau": tau, "tau_cl": tau_cl,
        "kp_pi": kp_pi, "ki_pi": ki_pi, "kp_ff": kp_ff, "ki_ff": ki_ff
    }


if __name__ == "__main__":
    ident = MotorIdentifier()
    try:
        v_dead_l, v_dead_r = ident.measure_deadzone()
        samples = ident.measure_step_response()

        csv_path = "step_identification.csv"
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "left_mps", "right_mps"])
            writer.writerows(samples)
        print(f"[Saved] Raw 100 Hz step data written to {csv_path}\n")

        times = [s[0] for s in samples]
        left_speeds = [s[1] for s in samples]
        right_speeds = [s[2] for s in samples]

        res_l = analyze_channel(times, left_speeds, STEP_VOLTAGE_V, v_dead_l, "Left")
        res_r = analyze_channel(times, right_speeds, STEP_VOLTAGE_V, v_dead_r, "Right")

        print("=== COPY-PASTE CONFIG FOR YOUR PID CONTROLLER ===")
        print(f"PID_LEFT_KP  = {res_l['kp_pi']:.2f}")
        print(f"PID_LEFT_KI  = {res_l['ki_pi']:.2f}")
        print(f"PID_RIGHT_KP = {res_r['kp_pi']:.2f}")
        print(f"PID_RIGHT_KI = {res_r['ki_pi']:.2f}")

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")
    finally:
        ident.stop()
        os._exit(0)
```

---

### 5.2 Closed-Loop Velocity & Square-Path Trajectory Controller (`PID.py`)

```python
import spidev
from gpiozero import PWMOutputDevice, OutputDevice
import time
import math
import struct
import os

# GPIO Pin Definitions (Raspberry Pi -> Pololu Dual MC33926)
LEFT_PWM_PIN  = 12
LEFT_DIR_PIN  = 5
RIGHT_PWM_PIN = 13
RIGHT_DIR_PIN = 6

# Motor Mounting Polarity (mirrored 180 deg on differential chassis)
LEFT_MOTOR_INVERTED  = False
RIGHT_MOTOR_INVERTED = True

# --- True Physical Dimensions from Schematic & Bottom Photo ---
SUPPLY_VOLTAGE        = 12.0      # V (Pololu #4752 & MC33926 nominal supply)
DRIVE_WHEEL_DIAM_M    = 0.060     # 60 mm diameter blue actuated wheels
DRIVE_WHEEL_BASE_M    = 0.180     # 2 * 90 mm center-to-center (blue wheels)
ODO_WHEEL_OFFSET_M    = 0.030     # 30 mm (3 cm) center-to-center outside each blue wheel
ODO_WHEEL_BASE_M      = DRIVE_WHEEL_BASE_M + 2.0 * ODO_WHEEL_OFFSET_M  # 0.240 m (240 mm)

# Drive Motor Encoder (Pololu 37D #4752: 30:1 gearbox, 32 ticks/rev in 2x edge decoding)
GEAR_RATIO            = 30.0
MOTOR_ENCODER_CPR     = 32        # 16 PPR * 2x edge decoding (960 ticks/drive rev)
FPGA_SAMPLE_FREQ_HZ   = 100.0     # Speed register updates at 100 Hz (10 ms)

TICKS_PER_DRIVE_REV   = MOTOR_ENCODER_CPR * GEAR_RATIO          # 960 ticks/rev
METERS_PER_MOTOR_TICK = (math.pi * DRIVE_WHEEL_DIAM_M) / TICKS_PER_DRIVE_REV

# Passive Dead-Wheel Encoder (CUI AMT10: 2048 PPR * 2x edge decoding = 4096 ticks/rev)
AMT10_PPR             = 1024      # Effective PPR giving 4096 ticks/rev
ODO_ENCODER_CPR       = AMT10_PPR * 4
ODO_WHEEL_DIAM_M      = 0.048     # ~48 mm black passive wheel diameter
METERS_PER_ODO_TICK   = (math.pi * ODO_WHEEL_DIAM_M) / ODO_ENCODER_CPR

# SPI Register Map
REG_LEFT_SPEED  = 0x10
REG_RIGHT_SPEED = 0x11
REG_LEFT_ODO    = 0x13
REG_RIGHT_ODO   = 0x14
REG_RESET       = 0x7F


class PIDController:
    """
    Discrete PID controller outputting motor voltage [-12.0V, +12.0V]
    from linear wheel velocity error (m/s).
    """
    def __init__(self, kp=18.0, ki=95.0, kd=0.0, out_min=-SUPPLY_VOLTAGE, out_max=SUPPLY_VOLTAGE):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, target_mps: float, measured_mps: float, dt: float) -> float:
        if dt <= 0:
            return 0.0

        error = target_mps - measured_mps
        self.integral += error * dt
        max_integral = (0.5 * self.out_max) / self.ki if self.ki > 0 else 0.0
        self.integral = max(min(self.integral, max_integral), -max_integral)

        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        voltage = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return max(min(voltage, self.out_max), self.out_min)

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0


class MinibotDriveOdo:
    def __init__(self):
        # 10 kHz PWM supported by MC33926 (up to 20 kHz)
        self.m1_pwm = PWMOutputDevice(pin=LEFT_PWM_PIN, frequency=10000, initial_value=0)
        self.m1_dir = OutputDevice(pin=LEFT_DIR_PIN, initial_value=False)
        self.m2_pwm = PWMOutputDevice(pin=RIGHT_PWM_PIN, frequency=10000, initial_value=0)
        self.m2_dir = OutputDevice(pin=RIGHT_DIR_PIN, initial_value=False)

        # Replace kp and ki with values output by identify_motors_imc.py
        self.pid_left = PIDController(kp=18.0, ki=95.0, kd=0.0)
        self.pid_right = PIDController(kp=18.0, ki=95.0, kd=0.0)

        self.spi = spidev.SpiDev()
        try:
            self.spi.open(0, 1)
            self.spi.max_speed_hz = 1000000
            self.spi.mode = 0
            print("[SPI] Connected on SPI0 CE1")
            self.reset_odometry()
        except Exception as e:
            print(f"[SPI ERROR] {e}")
            self.spi = None

    def reset_odometry(self):
        if self.spi:
            self.spi.xfer2([0xFF, 0x00, 0x00, 0x00, 0x00])

    def reset_pids(self):
        self.pid_left.reset()
        self.pid_right.reset()

    def set_motor_voltage(self, left_volts: float, right_volts: float):
        left_volts = max(min(left_volts, SUPPLY_VOLTAGE), -SUPPLY_VOLTAGE)
        right_volts = max(min(right_volts, SUPPLY_VOLTAGE), -SUPPLY_VOLTAGE)

        eff_left_v = -left_volts if LEFT_MOTOR_INVERTED else left_volts
        eff_right_v = -right_volts if RIGHT_MOTOR_INVERTED else right_volts

        self.m1_dir.value = eff_left_v < 0
        self.m1_pwm.value = abs(eff_left_v) / SUPPLY_VOLTAGE

        self.m2_dir.value = eff_right_v < 0
        self.m2_pwm.value = abs(eff_right_v) / SUPPLY_VOLTAGE

        return left_volts, right_volts

    def stop(self):
        self.set_motor_voltage(0.0, 0.0)
        self.reset_pids()

    def _read_spi_reg(self, reg_addr: int) -> int:
        if not self.spi:
            return 0
        rx = self.spi.xfer2([reg_addr & 0x7F, 0x00, 0x00, 0x00, 0x00])
        return struct.unpack(">i", bytes(rx[1:5]))[0]

    def get_telemetry(self):
        raw_l_spd = self._read_spi_reg(REG_LEFT_SPEED)
        raw_r_spd = self._read_spi_reg(REG_RIGHT_SPEED)
        raw_l_odo = self._read_spi_reg(REG_LEFT_ODO)
        raw_r_odo = self._read_spi_reg(REG_RIGHT_ODO)

        if LEFT_MOTOR_INVERTED:
            raw_l_spd, raw_l_odo = -raw_l_spd, -raw_l_odo
        if RIGHT_MOTOR_INVERTED:
            raw_r_spd, raw_r_odo = -raw_r_spd, -raw_r_odo

        left_mps  = raw_l_spd * FPGA_SAMPLE_FREQ_HZ * METERS_PER_MOTOR_TICK
        right_mps = raw_r_spd * FPGA_SAMPLE_FREQ_HZ * METERS_PER_MOTOR_TICK
        left_m    = raw_l_odo * METERS_PER_ODO_TICK
        right_m   = raw_r_odo * METERS_PER_ODO_TICK

        return left_mps, right_mps, left_m, right_m

    def update_speed_control(self, target_left_mps: float, target_right_mps: float, dt: float):
        l_mps, r_mps, l_m, r_m = self.get_telemetry()
        v_left = self.pid_left.update(target_left_mps, l_mps, dt)
        v_right = self.pid_right.update(target_right_mps, r_mps, dt)
        v_left, v_right = self.set_motor_voltage(v_left, v_right)
        return l_mps, r_mps, l_m, r_m, v_left, v_right

    def drive_straight(self, distance_m: float, max_speed_mps: float = 0.25, loop_hz: int = 50):
        """Drives straight for `distance_m` meters using the outer passive odometers."""
        self.reset_odometry()
        self.reset_pids()
        time.sleep(0.02)

        dt_nominal = 1.0 / loop_hz
        last_time = time.time()
        kp_dist = 2.0
        kp_heading = 1.5

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            _, _, l_m, r_m = self.get_telemetry()
            avg_dist = 0.5 * (l_m + r_m)
            remaining = distance_m - avg_dist

            if abs(remaining) <= 0.005:  # 5 mm tolerance
                break

            cmd_speed = max(min(kp_dist * remaining, max_speed_mps), -max_speed_mps)
            if 0 < cmd_speed < 0.08:
                cmd_speed = 0.08
            elif -0.08 < cmd_speed < 0:
                cmd_speed = -0.08

            trim = kp_heading * (l_m - r_m)
            self.update_speed_control(cmd_speed - trim, cmd_speed + trim, dt)
            time.sleep(dt_nominal)

        self.stop()

    def rotate_in_place(self, angle_deg: float, max_wheel_mps: float = 0.20, loop_hz: int = 50):
        """
        Rotates around the axle midpoint (x=0, y=0).
        Uses ODO_WHEEL_BASE_M (240 mm) for yaw angle feedback from the outer black wheels,
        while commanding the inner blue drive wheels at radius DRIVE_WHEEL_BASE_M / 2 (90 mm).
        Positive angle_deg = Counter-Clockwise (Left turn).
        """
        self.reset_odometry()
        self.reset_pids()
        time.sleep(0.02)

        target_rad = math.radians(angle_deg)
        dt_nominal = 1.0 / loop_hz
        last_time = time.time()
        kp_omega = 2.5

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            _, _, l_m, r_m = self.get_telemetry()
            # Yaw angle measured by the outer passive odometers (240 mm track width)
            current_rad = (r_m - l_m) / ODO_WHEEL_BASE_M
            remaining_rad = target_rad - current_rad

            if abs(remaining_rad) <= math.radians(1.0):  # 1 deg tolerance
                break

            # Desired chassis angular velocity omega (rad/s) -> inner blue wheel speed
            omega = kp_omega * remaining_rad
            wheel_speed = omega * (0.5 * DRIVE_WHEEL_BASE_M)
            wheel_speed = max(min(wheel_speed, max_wheel_mps), -max_wheel_mps)

            if 0 < wheel_speed < 0.07:
                wheel_speed = 0.07
            elif -0.07 < wheel_speed < 0:
                wheel_speed = -0.07

            self.update_speed_control(-wheel_speed, +wheel_speed, dt)
            time.sleep(dt_nominal)

        self.stop()

    def emergency_stop(self):
        try:
            self.stop()
        except Exception:
            pass
        os.system("pinctrl set 12,13,5,6 op dl > /dev/null 2>&1 || raspi-gpio set 12,13,5,6 op dl > /dev/null 2>&1")


if __name__ == "__main__":
    bot = MinibotDriveOdo()
    SIDE_LENGTH_M = 0.50   # 50 cm square sides
    TURN_ANGLE_DEG = 90.0  # +90 deg = CCW (left turn)

    try:
        print(f"\n=== EXECUTING {SIDE_LENGTH_M:.2f}m x {SIDE_LENGTH_M:.2f}m SQUARE PATH ===")
        for side in range(1, 5):
            print(f"[Side {side}/4] Driving forward {SIDE_LENGTH_M:.2f} m...")
            bot.drive_straight(SIDE_LENGTH_M, max_speed_mps=0.25)
            time.sleep(0.4)

            print(f"[Corner {side}/4] Rotating {TURN_ANGLE_DEG:+.0f} deg...")
            bot.rotate_in_place(TURN_ANGLE_DEG, max_wheel_mps=0.20)
            time.sleep(0.4)

        print("\n=== SQUARE PATH COMPLETE ===")

    except KeyboardInterrupt:
        print("\n[STOP] Script interrupted.")
    finally:
        bot.emergency_stop()
        os._exit(0)
```