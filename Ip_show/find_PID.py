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
MOTOR_ENCODER_CPR     = 64        # 4x quadrature on motor shaft
FPGA_SAMPLE_FREQ_HZ   = 100.0     # FPGA speed update rate (100 Hz)

TICKS_PER_DRIVE_REV   = MOTOR_ENCODER_CPR * GEAR_RATIO
METERS_PER_MOTOR_TICK = (math.pi * DRIVE_WHEEL_DIAM_M) / TICKS_PER_DRIVE_REV

# DE0-Nano SPI Registers
REG_LEFT_SPEED  = 0x10
REG_RIGHT_SPEED = 0x11
REG_RESET       = 0x7F

# Identification Experiment Settings
STEP_VOLTAGE_V        = 6.0       # Safe step amplitude (~0.30-0.35 m/s)
STEP_DURATION_S       = 1.5       # Long enough for >99% steady-state settling
SAMPLE_RATE_HZ        = 100.0     # Match FPGA 100 Hz update rate
DESIRED_TAU_CL_S      = 0.12      # Desired closed-loop time constant tau_cl (s)


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