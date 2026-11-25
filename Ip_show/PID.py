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

# Drive Motor Encoder (Pololu 37D #4752: 30:1 gearbox, 64 CPR motor shaft)
GEAR_RATIO            = 30.0
MOTOR_ENCODER_CPR     = 64        # 4x quadrature on motor shaft
FPGA_SAMPLE_FREQ_HZ   = 100.0     # FPGA speed register updates at 100 Hz (10 ms)

TICKS_PER_DRIVE_REV   = MOTOR_ENCODER_CPR * GEAR_RATIO          # 1920 ticks/rev
METERS_PER_MOTOR_TICK = (math.pi * DRIVE_WHEEL_DIAM_M) / TICKS_PER_DRIVE_REV

# Passive Dead-Wheel Encoder (CUI AMT10: default 2048 PPR -> 8192 CPR, ~48 mm black wheel)
AMT10_PPR             = 1024      # DIP switch setting on AMT10 (pre-quadrature PPR)
ODO_ENCODER_CPR       = AMT10_PPR * 4
ODO_WHEEL_DIAM_M      = 0.048     # ~48 mm black passive wheel diameter
METERS_PER_ODO_TICK   = (math.pi * ODO_WHEEL_DIAM_M) / ODO_ENCODER_CPR

# DE0-Nano SPI Register Map
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

        self.pid_left = PIDController(kp=18.0, ki=95.0, kd=0.0)
        self.pid_right = PIDController(kp=18.0, ki=95.0, kd=0.0)

        self.spi = spidev.SpiDev()
        try:
            self.spi.open(0, 1)
            self.spi.max_speed_hz = 1000000
            self.spi.mode = 0
            print("[SPI] Connected to DE0-Nano on SPI0 CE1")
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

    def drive_straight(self, distance_m: float, max_speed_mps: float = 0.25, loop_hz: int = 20):
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

    def rotate_in_place(self, angle_deg: float, max_wheel_mps: float = 0.20, loop_hz: int = 20):
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