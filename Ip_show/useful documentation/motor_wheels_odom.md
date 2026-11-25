# Minibot Wheels, Motors & Odometry Guide

## 1. System Overview

The Minibot drive and localization system consists of two distinct wheel sets and a split actuation/sensing pipeline:

1. **Motor Actuation:** The **Raspberry Pi GPIO pins** generate $10\text{ kHz}$ `PWM` and `DIR` signals routed through on-board level shifters (`U10` PI4ULS5V108) and gating logic (`U2`, `U3`, `U14`, `U15` 74LVC1G97DW-7) to a **Pololu Dual MC33926 H-bridge** ($12\text{ V}$ nominal supply) driving two **Pololu 37D #4752** gearmotors.
2. **Wheel & Odometry Telemetry:** Quadrature encoders on the two drive motors and two passive tracking wheels report tick counts to the Raspberry Pi over **SPI0 CE1 (`1 MHz`, Mode 0)**, where they are converted into linear wheel speeds ($\text{m/s}$), wheel travel ($\text{m}$), and 2D chassis pose $(x, y, \theta)$.

$$\text{Raspberry Pi GPIO} \xrightarrow{\text{10 kHz PWM / DIR}} \text{MC33926 H-Bridge} \xrightarrow{\pm 12\text{ V}} \mathbf{\text{Drive Motors (Blue Wheels)}}$$

$$\mathbf{\text{Motor \& Passive Encoders}} \xrightarrow{\text{SPI0 CE1}} \text{Raspberry Pi } \left(v_L, v_R\text{ in m/s};\; d_L, d_R\text{ in m};\; x, y, \theta\right)$$

---

## 2. Motor Pin Mapping, Polarity & PWM Configuration

### 2.1 Raspberry Pi $\leftrightarrow$ H-Bridge Pinout

| Function | Motor | Raspberry Pi BCM | Physical Header Pin | PCB Net Name | Logic State / Parameter |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Left PWM** | Motor 1 (M1) | **BCM 12** | Pin 32 | `PWM1` | $10\text{ kHz}$ PWM, duty cycle $0\%$ to $100\%$ ($0\text{ V}$ to $12\text{ V}$) |
| **Left DIR** | Motor 1 (M1) | **BCM 5** | Pin 29 | `GPIO05` / `M1DIR` | `0` = Forward ($+V$), `1` = Reverse ($-V$) |
| **Right PWM** | Motor 2 (M2) | **BCM 13** | Pin 33 | `PWM2` | $10\text{ kHz}$ PWM, duty cycle $0\%$ to $100\%$ ($0\text{ V}$ to $12\text{ V}$) |
| **Right DIR** | Motor 2 (M2) | **BCM 6** | Pin 31 | `GPIO06` / `M2DIR` | `0` = Reverse ($-V$), `1` = Forward ($+V$) *(mirrored)* |

### 2.2 Important Motor Configuration Details

1. **Mirrored Right-Motor Polarity (`RIGHT_MOTOR_INVERTED = True`):** Because Motor 1 and Motor 2 are mounted $180^\circ$ opposite each other on the differential chassis, driving both with `DIR = 0` spins the robot in place instead of moving straight. Both the **right motor `DIR` pin** and the **right encoder/odometer raw counts (`0x11`, `0x14`)** are inverted in software so positive numbers always correspond to forward robot motion.
2. **Ultrasonic PWM Frequency ($10\text{ kHz}$):** The Pololu Dual MC33926 supports PWM frequencies up to $20\text{ kHz}$. Setting `frequency=10000` instead of `1000` eliminates audible $1\text{ kHz}$ motor whine while keeping switching losses negligible.

---

## 3. The PCB Pull-Up Trap & Motor Safety Lock

### 3.1 Why Motors Run Away at 100% Speed on Exit

The `U10` level shifters on the Minibot interface PCB include pull-up resistors on their input lines. When a standard Python script finishes, crashes, or is interrupted with `Ctrl+C`, default GPIO libraries (`gpiozero` or `RPi.GPIO`) clean up by reverting pins to **High-Impedance Input Mode**. As soon as the pins float, the PCB pull-ups pull `PWM1` (`BCM 12`) and `PWM2` (`BCM 13`) up to $3.3\text{ V}$ ($100\%$ duty cycle), causing both motors to run at full speed indefinitely.

### 3.2 Three-Layer Safety Protection

1. **Immediate Terminal Kill Switch:** If the motors ever latch ON, force all four control pins low from the terminal:
   ```bash
   # Raspberry Pi OS Bookworm / Trixie
   pinctrl set 12,13,5,6 op dl

   # Legacy Raspberry Pi OS
   raspi-gpio set 12,13,5,6 op dl
   ```
2. **Boot-Time Pin Configuration (`/boot/firmware/config.txt`):** Keep the motor pins driven low from power-on:
   ```ini
   gpio=12,13,5,6=op,dl
   ```
3. **Python Exit Lock (`os._exit(0)`):** Inside the `finally:` block, set PWM duty cycles to `0`, execute `pinctrl set 12,13,5,6 op dl`, and call `os._exit(0)` to exit immediately without running `gpiozero`'s cleanup destructor.

---

## 4. Chassis Wheel Geometry & Dual-Track Kinematics

The Minibot uses **four coaxial wheels** aligned along the same transverse axle ($38\text{ mm}$ forward of the rear bumper):

| Parameter | Actuated Drive Wheels (Blue) | Passive Odometer Wheels (Black) | Physical Role |
| :--- | :--- | :--- | :--- |
| **Hardware** | Pololu 37D #4752 ($12\text{ V}$, $30:1$ gearbox) | CUI AMT10 capacitive rotary encoder | Blue wheels propel the robot; unpowered black dead-wheels track true ground motion without wheel slip |
| **Wheel Diameter ($D$)** | $0.060\text{ m}$ ($60\text{ mm}$, $R = 30\text{ mm}$) | $0.048\text{ m}$ ($48\text{ mm}$, $R = 24\text{ mm}$) | Determines linear distance per revolution ($C = \pi D$) |
| **Wheel Circumference ($C$)** | $0.18850\text{ m/rev}$ | $0.15080\text{ m/rev}$ | Scales encoder ticks into meters |
| **Centerline Offset ($y$)** | $\pm 0.090\text{ m}$ ($\pm 90\text{ mm}$) | $\pm 0.120\text{ m}$ ($\pm 120\text{ mm}$, $30\text{ mm}$ outboard) | Outer black wheels travel $\frac{0.240}{0.180} = 1.333\times$ farther than blue wheels during turns |
| **Track Width ($W$)** | $0.180\text{ m}$ ($180\text{ mm}$) | $0.240\text{ m}$ ($240\text{ mm}$) | Chassis yaw angle $\theta$ **must** divide by $W_{\text{odo}} = 0.240\text{ m}$ |

### Straight-Line vs. In-Place Rotation Kinematics

* **Straight Translation:** Both inner drive wheels and outer passive wheels trace the same linear distance. Forward travel $d$ and left/right drift are measured directly from the passive odometer distances $(d_L, d_R)$:
  $$d = \frac{d_L + d_R}{2}, \qquad \Delta d_{\text{drift}} = d_L - d_R$$
* **In-Place Rotation ($\omega$ around axle midpoint):** Because the outer black odometer wheels sit at radius $\frac{W_{\text{odo}}}{2} = 0.120\text{ m}$ while the inner blue drive wheels sit at radius $\frac{W_{\text{drive}}}{2} = 0.090\text{ m}$:
  * **Measured chassis yaw angle** from the outer passive odometers:
    $$\theta = \frac{d_R - d_L}{W_{\text{odo}}} = \frac{d_R - d_L}{0.240\text{ m}}$$
  * **Relationship between inner and outer wheel linear speeds** during a pure turn:
    $$v_{\text{drive}} = \omega \cdot \frac{W_{\text{drive}}}{2} = \frac{0.180}{0.240} v_{\text{odo}} = 0.75 \, v_{\text{odo}}$$

---

## 5. Translating Raw Telemetry into Real Physical Units ($\text{m/s}$, $\text{m}$, $\text{rad}$)

### 5.1 SPI Telemetry Register Summary (`SPI0 CE1`)

Each SPI read sends a 5-byte transaction `[addr & 0x7F, 0, 0, 0, 0]` and unpacks the 4 payload bytes as a signed 32-bit big-endian integer (`>i`):

| Register | Access | Content | Raw Units |
| :--- | :--- | :--- | :--- |
| **`0x10`** | Read (`>i`) | Left drive wheel speed | Signed ticks accumulated over $10\text{ ms}$ ($f_{\text{sample}} = 100\text{ Hz}$) |
| **`0x11`** | Read (`>i`) | Right drive wheel speed | Signed ticks accumulated over $10\text{ ms}$ ($f_{\text{sample}} = 100\text{ Hz}$) |
| **`0x13`** | Read (`>i`) | Left passive odometer | Cumulative signed ticks (`int32`) |
| **`0x14`** | Read (`>i`) | Right passive odometer | Cumulative signed ticks (`int32`) |
| **`0x7F` (`0xFF`)** | Write | Counter reset | Writing `0xFF` resets all speed and odometry counters to `0` |

### 5.2 Exact Encoder Resolution & Conversion Formulas

The hardware quadrature counter triggers on **both rising and falling edges of Channel A** ($2\times$ edge decoding per encoder cycle):

1. **Passive Odometer Distance ($d_L, d_R$ in meters):**
   * The CUI AMT10 encoder is configured to $2048\text{ PPR}$ (default DIP `0000`). Under $2\times$ edge decoding, one full revolution of the $48\text{ mm}$ black wheel produces:
     $$N_{\text{odo}} = 2048\text{ PPR} \times 2 = 4096\text{ ticks/rev}$$
   * Linear floor travel per odometer tick:
     $$\Delta s_{\text{odo}} = \frac{\pi \, D_{\text{odo}}}{N_{\text{odo}}} = \frac{\pi \times 0.048\text{ m}}{4096} = 3.68155 \times 10^{-5}\text{ m/tick} \quad (\approx 0.0368\text{ mm/tick})$$
   * Total distance traveled by each passive wheel in meters:
     $$d_L = \text{ticks}_{0x13} \times \Delta s_{\text{odo}}, \qquad d_R = \text{ticks}_{0x14} \times \Delta s_{\text{odo}}$$

2. **Actuated Drive Wheel Speed ($v_L, v_R$ in $\text{m/s}$):**
   * Pololu's #4752 motor shaft carries a $16\text{ PPR}$ magnetic encoder disc (often advertised as $64\text{ CPR}$ when decoded in $4\times$ mode). Under $2\times$ Channel-A edge decoding, the motor shaft outputs $16 \times 2 = 32\text{ ticks/rev}$, which after the $30:1$ gearbox yields:
     $$N_{\text{mot}} = 32 \times 30 = 960\text{ ticks/drive wheel rev}$$
   * Linear rim travel per motor tick on the $60\text{ mm}$ blue wheel:
     $$\Delta s_{\text{mot}} = \frac{\pi \, D_{\text{drive}}}{N_{\text{mot}}} = \frac{\pi \times 0.060\text{ m}}{960} = 1.96350 \times 10^{-4}\text{ m/tick} \quad (\approx 0.196\text{ mm/tick})$$
   * Since speed registers `0x10` and `0x11` report ticks counted during a $\Delta t = 0.010\text{ s}$ ($f_{\text{sample}} = 100\text{ Hz}$) window:
     $$v_L = \text{spd}_{0x10} \times f_{\text{sample}} \times \Delta s_{\text{mot}}, \qquad v_R = \text{spd}_{0x11} \times f_{\text{sample}} \times \Delta s_{\text{mot}}$$
   * **Physical Cross-Check:** During straight motion without wheel slip, $v_L$ and $v_R$ computed with $N_{\text{mot}} = 960$ match $\frac{\Delta d_L}{\Delta t}$ and $\frac{\Delta d_R}{\Delta t}$ measured by the passive odometers ($N_{\text{odo}} = 4096$).

3. **2D Differential-Drive Pose Integration $(x, y, \theta)$:**
   Given incremental displacements $\Delta d_L = d_{L,k} - d_{L,k-1}$ and $\Delta d_R = d_{R,k} - d_{R,k-1}$ from the passive odometers:
   $$\Delta s = \frac{\Delta d_R + \Delta d_L}{2}, \qquad \Delta \theta = \frac{\Delta d_R - \Delta d_L}{W_{\text{odo}}} \quad (W_{\text{odo}} = 0.240\text{ m})$$
   Using midpoint integration for accurate curved trajectories:
   $$x_k = x_{k-1} + \Delta s \cos\!\left(\theta_{k-1} + \frac{\Delta \theta}{2}\right), \qquad y_k = y_{k-1} + \Delta s \sin\!\left(\theta_{k-1} + \frac{\Delta \theta}{2}\right), \qquad \theta_k = \theta_{k-1} + \Delta \theta$$

---

## 6. Complete Verified Python Motor & Odometry Module (`minibot_wheels_odo.py`)

```python
import spidev
from gpiozero import PWMOutputDevice, OutputDevice
import time
import math
import struct
import os

# =============================================================================
# 1. HARDWARE PINOUT & MOTOR POLARITY
# =============================================================================
LEFT_PWM_PIN  = 12   # BCM 12 (Pin 32) -> PWM1
LEFT_DIR_PIN  = 5    # BCM 5  (Pin 29) -> M1DIR
RIGHT_PWM_PIN = 13   # BCM 13 (Pin 33) -> PWM2
RIGHT_DIR_PIN = 6    # BCM 6  (Pin 31) -> M2DIR

# Right motor & encoders are mounted 180 deg mirrored on the chassis
LEFT_MOTOR_INVERTED  = False
RIGHT_MOTOR_INVERTED = True

# =============================================================================
# 2. PHYSICAL DIMENSIONS & ENCODER SCALING CONSTANTS
# =============================================================================
SUPPLY_VOLTAGE        = 12.0      # V (Pololu #4752 & Dual MC33926 nominal supply)
SPEED_SAMPLE_FREQ_HZ  = 100.0     # Speed registers update every 10 ms (100 Hz)

# Blue Actuated Drive Wheels (Pololu 37D #4752: 30:1 gearbox, 60 mm diameter)
DRIVE_WHEEL_DIAM_M    = 0.060     # 60 mm diameter (R = 30 mm)
DRIVE_WHEEL_BASE_M    = 0.180     # 2 * 90 mm center-to-center track width
GEAR_RATIO            = 30.0
MOTOR_ENCODER_CPR     = 32        # 16 PPR * 2x edge decoding -> 960 ticks/wheel rev
TICKS_PER_DRIVE_REV   = MOTOR_ENCODER_CPR * GEAR_RATIO
METERS_PER_MOTOR_TICK = (math.pi * DRIVE_WHEEL_DIAM_M) / TICKS_PER_DRIVE_REV

# Black Passive Odometer Wheels (CUI AMT10: 2048 PPR * 2x edge decoding = 4096 ticks/rev)
ODO_WHEEL_DIAM_M      = 0.048     # 48 mm diameter (R = 24 mm)
ODO_WHEEL_OFFSET_M    = 0.030     # 30 mm (3 cm) outboard of each blue drive wheel
ODO_WHEEL_BASE_M      = DRIVE_WHEEL_BASE_M + 2.0 * ODO_WHEEL_OFFSET_M  # 0.240 m
ODO_ENCODER_CPR       = 4096      # 2048 PPR * 2x edge decoding
METERS_PER_ODO_TICK   = (math.pi * ODO_WHEEL_DIAM_M) / ODO_ENCODER_CPR

# =============================================================================
# 3. SPI TELEMETRY REGISTERS (SPI0 CE1, Mode 0)
# =============================================================================
REG_LEFT_SPEED  = 0x10   # int32: Left drive wheel ticks / 10 ms
REG_RIGHT_SPEED = 0x11   # int32: Right drive wheel ticks / 10 ms
REG_LEFT_ODO    = 0x13   # int32: Left passive odometer cumulative ticks
REG_RIGHT_ODO   = 0x14   # int32: Right passive odometer cumulative ticks
REG_RESET       = 0x7F   # Write 0xFF (WE=1 | 0x7F) to zero all counters


class MinibotWheelsOdo:
    def __init__(self):
        # 10 kHz ultrasonic PWM (eliminates 1 kHz whine on MC33926)
        self.m1_pwm = PWMOutputDevice(pin=LEFT_PWM_PIN, frequency=10000, initial_value=0)
        self.m1_dir = OutputDevice(pin=LEFT_DIR_PIN, initial_value=False)
        self.m2_pwm = PWMOutputDevice(pin=RIGHT_PWM_PIN, frequency=10000, initial_value=0)
        self.m2_dir = OutputDevice(pin=RIGHT_DIR_PIN, initial_value=False)

        # 2D Odometry Pose State (meters, radians)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_left_m = 0.0
        self._prev_right_m = 0.0

        # Initialize SPI0 CE1
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

    def _read_reg_i32(self, addr: int) -> int:
        if not self.spi:
            return 0
        rx = self.spi.xfer2([addr & 0x7F, 0x00, 0x00, 0x00, 0x00])
        return struct.unpack(">i", bytes(rx[1:5]))[0]

    def reset_odometry(self):
        """Resets hardware tick counters (0x7F) and zeroes local (x, y, theta) pose."""
        if self.spi:
            self.spi.xfer2([0x80 | REG_RESET, 0x00, 0x00, 0x00, 0x00])
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._prev_left_m = 0.0
        self._prev_right_m = 0.0

    def set_motor_voltage(self, left_volts: float, right_volts: float):
        """Commands left and right motor voltages in [-12.0 V, +12.0 V]."""
        left_volts = max(min(left_volts, SUPPLY_VOLTAGE), -SUPPLY_VOLTAGE)
        right_volts = max(min(right_volts, SUPPLY_VOLTAGE), -SUPPLY_VOLTAGE)

        eff_l = -left_volts if LEFT_MOTOR_INVERTED else left_volts
        eff_r = -right_volts if RIGHT_MOTOR_INVERTED else right_volts

        self.m1_dir.value = eff_l < 0
        self.m1_pwm.value = abs(eff_l) / SUPPLY_VOLTAGE

        self.m2_dir.value = eff_r < 0
        self.m2_pwm.value = abs(eff_r) / SUPPLY_VOLTAGE

    def update_telemetry(self):
        """
        Reads all 4 speed/odometry registers, converts counts into SI units (m/s and m),
        and integrates 2D differential-drive pose (x, y, theta).
        """
        raw_l_spd = self._read_reg_i32(REG_LEFT_SPEED)
        raw_r_spd = self._read_reg_i32(REG_RIGHT_SPEED)
        raw_l_odo = self._read_reg_i32(REG_LEFT_ODO)
        raw_r_odo = self._read_reg_i32(REG_RIGHT_ODO)

        if LEFT_MOTOR_INVERTED:
            raw_l_spd, raw_l_odo = -raw_l_spd, -raw_l_odo
        if RIGHT_MOTOR_INVERTED:
            raw_r_spd, raw_r_odo = -raw_r_spd, -raw_r_odo

        # Translate raw counts into real physical values
        v_left_mps  = raw_l_spd * SPEED_SAMPLE_FREQ_HZ * METERS_PER_MOTOR_TICK
        v_right_mps = raw_r_spd * SPEED_SAMPLE_FREQ_HZ * METERS_PER_MOTOR_TICK
        d_left_m    = raw_l_odo * METERS_PER_ODO_TICK
        d_right_m   = raw_r_odo * METERS_PER_ODO_TICK

        # Incremental displacement since last update
        dl = d_left_m - self._prev_left_m
        dr = d_right_m - self._prev_right_m
        self._prev_left_m = d_left_m
        self._prev_right_m = d_right_m

        ds = 0.5 * (dr + dl)
        dtheta = (dr - dl) / ODO_WHEEL_BASE_M

        mid_theta = self.theta + 0.5 * dtheta
        self.x += ds * math.cos(mid_theta)
        self.y += ds * math.sin(mid_theta)
        self.theta = (self.theta + dtheta + math.pi) % (2.0 * math.pi) - math.pi

        return {
            "v_left_mps": v_left_mps,
            "v_right_mps": v_right_mps,
            "d_left_m": d_left_m,
            "d_right_m": d_right_m,
            "x_m": self.x,
            "y_m": self.y,
            "theta_deg": math.degrees(self.theta),
        }

    def emergency_stop(self):
        """Drives PWM/DIR to 0V and locks GPIO pinmux LOW to beat PCB pull-ups."""
        try:
            self.set_motor_voltage(0.0, 0.0)
        except Exception:
            pass
        os.system("pinctrl set 12,13,5,6 op dl > /dev/null 2>&1 || raspi-gpio set 12,13,5,6 op dl > /dev/null 2>&1")
        print("[MOTORS] Driven 0V & Locked LOW")


if __name__ == "__main__":
    bot = MinibotWheelsOdo()
    try:
        print("\n=== Forward Motion Test (+4.5 V for 1.5 s) ===")
        bot.set_motor_voltage(4.5, 4.5)
        t0 = time.time()
        while time.time() - t0 < 1.5:
            t = bot.update_telemetry()
            print(
                f"t={time.time()-t0:4.2f}s | "
                f"vL={t['v_left_mps']:+.3f} m/s, vR={t['v_right_mps']:+.3f} m/s | "
                f"dL={t['d_left_m']:+.3f} m, dR={t['d_right_m']:+.3f} m | "
                f"Pose=({t['x_m']:+.3f} m, {t['y_m']:+.3f} m, {t['theta_deg']:+.1f} deg)"
            )
            time.sleep(0.05)

        bot.set_motor_voltage(0.0, 0.0)
        time.sleep(0.5)

        print("\n=== In-Place CCW Rotation (-4.0 V / +4.0 V for 1.0 s) ===")
        bot.set_motor_voltage(-4.0, +4.0)
        t0 = time.time()
        while time.time() - t0 < 1.0:
            t = bot.update_telemetry()
            print(
                f"t={time.time()-t0:4.2f}s | "
                f"vL={t['v_left_mps']:+.3f} m/s, vR={t['v_right_mps']:+.3f} m/s | "
                f"Pose=({t['x_m']:+.3f} m, {t['y_m']:+.3f} m, {t['theta_deg']:+.1f} deg)"
            )
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user.")
    finally:
        bot.emergency_stop()
        os._exit(0)
```