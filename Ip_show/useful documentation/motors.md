# Minibot Motor Control & Odometry Guide

## 1. System Architecture Overview

The Minibot motor system is split into two independent hardware control loops:

1. **Actuation (Motor Control):** Handled directly by the **Raspberry Pi GPIO pins** connected to the H-Bridge drivers via logic gates and level shifters.

2. **Sensing (Speed & Odometry Telemetry):** Handled by quadrature optical encoders and passive tracking wheels connected to the **DE0-Nano FPGA**, which communicates feedback to the Raspberry Pi over the **SPI bus (SPI0 CE1)**.

$$
\text{Raspberry Pi GPIO} \xrightarrow{\text{PWM / DIR}} \text{H-Bridge Driver} \xrightarrow{\text{Voltage}} \mathbf{\text{DC Motors}}
$$

$$
\mathbf{\text{Encoders / Odometers}} \xrightarrow{\text{Quadrature Signals}} \text{DE0-Nano FPGA} \xrightarrow{\text{SPI0 CE1}} \text{Raspberry Pi}
$$

## 2. Hardware Pin Mapping & Circuit Layout

The motor direction (`DIR`) and speed (`PWM`) signals pass through level shifters (`U10` PI4ULS5V108) and logic gates (`U2`, `U3`, `U14`, `U15` 74LVC1G97DW-7) on the main PCB (`Sheet_1`).

### Pinout Table

| Function | Motor | Raspberry Pi BCM | Physical Header Pin | PCB Net Name | Logic State | 
| ----- | ----- | ----- | ----- | ----- | ----- | 
| **Left PWM** | Motor 1 (M1) | **BCM 12** | Pin 32 | `PWM1` | Duty Cycle ($0\%$ to $100\%$) | 
| **Left DIR** | Motor 1 (M1) | **BCM 5** | Pin 29 | `GPIO05` / `M1DIR` | `0` = Forward, `1` = Reverse | 
| **Right PWM** | Motor 2 (M2) | **BCM 13** | Pin 33 | `PWM2` | Duty Cycle ($0\%$ to $100\%$) | 
| **Right DIR** | Motor 2 (M2) | **BCM 6** | Pin 31 | `GPIO06` / `M2DIR` | `0` = Forward, `1` = Reverse | 

## 3. The PCB Pull-Up Trap & Safety Lock

### 3.1 Why Motors Run Away at 100% Speed

The level shifters (`U10`) on the Minibot PCB feature internal/external pull-up resistors on the input logic lines.

When a standard Python script finishes, crashes, or is interrupted via `Ctrl+C`, default GPIO libraries (`gpiozero` or `RPi.GPIO`) clean up by resetting all pins back to **Input Mode (High-Impedance)**. As soon as the pins float high-impedance, the PCB pull-up resistors pull `PWM1` (BCM 12) and `PWM2` (BCM 13) up to **3.3V (HIGH)**, causing both motors to spin at **100% full speed indefinitely**.

### 3.2 Solutions for Motor Safety

#### Solution A: Terminal Emergency Stop

If the motors run away, execute this command directly from the Raspberry Pi terminal to lock the pins to 0V:

```
# Raspberry Pi OS Bookworm
pinctrl set 12,13,5,6 op dl

# Older Raspberry Pi OS versions
raspi-gpio set 12,13,5,6 op dl

```

#### Solution B: Permanent Hardware Boot Configuration

To prevent pins from floating HIGH during boot or script exit, add the following line to `/boot/firmware/config.txt` (or `/boot/config.txt`):

```
# Force Motor PWM & DIR pins to Output Driven Low on startup
gpio=12,13,5,6=op,dl

```

#### Solution C: Python Exit Lock (`os._exit(0)`)

In Python scripts, bypass `gpiozero`'s destructor cleanup by calling `os._exit(0)` inside the `finally` block after forcing `pinctrl set 12,13,5,6 op dl`.

## 4. Encoder vs. Odometer Hardware Telemetry

On the Minibot, two different types of rotational sensors exist:

1. **Drive Motor Encoders (`Enc 1` & `Enc 2`):** Mounted directly on the drive motor shafts. They measure **motor speed** (`0x10`). They register speed immediately when wheels spin, even if the robot is lifted in the air.

2. **Passive Odometers (`Odo 1` & `Odo 2`):** Unpowered trailing tracking wheels mounted under the robot chassis. They measure **actual physical translation across the floor** (`0x13`). If the robot is tested on a bench with wheels in the air, the odometer counter stays near `0`.

### FPGA Default SPI Register Map (`SPI0 CE1`)

| SPI Address | Access | Description | Firmware Status | 
| ----- | ----- | ----- | ----- | 
| **`0x10`** | Read | Left Motor Encoder Speed ($f = 100\text{ Hz}$) | Active in Baseline Firmware | 
| **`0x11`** | Read | Right Motor Encoder Speed | Unmapped in Baseline Firmware | 
| **`0x13`** | Read | Left Passive Odometer Tick Counter | Active in Baseline Firmware | 
| **`0x14`** | Read | Right Passive Odometer Tick Counter | Unmapped in Baseline Firmware | 
| **`0x17`** | Read | Diagnostic Test Register (`0x01 0x02 0x03 0x04`) | Active in Baseline Firmware | 
| **`0x7F`** | Write | Master Reset (Clears counters to 0) | Active (`WE=1` $\rightarrow$ `0xFF`) | 

*Note: Data payload bytes returned over SPI are 32-bit signed integers in **Big-Endian (`>i`)** format.*

## 5. Complete Integrated Python Motor Controller

The following script (`minibot_motors.py`) provides full dual-motor control, safe shutdown handling, SPI reading from the DE0-Nano, and odometer counter reset logic:

```
import spidev
from gpiozero import PWMOutputDevice, OutputDevice
import time
import struct
import os

# Pin Mapping
LEFT_PWM_PIN  = 12
LEFT_DIR_PIN  = 5
RIGHT_PWM_PIN = 13
RIGHT_DIR_PIN = 6

# SPI Registers
REG_LEFT_SPEED = 0x10
REG_LEFT_ODO   = 0x13
REG_RESET      = 0x7F

class MinibotMotorController:
    def __init__(self):
        # Initialize pins actively LOW (0V)
        self.m1_pwm = PWMOutputDevice(pin=LEFT_PWM_PIN, frequency=1000, initial_value=0)
        self.m1_dir = OutputDevice(pin=LEFT_DIR_PIN, initial_value=False)
        self.m2_pwm = PWMOutputDevice(pin=RIGHT_PWM_PIN, frequency=1000, initial_value=0)
        self.m2_dir = OutputDevice(pin=RIGHT_DIR_PIN, initial_value=False)

        # Initialize SPI0 CE1
        self.spi = spidev.SpiDev()
        try:
            self.spi.open(0, 1)  # Bus 0, Device 1 (CE1)
            self.spi.max_speed_hz = 1000000  # 1 MHz
            self.spi.mode = 0
            print("[SPI] Connected to DE0-Nano on SPI0 CE1")
            self.reset_fpga_counters()
        except Exception as e:
            print(f"[SPI ERROR] {e}")
            self.spi = None

    def reset_fpga_counters(self):
        """Sends WE=1 to Register 0x7F to reset internal FPGA counters."""
        if self.spi:
            # Command byte: WE bit 8 = 1 -> 0x80 | 0x7F = 0xFF
            tx = [0xFF, 0x00, 0x00, 0x00, 0x00]
            self.spi.xfer2(tx)
            print("[SPI] FPGA internal counters reset (0x7F)")

    def set_motors(self, left_speed: float, right_speed: float):
        """
        Sets speed for Left and Right motors (-1.0 to 1.0).
        """
        # Left Motor (M1)
        if left_speed >= 0:
            self.m1_dir.off()
            self.m1_pwm.value = min(1.0, left_speed)
        else:
            self.m1_dir.on()
            self.m1_pwm.value = min(1.0, abs(left_speed))

        # Right Motor (M2)
        if right_speed >= 0:
            self.m2_dir.off()
            self.m2_pwm.value = min(1.0, right_speed)
        else:
            self.m2_dir.on()
            self.m2_pwm.value = min(1.0, abs(right_speed))

    def read_left_telemetry(self):
        """Reads Left Motor Speed (0x10) and Left Passive Odometer (0x13)."""
        if not self.spi:
            return 0, 0
        
        # Read 0x10 (Left Speed)
        rx_spd = self.spi.xfer2([REG_LEFT_SPEED & 0x7F, 0, 0, 0, 0])
        spd = struct.unpack(">i", bytes(rx_spd[1:5]))[0]

        # Read 0x13 (Left Odometer)
        rx_odo = self.spi.xfer2([REG_LEFT_ODO & 0x7F, 0, 0, 0, 0])
        odo = struct.unpack(">i", bytes(rx_odo[1:5]))[0]

        return spd, odo

    def emergency_stop(self):
        """Drives PWM and DIR to 0V and locks hardware pinmux LOW."""
        try:
            self.m1_pwm.value = 0
            self.m2_pwm.value = 0
            self.m1_dir.off()
            self.m2_dir.off()
        except Exception:
            pass

        # Force SoC pinmux state to Output Driven Low (0V)
        os.system("pinctrl set 12,13,5,6 op dl > /dev/null 2>&1 || raspi-gpio set 12,13,5,6 op dl > /dev/null 2>&1")
        print("[MOTORS] Driven 0V & Locked LOW")

if __name__ == "__main__":
    bot = MinibotMotorController()
    header = f"{'Time(s)':<8} | {'Left Motor Speed (0x10)':<23} | {'Left Passive Odo (0x13)':<23}"
    
    try:
        print("\n=== Driving FORWARD (50%) ===")
        print(header)
        print("-" * 60)
        bot.set_motors(0.5, 0.5)
        t0 = time.time()
        for _ in range(20):
            spd, odo = bot.read_left_telemetry()
            print(f"{time.time()-t0:<8.2f} | {spd:<23} | {odo:<23}")
            time.sleep(0.1)

        bot.set_motors(0.0, 0.0)
        time.sleep(0.5)

        print("\n=== Driving BACKWARD (50%) ===")
        print(header)
        print("-" * 60)
        bot.set_motors(-0.5, -0.5)
        t0 = time.time()
        for _ in range(20):
            spd, odo = bot.read_left_telemetry()
            print(f"{time.time()-t0:<8.2f} | {spd:<23} | {odo:<23}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[STOP] Program interrupted by user.")
    finally:
        bot.emergency_stop()
        # Bypass gpiozero destructor cleanup to keep pins locked LOW
        os._exit(0)

```