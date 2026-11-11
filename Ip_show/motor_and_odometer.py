import spidev
from gpiozero import PWMOutputDevice, OutputDevice
import time
import struct
import os

LEFT_PWM_PIN  = 12
LEFT_DIR_PIN  = 5
RIGHT_PWM_PIN = 13
RIGHT_DIR_PIN = 6

# DE0-Nano Register Definitions
REG_LEFT_SPEED  = 0x10
REG_RIGHT_SPEED = 0x11  # Try 0x11 or 0x12 if right speed remains 0
REG_LEFT_ODO    = 0x13
REG_RIGHT_ODO   = 0x14  # Try 0x14 or 0x15 if right odo remains 0
REG_RESET       = 0x7F

class MinibotDriveOdo:
    def __init__(self):
        self.m1_pwm = PWMOutputDevice(pin=LEFT_PWM_PIN, frequency=1000, initial_value=0)
        self.m1_dir = OutputDevice(pin=LEFT_DIR_PIN, initial_value=False)
        self.m2_pwm = PWMOutputDevice(pin=RIGHT_PWM_PIN, frequency=1000, initial_value=0)
        self.m2_dir = OutputDevice(pin=RIGHT_DIR_PIN, initial_value=False)

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
        """Sends WE=1 to Register 0x7F to reset internal FPGA counters."""
        if self.spi:
            # Bit 8 = 1 (WE) -> 0x80 | 0x7F = 0xFF
            tx = [0xFF, 0x00, 0x00, 0x00, 0x00]
            self.spi.xfer2(tx)
            print("[SPI] Odometer reset command sent (0x7F)")

    def set_motors(self, left_speed: float, right_speed: float):
        if left_speed >= 0:
            self.m1_dir.off()
            self.m1_pwm.value = min(1.0, left_speed)
        else:
            self.m1_dir.on()
            self.m1_pwm.value = min(1.0, abs(left_speed))

        if right_speed >= 0:
            self.m2_dir.off()
            self.m2_pwm.value = min(1.0, right_speed)
        else:
            self.m2_dir.on()
            self.m2_pwm.value = min(1.0, abs(right_speed))

    def _read_spi_reg(self, reg_addr: int) -> int:
        if not self.spi:
            return 0
        tx = [reg_addr & 0x7F, 0x00, 0x00, 0x00, 0x00]
        rx = self.spi.xfer2(tx)
        return struct.unpack(">i", bytes(rx[1:5]))[0]

    def get_telemetry(self):
        return (
            self._read_spi_reg(REG_LEFT_SPEED),
            self._read_spi_reg(REG_RIGHT_SPEED),
            self._read_spi_reg(REG_LEFT_ODO),
            self._read_spi_reg(REG_RIGHT_ODO)
        )

    def emergency_stop(self):
        try:
            self.m1_pwm.value = 0
            self.m2_pwm.value = 0
            self.m1_dir.off()
            self.m2_dir.off()
        except Exception:
            pass
        os.system("pinctrl set 12,13,5,6 op dl > /dev/null 2>&1 || raspi-gpio set 12,13,5,6 op dl > /dev/null 2>&1")

if __name__ == "__main__":
    bot = MinibotDriveOdo()
    header = f"{'Time(s)':<8} | {'L Spd':<7} | {'R Spd':<7} | {'L Odo':<12} | {'R Odo':<12}"
    
    try:
        print("\n=== Driving FORWARD (50%) ===")
        print(header)
        print("-" * 65)
        bot.set_motors(0.5, 0.5)
        t0 = time.time()
        for _ in range(20):
            l_spd, r_spd, l_odo, r_odo = bot.get_telemetry()
            print(f"{time.time()-t0:<8.2f} | {l_spd:<7} | {r_spd:<7} | {l_odo:<12} | {r_odo:<12}")
            time.sleep(0.1)

        bot.set_motors(0.0, 0.0)
        time.sleep(0.5)

        print("\n=== Driving BACKWARD (50%) ===")
        print(header)
        print("-" * 65)
        bot.set_motors(-0.5, -0.5)
        t0 = time.time()
        for _ in range(20):
            l_spd, r_spd, l_odo, r_odo = bot.get_telemetry()
            print(f"{time.time()-t0:<8.2f} | {l_spd:<7} | {r_spd:<7} | {l_odo:<12} | {r_odo:<12}")
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n[STOP] Script interrupted.")
    finally:
        bot.emergency_stop()
        os._exit(0)