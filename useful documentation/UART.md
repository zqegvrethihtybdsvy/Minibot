# UART Communication & Control Guide

## 1. Overview of UART Communication

Universal Asynchronous Receiver-Transmitter (**UART**) is a hardware communication protocol that allows asynchronous serial communication between devices. Unlike synchronous protocols such as **I²C** or **SPI**, UART does not rely on a shared clock signal. Instead, both devices must agree on a predefined transmission speed (**Baud Rate**) and data framing parameters prior to communication.

### Core Principles
* **Asynchronous:** No separate clock line; data framing is controlled by start and stop bits.
* **Full-Duplex:** Dedicated channels for transmission ($\text{TX}$) and reception ($\text{RX}$) allow simultaneous bidirectional data flow.
* **Baud Rate ($B$):** The rate at which bits are transmitted per second ($\text{bits/s}$ or $\text{bps}$). The bit duration $T_{\text{bit}}$ is given by:

$$T_{\text{bit}} = \frac{1}{B}$$

For a standard baud rate of $9600 \text{ baud}$:

$$T_{\text{bit}} = \frac{1}{9600 \text{ s}^{-1}} \approx 104.17 \ \mu\text{s}$$

---

### **1.1 UART Data Framing & Packet Structure**

Because UART is asynchronous, there is no clock wire to tell the receiver exactly when a bit starts or ends. Instead, the transmitter and receiver rely on strict time synchronization governed by the baud rate, wrapped inside a standardized **data frame**:

```text
Idle (High) -> [ START (0) ] -> [ D0 ] -> [ D1 ] -> [ D2 ] -> [ D3 ] -> [ D4 ] -> [ D5 ] -> [ D6 ] -> [ D7 ] -> [ PARITY ] -> [ STOP (1) ] -> Idle (High)

```

1. **Idle State (`HIGH` / `1`):** When no data is being transmitted, the line is held at logic high (`3.3V` or `5V`).
2. **Start Bit (`LOW` / `0`):** A transition from high to low marks the beginning of a data packet, alerting the receiver's internal counter to start sampling bits at the specified baud rate interval ($T_{\text{bit}}$).
3. **Data Bits (typically 5 to 9 bits, usually 8):** The actual payload data, transmitted least significant bit ($\text{LSB}$) first.
4. **Parity Bit (Optional):** An error-checking bit.
* **Even Parity:** The bit is set so the total number of `1`s in the data payload + parity bit is even.
* **Odd Parity:** Set to make the total number of `1`s odd.
* *Note:* Often omitted in modern simple links (configured as "None").


5. **Stop Bit(s) (`HIGH` / `1`):** Lasting 1, 1.5, or 2 bit periods, this returns the line to the idle state and provides a safety margin for the receiver to reset before the next frame.

---

## 2. Hardware Architecture & Wiring

UART relies on cross-wired data lines and a common ground reference.

### Connection Pinout

| Raspberry Pi Pin | Signal | Target Device Pin |
| :--- | :--- | :--- |
| **GPIO 14** (Pin 8) | $\text{TX}$ (Transmit) | $\text{RX}$ (Receive) |
| **GPIO 15** (Pin 10) | $\text{RX}$ (Receive) | $\text{TX}$ (Transmit) |
| **GND** (Pin 6/9/14/...) | Ground | **GND** |

> **Warning on Voltage Levels:** Raspberry Pi GPIO operates at **3.3V TTL logic**. Connecting directly to 5V microcontrollers (e.g., standard Arduino Uno) without a logic level converter can permanently damage the Pi's GPIO pins.

---

To expand your guide and provide a deeper, more rigorous engineering understanding of the protocol, we can add a detailed breakdown of **Data Packet Framing**, **Hardware Flow Control (Handshaking)**, and a clear comparison against **I²C** and **SPI**.

Here are the sections you can integrate into your guide:

---

### **2.1 Hardware Flow Control (RTS / CTS)**

For basic point-to-point links (like your Raspberry Pi talking to a Minibot microcontroller), simple 3-wire connections ($\text{TX}$, $\text{RX}$, $\text{GND}$) are sufficient. However, for higher-speed or heavy data pipelines, **Hardware Flow Control** can be used via two additional lines:

* **RTS (Request to Send):** Output from the receiver indicating it is ready and has buffer space to accept data.
* **CTS (Clear to Send):** Input to the transmitter telling it that it is safe to start sending bytes.

This prevents **buffer overflow** on microcontrollers with limited RAM if the receiver gets temporarily busy processing other tasks.

---

## 3. Python Implementation (`pySerial`)

Below is a robust, modular Python class for sending string commands and receiving responses over UART.

```python
import serial
import time
from typing import Optional

class UARTController:
    """Handles UART serial communication with target microcontrollers."""

    def __init__(self, port: str = '/dev/serial0', baudrate: int = 9600, timeout: float = 1.0):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )
        # Allow time for port initialization
        time.sleep(0.1)

    def send_command(self, command: str, delimiter: str = '\n', delay: float = 0.05) -> Optional[str]:
        """
        Sends a command string over UART and reads the target response.
        
        :param command: String command to send.
        :param delimiter: End-of-line marker expected by target firmware.
        :param delay: Processing pause before reading response.
        :return: Decoded string response or None if timed out.
        """
        # 1. Clear stale data from receive buffer
        self.ser.reset_input_buffer()

        # 2. Append terminator and encode to bytes
        full_command = command + delimiter
        self.ser.write(full_command.encode('utf-8'))
        print(f"[UART OUT] Sent: {repr(command)}")

        # 3. Pause for firmware processing execution
        time.sleep(delay)

        # 4. Read response until newline
        raw_response = self.ser.readline()
        if raw_response:
            response_str = raw_response.decode('utf-8', errors='ignore').strip()
            print(f"[UART IN]  Received: {repr(response_str)}")
            return response_str
        else:
            print("[UART IN]  Warning: Timeout reached, no response received.")
            return None

    def close(self):
        """Closes the serial connection safely."""
        if self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    controller = UARTController(port='/dev/serial0', baudrate=9600)

    try:
        # Example LED toggling commands
        controller.send_command("Turn Led On")
        time.sleep(1)
        controller.send_command("Turn Led Off")
        
    finally:
        controller.close()
```

---
