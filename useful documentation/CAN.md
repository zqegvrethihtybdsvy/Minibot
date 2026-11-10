# Controller Area Network (CAN Bus) & MCP25050 Guide

## 1. Overview of CAN Bus Architecture

The **Controller Area Network (CAN)** is a robust, message-based protocol designed to allow microcontrollers and embedded devices to communicate without a host computer. Originally developed by Bosch for automotive application, CAN is now widely used in robotics, industrial automation, and mechatronic systems.

### Core Properties
- **Differential Signaling:** Highly immune to electrical noise.
- **Multi-Master / Broadcast:** Any node can transmit when the bus is free; every frame is visible to all nodes.
- **Message-Based Addressing:** Nodes do not have protocol addresses; instead, **Messages (Identifiers)** have priorities.
- **Non-Destructive Bitwise Arbitration:** If two nodes start transmitting simultaneously, the message with the lower numerical ID wins control of the bus without data corruption.

---

## 2. Physical Layer & Electrical Characteristics

CAN uses a balanced differential pair consisting of two lines: **CAN_H** (CAN High) and **CAN_L** (CAN Low).


- **Dominant State (Logic `0`):** Driven state. $\text{CAN\_H} \approx 3.5\text{V}$, $\text{CAN\_L} \approx 1.5\text{V}$ ($\Delta V \approx 2.0\text{V}$).
- **Recessive State (Logic `1`):** Idle/floating state via termination. $\text{CAN\_H} \approx 2.5\text{V}$, $\text{CAN\_L} \approx 2.5\text{V}$ ($\Delta V \approx 0.0\text{V}$).
- **Termination:** Requires a $120\,\Omega$ resistor at each physical end of the bus line to prevent signal reflection.

\*Note: The Output bits are given through $\text{CAN\_H}- \text{CAN\_L}$

---

## 3. CAN Frame Architecture

A standard CAN 2.0A frame consists of several distinct bit fields:

```text
| SOF | 11-bit Identifier | RTR | IDE | r0 | DLC (4 bits) | Data Field (0-8 bytes) | CRC (15 bits) | ACK | EOF |
```

### Key Field Descriptions
1. **Start of Frame (SOF):** Single dominant bit (`0`) to synchronize nodes: Recessive State → Dominant State 

   → similar to UART
   
2. **Identifier (ID):** 
   - **Standard CAN 2.0A:** 11-bit ID.
   - **Extended CAN 2.0B:** 29-bit ID.
   - *Determines message priority (lower numerical value = higher priority).*
3. **Remote Transmission Request (RTR):**
   - `0` (Dominant): Data Frame.
   - `1` (Recessive): Remote Frame (request for data from another node).
4. **Data Length Code (DLC):** 4-bit integer specifying the payload size ($0$ to $8$ bytes).
5. **Data Payload:** $0$ to $8$ bytes of application data.
6. **CRC Field:** 15-bit Cyclic Redundancy Check for detecting transmission errors.
7. **ACK Field:** Receiving nodes overwrite this bit with a dominant `0` if received error-free.

---

## 4. MCP25050 I/O Expander Architecture

The **Microchip MCP25050** (or emulated target) is a CAN-I/O expander device. It translates raw CAN frames directly into digital pin manipulation (GPIO) or ADC/PWM signals without needing a local CPU.

### 4.1 Addressing & Configuration (Minibot Context)
- **Base IDs:**
  - **Read Address:** `0x700`
  - **Write Address:** `0x708`
- **Identifier Type:** Standard (11-bit).

### 4.2 Register & Command Mapping (Datasheet Summary)

#### Command Opcodes (Table 4-2)
The MCP25050 uses specific payload structures to read or modify internal hardware registers.

| Command Type | Opcode Structure / Bits | Description |
| :--- | :--- | :--- |
| **Register Read** | Request payload specifying target register | Requests the chip to return register contents |
| **Register Write** | Byte string with Register Address, Data, and Mask | Modifies target bits within internal registers |
| **Bit Set/Clear** | Bit Modification Commands (`AND`/`OR` mask) | Safely toggles individual GPIO lines without overwriting adjacent pins |

#### Key GPIO Registers (Page 19 & Register 5-2)
- **`GPLAT` (GPIO Latch Register):** 
  - Controls output logic levels for physical I/O pins.
  - Writing a `1` to the specific GPIO bit sets the pin **HIGH** (illuminating an active-high LED or enabling a gate).
  - Writing a `0` clears the pin **LOW**.

---

## 5. Linux SocketCAN & Raspberry Pi Setup

Raspberry Pi uses **SocketCAN**, a network stack that exposes the MCP2515 CAN controller as a standard Linux network interface (`can0`).

### 5.1 System Interface Configuration
Verify and bring up the CAN interface at the required bit rate (e.g., $500\text{ kbit/s}$):

```bash
# Bring up can0 interface
sudo ip link set can0 up type can bitrate 500000

# Verify interface status
ifconfig can0
```

### 5.2 Command Line Utilities (`can-utils`)
- **Monitor Bus Traffic:**
  ```bash
  candump can0
  ```
- **Transmit Frame:**
  ```bash
  # Send data frame to ID 0x708 with 3 payload bytes
  cansend can0 708#1234FF
  ```

---

## 6. Python SocketCAN Implementation

Using Python's native `python-can` library to communicate with the CAN bus:

```python
import can
import time

def setup_can_bus(channel='can0', bustype='socketcan', bitrate=500000):
    """Initializes the CAN bus interface."""
    try:
        bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=bitrate)
        print(f"[CAN] Interface connected on {channel}")
        return bus
    except Exception as e:
        print(f"[CAN] Error connecting to interface: {e}")
        return None

def send_mcp25050_write(bus, target_id=0x708, reg_addr=0x1E, mask=0x01, data=0x01):
    """
    Sends a Register Write frame to the MCP25050.
    
    :param bus: Active CAN bus object.
    :param target_id: CAN Arbitration ID (0x708 for Write).
    :param reg_addr: Register address (e.g., GPLAT).
    :param mask: Bit mask to select which pins to alter.
    :param data: Pin values (1 = High, 0 = Low).
    """
    # Construction of MCP25050 register modification frame payload
    payload = [reg_addr, mask, data]
    
    msg = can.Message(
        arbitration_id=target_id,
        data=payload,
        is_extended_id=False
    )
    
    try:
        bus.send(msg)
        print(f"[CAN OUT] ID: 0x{target_id:03X} Payload: {[hex(x) for x in payload]}")
    except can.CanError:
        print("[CAN OUT] Message failed to transmit")

if __name__ == "__main__":
    bus = setup_can_bus()
    if bus:
        # Example: Set LED pin HIGH
        send_mcp25050_write(bus, target_id=0x708, reg_addr=0x1E, mask=0x01, data=0x01)
        time.sleep(1)
        # Example: Clear LED pin LOW
        send_mcp25050_write(bus, target_id=0x708, reg_addr=0x1E, mask=0x01, data=0x00)
```

---

## 7. Advantages and Disadvantages of CAN Bus

### Advantages

* **High Reliability & Noise Immunity:** Uses differential signaling ($\text{CANH} - \text{CANL}$), making it exceptionally resistant to electrical noise and electromagnetic interference (EMI) in harsh industrial or automotive environments.

* **Bus Topography (Reduced Wiring):** Replaces complex point-to-point wiring harnesses with a simple two-wire differential bus, significantly reducing weight, material cost, and installation complexity.

* **Built-in Error Handling & Fault Confinement:** Supports automatic retransmission of corrupted frames, CRC verification, and node status monitoring (Error Active, Error Passive, Bus-Off) to prevent a failing node from disrupting the network.

* **Nondestructive Bitwise Arbitration:** Priority-based message transmission using frame Identifiers ensures high-priority messages (e.g., emergency braking or motor safety) gain instant bus access without packet collision or data loss.

* **Multi-Master Architecture:** Any node on the bus can transmit data whenever the bus is idle, eliminating single points of failure associated with master-slave configurations.

### Disadvantages

* **Limited Payload Size:** Standard CAN (2.0A/B) limits data payloads to a maximum of 8 bytes per frame, making it inefficient for transmitting large data streams (such as video feeds or high-resolution imagery).

* **Bandwidth Limitations:** Maximum data rates are limited to $1\text{ Mbps}$ at standard cable lengths (and decrease significantly as bus length increases), which is slower compared to Modern Ethernet or CAN FD.

* **Bus Termination Dependency:** Requires precise impedance matching ($120\,\Omega$ termination resistors at both physical ends of the bus line) to prevent signal reflections and bus corruption.

* **Complex Debugging & Software Overhead:** Protocol framing, identifier arbitration, filtering masks, and state-machine transitions require specialized hardware analyzers and software stacks to diagnose electrical or logical bus faults.

* **Security Constraints:** Native CAN 2.0 lacks built-in encryption, message authentication, or sender verification, making raw buses vulnerable to spoofing or packet injection if physical access is compromised.