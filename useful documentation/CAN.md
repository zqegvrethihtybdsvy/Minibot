# Controller Area Network (CAN Bus) & MCP25050 Guide



## 1. Overview of CAN Bus Architecture



The **Controller Area Network (CAN)** is a robust, message-based protocol designed to allow microcontrollers and embedded devices to communicate without a host computer. Originally developed by Bosch for automotive applications, CAN is now widely used in robotics, industrial automation, and mechatronic systems.

### Core Properties



* **Differential Signaling:** Highly immune to electrical noise.


* **Multi-Master / Broadcast:** Any node can transmit when the bus is free; every frame is visible to all nodes.


* **Message-Based Addressing:** Nodes do not have protocol addresses; instead, **Messages (Identifiers)** have priorities.


* **Non-Destructive Bitwise Arbitration:** If two nodes start transmitting simultaneously, the message with the lower numerical ID wins control of the bus without data corruption.



### 1.1 Hardware Architecture: Controller vs. Transceiver

In systems such as the Minibot, two distinct ICs are required to implement the full CAN stack. They perform two completely different hardware functions:

1. **MCP2515 = CAN Controller (Logic & Protocol)**
* **Role:** Handles the digital CAN protocol logic. Manages message frames, bit timing, acceptance masks, receive/transmit buffers, and CRC error checking.
* **Interfacing:** Connects to the host microcontroller (e.g., Raspberry Pi) via the SPI bus (`SCK`, `SI`, `SO`, `CS`).


2. **MCP2551 (or MCP2550/MCP2561) = CAN Transceiver (Physical Layer)**
* **Role:** Translates digital logic levels ($0\text{V} / 3.3\text{V} / 5\text{V}$ TTL from `TXD`/`RXD`) into high-voltage differential signals (`CANH` and `CANL`) capable of driving physical wires over long distances without interference.
* **Interfacing:** Connects directly between the MCP2515 controller's TX/RX logic pins and the physical two-wire differential bus.



$$\text{Raspberry Pi} \xrightarrow{\text{SPI}} \mathbf{\text{MCP2515 (Controller)}} \xrightarrow{\text{TX/RX}} \mathbf{\text{MCP2551 (Transceiver)}} \xrightarrow{\text{CANH/CANL}} \text{CAN Bus}$$

---

## 2. Physical Layer & Electrical Characteristics



CAN uses a balanced differential pair consisting of two lines: **CAN_H** (CAN High) and **CAN_L** (CAN Low).

* **Dominant State (Logic `0`):** Driven state. $\text{CAN\_H} \approx 3.5\text{V}$, $\text{CAN\_L} \approx 1.5\text{V}$ ($\Delta V \approx 2.0\text{V}$).


* **Recessive State (Logic `1`):** Idle/floating state via termination. $\text{CAN\_H} \approx 2.5\text{V}$, $\text{CAN\_L} \approx 2.5\text{V}$ ($\Delta V \approx 0.0\text{V}$).


* **Termination:** Requires a $120\,\Omega$ resistor at each physical end of the bus line to prevent signal reflection.



*Note: Differential output voltage is given by $\text{CAN\_H} - \text{CAN\_L}$.

---

## 3. Detailed CAN 2.0A Frame Architecture

### 3.1 Standard Data Frame Bit Architecture

```text
+---+----------------+-----+-----+----+----------+----------------+----------------+-----+----------+-----+----------+-----------+
|SOF| Identifier (ID)| RTR | IDE | r0 | DLC (4b) |  Data (0-8B)   |   CRC (15b)    | DEL | ACK Slot | DEL | EOF (7b) | ITM (3b)  |
+---+----------------+-----+-----+----+----------+----------------+----------------+-----+----------+-----+----------+-----------+
 1b       11b          1b    1b   1b     4b         0 to 64b            15b          1b       1b       1b      7b         3b

```

### 3.2 Field Mapping & Comparison

| Section | Image Diagram Field | Text Description Field | Purpose & Details |
| --- | --- | --- | --- |
| **Start of Frame** | SOF | SOF | Single dominant bit (`0`) to synchronize node clocks. |
| **Arbitration Field** | Identifier (11 bits)<br>RTR (1 bit) | 11-bit Identifier<br>RTR | **ID:** Identifies priority and message content.<br>**RTR:** `0` = Data frame, `1` = Remote request frame. |
| **Control Field** | IDE (1 bit)<br>r / r0 (1 bit)<br>DLC (4 bits) | IDE<br>r0<br>DLC (4 bits) | **IDE:** Identifier Extension bit (`0` = Standard 11-bit, `1` = Extended 29-bit).<br>**r0:** Reserved bit (must be `0`).<br>**DLC:** Payload length specification (0 to 8 bytes). |
| **Data Field** | Data Field | Data Field (0-8 bytes) | Payload (0 to 64 bits / 0 to 8 bytes). |
| **CRC Field** | CRC Sequence (15 bits)<br>DEL (1 bit) | CRC (15 bits) | **CRC Sequence:** Error detection polynomial.<br>**DEL (CRC Delimiter):** A required single recessive bit (`1`) separating CRC from ACK. |
| **ACK Field** | ACK (1 bit)<br>DEL (1 bit) | ACK | **ACK Slot:** Transmitter sends `1`; receiver overwrites with `0` if frame is valid.<br>**DEL (ACK Delimiter):** A required single recessive bit (`1`). |
| **End of Frame** | EOF (7 bits) | EOF | 7 consecutive recessive bits (`1111111`) signaling frame end. |
| **Interframe Space** | ITM (3 bits) | *(Omitted in brief text)* | **Intermission (ITM / IFS):** 3 recessive bits required between consecutive frames before the bus can return to idle. |

### 3.3 Standard Spec vs. Simplified Representations

1. **Logical Groupings:**
   * **Arbitration Field:** ID + RTR
   * **Control Field:** IDE + r0 + DLC
   * **CRC Field:** CRC Sequence + CRC Delimiter
   * **ACK Field:** ACK Slot + ACK Delimiter
2. **Intermission (ITM):** The mandatory 3-bit Interframe Space that must elapse before another SOF bit or Bus Idle state can occur.

---


### 3.4 Detailed Bit-by-Bit Field Specifications & Low-Level Coding Guide

1. **Start of Frame (SOF) — 1 Bit**
* **Bit Value:** `0` (Dominant).
* **Function:** Triggers hard synchronization across all receivers on the bus by transitioning the line from Recessive (`1`) to Dominant (`0`).


* **Coding Note:** Managed automatically by the CAN MAC protocol engine (e.g., MCP2515). You do not manually write this bit in software.


2. **Arbitration Field (12 Bits Total)**
* **Identifier ($\text{ID}[10:0]$) — 11 Bits:**
* **Function:** Uniquely identifies the message and establishes bus priority during arbitration (lower numerical value = higher priority). Sent MSB-first ($\text{ID10} \rightarrow \text{ID0}$).


* **Low-Level Register Packing (e.g., MCP2515 `TXBnSIDH` / `TXBnSIDL`):**
* `TXBnSIDH[7:0]` = $\text{ID}[10:3]$
* `TXBnSIDL[7:5]` = $\text{ID}[2:0]$




* **Remote Transmission Request (RTR) — 1 Bit:**
* **Bit Value:** `0` (Dominant) for Data Frame; `1` (Recessive) for Remote Frame (requesting data from another node).


* **Coding Note:** When building a SocketCAN `can_frame` struct in Python or C, setting the `CAN_RTR_FLAG` bit in the ID field automatically toggles this bit to `1`.




3. **Control Field (6 Bits Total)**
* **Identifier Extension (IDE) — 1 Bit:**
* **Bit Value:** `0` (Dominant) for Standard 11-bit CAN 2.0A; `1` (Recessive) for Extended 29-bit CAN 2.0B.




* **Reserved Bit (r0) — 1 Bit:**
* **Bit Value:** `0` (Dominant). Required by ISO 11898 for backward compatibility.


* **Data Length Code ($\text{DLC}[3:0]$) — 4 Bits:**
* **Bit Value:** Binary representation of payload size (`0000` = 0 bytes, `1000` = 8 bytes).


* **Coding Note:** Values `1001` through `1111` ($9\text{--}15$) are accepted by CAN controllers but still cap the payload size at 8 bytes.




4. **Data Payload (0 to 64 Bits / 0 to 8 Bytes)**
* **Field Length:** $N \times 8 \text{ bits}$ (where $0 \le N \le 8$).


* **Transmission Order:** Transmitted byte-by-byte ($\text{Byte } 0 \rightarrow \text{Byte } N-1$), with the most significant bit ($\text{D7}$) of each byte sent first.
* **Coding Note:** Byte ordering depends on higher-layer protocols.


5. **Cyclic Redundancy Check (CRC) Field (16 Bits Total)**
* **CRC Sequence — 15 Bits:**
* **Polynomial:** $X^{15} + X^{14} + X^{10} + X^8 + X^7 + X^4 + X^3 + 1$.
* **Coverage:** Calculated over the unstuffed bit stream from SOF through the end of the Data Field.


* **CRC Delimiter (DEL) — 1 Bit:**
* **Bit Value:** `1` (Recessive). Provides a mandatory time buffer for processing the CRC calculation before the ACK phase.




6. **Acknowledge (ACK) Field (2 Bits Total)**
* **ACK Slot — 1 Bit:**
* **Transmitter Output:** Sends a `1` (Recessive).
* **Receiver Output:** Any node that receives the frame without CRC or form errors overwrites this bit with a `0` (Dominant).


* **Coding Note:** If no other active node acknowledges the frame, a Transmit ACK Error occurs in hardware, triggering automatic frame retransmission.


* **ACK Delimiter (DEL) — 1 Bit:**
* **Bit Value:** `1` (Recessive). Ensures the bus returns to logic high before EOF.




7. **End of Frame (EOF) — 7 Bits**
* **Bit Value:** `1111111` (7 consecutive Recessive bits).


* **Function:** Unambiguously marks the termination of the CAN message frame. (Bit stuffing is disabled during EOF).


8. **Interframe Space / Intermission (ITM / IFS) — 3 Bits**
* **Bit Value:** `111` (3 consecutive Recessive bits).
* **Function:** Mandatory minimum pause required between consecutive frames. Gives the controller protocol state machine time to move received frames into buffer RAM (`RXB0`/`RXB1`) before the next SOF can be asserted.



---

## 4. MCP25050 I/O Expander Architecture



The **Microchip MCP25050** (or emulated target) is a CAN-I/O expander device. It translates raw CAN frames directly into digital pin manipulation (GPIO) or ADC/PWM signals without needing a local CPU.

### 4.1 Addressing & Configuration (Minibot Context)



* **Base IDs:**


* **Read Address:** `0x700`


* **Write Address:** `0x708`




* **Identifier Type:** Standard (11-bit).



### 4.2 Register & Command Mapping (Datasheet Summary)



#### Key GPIO Registers (Page 19 & Register 5-2)



* **`GPLAT` (GPIO Latch Register):**


* Controls output logic levels for physical I/O pins.


* Writing a `1` to the specific GPIO bit sets the pin **HIGH** (illuminating an active-high LED or enabling a gate).


* Writing a `0` clears the pin **LOW**.





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



* **Monitor Bus Traffic:**


```bash
candump can0

```


* **Transmit Frame:**


```bash
# Send data frame to ID 0x708 with 3 payload bytes
cansend can0 708#1234FF

```



## 7. Advantages and Disadvantages of CAN Bus



### Advantages



* **High Reliability & Noise Immunity:** Uses differential signaling ($\text{CANH} - \text{CANL}$), making it exceptionally resistant to electrical noise and electromagnetic interference (EMI) in harsh industrial or automotive environments.


* **Bus Topography (Reduced Wiring):** Replaces complex point-to-point wiring harnesses with a simple two-wire differential bus, significantly reducing weight, material cost, and installation complexity.


* **Built-in Error Handling & Fault Confinement:** Supports automatic retransmission of corrupted frames, CRC verification, and node status monitoring (Error Active, Error Passive, Bus-Off) to prevent a failing node from disrupting the network.


* **Nondestructive Bitwise Arbitration:** Priority-based message transmission using frame Identifiers ensures high-priority messages gain instant bus access without packet collision or data loss.


* **Multi-Master Architecture:** Any node on the bus can transmit data whenever the bus is idle, eliminating single points of failure associated with master-slave configurations.



### Disadvantages



* **Limited Payload Size:** Standard CAN (2.0A/B) limits data payloads to a maximum of 8 bytes per frame, making it inefficient for transmitting large data streams.


* **Bandwidth Limitations:** Maximum data rates are limited to $1\text{ Mbps}$ at standard cable lengths (and decrease as bus length increases), which is slower compared to Ethernet or CAN FD.


* **Bus Termination Dependency:** Requires precise impedance matching ($120\,\Omega$ termination resistors at both physical ends of the bus line) to prevent signal reflections and bus corruption.


* **Complex Debugging & Software Overhead:** Protocol framing, identifier arbitration, filtering masks, and state-machine transitions require specialized hardware analyzers and software stacks.


* **Security Constraints:** Native CAN 2.0 lacks built-in encryption, message authentication, or sender verification, making raw buses vulnerable to spoofing if physical access is compromised.