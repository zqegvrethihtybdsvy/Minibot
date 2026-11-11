# SPI Communication & DE0-Nano Protocol Guide

## 1. Overview of the SPI Bus Architecture

The Minibot relies on the **Serial Peripheral Interface (SPI)** to facilitate high-speed, full-duplex, synchronous communication between the main processing unit (Raspberry Pi) and its hardware peripherals.

On the Raspberry Pi, the primary SPI interface (**SPI0**) acts as the Master, driving the clock line (`SCK`) and managing data flow via the Master Out Slave In (`MOSI`) and Master In Slave Out (`MISO`) lines.

The bus is shared between two target devices, distinguished by their Chip Enable (CE) or Slave Select (SS) lines:

* **CE0 (Chip Enable 0):** CAN Controller (MCP2515)

* **CE1 (Chip Enable 1):** DE0-Nano FPGA

When communicating with the DE0-Nano, the Raspberry Pi pulls `CE1` low, signaling the FPGA to listen to the incoming clock and data signals.

## 2. DE0-Nano Custom 5-Byte Protocol

The DE0-Nano is programmed with a custom SPI slave interface designed to exchange exact **5-byte frames** for every transaction. Because SPI is full-duplex, for every byte the Master sends out, it simultaneously receives a byte back from the Slave.

### 2.1 Message Frame Structure

The 5-byte message is structured as a Command/Address byte followed by 4 bytes (32 bits) of Data payload.

| Byte Index | Field Name | Description | 
| ----- | ----- | ----- | 
| **Byte 1** | `CMD / ADDR` | Bit 8 (MSB): Write Enable (`We`). Bits 7-1: Target Register Address. | 
| **Byte 2** | `DATA 1` | Payload Data Byte 1 (Most Significant Byte) | 
| **Byte 3** | `DATA 2` | Payload Data Byte 2 | 
| **Byte 4** | `DATA 3` | Payload Data Byte 3 | 
| **Byte 5** | `DATA 4` | Payload Data Byte 4 (Least Significant Byte) | 

### 2.2 Constructing the First Byte (`CMD / ADDR`)

The first byte dictates whether the transaction is a **Read** or a **Write**, and targets a specific internal register address on the DE0-Nano.

* **Write/Enable Bit (Bit 8 / MSB):**

  * `0`: **Read Only.** The Raspberry Pi reads data from the DE0-Nano. The payload sent in Bytes 2-5 is typically ignored (dummy data, usually `0x00`), and the relevant data is returned on the `MISO` line.

  * `1`: **Write.** The Raspberry Pi writes the 32-bit payload (Bytes 2-5) to the specified DE0-Nano address.

* **Address Bits (Bits 7-1):** The 7-bit identifier of the register.

**Example Bitwise Construction:**

* To **Read** from Address `0x10`: Byte 1 = `0x10 & 0x7F = 0x10` (Binary: `0001 0000`)

* To **Write** to Address `0x10`: Byte 1 = `0x10 | 0x80 = 0x90` (Binary: `1001 0000`)

## 3. Register Address Map

The default program flashed to the DE0-Nano exposes several registers for interacting with the robot's hardware sensors (like odometers and encoders).

| Address (Hex) | Access Type | Function / Description | 
| ----- | ----- | ----- | 
| **`0x10`** | Read | Return the calculated speed from the left encoder. Update frequency $f = 100 \text{ Hz}$. | 
| **`0x13`** | Read | Return the raw tick counter from the left odometer. | 
| **`0x17`** | Read | Diagnostics / Test Register. Always returns the sequence: `0x01, 0x02, 0x03, 0x04`. | 
| **`0x7F`** | Write | Reset internal values (clear counters/errors). | 

## 4. Software Implementation (Python `spidev`)

To interface with the DE0-Nano from the Raspberry Pi, you can use the standard `spidev` library. Below is an example demonstrating how to construct and send the 5-byte frames.

```
import spidev
import time

def setup_spi():
    """Initializes the SPI bus for the DE0-Nano on CE1."""
    spi = spidev.SpiDev()
    # Bus 0, Device 1 (CE1)
    spi.open(0, 1)
    # Configure SPI settings (adjust max_speed_hz based on DE0-Nano spec)
    spi.max_speed_hz = 1000000 
    spi.mode = 0
    return spi

def de0_read_register(spi, address):
    """
    Reads a 32-bit value from the specified DE0-Nano address.
    """
    # Bit 8 is 0 for Read, so we just use the address directly (masked to 7 bits)
    cmd_byte = address & 0x7F
    
    # We send the command byte followed by 4 dummy bytes (0x00) to clock out the response
    msg = [cmd_byte, 0x00, 0x00, 0x00, 0x00]
    
    # xfer2 keeps CS (CE1) active for the entire 5-byte transaction
    response = spi.xfer2(msg)
    
    # The DE0 returns data simultaneously; bytes 1-4 contain the actual register data
    data_bytes = response[1:5]
    print(f"Read ADDR 0x{address:02X} -> Data: {[hex(b) for b in data_bytes]}")
    return data_bytes

def de0_test_communication(spi):
    """
    Reads the diagnostic register 0x17. 
    Expected response: [0x01, 0x02, 0x03, 0x04]
    """
    print("Testing DE0-Nano SPI Connection...")
    data = de0_read_register(spi, 0x17)
    if data == [0x01, 0x02, 0x03, 0x04]:
        print("SUCCESS: DE0-Nano is communicating correctly.")
    else:
        print("ERROR: Unexpected response from DE0-Nano.")

if __name__ == "__main__":
    spi_bus = setup_spi()
    
    try:
        # 1. Test the connection
        de0_test_communication(spi_bus)
        
        # 2. Read Left Odometer Tick Counter
        de0_read_register(spi_bus, 0x13)
        
    finally:
        spi_bus.close()

```

## 5. ⚠️ Critical Safety & Hardware Warnings

### Persistent Flashing

The DE0-Nano uses volatile memory for its default operation. If power is lost, the default program may be cleared unless it is flashed to the onboard persistent EEPROM/Flash.

* **Action:** Refer to the tutorial available on Moodle in the **Minibots section** to learn how to flash your DE0-Nano persistently.

### Dual Power Source Danger (Hardware Destruction Risk)

The Minibot supplies power to the DE0-Nano directly through the internal GPIO/wiring harness. Connecting the DE0-Nano to your laptop via USB while the Minibot's battery/power system is active creates a **dual power source conflict**.

> 🛑 **CRITICAL RULE:**
> **BEFORE** plugging the DE0-Nano into your computer via USB (e.g., to reprogram or flash it using Quartus), you **MUST** physically shut off the Minibot's main power. Connecting two different power sources simultaneously can cause reverse currents that will permanently destroy the DE0-Nano, the Raspberry Pi, or your computer's USB port.