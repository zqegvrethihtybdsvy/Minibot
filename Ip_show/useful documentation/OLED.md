# SSD1327 1.5-Inch OLED Display Technical Reference Guide

This document provides a comprehensive technical reference for the **Waveshare 1.5-inch $128 \times 128$ Grayscale OLED Display** controlled by the **Solomon Systech SSD1327 driver IC** operating over $\text{I}^2\text{C}$ for embedded systems and robotics projects (such as the Minibot platform).

---

## 1. Core Specifications & Features

* **Display Driver:** Solomon Systech SSD1327.
* **Resolution:** $128 \times 64$ to $128 \times 128$ pixels ($128 \times 128$ on this 1.5-inch module).
* **Color Depth:** 16-level grayscale ($4\text{ bits}$ per pixel).
* **Technology:** Passive Matrix OLED (self-illuminating, high contrast, wide viewing angle).
* **Operating Voltage (**$V_{\text{DD}}$**):** $3.3\text{ V}$ to $5.0\text{ V}$.
* **Communication Modes:** 4-wire SPI (Factory Default) or $\text{I}^2\text{C}$ (selected via onboard `BS` jumper/resistor).
* **Physical Dimensions:** $44.5\text{ mm} \times 37.0\text{ mm}$ (Active Display: $26.86\text{ mm} \times 26.86\text{ mm}$).

---

## 2. Pinout & $\text{I}^2\text{C}$ Address Configuration

### Module Pinout

| Pin Name | Type | $\text{I}^2\text{C}$ Signal | Description |
| :--- | :--- | :--- | :--- |
| **VCC** | Power | $3.3\text{ V} / 5\text{ V}$ | Main power supply input. |
| **GND** | Power | Ground ($0\text{ V}$) | Common system ground connection. |
| **DIN** | Digital Input | **SDA** | Serial Data Line ($\text{I}^2\text{C}$ mode). |
| **CLK** | Digital Input | **SCL** | Serial Clock Line ($\text{I}^2\text{C}$ mode). |
| **CS** | Digital Input | NC / Dangling | Chip Select (Not connected in $\text{I}^2\text{C}$ mode). |
| **DC** | Digital Input | `0` or `1` | Address select pin in $\text{I}^2\text{C}$ mode (`GND` = `0x3C`, `VCC` = `0x3D`). |
| **RST** | Digital Input | Reset | Hardware Reset pin (Active Low). |

### Hardware Address Selection

The 7-bit $\text{I}^2\text{C}$ address depends on the physical level applied to the **`DC` pin**:

| `DC` Pin Connection | 7-Bit Address (Binary) | 7-Bit Address (Hex) | Usage |
| :--- | :--- | :--- | :--- |
| **`GND` (`0`)** | `0111100b` | **`0x3C`** | Default manufacturer Address |
| **`VCC` (`1`)** | `0111101b` | **`0x3D`** | Alternate Address - **Minbot's default adress for the OLED** |

---

## 3. $\text{I}^2\text{C}$ Frame & Control Byte Protocol

Every $\text{I}^2\text{C}$ transmission to the SSD1327 begins with a **Control Byte** preceding the payload to differentiate between setup commands and image pixel data.

```
I2C Frame: [START] -> [0x3C + Write] -> [Control Byte*] -> [Command or Data Payload] -> [STOP]
```

### Control Byte Structure (`B7..B0`)

Unlike standard sensors with direct register addresses, the OLED controller uses the byte following the device address as a **Control Byte**[cite: 5]. This byte determines how the controller interprets the payload bytes that follow:

```text
Bit:   |   B7   |   B6   | B5 B4 B3 B2 B1 B0 |
       |--------|--------|-------------------|
       | Co (C0)|  D/C#  |      0 0 0 0 0 0  |

```

* **Bit 7 (`Co` — Continuation Bit):**
* `0b` = **Stream mode**: All subsequent bytes in the packet are treated continuously as data or commands (most common for efficiency).
* `1b` = **Single-byte mode**: Each individual payload byte must be preceded by its own control byte.


* **Bit 6 (`D/C#` — Data/Command Select Bit):**
* `0b` = **Command Byte**: Instructs the controller to interpret the payload as an operational configuration command or opcode (e.g., `0xAF` to turn display on).


* `1b` = **Data Byte**: Instructs the controller to write the payload directly into the Display RAM (GDDRAM) as pixel values.



* **Bits 5–0:**
* Reserved / set to `0`.


### Frequently Used Control Bytes

| Hex Code | Binary Code | Mode Target |
| :--- | :--- | :--- |
| **`0x00`** | `0000 0000b` | **Command Byte** (`Co = 0`, `D/C# = 0`) |
| **`0x40`** | `0100 0000b` | **Display Data Byte** (`Co = 0`, `D/C# = 1`) |



## 4. Key SSD1327 Register Commands

### Display Power & Power Management Commands

| Command Hex | Binary Code | Name | Function Description |
| :--- | :--- | :--- | :--- |
| **`0xAE`** | `1010 1110b` | **Display OFF** | Puts display into Sleep Mode (Turns off OLED panel driver). |
| **`0xAF`** | `1011 1111b` | **Display ON** | Turns on display panel and resumes rendering RAM. |
| **`0xA4`** | `1010 0100b` | **Normal Display Mode** | Display output follows RAM content. |
| **`0xA5`** | `1010 0101b` | **Entire Display ON** | Forces all pixels to maximum brightness (Test pattern). |
| **`0xA6`** | `1010 0110b` | **Normal Mode** | Normal video polarity. |
| **`0xA7`** | `1010 0111b` | **Inverse Mode** | Inverts pixel intensity ($15 - \text{intensity}$). |

### Window Address Setting Commands

Setting the column and row windows defines where incoming pixel data will be written:

| Command | Sequence | Description |
| :--- | :--- | :--- |
| **Set Column Address** | `0x15 [Start, End]` | Column range from `0` to `63` (each column address contains 2 pixels). |
| **Set Row Address** | `0x75 [Start, End]` | Row range from `0` to `127`. |

---

## 5. Memory Organization (GDDRAM Structure)

The Graphic Display Data RAM (GDDRAM) stores the pixel intensities for the $128 \times 128$ matrix.

* **Resolution:** $128 \times 128$ pixels.
* **Grayscale Depth:** 16 levels ($0\text{ to }15$, represented by $4\text{ bits}$).
* **Byte Packing:** **Two pixels per byte** ($8\text{ bits} = 2 \times 4\text{-bit pixels}$).
  * **Upper 4 bits (`D[7:4]`):** Left Pixel intensity ($0\text{--}15$).
  * **Lower 4 bits (`D[3:0]`):** Right Pixel intensity ($0\text{--}15$).
* **Total RAM Capacity:** 
  $$\text{RAM Size} = \frac{128 \times 128 \text{ pixels}}{2 \text{ pixels/byte}} = 8192 \text{ bytes} \ (8\text{ KB})$$

---

## 6. Python Driver Implementation (`smbus`)

```python
#!/usr/bin/env python3
"""
SSD1327 1.5-inch 128x128 OLED Display Driver for Raspberry Pi
Targeted for Minibot Platform (Address 0x3C)
"""

import smbus
import time

I2C_BUS = 1
OLED_ADDR = 0x3C

CMD_MODE = 0x00
DATA_MODE = 0x40

CMD_DISPLAY_OFF = 0xAE
CMD_DISPLAY_ON = 0xAF

def init_ssd1327(bus):
    """Executes hardware setup sequence for 128x128 SSD1327 OLED."""
    init_cmds = [
        CMD_DISPLAY_OFF,   # 0xAE: Display OFF
        0x15, 0x00, 0x3F,  # Set Column Address: 0 to 63 (128 pixels / 2)
        0x75, 0x00, 0x7F,  # Set Row Address: 0 to 127
        0x81, 0x7F,        # Set Contrast Control to mid level (127)
        0xA0, 0x51,        # Set Re-map format: Enable Column address re-map, Nibble re-map
        0xA1, 0x00,        # Set Display Start Line (0)
        0xA2, 0x00,        # Set Display Offset (0)
        0xA4,              # Normal Display Mode
        0xA8, 0x7F,        # Set Multiplex Ratio (128 lines)
        0xB1, 0x11,        # Set Phase Length
        0xB3, 0x00,        # Set Display Clock Divide Ratio / Oscillator
        0xAB, 0x01,        # Enable internal VDD regulator
        0xB6, 0x0F,        # Set Second Pre-charge Period
        0xBE, 0x07,        # Set VCOMH voltage level
        0xBC, 0x07,        # Set Pre-charge Voltage level
        0xD5, 0x62,        # Set Function Selection B
        CMD_DISPLAY_ON     # 0xAF: Display ON
    ]
    bus.write_i2c_block_data(OLED_ADDR, CMD_MODE, init_cmds)
    print("SSD1327 1.5-inch OLED Initialized (Display ON)")

def clear_display(bus):
    """Fills the 8192-byte GDDRAM with zeros to clear screen."""
    # Set window bounds for entire screen
    bus.write_i2c_block_data(OLED_ADDR, CMD_MODE, [0x15, 0x00, 0x3F])
    bus.write_i2c_block_data(OLED_ADDR, CMD_MODE, [0x75, 0x00, 0x7F])
    
    # Transmit zero-filled chunks (32 bytes per chunk)
    zero_chunk = [0x00] * 32
    for _ in range(256):  # 256 * 32 = 8192 bytes
        bus.write_i2c_block_data(OLED_ADDR, DATA_MODE, zero_chunk)
    print("Screen cleared.")

def main():
    try:
        bus = smbus.SMBus(I2C_BUS)
        init_ssd1327(bus)
        clear_display(bus)
        time.sleep(2)
        
        # Test display toggle commands
        print("Turning Display OFF...")
        bus.write_byte_data(OLED_ADDR, CMD_MODE, CMD_DISPLAY_OFF)
        time.sleep(1)
        
        print("Turning Display ON...")
        bus.write_byte_data(OLED_ADDR, CMD_MODE, CMD_DISPLAY_ON)
        
    except Exception as e:
        print(f"Error communicating with SSD1327 OLED: {e}")

if __name__ == "__main__":
    main()
```

---

## 7. Minibot Academic Documentation Reference Table

For completing assignment reports (e.g., section 2.1.4 of your lab manual):

| Section | Target Function | Address | Request Byte Sequence | Expected Response |
| :--- | :--- | :--- | :--- | :--- |
| **2.1.4.1** | **Turn OLED Display ON** | `0x3C` | `0x00 0xAF` | No response (Write-only) |
| **2.1.4.2** | **Turn OLED Display OFF** | `0x3C` | `0x00 0xAE` | No response (Write-only) |
| **2.1.4.3** | **Clear Display RAM** | `0x3C` | `0x40 0x00 0x00 ...` | No response (Write-only) |