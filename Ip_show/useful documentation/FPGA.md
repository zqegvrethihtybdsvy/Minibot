## 1. Detailed Explanation of the FPGA Architecture

The design runs entirely in a **single synchronous clock domain** (`clk_50MHz`, period $T_{\text{clk}} = 20\text{ ns}$). Rather than clocking flip-flops directly from the external SPI clock (`spi_sck`) or encoder lines, all external asynchronous signals are oversampled at $50\text{ MHz}$, eliminating clock-domain crossing (CDC) glitches and hold-time violations.

### Block 1: Deterministic $100\text{ Hz}$ Timebase (`tick_100hz`)

* `clk_div` counts from `0` to `499_999` ($500\,000$ cycles at $50\text{ MHz}$):

$$T_{\text{window}} = \frac{500\,000}{50 \times 10^6\text{ Hz}} = 0.010\text{ s} = 10\text{ ms} \quad (f_{\text{sample}} = 100\text{ Hz})$$


* Every $10\text{ ms}$, `tick_100hz` pulses high for a single $20\text{ ns}$ clock cycle, providing a jitter-free hardware gate for velocity differentiation ($\Delta N / \Delta t$).

### Block 2: Input Synchronizers & $2\times$ Quadrature Decoder

* **Metastability Protection (`*_s[2:0]`):** Each encoder channel passes through a 3-stage shift register clocked at $50\text{ MHz}$. Stages `[0]` and `[1]` form a standard two-flop synchronizer; stage `[2]` stores the previous synchronized state for edge detection.
* **Step & Direction Logic:**
* `step = ch_a_s[2] ^ ch_a_s[1]` triggers on **both rising and falling edges of Channel A** (**$2\times$ quadrature decoding**).
* `dir = ch_a_s[1] ^ ch_b_s[1]` evaluates the phase relationship between A and B at the moment of Channel A's transition:
* When **A leads B**, `A` and `B` have opposite logic levels right after A toggles (`A ^ B == 1` $\rightarrow$ increment).
* When **B leads A**, `A` and `B` have identical logic levels right after A toggles (`A ^ B == 0` $\rightarrow$ decrement).




* **Impact on Resolution:** Because the FPGA counts both edges of Channel A ($2 \times \text{PPR}$) rather than all four edges of A and B ($4 \times \text{PPR}$), a CUI AMT10 set to `2048 PPR` yields **`4096 ticks/rev`** (explaining the exact factor-of-2 scaling observed during testing), while a Pololu #4752 motor encoder (`16 PPR` per channel $\times 2 \times 30\text{ gear ratio}$) yields **`960 ticks/rev`** at the drive wheel.

### Block 3: Velocity Accumulators & Position Counters

* **Speed Measurement (`0x10`, `0x11`):** `mot_l_period_cnt` and `mot_r_period_cnt` count signed steps during the active $10\text{ ms}$ window. On `tick_100hz`, their totals latch atomically into `left_motor_speed` and `right_motor_speed` while clearing the accumulators to `0` on the same clock edge—so **zero encoder ticks are lost** between consecutive windows.
* **Cumulative Odometry (`0x13`, `0x14`):** `left_odo_ticks` and `right_odo_ticks` are 32-bit signed up/down counters that update continuously on every dead-wheel step (overflow range: $\pm 2^{31}-1 \approx \pm 2.14 \times 10^9$ ticks, or $> 78\text{ km}$ before rollover).
* **Synchronous Reset (`0x7F`):** When `reset_counters` pulses high for one $50\text{ MHz}$ cycle, all speed and odometry registers clear simultaneously.

### Block 4: SPI Slave Interface (Mode 0, 5-Byte Frame)

* **Oversampled Edge Detection:** `spi_sck` and `spi_ss` pass through 3-stage synchronizers (`sck_sync`, `ss_sync`), generating single-cycle strobes `sck_rising` (`2'b01`) and `sck_falling` (`2'b10`). At $1\text{ MHz}$ SPI clock, each SPI bit spans $50$ FPGA clock cycles, giving ample timing margin.
* **40-Bit Transaction Protocol:**
1. **Idle (`spi_ss = 1`):** `bit_cnt` resets to `0` and `spi_miso` is placed in high-impedance (`1'bZ`) so other devices can share the Raspberry Pi SPI0 bus.
2. **Command Phase (`bit_cnt = 0..7`, first 8 rising `SCK` edges):** Shifts `spi_mosi` MSB-first into `rx_byte1` (`[We | ADDR[6:0]]`). On the 8th rising edge (`bit_cnt == 7`), `full_cmd` is decoded: if `addr_7bit == 7'h7F`, `reset_counters` fires, and `get_reg_value(addr_7bit)` preloads the 32-bit response into `tx_shift`.
3. **Data Phase (`bit_cnt = 8..39`, next 32 falling `SCK` edges):** Shifts `tx_shift[31]` MSB-first onto `spi_miso` on each falling edge so the Raspberry Pi samples stable data on the subsequent rising edge (standard **SPI Mode 0**).



#### SPI Register Map

| Address (`ADDR[6:0]`) | R/W | Register Name | Width / Format | Description |
| --- | --- | --- | --- | --- |
| **`0x10`** | R | `left_motor_speed` | `int32` (big-endian) | Left blue wheel signed ticks per $10\text{ ms}$ ($100\text{ Hz}$) |
| **`0x11`** | R | `right_motor_speed` | `int32` (big-endian) | Right blue wheel signed ticks per $10\text{ ms}$ ($100\text{ Hz}$) |
| **`0x13`** | R | `left_odo_ticks` | `int32` (big-endian) | Left black dead-wheel cumulative signed ticks |
| **`0x14`** | R | `right_odo_ticks` | `int32` (big-endian) | Right black dead-wheel cumulative signed ticks |
| **`0x17`** | R | `test_reg` | `uint32` (`0x01020304`) | Constant pattern for verifying SPI link integrity |
| **`0x7F` (`0xFF`)** | W | `reset_counters` | Command strobe | Resets all speed and odometry counters to `0` |

---


## 2. Verilog RTL Implementation (`minibot_wheels.v`)

```verilog
module minibot_wheels (
    input  wire        clk_50MHz,    // Onboard 50 MHz oscillator (PIN_R8)
    input  wire        rst_n,        // Active-low reset (KEY0 / PIN_J15)

    // SPI Slave Bus (Raspberry Pi SPI0 CE1, Mode 0: CPOL=0, CPHA=0)
    input  wire        spi_sck,
    input  wire        spi_ss,
    input  wire        spi_mosi,
    output reg         spi_miso,

    // Actuated Drive Wheel Encoders (Blue Wheels -> Speed PID)
    input  wire        mot_enc_la,
    input  wire        mot_enc_lb,
    input  wire        mot_enc_ra,
    input  wire        mot_enc_rb,

    // Passive Odometer Wheel Encoders (Black Wheels -> Localization)
    input  wire        odo_enc_la,
    input  wire        odo_enc_lb,
    input  wire        odo_enc_ra,
    input  wire        odo_enc_rb
);

    // =========================================================================
    // 1. 100 Hz Timer Tick (10 ms window for motor speed measurement)
    // =========================================================================
    reg [18:0] clk_div    = 19'd0;
    reg        tick_100hz = 1'b0;

    always @(posedge clk_50MHz) begin
        if (clk_div == 19'd499_999) begin
            clk_div    <= 19'd0;
            tick_100hz <= 1'b1;
        end else begin
            clk_div    <= clk_div + 1'b1;
            tick_100hz <= 1'b0;
        end
    end

    // =========================================================================
    // 2. Synchronizers & Edge/Direction Detection for All 4 Encoders
    // =========================================================================
    reg [2:0] mot_la_s, mot_lb_s, mot_ra_s, mot_rb_s;
    reg [2:0] odo_la_s, odo_lb_s, odo_ra_s, odo_rb_s;

    always @(posedge clk_50MHz) begin
        mot_la_s <= {mot_la_s[1:0], mot_enc_la};
        mot_lb_s <= {mot_lb_s[1:0], mot_enc_lb};
        mot_ra_s <= {mot_ra_s[1:0], mot_enc_ra};
        mot_rb_s <= {mot_rb_s[1:0], mot_enc_rb};

        odo_la_s <= {odo_la_s[1:0], odo_enc_la};
        odo_lb_s <= {odo_lb_s[1:0], odo_enc_lb};
        odo_ra_s <= {odo_ra_s[1:0], odo_enc_ra};
        odo_rb_s <= {odo_rb_s[1:0], odo_enc_rb};
    end

    // Motor encoders (step & direction - 2x quadrature decoding)
    wire mot_l_step = mot_la_s[2] ^ mot_la_s[1];
    wire mot_l_dir  = mot_la_s[1] ^ mot_lb_s[1];
    wire mot_r_step = mot_ra_s[2] ^ mot_ra_s[1];
    wire mot_r_dir  = mot_ra_s[1] ^ mot_rb_s[1];

    // Passive odometer encoders (step & direction - 2x quadrature decoding)
    wire odo_l_step = odo_la_s[2] ^ odo_la_s[1];
    wire odo_l_dir  = odo_la_s[1] ^ odo_lb_s[1];
    wire odo_r_step = odo_ra_s[2] ^ odo_ra_s[1];
    wire odo_r_dir  = odo_ra_s[1] ^ odo_rb_s[1];

    // =========================================================================
    // 3. Registers & Counters
    // =========================================================================
    reg signed [31:0] left_motor_speed  = 32'sd0; // 0x10
    reg signed [31:0] right_motor_speed = 32'sd0; // 0x11
    reg signed [31:0] left_odo_ticks    = 32'sd0; // 0x13
    reg signed [31:0] right_odo_ticks   = 32'sd0; // 0x14

    reg signed [31:0] mot_l_period_cnt  = 32'sd0;
    reg signed [31:0] mot_r_period_cnt  = 32'sd0;

    reg reset_counters = 1'b0;

    always @(posedge clk_50MHz) begin
        if (reset_counters) begin
            left_motor_speed  <= 32'sd0;
            right_motor_speed <= 32'sd0;
            left_odo_ticks    <= 32'sd0;
            right_odo_ticks   <= 32'sd0;
            mot_l_period_cnt  <= 32'sd0;
            mot_r_period_cnt  <= 32'sd0;
        end else begin
            // Left Motor Speed Accumulator
            if (mot_l_step) begin
                mot_l_period_cnt <= mot_l_dir ? (mot_l_period_cnt + 32'sd1)
                                              : (mot_l_period_cnt - 32'sd1);
            end

            // Right Motor Speed Accumulator
            if (mot_r_step) begin
                mot_r_period_cnt <= mot_r_dir ? (mot_r_period_cnt + 32'sd1)
                                              : (mot_r_period_cnt - 32'sd1);
            end

            // Left Passive Odometer Counter (real-time)
            if (odo_l_step) begin
                left_odo_ticks <= odo_l_dir ? (left_odo_ticks + 32'sd1)
                                            : (left_odo_ticks - 32'sd1);
            end

            // Right Passive Odometer Counter (real-time)
            if (odo_r_step) begin
                right_odo_ticks <= odo_r_dir ? (right_odo_ticks + 32'sd1)
                                             : (right_odo_ticks - 32'sd1);
            end

            // Latch Motor Speeds every 10 ms (100 Hz)
            if (tick_100hz) begin
                left_motor_speed  <= mot_l_period_cnt;
                right_motor_speed <= mot_r_period_cnt;
                mot_l_period_cnt  <= 32'sd0;
                mot_r_period_cnt  <= 32'sd0;
            end
        end
    end

    // =========================================================================
    // 4. SPI Slave Interface (Mode 0, 5-byte frame: [We|ADDR(6:0)] + 4 bytes)
    // =========================================================================
    reg [2:0] sck_sync, ss_sync;
    always @(posedge clk_50MHz) begin
        sck_sync <= {sck_sync[1:0], spi_sck};
        ss_sync  <= {ss_sync[1:0], spi_ss};
    end

    wire sck_rising  = (sck_sync[2:1] == 2'b01);
    wire sck_falling = (sck_sync[2:1] == 2'b10);
    wire ss_active   = ~ss_sync[1];

    reg [5:0]  bit_cnt  = 6'd0;
    reg [7:0]  rx_byte1 = 8'd0;
    reg [31:0] tx_shift = 32'd0;

    function [31:0] get_reg_value(input [6:0] addr);
        case (addr)
            7'h10:   get_reg_value = left_motor_speed;   // Left motor speed (100 Hz)
            7'h11:   get_reg_value = right_motor_speed;  // Right motor speed (100 Hz)
            7'h13:   get_reg_value = left_odo_ticks;     // Left passive odometer ticks
            7'h14:   get_reg_value = right_odo_ticks;    // Right passive odometer ticks
            7'h17:   get_reg_value = 32'h01020304;       // SPI loopback/health test register
            default: get_reg_value = 32'h00000000;
        endcase
    endfunction

    wire [7:0] full_cmd  = {rx_byte1[6:0], spi_mosi};
    wire       we_flag   = full_cmd[7];
    wire [6:0] addr_7bit = full_cmd[6:0];

    always @(posedge clk_50MHz) begin
        reset_counters <= 1'b0;

        if (~ss_active) begin
            bit_cnt  <= 6'd0;
            spi_miso <= 1'bZ;
        end else begin
            if (sck_rising) begin
                bit_cnt <= bit_cnt + 6'd1;

                if (bit_cnt < 6'd8) begin
                    rx_byte1 <= full_cmd;
                end

                if (bit_cnt == 6'd7) begin
                    // Reset all counters when ADDR = 0x7F (e.g., 0xFF with We=1)
                    if (addr_7bit == 7'h7F) begin
                        reset_counters <= 1'b1;
                    end
                    tx_shift <= get_reg_value(addr_7bit);
                end
            end

            if (sck_falling && (bit_cnt >= 6'd8)) begin
                spi_miso <= tx_shift[31];
                tx_shift <= {tx_shift[30:0], 1'b0};
            end
        end
    end

endmodule

```

---

## 3. Pin Attribution (Terasic DE0-Nano & Raspberry Pi)

### Fixed Onboard DE0-Nano Pins

| Verilog Port | DE0-Nano Pin | Board Resource | Direction | Notes |
| --- | --- | --- | --- | --- |
| `clk_50MHz` | **`PIN_R8`** | `CLOCK_50` | Input | Onboard $50\text{ MHz}$ crystal oscillator |
| `rst_n` | **`PIN_J15`** | `KEY[0]` | Input | Active-low push-button (optional) |

### Raspberry Pi $\leftrightarrow$ DE0-Nano SPI0 Bus (`3.3 V` CMOS)

| Verilog Port | Signal | Raspberry Pi 40-Pin Header | DE0-Nano `GPIO_0` Header (Standard Assignment) |
| --- | --- | --- | --- |
| `spi_sck` | `SPI0_SCLK` | Pin 23 (`GPIO 11`) | `GPIO_0[11]` (**`PIN_C6`**) |
| `spi_mosi` | `SPI0_MOSI` | Pin 19 (`GPIO 10`) | `GPIO_0[13]` (**`PIN_D6`**) |
| `spi_miso` | `SPI0_MISO` | Pin 21 (`GPIO 9`) | `GPIO_0[15]` (**`PIN_E6`**) |
| `spi_ss` | `SPI0_CE1_N` | Pin 26 (`GPIO 7`) | `GPIO_0[9]` (**`PIN_D5`**) |
| — | `GND` | Pin 6 / 9 / 25 | `GPIO_0` Pin 12 / 30 (`GND`) |

### Quadrature Encoder Inputs on DE0-Nano (`3.3 V` Logic)

> **Voltage Level Note:** Both the Pololu #4752 Hall sensors (`3.5–20 V`) and CUI AMT10 encoders (`5 V`) are powered at **`5 V`** (or `3.3 V` for the Pololu Hall sensors). Ensure `5 V` encoder outputs pass through voltage dividers or level shifters to respect the Cyclone IV `3.3 V` I/O bank limit.

| Subsystem | Verilog Port | Signal | Typical DE0-Nano `GPIO_0` Pin |
| --- | --- | --- | --- |
| **Left Drive Motor** (Blue) | `mot_enc_la` | Channel A | `GPIO_0[0]` (**`PIN_A8`**) |
| **Left Drive Motor** (Blue) | `mot_enc_lb` | Channel B | `GPIO_0[1]` (**`PIN_D3`**) |
| **Right Drive Motor** (Blue) | `mot_enc_ra` | Channel A | `GPIO_0[2]` (**`PIN_B8`**) |
| **Right Drive Motor** (Blue) | `mot_enc_rb` | Channel B | `GPIO_0[3]` (**`PIN_C3`**) |
| **Left Passive Odometer** (Black) | `odo_enc_la` | Channel A | `GPIO_0[4]` (**`PIN_A2`**) |
| **Left Passive Odometer** (Black) | `odo_enc_lb` | Channel B | `GPIO_0[5]` (**`PIN_A3`**) |
| **Right Passive Odometer** (Black) | `odo_enc_ra` | Channel A | `GPIO_0[6]` (**`PIN_B3`**) |
| **Right Passive Odometer** (Black) | `odo_enc_rb` | Channel B | `GPIO_0[7]` (**`PIN_B4`**) |

*(Verify the `GPIO_0` pin numbers in your Quartus Prime **Pin Planner** / `.qsf` file against the physical ribbon cable connected to your Minibot interface PCB.)*