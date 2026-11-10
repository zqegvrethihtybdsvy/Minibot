# ==============================================================================
# ADS1115 ADC (Analog-to-Digital Converter) I2C Reading Script
# ==============================================================================

# Import the SMBus module to enable I2C communication on the Raspberry Pi
import smbus

# Open physical I2C Bus 1 (GPIO 2 = SDA, GPIO 3 = SCL on the Raspberry Pi header)
bus = smbus.SMBus(1)

# Define the hardware I2C address of the ADS1115 chip on the Minibot PCB
ADC_ADDRESS = 0x48

# Define internal register pointer addresses inside the ADS1115 chip:
CONFIG_REGISTER = 0x01     # Pointer 0x01: Where we write setup commands & settings
CONVERSION_REGISTER = 0x00 # Pointer 0x00: Where the chip outputs the 16-bit digitized result

LSB_SIZE = 2.048/(2**16)
# ------------------------------------------------------------------------------
# STEP 1: Define configuration settings (2 bytes / 16 bits total)
# ------------------------------------------------------------------------------
# High Byte (0x84 = 10000100 in binary):
#   - Bit 15 = 1 : Start a single-shot measurement
#   - Bits 14-12 = 000 : Select channel AIN0 (relative to ground)
#   - Bits 11-9 = 010 : Set full scale range to +/- 2.048 Volts
#   - Bit 8 = 0 : Single-shot mode active
#
# Low Byte (0x83 = 10000011 in binary):
#   - Bits 7-5 = 100 : Set sampling speed to 128 samples per second
#   - Bits 4-0 = 00011 : Disable internal alert comparator queue
config = [0x84, 0x83]

# Write the 2 configuration bytes to device 0x48 inside Config Register 0x01
bus.write_i2c_block_data(ADC_ADDRESS, CONFIG_REGISTER, config)

# ------------------------------------------------------------------------------
# STEP 2: Read the voltage conversion result
# ------------------------------------------------------------------------------
# Fetch 2 bytes from device 0x48 starting at Conversion Register 0x00
# The returned array contains: [High Byte (data[0]), Low Byte (data[1])]
data = bus.read_i2c_block_data(ADC_ADDRESS, CONVERSION_REGISTER, 2)

# ------------------------------------------------------------------------------
# STEP 3: Combine the two 8-bit bytes into one 16-bit number
# ------------------------------------------------------------------------------
# Shift the High Byte (data[0]) left by 8 positions to occupy bits 15 through 8,
# then use Bitwise OR (|) to insert the Low Byte (data[1]) into bits 7 through 0.
raw_adc = (data[0] << 8) | data[1]
voltage = raw_adc * LSB_SIZE

# Print the final 16-bit integer value to the terminal

print(f"Raw ADC: {raw_adc:6d}  |  Voltage: {voltage:.4f} V")

