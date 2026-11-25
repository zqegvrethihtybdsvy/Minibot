import smbus

bus = smbus.SMBus(1)

OLED_ADDR = 0x3D 

# Control byte prefix for commands
CMD_MODE = 0x00 

# Standard SSD1306 OLED Commands
OLED_CMD_DISPLAY_ON = 0xAF
OLED_CMD_DISPLAY_OFF = 0xAE

def turn_oled_on():
    # Sends Control Byte (0x00) followed by Display ON Command (0xAF)
    bus.write_byte_data(OLED_ADDR, CMD_MODE, OLED_CMD_DISPLAY_ON)
    print("OLED turned ON")

def turn_oled_off():
    # Sends Control Byte (0x00) followed by Display OFF Command (0xAE)
    bus.write_byte_data(OLED_ADDR, CMD_MODE, OLED_CMD_DISPLAY_OFF)
    print("OLED turned OFF")

if __name__ == "__main__":
    turn_oled_on()