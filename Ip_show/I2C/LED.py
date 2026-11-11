import smbus

bus = smbus.SMBus(1)

LED_ADDRESS = 0x08
CMD_LED_ON = 0x01
CMD_LED_OFF = 0x00

def set_led(state: bool):
    """
    Turns the LED on or off.
    :param state: True for ON (0x01), False for OFF (0x00)
    """
    cmd = CMD_LED_ON if state else CMD_LED_OFF
    bus.write_byte(LED_ADDRESS, cmd)

if __name__ == "__main__":
    #set_led(True)   # Turns LED ON
    set_led(False) # Turns LED OFF