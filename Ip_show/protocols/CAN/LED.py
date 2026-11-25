import can
import sys

def turn_led_on():
    # 1. Connect to the SocketCAN interface (can0)
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
    except Exception as e:
        print(f"Error opening can0 interface: {e}")
        sys.exit(1)

    # 2. Configure CAN Write Frame
    # Arbitration ID: 0x708 (MCP25050 Standard Write ID)
    # Byte 0: 0x1E -> GPLAT Register Address (GPIO Output Latch, Page 19 / Reg 5-2)
    # Byte 1: 0xFF -> Set output latch bits HIGH (Turns LED ON)
    can_id = 0x708
    payload = [0x1E, 0xFF]

    msg = can.Message(
        arbitration_id=can_id,
        data=payload,
        is_extended_id=False  # Standard 11-bit CAN frame
    )

    # 3. Transmit Frame
    try:
        bus.send(msg)
        print(f"[CAN OUT] Sent Frame -> ID: 0x{can_id:03X} | Data: {[hex(b) for b in payload]}")
    except can.CanError as e:
        print(f"Failed to transmit CAN frame: {e}")

if __name__ == "__main__":
    turn_led_on()