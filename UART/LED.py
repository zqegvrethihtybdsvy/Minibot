import serial
import time

# Initialize UART serial port
# Baud rate must match the target device (usually 9600 or 115200 baud)
ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1.0)

def send_uart_command(command_str: str):
    # 1. Clear input buffer
    ser.reset_input_buffer()

    # 2. Convert text string to bytes and send over TX line
    # (Adding '\n' newline if expected by the target firmware)
    full_command = command_str + '\n'
    ser.write(full_command.encode('utf-8'))
    print(f"Sent: {command_str}")

    # 3. Read back response string from RX line
    time.sleep(0.05)  # Brief delay to allow target to process and reply
    response = ser.readline().decode('utf-8').strip()
    
    print(f"Received: {response}")
    return response

if __name__ == "__main__":
    # 2.2.1 Turn on the LED
    
    send_uart_command("Turn Led On")

    time.sleep(1)

    send_uart_command("Turn Led Off")
    