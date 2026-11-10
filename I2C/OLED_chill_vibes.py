import smbus
import time
import math
from PIL import Image, ImageDraw

# --- Hardware Setup ---
I2C_BUS = 1
OLED_ADDR = 0x3D  # Target Minibot OLED address

CMD_MODE = 0x00
DATA_MODE = 0x40

bus = smbus.SMBus(I2C_BUS)

def send_command(cmd: int):
    bus.write_byte_data(OLED_ADDR, CMD_MODE, cmd)

def send_data_block(data_list):
    """Chunk data stream into 32-byte I2C SMBus transfers."""
    for i in range(0, len(data_list), 32):
        bus.write_i2c_block_data(OLED_ADDR, DATA_MODE, data_list[i:i+32])

def init_ssd1327():
    """Configures display timing, contrast, and 4-bit grayscale mapping."""
    init_cmds = [
        0xAE,              # Display OFF
        0x15, 0x00, 0x3F,  # Column Start=0, End=63 (128 pixels @ 2 px/byte)
        0x75, 0x00, 0x7F,  # Row Start=0, End=127
        0x81, 0x9F,        # High Contrast boost
        0xA0, 0x51,        # Set Re-map (Column Re-map + COM Split Odd/Even)
        0xA1, 0x00,        # Start line = 0
        0xA2, 0x00,        # Offset = 0
        0xA4,              # Normal Display Mode
        0xA8, 0x7F,        # MUX Ratio = 128
        0xB1, 0x11,        # Phase Length
        0xB3, 0x00,        # Clock Divider / Oscillator Freq
        0xBC, 0x07,        # Pre-charge Voltage
        0xBE, 0x07,        # VCOMH Voltage
        0xB8, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 
        0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, # Linear Gray Scale
        0xAF               # Display ON
    ]
    for cmd in init_cmds:
        send_command(cmd)

def render_buffer(image: Image.Image):
    """Packs 8-bit PIL grayscale image into SSD1327 4-bit nibble stream."""
    pixels = image.load()
    buffer = bytearray(8192) # 128 * 64 bytes
    idx = 0
    for y in range(128):
        for x in range(0, 128, 2):
            p1 = pixels[x, y] >> 4
            p2 = pixels[x+1, y] >> 4
            buffer[idx] = (p1 << 4) | p2
            idx += 1
            
    # Reset memory window pointers to top-left
    send_command(0x15); send_command(0x00); send_command(0x3F)
    send_command(0x75); send_command(0x00); send_command(0x7F)
    
    send_data_block(list(buffer))

def draw_beach_frame(t: float) -> Image.Image:
    """Generates a single frame at time t (in seconds)."""
    img = Image.new('L', (128, 128), 0)
    draw = ImageDraw.Draw(img)

    # 1. Sunset Sky Gradient (Dithered Gray Ramp)
    for y in range(50):
        intensity = int(20 + (y / 50.0) * 110)
        draw.line([(0, y), (128, y)], fill=intensity)

    # 2. Breathing Glowing Sun
    sun_pulse = math.sin(t * 3.0) * 2.0
    sun_r = 13 + sun_pulse
    cx, cy = 96, 26
    
    # Outer aura rings
    draw.ellipse([cx - sun_r - 7, cy - sun_r - 7, cx + sun_r + 7, cy + sun_r + 7], fill=0, outline=90)
    draw.ellipse([cx - sun_r - 4, cy - sun_r - 4, cx + sun_r + 4, cy + sun_r + 4], fill=0, outline=170)
    # Sun core
    draw.ellipse([cx - sun_r, cy - sun_r, cx + sun_r, cy + sun_r], fill=255)

    # 3. Dynamic Ocean Water & Rolling Wave Physics
    draw.rectangle([0, 50, 128, 68], fill=60)
    
    # Layer 1: Foreground sine wave
    wave_pts_1 = [(0, 68)]
    for x in range(0, 129, 4):
        y = 54 + math.sin(x * 0.08 + t * 4.0) * 2.5
        wave_pts_1.append((x, y))
    wave_pts_1.extend([(128, 68), (0, 68)])
    draw.polygon(wave_pts_1, fill=130)

    # Layer 2: Whitecap foam crests
    for x in range(0, 128, 8):
        y_foam = 56 + math.sin(x * 0.12 + t * 5.0) * 2.0
        draw.line([(x, y_foam), (x + 4, y_foam)], fill=240, width=1)

    # 4. Sandy Shoreline
    draw.polygon([(0, 66), (128, 60), (128, 128), (0, 128)], fill=30)

    # 5. Swaying Palm Tree
    sway = math.sin(t * 2.0) * 4.0
    trunk_base = (20, 110)
    trunk_top = (35 + sway, 42)
    
    # Curved trunk (quadratic Bezier control points)
    ctrl_x = 10 + sway * 0.5
    ctrl_y = 75
    trunk_pts = []
    for step in range(11):
        u = step / 10.0
        bx = (1-u)**2 * trunk_base[0] + 2*(1-u)*u * ctrl_x + u**2 * trunk_top[0]
        by = (1-u)**2 * trunk_base[1] + 2*(1-u)*u * ctrl_y + u**2 * trunk_top[1]
        trunk_pts.append((bx, by))
    
    for i in range(len(trunk_pts) - 1):
        draw.line([trunk_pts[i], trunk_pts[i+1]], fill=190, width=3)

    # Swaying Palm Fronds
    tx, ty = trunk_top
    leaf_angles = [200, 240, 280, 330, 20, 60]
    for angle in leaf_angles:
        rad = math.radians(angle + sway * 1.5)
        length = 22
        ex = tx + math.cos(rad) * length
        ey = ty + math.sin(rad) * length
        # Arching leaves
        mid_x = (tx + ex) / 2
        mid_y = ((ty + ey) / 2) - 4
        draw.line([(tx, ty), (mid_x, mid_y), (ex, ey)], fill=255, width=2)

    # Coconuts
    draw.ellipse([tx - 2, ty, tx + 2, ty + 4], fill=100)
    draw.ellipse([tx + 2, ty + 1, tx + 6, ty + 5], fill=80)

    # 6. Beach Chair & Umbrella
    # Chair
    draw.line([(68, 92), (80, 78)], fill=255, width=2)
    draw.line([(80, 78), (110, 88)], fill=255, width=2)
    draw.line([(72, 90), (72, 98)], fill=160, width=1)
    draw.line([(102, 86), (102, 98)], fill=160, width=1)

    # Umbrella Pole & Canopy
    draw.line([(95, 84), (100, 60)], fill=180, width=2)
    draw.chord([80, 48, 120, 68], start=180, end=360, fill=230, outline=255)

    # 7. Animated Floating Text Banner
    text_y = int(112 + math.sin(t * 3.0) * 1.5)
    draw.text((32, text_y), "~ CHILL VIBES ~", fill=255)

    return img

if __name__ == "__main__":
    init_ssd1327()
    print("Running Beach Animation (Press Ctrl+C to stop)...")
    
    start_time = time.time()
    try:
        while True:
            t = time.time() - start_time
            frame = draw_beach_frame(t)
            render_buffer(frame)
            time.sleep(0.03)  # Target ~30 FPS
    except KeyboardInterrupt:
        send_command(0xAE)  # Turn off display on exit
        print("\nDisplay powered down.")