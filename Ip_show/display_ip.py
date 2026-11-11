import DEV_Config
import OLED_Driver
import socket
from PIL import Image,ImageDraw,ImageFont

from time import sleep
from subprocess import check_output

def fetch_Ip(line):
    if "inet " in line:
        return line.split("inet ")[1].split("/")[0]
    return "No address :("

def fetch_Ips():
  Ip = str(check_output(["ip","address"])).split(": eth0")[1].split(": wlan")
  Eth0 = fetch_Ip(Ip[0])
  Wlan = fetch_Ip(Ip[1])
  return [Eth0, Wlan]


#Init OLED
OLED = OLED_Driver.OLED()
OLED_ScanDir = OLED_Driver.SCAN_DIR_DFT  #SCAN_DIR_DFT = D2U_L2R
OLED.OLED_Init(OLED_ScanDir)
DEV_Config.Driver_Delay_ms(20)
image = Image.new("L", (OLED.OLED_Dis_Column, OLED.OLED_Dis_Page), 0)# grayscale (luminance)
draw = ImageDraw.Draw(image)

hostname = str(check_output(["hostname"]))[2:-3]

draw.text((0, 0), f"I am {hostname}", fill = "White")

i_max = 5
for i in range(i_max):
  Ips = fetch_Ips()
  draw.rectangle([0,12,130,60],fill="Black")
  OLED.OLED_ShowImage(image,0,0)
  draw.text((0,22), f"Wifi: {Ips[1]}", fill = "White")
  draw.text((0,32), f"Eth:  {Ips[0]}", fill = "White")

  if "No address :(" not in Ips or i == i_max - 1:
      OLED.OLED_ShowImage(image,0,0)
      break
  draw.text((0,42), f"Still trying... ({i+1}/{i_max})", fill="White")
  OLED.OLED_ShowImage(image,0,0)
  sleep(3)

#draw.text((0, 22), 'My IP address is ', fill = "White")
#draw.text((0, 32), str(check_output(['hostname', '-I']))[2:13], fill = "White")
#OLED.OLED_ShowImage(image,0,0)
