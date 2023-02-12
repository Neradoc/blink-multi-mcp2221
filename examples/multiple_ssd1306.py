import atexit
import time
from mcp2221 import mcp2221, i2c, get_mcp_list

import datetime
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306

class Display:
    def __init__(self, bus, address, pos=None):
        self.pos = pos

        if address == 0x3C:
            height = 32
        else:
            height = 64

        self.oled = adafruit_ssd1306.SSD1306_I2C(128, height, bus, addr=address)
        # Clear display.
        self.oled.fill(0)
        self.oled.show()

        # Create blank image for drawing.
        self.image = Image.new("1", (self.oled.width, self.oled.height))
        self.draw = ImageDraw.Draw(self.image)

        # Load a font in 2 different sizes.
        self.font = ImageFont.truetype("DejaVuSans.ttf", 28)
        self.font2 = ImageFont.truetype("DejaVuSans.ttf", 14)

        # Draw the text
        self.draw.text((0, 30), "Hello!", font=self.font2, fill=255)
        self.draw.text((34, 46), "Hello!", font=self.font2, fill=255)
        self.oled.image(self.image)
        # self.oled.show()
        self.update()

    def write_text(self, text):
        self.draw.rectangle((0, 0, 128, 28), outline=0, fill=0)
        self.draw.text((0, 0), text, font=self.font, fill=255)
        # Display image
        self.oled.image(self.image)
        self.oled.show()

    def update(self):
        # text = f"Num {self.pos}"
        now = datetime.datetime.now()
        text = f"{now.hour}:{now.minute:02d}:{now.second:02d}"
        self.write_text(text)


if __name__ == "__main__":

    mcps = get_mcp_list(timeout=10, num=2)
    print(mcps)

    disps = []
    for pos, mcp in enumerate(mcps):
        scan = mcp.i2c_scan()
        print("-"*70)
        print("MCP found")
        print("  I2C scan:", [hex(addr) for addr in scan])
        print("  Pins:", [mcp.gpio_get_pin(i) for i in range(4)])
        print("  Serial number:", mcp.serial_number)
        print("  As an hex int:", int(mcp.serial_number, 16))

        if 0x3D in scan:
            bus = i2c.I2C(mcp)
            disps.append(Display(bus, 0x3D, pos))
        if 0x3C in scan:
            bus = i2c.I2C(mcp)
            disps.append(Display(bus, 0x3C, pos))

    while disps:
        for disp in disps:
            disp.update()

