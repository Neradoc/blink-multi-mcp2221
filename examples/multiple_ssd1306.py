import atexit
import time
from mcp2221_extended import mcp2221, i2c, get_mcp_list

import datetime
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import adafruit_si7021
import adafruit_bh1750

temperature = None
luminix = None

class Display:
    def __init__(self, bus, address, pos=None):
        self.pos = pos
        self.bus = bus

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
        self.draw.rectangle((0, 0, 128, self.oled.height), outline=0, fill=0)
        self.draw.text((0, 0), text, font=self.font, fill=255)
        if self.oled.height > 32:
            if temperature:
                try:
                    temp = temperature.temperature
                    humi = temperature.relative_humidity
                    text = f"Temperature: {temp:.1f} C\nHumidity: {humi:.1f} %"
                    self.draw.text((0, 32), text, font=self.font2, fill=255)
                except RuntimeError:
                    pass
            if luminix:
                try:
                    text = f"Lum: {luminix.lux:.1f}"
                    self.draw.text((0, 48), text, font=self.font2, fill=255)
                except RuntimeError:
                    pass
        # Display image
        self.oled.image(self.image)
        try:
            self.oled.show()
        except Exception as err:
            print(self.pos, err)
            self.bus._mcp._reset()

    def update(self):
        # text = f"Num {self.pos}"
        now = datetime.datetime.now()
        text = f"{now.hour}:{now.minute:02d}:{now.second:02d}"
        self.write_text(text)


if __name__ == "__main__":

    mcps = get_mcp_list(timeout=2, num=2)
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

        bus = mcp.board.I2C()
        print(bus)

        if 0x3D in scan:
            disps.append(Display(bus, 0x3D, pos))
        if 0x3C in scan:
            disps.append(Display(bus, 0x3C, pos))
        if 0x40 in scan:
            temperature = adafruit_si7021.SI7021(bus)
        if 0x23 in scan:
            luminix = adafruit_bh1750.BH1750(bus)

    while disps:
        for disp in disps:
            print(disp)
            disp.update()
            time.sleep(.1)

