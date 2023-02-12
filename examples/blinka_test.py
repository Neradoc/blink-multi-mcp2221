import busio
import digitalio
# import analogio
from mcp2221_extended import mcp2221, get_mcp_list, pin, analogio

mcps = get_mcp_list(timeout=5, num=2)

print(mcps)
if not mcps:
    print("No MCP2221 found")

for pos, mcp in enumerate(mcps):
    scan = mcp.i2c_scan()
    print("-"*70)
    print("MCP Serial number:", mcp.serial_number)

    # using the internal board instance as board module
    # use board.I2C as usual
    board = mcp.board
    i2c = board.I2C()
    while not i2c.try_lock():
        i2c.unlock()
    print("Scan", i2c.scan())
    i2c.unlock()

    # this is a hack that hijacks digitalio
#     class Pin(pin.Pin):
#         def __init__(self, pin_id=None):
#             super().__init__(mcp, pin_id)
# 
#     digitalio.Pin = Pin

    # doing this without the hack requires a change in digitalio
    pin1 = digitalio.DigitalInOut(board.G1)
    print("pin1", pin1.value)
    pin1.switch_to_output(True)

    # import the mcp2221 analogio because that's how it works
    adc = analogio.AnalogIn(board.G2)
    print("adc2", adc.value)

