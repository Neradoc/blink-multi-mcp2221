import atexit
import time
from mcp2221 import mcp2221


def get_mcp_list(num=999, timeout=2):
    mcps = []
    t0 = time.monotonic()
    while True:
        try:
            mcp = mcp2221.MCP2221()
            mcps.append(mcp)
            atexit.register(mcp.deinit)
        except OSError as err:
            if len(mcps) >= num:
                break
            if time.monotonic() > t0 + timeout:
                break
        time.sleep(0.1)
    mcps.sort(key=lambda m: m.serial_number)
    return mcps


def demo_list():
    mcps = get_mcp_list(timeout=10, num=2)
    print(mcps)

    disps = []
    for pos, mcp in enumerate(mcps):
        scan = mcp.i2c_scan()
        print("-"*70)
        print(f"MCP found {pos}")
        print("  I2C scan:", [hex(addr) for addr in scan])
        print("  Pins:", [mcp.gpio_get_pin(i) for i in range(4)])
        print("  Serial number:", mcp.serial_number)
        print("  As an hex int:", int(mcp.serial_number, 16))


if __name__ == "__main__":
    demo_list()
