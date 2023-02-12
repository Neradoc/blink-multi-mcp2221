import time
from . import mcp2221


def get_mcp_list(num=999, timeout=5, debug=False):
    mcps = []
    t0 = time.monotonic()
    while True:
        try:
            mcp = mcp2221.MCP2221()
            mcps.append(mcp)
        except OSError as err:
            if debug:
                print(len(mcps), err)
            if time.monotonic() > t0 + timeout:
                if debug:
                    print("get_mcp_list: Timeout")
                break
        if len(mcps) >= num:
            if debug:
                print("get_mcp_list: All found")
            break
        time.sleep(0.1)
    mcps.sort(key=lambda m: m.serial_number)
    return mcps


def demo_list(mcps=None):
    if mcps is None:
        mcps = get_mcp_list(timeout=10, num=2)
        print(mcps)

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
