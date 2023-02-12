# blink-multi-mcp2221
A library to find and use multiple MCP2221A chips with Blinka

```py
from mcp2221_extended import get_mcp_list, demo_list

mcps = get_mcp_list(timeout=5)

for pos, mcp in enumerate(mcps):
    scan = mcp.i2c_scan()
    print("-"*70)
    print(f"MCP found {pos}")
    print("  I2C scan:", [hex(addr) for addr in scan])
    print("  Pins:", [mcp.gpio_get_pin(i) for i in range(4)])
    print("  Serial number:", mcp.serial_number)
    print("  As an hex int:", int(mcp.serial_number, 16))
```
