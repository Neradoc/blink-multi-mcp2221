# borrowed from:
# https://gist.github.com/twitchyliquid64/a093ce11245274a2adeb631ccd2ba7eb

class ByteDecoder(object):
    def __init__(self, b, multiplier):
        self.b = b
        self.multiplier = multiplier
    def value(self, data):
        return data[self.b] * self.multiplier

class HexDecoder(object):
    def __init__(self, start, end):
        self.start = start
        self.end = end
    def value(self, data):
        section = data[self.start:self.end]
        section.reverse()
        return ''.join('{:02x}'.format(x) for x in section)

class BitDecoder(object):
    def __init__(self, byte, bit):
        self.byte = byte
        self.bit = bit
    def value(self, data):
        return bool(data[self.byte] & (1 << self.bit))

class EnumDecoder(object):
    def __init__(self, byte, mask, opts):
        self.byte = byte
        self.mask = mask
        self.opts = opts
    def value(self, data):
        return self.opts[data[self.byte] & self.mask]

CHIP_SETTINGS_MAP = {
    'Provide serial number on enumeration': BitDecoder(4, 7),
    'USB vendorID': HexDecoder(8, 10),
    'USB productID': HexDecoder(10, 12),
    'Requested mA': ByteDecoder(13, 2),
    'Chip security': EnumDecoder(4, 0b11, {0: 'Unsecured', 1: 'Password-protected', 2: 'Permanently-locked', 3: 'Permanently-locked'})
}

FLASH_SETTINGS_SUBCODE = b"\x00"

def settings(mcp):
    chip_settings = mcp._read_flash(FLASH_SETTINGS_SUBCODE)
    output = dict()
    for attr in CHIP_SETTINGS_MAP:
        output[attr] = CHIP_SETTINGS_MAP[attr].value(chip_settings)
    return output

