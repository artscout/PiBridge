#!/usr/bin/env python3
"""EDID override for the Pimax Crystal Super on NVIDIA (Linux).

From the headset's own 256-byte EDID (base block + DisplayID 1.2 with the 7104x3840 timings):
  * base block byte 20: digital input, colour depth undefined (0x85) -> 8 bits per colour (0xA5); with "undefined"
    NVKMS validates the mode at 6 bpc, and DSC refuses 6 bpc, so both modes fail the DP bandwidth check;
  * a CTA-861 block with only a Microsoft VSDB v2 (the kernel marks the connector non-desktop, so the compositor
    offers it for a DRM lease instead of using it as a monitor);
  * the headset's DisplayID 1.2 block unchanged (its timings are what NVKMS accepted).
Usage: make_crystal_edid.py <in.bin> <out.bin>
"""
import sys
import uuid

src = open(sys.argv[1], 'rb').read()
assert len(src) == 256 and src[:8] == bytes([0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0]), 'not the 256-byte headset EDID'
assert src[8:10] == bytes([0x42, 0xd2]), 'not a PVR EDID'
assert src[128] == 0x70, 'block 1 is not DisplayID'

base = bytearray(src[:128])
base[20] = 0xA5      # digital, 8 bpc, DisplayPort
base[126] = 2        # two extension blocks
base[127] = (-sum(base[:127])) & 0xff

msvsdb = bytes([0x75, 0x5c, 0x12, 0xca, 0x02, 0x07]) + uuid.uuid4().bytes   # tag 3 len 21: OUI CA-12-5C, v2, use 7 = VR headset
cta = bytearray([0x02, 0x03, 4 + len(msvsdb), 0x80]) + msvsdb   # underscan, no audio, no DTDs
cta = cta.ljust(127, b'\0')
cta.append((-sum(cta)) & 0xff)

out = bytes(base) + bytes(cta) + src[128:256]
assert len(out) == 384
open(sys.argv[2], 'wb').write(out)
