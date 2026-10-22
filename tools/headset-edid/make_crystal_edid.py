#!/usr/bin/env python3
"""EDID override for a Pimax headset on NVIDIA (Linux): Crystal Super, Crystal (original), Crystal Light.

From the headset's own EDID (/sys/class/drm/cardX-DP-N/edid):
  * base block byte 20 -> digital, 8 bits per colour, DisplayPort (0xA5); the Crystal Super says "colour depth
    undefined" (0x85), with which NVKMS validates the mode at 6 bpc, and DSC refuses 6 bpc, so its modes fail the DP
    bandwidth check;
  * a Microsoft VSDB v2 "VR headset" (the kernel marks the connector non-desktop, so the compositor offers it for a DRM
    lease instead of extending the desktop onto it): written into the headset's CTA-861 block when it has one (the
    original Crystal), otherwise in a new CTA block (the Crystal Super, whose EDID is base + DisplayID only);
  * every other block unchanged (their timings are what NVKMS accepted).
NVIDIA's override belongs to a GPU output, not to a headset: --add appends another headset's DisplayID timings, so
either headset gets its own modes on that output (pibridge-headset-edid adds every known Pimax this way). Both
headsets report the same vendor and product (PVR 0x1019), so the base block's identity does not matter.
Running it again on its own output changes nothing.
Usage: make_crystal_edid.py <in.bin> [--add <other-headset.bin>]... <out.bin>
"""
import sys
import uuid

HDR = bytes([0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0])
PVR = bytes([0x42, 0xd2])
MS_OUI = bytes([0x5c, 0x12, 0xca])     # CA-12-5C, little endian


def is_pimax(edid):
    return edid[:8] == HDR and edid[8:10] == PVR


def blocks(edid, name='EDID'):
    """(base block, extension blocks) of a checked Pimax EDID."""
    assert edid[:8] == HDR, f'{name}: not an EDID'
    assert edid[8:10] == PVR, f'{name}: not a Pimax (PVR) EDID'
    assert len(edid) == 128 * (edid[126] + 1), f'{name}: {len(edid)} bytes, but the base block counts {edid[126]} extensions'
    return edid[:128], [edid[128 * i:128 * (i + 1)] for i in range(1, edid[126] + 1)]


def has_msvsdb(edid):
    """True when a CTA block of the EDID carries a Microsoft VSDB (a VR headset flag, ours or the maker's)."""
    for i in range(1, len(edid) // 128):
        cta = edid[128 * i:128 * (i + 1)]
        if cta[0] != 0x02:
            continue
        j = 4
        while j < min(cta[2], 127):
            tag, n = cta[j] >> 5, cta[j] & 0x1f
            if tag == 3 and n >= 3 and cta[j + 1:j + 4] == MS_OUI:
                return True
            j += 1 + n
    return False


def with_msvsdb(cta, vsdb):
    """The CTA block (127 bytes, no checksum) with the VSDB appended to its data blocks; the detailed timings after
    them move along."""
    d = cta[2]
    body = bytes(cta[d:127]).rstrip(b'\0')
    dtds = body + b'\0' * (-len(body) % 18)     # whole 18-byte descriptors
    assert d + len(vsdb) + len(dtds) <= 127, 'no room for the VR headset flag in the CTA block'
    out = bytearray(cta[:d]) + vsdb + dtds
    out[2] = d + len(vsdb)
    return out.ljust(127, b'\0')


def build(edid, others=()):
    """The override EDID for a headset's own EDID, plus the DisplayID timings of the other headsets' EDIDs."""
    base, exts = blocks(edid)
    if not has_msvsdb(edid):
        # tag 3 len 21: OUI CA-12-5C, version 2, desktop usage 7 = VR headset, a container id fixed per headset
        vsdb = bytes([0x75]) + MS_OUI + bytes([0x02, 0x07]) + uuid.uuid5(uuid.NAMESPACE_OID, base[8:18].hex()).bytes
        ctas = [i for i, b in enumerate(exts) if b[0] == 0x02]
        if ctas:
            exts[ctas[0]] = bytes(with_msvsdb(exts[ctas[0]], vsdb)) + b'\0'
        else:
            cta = bytearray([0x02, 0x03, 4, 0x80]).ljust(127, b'\0')   # rev 3, underscan, no audio, no DTDs
            exts.insert(0, bytes(with_msvsdb(cta, vsdb)) + b'\0')
    for other in others:
        for b in blocks(other)[1]:
            if b[0] == 0x70 and b not in exts:
                exts.append(b)
    base = bytearray(base)
    base[20] = 0xA5          # digital, 8 bpc, DisplayPort
    base[126] = len(exts)
    return b''.join(bytes(b[:127]) + bytes([(-sum(b[:127])) & 0xff]) for b in [base] + exts)


def main(argv):
    adds = []
    while '--add' in argv:
        i = argv.index('--add')
        adds.append(argv[i + 1])
        del argv[i:i + 2]
    if len(argv) != 2:
        sys.exit(__doc__)
    out = build(open(argv[0], 'rb').read(), [open(p, 'rb').read() for p in adds])
    open(argv[1], 'wb').write(out)
    print(f'{argv[1]}: {len(out)} bytes, {len(out) // 128 - 1} extension blocks')


if __name__ == '__main__':
    main(sys.argv[1:])
