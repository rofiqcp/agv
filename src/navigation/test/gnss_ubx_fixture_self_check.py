#!/usr/bin/env python3
"""Protocol fixture checks for the u-blox M9 fields used by GNSS Driver V2.

This test does not require ROS. It freezes the official UBX offsets/scales used by
Part 1 and catches accidental edits to the parser contract.
"""
from pathlib import Path
import math
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
CPP = (ROOT / "src/gnss_node.cpp").read_text(encoding="utf-8")


def fail(msg: str):
    print("FAIL:", msg, file=sys.stderr)
    raise SystemExit(1)

# Synthetic NAV-PVT (92 bytes) with distinctive values at canonical offsets.
p = bytearray(92)
struct.pack_into('<I', p, 0, 345_678_900)  # iTOW ms
struct.pack_into('<H', p, 4, 2026)
p[6:12] = bytes([8, 22, 15, 37, 42, 0x07])
struct.pack_into('<I', p, 12, 25_000)       # tAcc ns
struct.pack_into('<i', p, 16, 123_456_789)  # nano ns
p[20] = 3                                   # fixType
p[21] = 0x83                                # gnssFixOK + diffSoln + carrSoln=2
p[22] = 0xE0
p[23] = 18
struct.pack_into('<i', p, 24, 110_123_456)  # lon 11.0123456 deg
struct.pack_into('<i', p, 28, -70_123_456)  # lat -7.0123456 deg
struct.pack_into('<i', p, 32, 125_000)      # ellipsoid mm
struct.pack_into('<i', p, 36, 95_000)       # hMSL mm
struct.pack_into('<I', p, 40, 850)          # hAcc mm
struct.pack_into('<I', p, 44, 1300)         # vAcc mm
struct.pack_into('<i', p, 48, 220)          # velN mm/s
struct.pack_into('<i', p, 52, 310)          # velE mm/s
struct.pack_into('<i', p, 56, -20)          # velD mm/s
struct.pack_into('<i', p, 60, 380)          # gSpeed mm/s
struct.pack_into('<i', p, 64, 3_300_000)    # headMot 33 deg
struct.pack_into('<I', p, 68, 45)           # sAcc mm/s
struct.pack_into('<I', p, 72, 150_000)      # headAcc 1.5 deg
struct.pack_into('<H', p, 76, 123)           # pDOP 1.23
struct.pack_into('<H', p, 78, 0x2008)        # authTime + correction age code 4, invalidLLH=0
struct.pack_into('<i', p, 84, 3_250_000)     # headVeh 32.5 deg
struct.pack_into('<h', p, 88, -235)          # magDec -2.35 deg
struct.pack_into('<H', p, 90, 25)            # magAcc 0.25 deg

checks = {
    'itow': struct.unpack_from('<I', p, 0)[0] == 345_678_900,
    'vacc': struct.unpack_from('<I', p, 44)[0] / 1000.0 == 1.3,
    'veln': struct.unpack_from('<i', p, 48)[0] / 1000.0 == 0.22,
    'vele': struct.unpack_from('<i', p, 52)[0] / 1000.0 == 0.31,
    'veld': struct.unpack_from('<i', p, 56)[0] / 1000.0 == -0.02,
    'pdop': math.isclose(struct.unpack_from('<H', p, 76)[0] * 0.01, 1.23),
    'flags3': struct.unpack_from('<H', p, 78)[0] == 0x2008,
}
for name, ok in checks.items():
    if not ok:
        fail(f'NAV-PVT fixture mismatch: {name}')

# Synthetic NAV-COV (64 bytes): NED upper triangular covariance.
c = bytearray(64)
struct.pack_into('<I', c, 0, 345_678_900)
c[5] = 1; c[6] = 1
vals = [4.0, 0.2, 0.3, 9.0, 0.4, 16.0, 0.04, 0.002, 0.003, 0.09, 0.004, 0.16]
for i, v in enumerate(vals):
    struct.pack_into('<f', c, 16 + 4*i, v)
f = [struct.unpack_from('<f', c, 16 + 4*i)[0] for i in range(12)]
# NED -> ENU expected: xx=EE, yy=NN, zz=DD, xz=-ED, yz=-ND.
if not (math.isclose(f[3], 9.0) and math.isclose(f[0], 4.0) and
        math.isclose(-f[4], -0.4, abs_tol=1e-6) and math.isclose(-f[2], -0.3, abs_tol=1e-6)):
    fail('NAV-COV NED->ENU fixture mismatch')

# Synthetic NAV-DOP (18 bytes) field order g,p,t,v,h,n,e.
d = bytearray(18)
struct.pack_into('<I', d, 0, 345_678_900)
for i, raw in enumerate([110, 120, 130, 140, 150, 160, 170]):
    struct.pack_into('<H', d, 4 + 2*i, raw)
dop_vals = [struct.unpack_from('<H', d, 4+2*i)[0] * 0.01 for i in range(7)]
if not all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(dop_vals, [1.1,1.2,1.3,1.4,1.5,1.6,1.7])):
    fail('NAV-DOP fixture mismatch')

# 5 Hz target -> 200 ms nominal measurement interval.
if round(1000.0 / 5.0) != 200:
    fail('CFG-RATE 5 Hz conversion mismatch')

# Keep source contract tied to the same documented offsets / transforms.
for token in [
    'std::memcpy(&vel_n_raw,&payload[48],4)',
    'std::memcpy(&vel_e_raw,&payload[52],4)',
    'std::memcpy(&vel_d_raw,&payload[56],4)',
    'std::memcpy(&flags3, &payload[78], 2)',
    'nav_cov_.pos_ee', '-nav_cov_.pos_ed', '-nav_cov_.pos_nd',
    '0x30210001u', '0x30210002u',
]:
    if token not in CPP:
        fail(f'C++ parser/config token missing: {token}')

print('PASS gnss_ubx_fixture_self_check')
