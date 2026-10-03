"""GPS L1 C/A codes (IS-GPS-200, Gold codes from the G1 and G2 shift registers)."""
from __future__ import annotations

from functools import lru_cache

import numpy as np

CHIP_RATE = 1.023e6
CODE_LEN = 1023

# G2 phase-selector taps per PRN (IS-GPS-200 Table 3-Ia).
G2_TAPS = {1: (2, 6), 2: (3, 7), 3: (4, 8), 4: (5, 9), 5: (1, 9), 6: (2, 10), 7: (1, 8), 8: (2, 9),
           9: (3, 10), 10: (2, 3), 11: (3, 4), 12: (5, 6), 13: (6, 7), 14: (7, 8), 15: (8, 9),
           16: (9, 10), 17: (1, 4), 18: (2, 5), 19: (3, 6), 20: (4, 7), 21: (5, 8), 22: (6, 9),
           23: (1, 3), 24: (4, 6), 25: (5, 7), 26: (6, 8), 27: (7, 9), 28: (8, 10), 29: (1, 6),
           30: (2, 7), 31: (3, 8), 32: (4, 9)}

# First 10 chips in octal, IS-GPS-200 Table 3-Ia: a check on the generator.
FIRST10_OCT = {1: 0o1440, 2: 0o1620, 3: 0o1710, 4: 0o1744, 5: 0o1133, 6: 0o1455, 7: 0o1131,
               8: 0o1454, 9: 0o1626, 10: 0o1504, 11: 0o1642, 12: 0o1750, 13: 0o1764, 14: 0o1772,
               15: 0o1775, 16: 0o1776, 17: 0o1156, 18: 0o1467, 19: 0o1633, 20: 0o1715, 21: 0o1746,
               22: 0o1763, 23: 0o1063, 24: 0o1706, 25: 0o1743, 26: 0o1761, 27: 0o1770, 28: 0o1774,
               29: 0o1127, 30: 0o1453, 31: 0o1625, 32: 0o1712}


@lru_cache(maxsize=64)
def ca_bits(prn: int) -> np.ndarray:
    """The 1023 chips as 0/1."""
    g1 = [1] * 10
    g2 = [1] * 10
    t1, t2 = G2_TAPS[prn]
    out = np.empty(CODE_LEN, np.int8)
    for i in range(CODE_LEN):
        out[i] = g1[9] ^ g2[t1 - 1] ^ g2[t2 - 1]
        f1 = g1[2] ^ g1[9]
        f2 = g2[1] ^ g2[2] ^ g2[5] ^ g2[7] ^ g2[8] ^ g2[9]
        g1 = [f1] + g1[:9]
        g2 = [f2] + g2[:9]
    return out


def ca_code(prn: int) -> np.ndarray:
    """The 1023 chips as +-1 floats (bit 0 -> +1)."""
    return (1.0 - 2.0 * ca_bits(prn)).astype(np.float32)


def check() -> None:
    for prn, octv in FIRST10_OCT.items():
        v = int("".join(str(b) for b in ca_bits(prn)[:10]), 2)
        if v != octv:
            raise AssertionError(f"PRN {prn}: first chips {oct(v)}, ICD {oct(octv)}")
