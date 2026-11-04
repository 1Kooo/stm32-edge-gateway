#!/usr/bin/env python3
"""
CRC32 高位优先版（CRC-32/BZIP2 变体），用于和 MCU 上的 ota_crc32() 对齐。

为什么不能用 zlib.crc32：
    zlib / 标准 CRC-32（ISO-HDLC）是「反射」实现，数据位先反转再算；
    MCU 上那份是「非反射」实现，高位先进、左移、多项式 0x04C11DB7。
    两者计算结果完全不同，混用会让板上校验永远失败，而且两边各自看都合理。

参数：poly=0x04C11DB7 init=0xFFFFFFFF refin=False refout=False xorout=0xFFFFFFFF
校验值：crc32_msb(b"123456789") == 0xFC891918
"""

POLY = 0x04C11DB7
MASK = 0xFFFFFFFF


def _build_table():
    """预先算好 256 项查表，避免逐位循环（400KB 数据快 8 倍左右）。"""
    table = []
    for i in range(256):
        crc = i << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ POLY) & MASK
            else:
                crc = (crc << 1) & MASK
        table.append(crc)
    return table


_TABLE = _build_table()


def crc32_msb(data):
    """一次性算完一整段数据。"""
    crc = 0xFFFFFFFF
    for byte in data:
        idx = ((crc >> 24) ^ byte) & 0xFF
        crc = ((crc << 8) & MASK) ^ _TABLE[idx]
    return crc ^ MASK


def crc32_begin():
    return 0xFFFFFFFF


def crc32_update(crc, data):
    """增量式：crc 传入上一次的中间值（首次用 crc32_begin()）。"""
    for byte in data:
        idx = ((crc >> 24) ^ byte) & 0xFF
        crc = ((crc << 8) & MASK) ^ _TABLE[idx]
    return crc


def crc32_end(crc):
    return crc ^ MASK


if __name__ == "__main__":
    check = crc32_msb(b"123456789")
    print("crc32_msb(b'123456789') = 0x%08X" % check)
    if check != 0xFC891918:
        print("FAIL: 期望 0xFC891918，算法实现有误")
        raise SystemExit(1)

    # 增量与一次性必须一致
    whole = crc32_msb(b"123456789")
    step = crc32_end(crc32_update(crc32_update(crc32_begin(), b"1234"), b"56789"))
    if whole != step:
        print("FAIL: 增量版与一次性版结果不一致")
        raise SystemExit(1)
    print("OK: 查表版与增量版一致")
