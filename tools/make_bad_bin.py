#!/usr/bin/env python3
"""make_bad_bin.py - 生成"必砖固件"，用于测试 OTA 回滚链路。

原理：翻转 bin 里复位向量的最低字节（偏移 4）。
    复位向量的 Thumb 位（bit0）原本必为 1，翻转后变 0，
    boot 跳过去的一瞬间触发 invalid-state HardFault，固件必死。
    但向量表位置仍在 APP 区内，boot 的 sp/pc 合法性检查照样通过
    —— 完美模拟"CRC 全对、就是跑不起来"的坏固件。

关键点：本工具不改 CRC！ota_server.py 启动时会对它读到的文件
    重新计算 CRC 并写进协议头，所以：
    下载端校验通过 → 安装端 CRC 终检通过 → 跳转 → 死机。
    这正是要测的场景：校验全绿，固件本身是坏的。

用法：
    python make_bad_bin.py                    # gwbase.bin -> bad.bin
    python make_bad_bin.py good.bin bad.bin
"""

import os
import sys

from ota_crc import crc32_msb


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(__file__), "gwbase.bin")
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(__file__), "bad.bin")

    if not os.path.isfile(src):
        print("找不到固件：%s" % src)
        return 1

    with open(src, "rb") as f:
        data = bytearray(f.read())

    if len(data) < 8:
        print("文件太小，不像固件：%d 字节" % len(data))
        return 1

    # 偏移 0 是初始栈顶，偏移 4 是复位向量。翻最低字节 = 清掉 Thumb 位
    data[4] ^= 0xFF

    with open(dst, "wb") as f:
        f.write(data)

    print("输入   : %s (%d 字节)" % (src, len(data)))
    print("翻转   : 偏移 4（复位向量 LSB）0x%02X -> 0x%02X"
          % (data[4] ^ 0xFF, data[4]))
    print("输出   : %s" % dst)
    print("CRC32  : 0x%08X（server 会自动用它，下载/安装校验都会通过）"
          % crc32_msb(bytes(data)))
    print("提示   : python ota_server.py --bin bad.bin --version 99.0")
    return 0


if __name__ == "__main__":
    sys.exit(main())
