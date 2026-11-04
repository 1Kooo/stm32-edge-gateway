#!/usr/bin/env python3
"""
把 Keil 编出来的 gwbase.axf 转成裸二进制 gwbase.bin，并打印长度和 CRC。

为什么要 .bin 而不是 .hex：
    hex 是带地址记录的文本格式，MCU 侧要自己解析；bin 是纯字节流，
    从 0x08010000 开始一段连续内容，直接往外部 Flash 里搬就行。

用法：
    python make_bin.py                  # 用默认路径
    python make_bin.py --axf xxx.axf --out yyy.bin
"""

import argparse
import os
import subprocess
import sys

from ota_crc import crc32_msb

FROMELF = r"D:\keil5\ARM\ARMCC\bin\fromelf.exe"
DEFAULT_AXF = r"D:\gw\gwbase\Obj\gwbase.axf"
DEFAULT_BIN = r"D:\gw\tools\gwbase.bin"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axf", default=DEFAULT_AXF)
    ap.add_argument("--out", default=DEFAULT_BIN)
    args = ap.parse_args()

    if not os.path.isfile(FROMELF):
        print("找不到 fromelf：%s" % FROMELF)
        return 1
    if not os.path.isfile(args.axf):
        print("找不到 axf：%s（先在 Keil 里编译一次）" % args.axf)
        return 1

    # --bin 输出裸二进制，不带地址填充
    cmd = [FROMELF, "--bin", "--output=" + args.out, args.axf]
    print("运行：%s" % " ".join(cmd))
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("fromelf 失败：")
        print(r.stdout)
        print(r.stderr)
        return 1

    if not os.path.isfile(args.out):
        print("fromelf 说成功了，但没有生成 %s" % args.out)
        return 1

    with open(args.out, "rb") as f:
        data = f.read()

    # 前 8 个字节应该是向量表：栈顶 + 复位入口
    sp = int.from_bytes(data[0:4], "little") if len(data) >= 4 else 0
    pc = int.from_bytes(data[4:8], "little") if len(data) >= 8 else 0

    print("")
    print("输出   : %s" % args.out)
    print("长度   : %d 字节 (%.1f KB)" % (len(data), len(data) / 1024.0))
    print("CRC32  : 0x%08X   <- 板上算出来必须是这个值" % crc32_msb(data))
    print("栈顶   : 0x%08X  (应在 0x20000000~0x20020000)" % sp)
    print("复位入口: 0x%08X  (应在 0x08010000 之后，且最低位为 1)" % pc)

    if not (0x20000000 <= sp <= 0x20020000):
        print("警告：栈顶地址不像 RAM，确认 axf 是不是链接到了 0x08010000")
    if not (0x08010000 <= pc < 0x08100000):
        print("警告：复位入口不在 APP 区，散转文件或起始地址可能不对")

    return 0


if __name__ == "__main__":
    sys.exit(main())
