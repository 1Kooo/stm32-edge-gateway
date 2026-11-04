#!/usr/bin/env python3
"""
模拟板子走一遍 OTA 拉取协议，用来验证服务端是否按协议说话。

这一步完全不碰硬件：本机连本机，收完把内容存成 received.bin，
和原始固件逐字节比对 + 校验 CRC。对不上就说明协议或服务端有问题。

用法：
    python ota_client_test.py
    python ota_client_test.py --host 192.168.137.1 --port 8888
"""

import argparse
import os
import socket
import struct
import sys

from ota_crc import crc32_begin, crc32_update, crc32_end

MAGIC = 0x57465747


def recv_exact(conn, n):
    """收满 n 字节，收不满就报错（对应 MCU 那边的读超时逻辑）。"""
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise IOError("连接提前关闭，还差 %d 字节" % (n - len(buf)))
        buf += chunk
    return buf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--chunk", type=int, default=2048)
    ap.add_argument("--save", default=os.path.join(os.path.dirname(__file__), "received.bin"))
    args = ap.parse_args()

    conn = socket.create_connection((args.host, args.port), timeout=5.0)
    conn.settimeout(10.0)
    print("已连上 %s:%d" % (args.host, args.port))

    # ---- ① 握手：要固件头 ----
    conn.sendall(b"OTA?\r\n")
    raw = recv_exact(conn, 20)

    magic, version, fw_len, fw_crc, hdr_crc = struct.unpack("<IIIII", raw)
    print("固件头 : magic=0x%08X ver=%d.%d len=%d crc=0x%08X"
          % (magic, version >> 16, version & 0xFFFF, fw_len, fw_crc))

    if magic != MAGIC:
        print("FAIL: magic 不对")
        return 1

    # ---- ② 校验头部 CRC（对应板上 hdr_crc 检查）----
    calc = crc32_end(crc32_update(crc32_begin(), raw[0:16]))
    if calc != hdr_crc:
        print("FAIL: 头 CRC 不符，算出来 0x%08X，头里写 0x%08X" % (calc, hdr_crc))
        return 1
    print("头校验 : OK")

    if fw_len <= 0 or fw_len > 1024 * 1024 - 256:
        print("FAIL: 固件长度不合理 %d" % fw_len)
        return 1

    # ---- ③ 拉数据 ----
    data = bytearray()
    crc = crc32_begin()

    while len(data) < fw_len:
        offset = len(data)
        conn.sendall(("GET %d\r\n" % offset).encode("ascii"))
        n = min(args.chunk, fw_len - offset)
        block = recv_exact(conn, n)
        data += block
        crc = crc32_update(crc, block)

        if (offset // args.chunk) % 20 == 0:
            print("  拉取 %6d / %d  (%.1f%%)" % (len(data), fw_len, len(data) * 100.0 / fw_len))

    conn.close()

    got_crc = crc32_end(crc)
    print("")
    print("收到   : %d 字节" % len(data))
    print("CRC32  : 0x%08X   （头里声明 0x%08X）" % (got_crc, fw_crc))

    if got_crc != fw_crc:
        print("FAIL: 正文 CRC 不符")
        return 1

    with open(args.save, "wb") as f:
        f.write(bytes(data))
    print("已保存 : %s" % args.save)

    # ---- ④ 如果有原始固件，逐字节比对 ----
    src = os.path.join(os.path.dirname(__file__), "gwbase.bin")
    if os.path.isfile(src):
        with open(src, "rb") as f:
            origin = f.read()
        if origin == bytes(data):
            print("比对   : 与 gwbase.bin 完全一致，共 %d 字节" % len(origin))
        else:
            print("比对   : 不一致！原始 %d 字节，收到 %d 字节" % (len(origin), len(data)))
            for i in range(min(len(origin), len(data))):
                if origin[i] != data[i]:
                    print("         首个差异在第 %d 字节：0x%02X vs 0x%02X"
                          % (i, origin[i], data[i]))
                    break
            return 1

    print("")
    print("=== 全部通过：服务端协议正确 ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
