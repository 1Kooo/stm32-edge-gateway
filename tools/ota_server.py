#!/usr/bin/env python3
"""
OTA 固件服务端（跑在 PC 上，板子通过 ESP-01S 主动来拉）。

协议（全部小端）：
    ① MCU -> PC   "OTA?\r\n"
    ② PC  -> MCU  20 字节固件头
                  magic    32  固定 0x57465747 ('GWFW')
                  version  32  高 16 位主版本，低 16 位次版本
                  fw_len   32  固件正文字节数
                  fw_crc   32  正文 CRC32（高位优先变体，见 ota_crc.py）
                  hdr_crc  32  上面 16 字节的 CRC32
    ③ MCU -> PC   "GET <偏移>\r\n"
    ④ PC  -> MCU  从该偏移开始的 min(chunk, 剩余) 字节
    ⑤ 重复 ③④ 直到收完，客户端不回任何东西，直接断开

为什么是「拉取」而不是「服务器一直推」：
    ESP 串口 115200，外部 Flash 每擦一个 4KB 扇区要阻塞 60~100ms，
    这期间收不了数据。服务器猛推会导致 ESP 缓冲溢出丢字节，
    丢在哪都查不出来。让 MCU 要一块收一块，天然限速。

用法：
    python ota_server.py
    python ota_server.py --bin gwbase.bin --port 8888 --version 0.2
"""

import argparse
import os
import socket
import struct
import sys
import time

from ota_crc import crc32_msb

MAGIC = 0x57465747  # 'G' 'W' 'F' 'W'


def build_header(payload, version_text):
    """按协议拼 20 字节头。version_text 形如 '0.2' -> 0x00000002。"""
    parts = version_text.split(".")
    major = int(parts[0])
    minor = int(parts[1]) if len(parts) > 1 else 0
    version = ((major & 0xFFFF) << 16) | (minor & 0xFFFF)

    body = struct.pack("<IIII", MAGIC, version, len(payload), crc32_msb(payload))
    hdr = struct.pack("<I", crc32_msb(body))
    return body + hdr, version


def read_line(conn, buf):
    """从连接里读一行（\\r\\n 结尾）。返回 (line, 剩余buf)；超时返回 (None, buf)。"""
    while b"\r\n" not in buf:
        try:
            chunk = conn.recv(256)
        except socket.timeout:
            return None, buf
        except OSError:
            return None, buf
        if not chunk:
            return None, buf
        buf += chunk
    line, _, rest = buf.partition(b"\r\n")
    return line, rest


def safe_send(conn, data):
    """发一段数据，对方中途断开时返回 False，别让异常把整个服务端带崩。

    板子在下载途中掉电 / 复位 / WiFi 掉线，这一端拿到的就是
    BrokenPipeError、ConnectionResetError 这类 OSError（都是 OSError 子类）。
    原来是直接抛出去的：serve_conn 没接，main 里也只 catch 了
    KeyboardInterrupt，于是整个服务端进程跟着退出，下一次还得手动重开。
    现场调试时这个现象很像"服务器自己挂了"，很难往"板子断了一下"上想。"""
    try:
        conn.sendall(data)
        return True
    except OSError:
        return False


def serve_conn(conn, peer, header, payload, chunk_size):
    buf = b""
    sent_chunks = 0
    total = len(payload)
    t0 = time.time()

    print("")
    print("[连接] %s:%d  固件 %d 字节，分 %d 块"
          % (peer[0], peer[1], total, (total + chunk_size - 1) // chunk_size))

    while True:
        line, buf = read_line(conn, buf)
        if line is None:
            print("[连接] 客户端断开（已发 %d 块）" % sent_chunks)
            return

        try:
            text = line.decode("ascii", "replace").strip()
        except Exception:
            text = ""

        if text.startswith("OTA?"):
            if not safe_send(conn, header):
                print("[连接] 客户端断开（发固件头时）")
                return
            # 板子拿到固件头之后不会马上要第一块：它要先把暂存区的 141 个
            # 扇区擦干净（每扇区 45~400ms，合计 6~14 秒），这段时间一个字节
            # 都不发。沿用 10 秒读超时就会在擦除中途把连接掐断，现象是
            # "第一次一块都没发就断开、第二次才下得动"（第二次扇区已是
            # 擦除态，擦得快，赶在 10 秒内发出了 GET 0）。
            # 所以这里单独放宽；收到第一个 GET 后立刻收回 10 秒。
            conn.settimeout(60.0)
            print("[握手] 已发送固件头  magic=0x%08X" % struct.unpack("<I", header[0:4])[0])
            continue

        if text.startswith("GET"):
            try:
                offset = int(text[3:].strip())
            except ValueError:
                print("[错误] 无法解析偏移：%r" % text)
                return

            # 进入下载阶段，把超时收回 10 秒：每块之间有数据流动，
            # 10 秒还没动静就是真断线，不该再等（否则故障被藏起来）。
            conn.settimeout(10.0)

            if offset >= total:
                print("[错误] 偏移 %d 超出固件长度 %d" % (offset, total))
                return

            n = min(chunk_size, total - offset)
            if not safe_send(conn, payload[offset:offset + n]):
                print("[连接] 客户端断开（已发 %d 块，第 %d 块没发出去）"
                      % (sent_chunks, sent_chunks + 1))
                return
            sent_chunks += 1

            # 每 20 块打一次，约 40KB，不刷屏
            if sent_chunks % 20 == 0 or offset + n >= total:
                pct = (offset + n) * 100.0 / total
                print("[进度] %6d / %d  (%5.1f%%)  %d 块  %.1fs"
                      % (offset + n, total, pct, sent_chunks, time.time() - t0))
            continue

        if text in ("QUIT", "BYE"):
            print("[连接] 客户端主动结束")
            return

        print("[忽略] 未知请求：%r" % text)


def main():
    # 输出被重定向到文件时 Python 默认整块缓冲，进度会看不见。
    # 强制按行刷新，边跑边能看到日志。
    sys.stdout.reconfigure(line_buffering=True)

    ap = argparse.ArgumentParser()
    ap.add_argument("--bin", default=os.path.join(os.path.dirname(__file__), "gwbase.bin"))
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--version", default="0.2")
    ap.add_argument("--chunk", type=int, default=2048)
    args = ap.parse_args()

    if not os.path.isfile(args.bin):
        print("找不到固件：%s" % args.bin)
        print("先跑 python make_bin.py 生成")
        return 1

    with open(args.bin, "rb") as f:
        payload = f.read()

    header, version = build_header(payload, args.version)

    print("固件   : %s" % args.bin)
    print("长度   : %d 字节 (%.1f KB)" % (len(payload), len(payload) / 1024.0))
    print("版本   : %d.%d  (0x%08X)" % (version >> 16, version & 0xFFFF, version))
    print("CRC32  : 0x%08X" % crc32_msb(payload))
    print("块大小 : %d 字节" % args.chunk)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.host, args.port))
    srv.listen(2)

    print("")
    print("监听 %s:%d …  （Ctrl+C 退出）" % (args.host, args.port))
    print("板子上的 OTA_SERVER_IP 要填这台机器的地址")

    try:
        while True:
            try:
                conn, peer = srv.accept()
            except OSError as e:
                # 端口突然不可用之类的瞬时错误：歇一下接着等，不要整个退出
                print("[错误] accept 失败：%s，1 秒后重试" % e)
                time.sleep(1.0)
                continue

            conn.settimeout(10.0)
            try:
                serve_conn(conn, peer, header, payload, args.chunk)
            except OSError as e:
                # serve_conn 里的收发都过 safe_send / read_line，正常不会到这；
                # 留着兜底，免得一个罕见错误把监听也带走
                print("[连接] 异常断开：%s" % e)
            finally:
                try:
                    conn.close()
                except OSError:
                    pass
    except KeyboardInterrupt:
        print("")
        print("已停止")
    finally:
        srv.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
