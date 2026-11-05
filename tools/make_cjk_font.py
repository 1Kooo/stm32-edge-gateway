#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成 LVGL 中文字体 lv_font_cn_14.c

字表分两部分：
  1. 扫 gwbase 下所有 .c/.h 里出现过的汉字（跳过 Obj 和 LVGL 库）
     代码注释里写的词就是界面会用到的词，自动收全，不用手工数缺哪个字。
  2. tools/font_chars_extra.txt 里手工补的字（注释里没有的界面字、中文标点）

改完界面文案后重跑一次本脚本就行。

用法：
    python tools/make_cjk_font.py

依赖：
    tools/.fonttools/node_modules/lv_font_conv（首次运行前执行
    npm i lv_font_conv@1.5.3 --prefix tools/.fonttools）
"""

import os
import re
import subprocess
import sys

# 工程根目录（本脚本在 tools/ 下）
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(ROOT, "gwbase")
OUT = os.path.join(BASE, "APP", "lv_font_cn_14.c")
EXTRA = os.path.join(ROOT, "tools", "font_chars_extra.txt")
CONV = os.path.join(ROOT, "tools", ".fonttools", "node_modules",
                    "lv_font_conv", "lv_font_conv.js")

# 思源黑体简体版，SIL OFL 授权，可自由嵌入固件。
# 不用微软雅黑/黑体：那两种字形版权归方正，嵌进产品固件有授权风险。
FONT = "C:/Windows/Fonts/SourceHanSansCN-Normal.ttf"
SIZE = 14        # 与界面现有 14px 一致（行高 16 ~ 17）
BPP = 4          # 4 位灰度。14px 汉字横画只有 1 像素宽，降到 2 位会断断续续
FONT_NAME = "lv_font_cn_14"

# 扫描时跳过的目录：编译产物，以及 LVGL 库本身
# （库里带的 CJK 字体是日文字集，混进字表没意义）
SKIP_DIRS = {"Obj", ".git", "LVGL", "__pycache__"}


def collect_from_sources():
    """收集 gwbase 下所有源文件里出现过的汉字。"""
    chars = set()
    for root, dirs, files in os.walk(BASE):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            if not name.endswith((".c", ".h")):
                continue
            path = os.path.join(root, name)
            with open(path, encoding="utf-8", errors="ignore") as f:
                text = f.read()
            for ch in text:
                if "\u4e00" <= ch <= "\u9fff":
                    chars.add(ch)
    return chars


def collect_extra():
    """读手工补充表，一行一串字。"""
    chars = set()
    if not os.path.exists(EXTRA):
        return chars
    with open(EXTRA, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            chars.update(line)
    # 空格和制表符由 -r 0x20-0x7F 覆盖，这里别重复收
    chars.discard(" ")
    chars.discard("\t")
    return chars


def count_bitmap_bytes(path):
    """数生成文件里 glyph_bitmap 的实际字节数，用来估算 Flash 占用。"""
    with open(path, encoding="utf-8", errors="ignore") as f:
        text = f.read()
    i = text.find("glyph_bitmap[] = {")
    j = text.find("};", i)
    return len(re.findall(r"0x[0-9a-fA-F]{1,2}", text[i:j]))


def main():
    if not os.path.exists(CONV):
        print("找不到 lv_font_conv，先执行：")
        print("  npm i lv_font_conv@1.5.3 --prefix tools/.fonttools")
        return 1
    if not os.path.exists(FONT):
        print("找不到字体文件：" + FONT)
        return 1

    from_src = collect_from_sources()
    extra = collect_extra()
    # 排序是为了每次生成结果完全一致，git 里不会出现无意义的顺序变动
    symbols = "".join(sorted(from_src | extra))

    print("源码里的汉字: %d" % len(from_src))
    print("补充表新增  : %d" % len(extra - from_src))
    print("合计字符    : %d（另有 ASCII 0x20-0x7F 共 95 个）" % len(symbols))

    cmd = [
        "node", CONV,
        "--font", FONT,
        "--size", str(SIZE),
        "--bpp", str(BPP),
        "--format", "lvgl",
        "--no-compress",
        "--no-prefilter",
        "--lv-font-name", FONT_NAME,
        "--lv-include", "lvgl.h",
        "-r", "0x20-0x7F",
        "--symbols", symbols,
        "-o", OUT,
    ]
    r = subprocess.run(cmd)
    if r.returncode != 0:
        print("生成失败，返回码 %d" % r.returncode)
        return r.returncode

    n = count_bitmap_bytes(OUT)
    print("已生成: %s" % OUT)
    print("位图字节: %d（%.1f KB）" % (n, n / 1024.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
