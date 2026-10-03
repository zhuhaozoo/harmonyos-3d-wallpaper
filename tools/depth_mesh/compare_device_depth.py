#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段1 判据 · 设备端深度 vs PC 端参考深度（比对 + 出图）

用途：真机跑「深度自检」后，把沙箱里的 `<prefix>.depth.f32.bin`（728×546，原始相对逆深度）
取回本地，与本工具产出的 PC 参考（1080×1440，`ref_depth.py` 输出）比对——
证明"设备侧 MindSpore Lite + .ms 的推理结果"与"PC 端 onnxruntime + ONNX 的参考结果"一致。

用法：
  python compare_device_depth.py --dev <设备bin> --ref <PC前缀> [--dev-meta <设备meta.txt>]

判据（阶段1 验收）：
  [1] 尺寸/元素数可读（设备 bin 元素数 = H*W，与 meta 一致）
  [2] 皮尔逊相关 r ≥ 0.95（两者都是"越大越近"，逐像素相关）
  [3] 各自 p1/p99 归一化后：平均绝对差 ≤ 0.08，且 ≥ 70% 像素差 ≤ 0.05
  附：出对比图 <dev>.cmp.png（左设备/中PC/右差异）

依赖：numpy pillow
"""
import argparse, os, sys
import numpy as np


def load_dev(path, meta_path):
    a = np.fromfile(path, dtype='<f4')
    h, w = 0, 0
    if meta_path and os.path.exists(meta_path):
        txt = open(meta_path).read()
        for tok in txt.split():
            if tok.startswith('out='):
                ww, hh = tok[4:].split('x')
                w, h = int(ww), int(hh)
    if w <= 0 or h <= 0:                    # meta 缺失时按 546×728（阶段1 默认）推断
        w, h = 546, 728
    if a.size != w * h:
        print(f'⚠️ 设备 bin 元素数 {a.size} ≠ {w}*{h}={w*h}（按 546×728 再试）')
        w, h = 546, 728
        if a.size != w * h:
            print('❌ 无法推断设备 bin 形状，请检查 meta'); sys.exit(1)
    return a.reshape(h, w)


def norm01(a):
    p1, p99 = np.percentile(a, 1), np.percentile(a, 99)
    return np.clip((a - p1) / max(1e-9, p99 - p1), 0, 1)


def resize_bilinear(a, ow, oh):
    h, w = a.shape
    ys = np.clip((np.arange(oh) + 0.5) * h / oh - 0.5, 0, h - 1)
    xs = np.clip((np.arange(ow) + 0.5) * w / ow - 0.5, 0, w - 1)
    y0 = np.floor(ys).astype(int); x0 = np.floor(xs).astype(int)
    y1 = np.minimum(h - 1, y0 + 1); x1 = np.minimum(w - 1, x0 + 1)
    ty = (ys - y0)[:, None]; tx = (xs - x0)[None, :]
    top = a[y0][:, x0] * (1 - tx) + a[y0][:, x1] * tx
    bot = a[y1][:, x0] * (1 - tx) + a[y1][:, x1] * tx
    return top * (1 - ty) + bot * ty


def parse_ascii_log(path):
    """从 hilog 文本里解析 native 打的 48×32 深度缩略图（'SRD depth dNN <48 chars>'，近=亮）"""
    levels = " .:-=+*#%@"
    rows = {}
    for line in open(path, encoding='utf-8', errors='ignore'):
        i = line.find('depth d')
        if i < 0:
            continue
        seg = line[i + 7:]
        if len(seg) < 2 or not seg[0:2].isdigit():
            continue
        r = int(seg[0:2])
        body = seg[2:]
        if body.startswith(' '):        # 格式是 "dNN <48字符>"：只去掉紧邻的那个分隔空格
            body = body[1:]
        body = body.rstrip('\r\n')      # 注意：行首/行内的空格是合法档位（远=暗），不可 lstrip
        if len(body) < 48:
            body = body + ' ' * (48 - len(body))   # 截断按最暗档处理
        rows[r] = [levels.index(c) / 9.0 if c in levels else 0.0 for c in body[:48]]
    if len(rows) < 32:
        print(f'⚠️ 日志里只解析到 {len(rows)}/32 行缩略图')
        return None
    return np.array([rows[r] for r in range(32)], dtype=np.float32)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dev', default='', help='设备端 <prefix>.depth.f32.bin（能取回时用）')
    ap.add_argument('--dev-ascii', default='', help='设备日志文本（含 SRD depth dNN 缩略图；沙箱取不出时用）')
    ap.add_argument('--dev-meta', default='', help='设备端 .meta.txt（取 out=WxH）')
    ap.add_argument('--ref', required=True, help='PC 端前缀（ref_depth.py 的 --prefix）')
    args = ap.parse_args()
    if not args.dev and not args.dev_ascii:
        print('需要 --dev <bin> 或 --dev-ascii <日志>'); sys.exit(2)

    ref_raw = np.fromfile(args.ref + '.depth.f32.bin', dtype='<f4')
    ref = ref_raw.reshape(1440, 1080)                      # PC 参考固定 1080×1440

    if args.dev:
        dev = load_dev(args.dev, args.dev_meta)
        ref_rs = resize_bilinear(ref, dev.shape[1], dev.shape[0])
        tag = f'设备 {dev.shape[1]}x{dev.shape[0]}'
    else:
        dev = parse_ascii_log(args.dev_ascii)
        if dev is None:
            sys.exit(1)
        # 复刻端侧 ASCII 的**点采样**规则（native：sx = tx*W/48, sy = ty*H/32，取整后直接取值）——
        # 不可用双线性缩放（那等于额外做了均值滤波，会系统性拉低两者的相关性）
        h_r, w_r = ref.shape
        ref_rs = np.array([[ref[int(ty * h_r / 32), int(tx * w_r / 48)] for tx in range(48)]
                           for ty in range(32)], dtype=np.float32)
        tag = '设备(日志缩略图 48x32)'
    print(f'{tag}  PC(缩放到同尺寸) {ref_rs.shape[1]}x{ref_rs.shape[0]}')

    d, r = norm01(dev), norm01(ref_rs)
    diff = np.abs(d - r)
    rr = float(np.corrcoef(d.ravel(), r.ravel())[0, 1])
    print(f'[2] 皮尔逊相关 r={rr:.4f}（阈值 0.95）→ {"PASS" if rr >= 0.95 else "FAIL"}')
    mad = float(diff.mean()); frac = float((diff <= 0.05).mean())
    ok3 = mad <= 0.08 and frac >= 0.70
    print(f'[3] 归一化平均绝对差={mad:.4f}（≤0.08）  差≤0.05 像素占比={frac*100:.1f}%（≥70%）→ {"PASS" if ok3 else "FAIL"}')
    print(f'   （设备 raw: min={dev.min():.3f} max={dev.max():.3f}  PC raw: min={ref.min():.3f} max={ref.max():.3f}）')

    try:
        from PIL import Image
        vis = np.concatenate([
            (d * 255).astype(np.uint8),
            (r * 255).astype(np.uint8),
            (diff / max(1e-6, diff.max()) * 255).astype(np.uint8),
        ], axis=1)
        Image.fromarray(vis).save(args.dev + '.cmp.png')
        print(f'对比图（左=设备 / 中=PC / 右=差异）→ {args.dev}.cmp.png')
    except Exception as e:  # 出图失败不影响判据结论
        print(f'(出图跳过: {e})')
    sys.exit(0 if (rr >= 0.95 and ok3) else 1)


if __name__ == '__main__':
    main()
