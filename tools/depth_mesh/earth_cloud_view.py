# -*- coding: utf-8 -*-
"""点云三视图（XY/XZ/YZ）——把模型直接画出来，颜色 = SH-DC 亮度（黑点/亮盘一眼可辨）。

为什么自己画：PLY 的统计量在"球心偏移 + 碎块 + 前挡黑点"这类病态云上会互相矛盾；
一张三视图能同时回答"球在哪、壳在哪、暗点在哪、相机在哪"。

用法：
  python earth_cloud_view.py <model.ply> [--out out/_x/cloud.png] [--span 3.0]
                             [--size 900] [--cam-in-glb <product.mp4>]
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from earth_ply_probe import read_ply, col  # noqa: E402


def glb_cam(path):
    d = open(path, 'rb').read()
    off = d.find(b'glTF') if path.lower().endswith(('.mp4', '.mov')) else 0
    jl = struct.unpack_from('<I', d, off + 12)[0]
    g = json.loads(d[off + 20:off + 20 + jl].decode('utf-8', 'ignore'))
    for n in g.get('nodes', []):
        if n.get('name') == 'Initial Camera':
            return np.array(n.get('translation', [0, 0, 0]), float)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--out', default=None)
    ap.add_argument('--span', type=float, default=3.0, help='单视图覆盖的半宽（模型单位）')
    ap.add_argument('--size', type=int, default=900)
    ap.add_argument('--cam', default=None, help='相机位置 x,y,z（或 --cam-in-glb）')
    ap.add_argument('--cam-in-glb', default=None)
    ap.add_argument('--dark', type=float, default=20.0)
    a = ap.parse_args()

    names, offs, stride, n, buf = read_ply(a.ply)
    P = np.stack([col(buf, offs, k, n, stride) for k in ('x', 'y', 'z')], 1)
    dc = [col(buf, offs, f'f_dc_{i}', n, stride) for i in range(3)]
    lum = np.clip(255.0 * (0.5 + 0.2820948 * (dc[0] + dc[1] + dc[2]) / 3.0), 0, 255)
    cam = None
    if a.cam_in_glb:
        cam = glb_cam(a.cam_in_glb)
    elif a.cam:
        cam = np.array([float(x) for x in a.cam.split(',')])
    near = np.linalg.norm(P, axis=1) < 20.0
    print(f'{os.path.basename(a.ply)}  n={n} 近场={near.sum()} 亮度分位 '
          f'5%={np.percentile(lum[near], 5):.0f} 50%={np.percentile(lum[near], 50):.0f} '
          f'95%={np.percentile(lum[near], 95):.0f} 暗点={int((lum[near] < a.dark).sum())}'
          + (f'  相机={cam.round(3)}' if cam is not None else ''))

    S, sp = a.size, a.span
    views = [('XY', 0, 1), ('XZ', 0, 2), ('YZ', 1, 2)]
    sheet = Image.new('RGB', (S * 3 + 20, S + 24), (255, 255, 255))
    from PIL import ImageDraw
    for k, (nm, i, j) in enumerate(views):
        im = Image.new('RGB', (S, S), (250, 250, 250))
        dr = ImageDraw.Draw(im)
        # 网格（每 0.5 单位一条淡线），原点十字
        for t in np.arange(-sp, sp + 1e-9, 0.5):
            p = (t + sp) / (2 * sp) * (S - 1)
            colr = (215, 215, 215) if abs(t - round(t)) > 1e-6 else (190, 190, 190)
            dr.line([(p, 0), (p, S - 1)], fill=colr)
            dr.line([(0, p), (S - 1, p)], fill=colr)
        px = np.round((P[near, i] + sp) / (2 * sp) * (S - 1)).astype(int)
        py = np.round((sp - P[near, j]) / (2 * sp) * (S - 1)).astype(int)   # y 轴向上
        ok = (px >= 0) & (px < S) & (py >= 0) & (py < S)
        px, py = px[ok], py[ok]
        lv = lum[near][ok]
        rgb = np.stack([lv, lv, lv], 1).astype(np.uint8)
        # 暗点标红（便于判"黑点挡前"）
        red = lv < a.dark
        rgb[red] = (255, 0, 0)
        arr = np.asarray(im).copy()
        # 叠加：后画的覆盖先画的；用 numpy 直接赋值
        arr[py, px] = rgb
        im = Image.fromarray(arr)
        dr = ImageDraw.Draw(im)
        if cam is not None:
            cxp = (cam[i] + sp) / (2 * sp) * (S - 1)
            cyp = (sp - cam[j]) / (2 * sp) * (S - 1)
            dr.ellipse([cxp - 6, cyp - 6, cxp + 6, cyp + 6], outline=(0, 128, 255), width=3)
            dr.text((cxp + 8, cyp - 8), 'CAM', fill=(0, 128, 255))
        dr.text((8, 6), f'{nm}  span±{sp}  红=亮度<{a.dark:g}', fill=(60, 60, 60))
        sheet.paste(im, (k * (S + 10), 0))
    out = a.out or os.path.splitext(a.ply)[0] + '_cloud.png'
    sheet.save(out)
    print('->', out)


if __name__ == '__main__':
    main()
