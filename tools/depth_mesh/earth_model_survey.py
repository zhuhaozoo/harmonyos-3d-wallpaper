# -*- coding: utf-8 -*-
"""模型体检（地球预置现役配置）——把成片上的两个缺陷**在模型里**量出来：
  ① 横向拉宽：球壳三个半轴之比（理想 = 1:1:1）
  ② 黑背景穿前：球面**前方**（相机侧、半径比 <1）的近黑点数量/位置/不透明度

输入 = Kit 的 PLY 侧车（68B 属性，见 earth_ply_probe.read_ply），相机方向取自同一轮的
成片 GLB（--glb，取 Initial Camera 的 translation：球在原点、相机在 +Z 附近）。

判据口径：
  · 球壳半轴：对每个坐标轴的 ±25° 圆锥内的点，取半径的 98 分位（稳，且不被少量飞点带跑）
  · 穿前暗点：ρ = |p−c| / R_axis(方向) < 0.98（在球面之内）且 dot(p−c, ĉam) > 0（朝相机侧）
    —— 这两条同时成立的点，渲染时必然叠在地球上（<-用户看到的"黑背景突出到地球前方"）

用法：python earth_model_survey.py <model.ply> [--glb <product.mp4|.glb>] [--dark 20]
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from earth_ply_probe import read_ply, col  # noqa: E402


def glb_camera(path):
    """从 mp4（扫 glTF 魔数）或 .glb 里取 Initial Camera 的 translation/rotation 与 yfov。"""
    d = open(path, 'rb').read()
    off = 0
    if path.lower().endswith(('.mp4', '.mov')):
        off = d.find(b'glTF')
        if off < 0:
            raise RuntimeError('mp4 内未找到 glTF 魔数')
    jl = struct.unpack_from('<I', d, off + 12)[0]
    g = json.loads(d[off + 20:off + 20 + jl].decode('utf-8', 'ignore'))
    node = None
    for n in g.get('nodes', []):
        if n.get('name') == 'Initial Camera':
            node = n
    cam = g.get('cameras', [{}])[0].get('perspective', {})
    return (np.array(node.get('translation', [0, 0, 0]), float) if node else None,
            np.array(node.get('rotation', [0, 0, 0, 1]), float) if node else None,
            cam.get('yfov'), cam.get('aspectRatio'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--glb', default=None)
    ap.add_argument('--dark', type=float, default=20.0)
    ap.add_argument('--cone', type=float, default=25.0, help='轴半轴估计的圆锥半角（度）')
    a = ap.parse_args()

    names, offs, stride, n, buf = read_ply(a.ply)
    P = np.stack([col(buf, offs, k, n, stride) for k in ('x', 'y', 'z')], axis=1)
    dc = [col(buf, offs, f'f_dc_{i}', n, stride) for i in range(3)]
    op = col(buf, offs, 'opacity', n, stride)
    lum = 0.299 * dc[0] + 0.587 * dc[1] + 0.114 * dc[2]
    print(f'{os.path.basename(a.ply)}  n={n} stride={stride}B 属性={names}')

    far = np.linalg.norm(P, axis=1) > 20.0
    print(f'远景点（|p|>20）：{far.sum()} 点；其亮度 50%={np.median(lum[far]):.1f} '
          f'不透明度(logit) 50%={np.median(op[far]):.2f}'
          if far.sum() else '无远景点')
    Pn, ln, on = P[~far], lum[~far], op[~far]
    r = np.linalg.norm(Pn, axis=1)
    print(f'近场点 {Pn.shape[0]}：|p| 分位 5%={np.percentile(r, 5):.3f} 25%={np.percentile(r, 25):.3f} '
          f'50%={np.percentile(r, 50):.3f} 75%={np.percentile(r, 75):.3f} 90%={np.percentile(r, 90):.3f} '
          f'99%={np.percentile(r, 99):.3f} max={r.max():.3f}')

    # ---- ① 球壳半轴（沿三轴圆锥内的 98 分位半径）----
    axis_names = ['X', 'Y', 'Z']
    axes = [np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0])]
    semis = []
    for ax in axes:
        cosang = (Pn @ ax) / np.maximum(r, 1e-9)
        sel = cosang > math.cos(math.radians(a.cone))
        rr = r[sel]
        semis.append(float(np.percentile(rr, 98)) if rr.size > 20 else float('nan'))
        print(f'  圆锥 {axis_names[int(np.argmax(ax))]}±{a.cone:.0f}°：{rr.size} 点  '
              f'半径 50%={np.median(rr):.3f} 98%={np.percentile(rr, 98):.3f}')
    print(f'半轴 X={semis[0]:.4f} Y={semis[1]:.4f} Z={semis[2]:.4f} '
          f'→ X/Y={semis[0] / semis[1]:.4f} X/Z={semis[0] / semis[2]:.4f} Y/Z={semis[1] / semis[2]:.4f}'
          f'（理想 1.0000）')

    # ---- ② 球面内/外的暗点（相机侧）----
    cam = None
    if a.glb:
        t, q, yfov, aspect = glb_camera(a.glb)
        if t is not None:
            cam = t
            print(f'相机（GLB Initial Camera）pos={t.round(4)} |pos|={np.linalg.norm(t):.4f} '
                  f'yfov={yfov} aspect={aspect}')
    dark = ln < a.dark
    print(f'近场暗点（亮度<{a.dark:g}）：{dark.sum()} / {Pn.shape[0]} = '
          f'{dark.sum() / Pn.shape[0] * 100:.2f}%；暗点 |p| 分位 50%={np.median(r[dark]):.3f}')
    if cam is not None:
        cv = cam / np.linalg.norm(cam)
        front = (Pn @ cv) > 0                       # 朝向相机的一侧
        # 半径比：方向上的球面半径用"该方向的半轴椭球"近似 ⇒ 用三半轴做各向异性归一
        ax_semi = np.array(semis)
        rho = np.sqrt(((Pn / ax_semi) ** 2).sum(axis=1))
        inside = rho < 0.98
        sel = dark & front & inside
        print(f'★ 球面之内（ρ<0.98）+ 朝相机侧：{sel.sum()} 点（占近场 {sel.sum() / Pn.shape[0] * 100:.2f}%）'
              f'；不透明度(logit) 50%={np.median(on[sel]):.2f} 90%={np.percentile(on[sel], 90):.2f}'
              if sel.sum() else '★ 球面之内 + 朝相机侧：0 点')
        if sel.sum():
            v = Pn[sel]
            ang = np.degrees(np.arccos(np.clip(v @ cv / np.maximum(np.linalg.norm(v, axis=1), 1e-9), -1, 1)))
            print(f'   与相机轴夹角（度）分位：10%={np.percentile(ang, 10):.1f} '
                  f'50%={np.median(ang):.1f} 90%={np.percentile(ang, 90):.1f}'
                  f'（90°=正好在圆盘边缘/临边）')
            print(f'   ρ 分位：5%={np.percentile(rho[sel], 5):.3f} 50%={np.median(rho[sel]):.3f} '
                  f'95%={np.percentile(rho[sel], 95):.3f}')
        # 盘内（投影落在盘内）的暗点：与相机轴夹角 < asin(R/d) 的等效判据用 80° 粗代
        proj = np.degrees(np.arccos(np.clip(Pn @ cv / np.maximum(np.linalg.norm(Pn, axis=1), 1e-9), -1, 1)))
        s2 = dark & (proj < 80.0) & (rho < 1.0)
        print(f'   保守口径（投影角<80° 且 ρ<1）暗点：{s2.sum()} 点')


if __name__ == '__main__':
    main()
