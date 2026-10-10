# -*- coding: utf-8 -*-
"""离线 3DGS 渲染器（吃 Kit 的 PLY 侧车 + 导出 GLB 的相机）——把"模型到底长什么样"直接画出来。

为什么必须自己渲染：PLY 的**统计量**（半径直方图/球拟合）在"球心偏移 + 碎块 + 前挡黑点"这类
病态云上会互相矛盾（实测：某轮几何算出的 r/d≈0.22，而成片里明明 r/d≈0.55），只有**成像**
才能一锤定音，也才能和成片/壁纸逐帧对照。

实现：INRIA 3DGS 属性的标准前向光栅化（简化版）
  · 位置/不透明度(sigmoid)/尺度(exp)/四元数(PLY 存 wxyz → 转 R)
  · 世界→相机（GLB `Initial Camera` 的 translation+rotation，glTF 四元数 xyzw）
  · 投影：Σ' = J·W·(R S Sᵀ Rᵀ)·Wᵀ·Jᵀ + εI；按深度从近到远做前向 alpha 合成
  · 内参：GLB `cameras[0].perspective`（yfov + aspectRatio）⇒ f_x = (W/2)/tan(xfov/2)，
    tan(xfov/2) = tan(yfov/2)·aspectRatio（与"按宽度贴合"口径一致）

用法：
  python earth_splat_render.py <model.ply> --glb <model.glb> --out out/x.png [--size 1080x1440]
                               [--time 0|1..5（用动画第 n 个关键帧）] [--bg 0]
"""
import argparse
import json
import math
import os
import struct
import sys
import time as _time

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from earth_ply_probe import read_ply, col  # noqa: E402


def quat_wxyz_to_R(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_xyzw_to_R(q):
    x, y, z, w = q
    return quat_wxyz_to_R((w, x, y, z))


def glb_camera(path, key=0):
    d = open(path, 'rb').read()
    jl = struct.unpack_from('<I', d, 12)[0]
    g = json.loads(d[20:20 + jl].decode('utf-8', 'ignore'))
    bin_off = 20 + jl + ((4 - jl % 4) % 4) + 8

    def acc(ai):
        a = g['accessors'][ai]
        bv = g['bufferViews'][a['bufferView']]
        nc = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}[a['type']]
        base = bin_off + bv.get('byteOffset', 0) + a.get('byteOffset', 0)
        return np.array(struct.unpack_from(f'<{a["count"] * nc}f', d, base)).reshape(a['count'], nc)

    node = [x for x in g['nodes'] if x.get('name') == 'Initial Camera'][0]
    trans = np.array(node['translation'], dtype=np.float64)
    rot = np.array(node['rotation'], dtype=np.float64)
    cam = g['cameras'][0]['perspective']
    yfov = cam['yfov']
    aspect = cam.get('aspectRatio', 1.0)
    # 动画：6 个关键帧，取第 key 个
    for an in g.get('animations', []):
        for ch in an['channels']:
            p = ch['target'].get('path')
            if p == 'translation':
                trans = acc(an['samplers'][ch['sampler']]['output'])[key].astype(np.float64)
            elif p == 'rotation':
                rot = acc(an['samplers'][ch['sampler']]['output'])[key].astype(np.float64)
    return trans, rot, yfov, aspect


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--glb', default='', help='导出 GLB（取 Initial Camera / yfov）；给了 --cam-pos 可省')
    ap.add_argument('--cam-pos', default='', help='直接给相机位置 "x,y,z"（日志 glmmeta initialCamera pos=）')
    ap.add_argument('--cam-quat', default='', help='直接给相机姿态 "x,y,z,w"（日志 glmmeta initialCamera quat=）')
    ap.add_argument('--yfov', type=float, default=0.0, help='直接给 yfov（弧度）')
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', default='1080x1440')
    ap.add_argument('--time', type=int, default=0)
    ap.add_argument('--bg', type=float, default=0.0)
    ap.add_argument('--drop-dark', type=float, default=-1.0, help='丢弃亮度<该值的点（诊断用；-1=关）')
    ap.add_argument('--tag-dark', type=float, default=-1.0, help='把亮度<该值的点染成品红（诊断用；-1=关）')
    ap.add_argument('--scale-mul', type=float, default=1.0, help='高斯尺度乘子（调试：填 2 补洞）')
    a = ap.parse_args()
    W, H = (int(x) for x in a.size.lower().split('x'))

    t0 = _time.time()
    names, offs, stride, count, buf = read_ply(a.ply)
    P = np.stack([col(buf, offs, 'x', count, stride), col(buf, offs, 'y', count, stride),
                  col(buf, offs, 'z', count, stride)], axis=1)
    op = col(buf, offs, 'opacity', count, stride)
    scl = np.stack([col(buf, offs, f'scale_{i}', count, stride) for i in range(3)], axis=1)
    rot = np.stack([col(buf, offs, f'rot_{i}', count, stride) for i in range(4)], axis=1)
    dc = np.stack([col(buf, offs, f'f_dc_{i}', count, stride) for i in range(3)], axis=1)
    rgb = np.clip(0.5 + 0.2820948 * dc, 0.0, 1.0)          # SH DC → RGB(0-1)
    alpha = 1.0 / (1.0 + np.exp(-op)) * 0.9                 # sigmoid；0.9 上限（与原实现同量级）
    S = np.exp(scl) * a.scale_mul
    if a.drop_dark >= 0:
        # 诊断：直接**丢弃**近黑点（黑区若本就是"挡在球前的黑点"，丢掉即露出球面）
        keepm = 255.0 * rgb.mean(axis=1) >= a.drop_dark
        print(f'--drop-dark {a.drop_dark}: 丢弃 {int((~keepm).sum())} 个近黑点')
        P = P[keepm]
        rgb = rgb[keepm]
        alpha = alpha[keepm]
        S = S[keepm]
        rot = rot[keepm]
        count = P.shape[0]
    if a.tag_dark >= 0:
        # 诊断：把近黑点染成纯红——渲染后若黑区变红 ⇒ 黑区是**染黑的点**（不是缺几何）
        m = (255.0 * rgb.mean(axis=1)) < a.tag_dark
        rgb[m] = np.array([1.0, 0.0, 0.0])
        print(f'--tag-dark {a.tag_dark}: 染色 {int(m.sum())} 个近黑点（红）')
    trans, rotq, yfov, aspect = glb_camera(a.glb, a.time) if a.glb else (
        np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]), 1.5299857, 0.75)
    if a.cam_pos:
        trans = np.array([float(x) for x in a.cam_pos.split(',')], dtype=np.float64)
    if a.cam_quat:
        rotq = np.array([float(x) for x in a.cam_quat.split(',')], dtype=np.float64)
    if a.yfov > 0:
        yfov = a.yfov
    R_wc = quat_xyzw_to_R(rotq)                             # 相机→世界
    R_cw = R_wc.T                                           # 世界→相机
    cam_pos = trans
    print(f'PLY n={count} 相机 pos={np.round(cam_pos,4)} yfov={math.degrees(yfov):.2f}° '
          f'aspect={aspect} 画幅 {W}x{H}')

    Pc = (P - cam_pos) @ R_cw.T                             # 相机系（z 向前 = +z? glTF 相机看向 -z）
    # glTF 相机局部 -Z 为视线；R_wc 的第三列 = 相机 +Z 在世界系 → 相机系 z 分量需取负
    zc = -Pc[:, 2]
    # 内参口径 = **设备实测**（成片与锁屏壁纸都吻合）：fy = (H/2)/tan(yfov/2)，**方像素** fx=fy
    # ⇒ 画幅宽高比 ≠ 相机 aspectRatio 时按高度贴合、两侧裁切（实测：非洲成片 48.0% ≈ 预测 48.6%；
    #   锁屏壁纸地球被屏幕横向裁切）。
    fy = (H / 2) / math.tan(yfov / 2)
    fx = fy
    cx, cy = W / 2, H / 2
    front = zc > 1e-4
    u = cx + fx * Pc[:, 0] / np.maximum(zc, 1e-9)
    v = cy - fy * Pc[:, 1] / np.maximum(zc, 1e-9)            # 图像 y 向下
    # 2D 协方差
    Rg = np.stack([quat_wxyz_to_R(q) for q in rot])         # (n,3,3)
    M = Rg * S[:, None, :]                                  # 列缩放
    Sig = M @ np.transpose(M, (0, 2, 1))
    J = np.zeros((count, 2, 3))
    J[:, 0, 0] = fx / np.maximum(zc, 1e-9)
    J[:, 0, 2] = -fx * Pc[:, 0] / np.maximum(zc ** 2, 1e-9)
    J[:, 1, 1] = -fy / np.maximum(zc, 1e-9)
    J[:, 1, 2] = fy * Pc[:, 1] / np.maximum(zc ** 2, 1e-9)
    Sig2 = J @ Sig @ np.transpose(J, (0, 2, 1))
    Sig2[:, 0, 0] += 0.3
    Sig2[:, 1, 1] += 0.3
    det = Sig2[:, 0, 0] * Sig2[:, 1, 1] - Sig2[:, 0, 1] ** 2
    ok = front & (det > 1e-9) & (u > -50) & (u < W + 50) & (v > -50) & (v < H + 50)
    idx = np.flatnonzero(ok)
    order = idx[np.argsort(zc[idx])]                        # 近→远
    print(f'可见 {len(idx)} / {count} 高斯；开始合成…')

    img = np.full((H, W, 3), a.bg, dtype=np.float32)
    T = np.ones((H, W), dtype=np.float32)
    inv = np.zeros((len(order), 2, 2))
    for k, i in enumerate(order):
        inv[k] = np.linalg.inv(Sig2[i])
    for k, i in enumerate(order):
        s2 = Sig2[i]
        r = int(np.ceil(3 * math.sqrt(max(s2[0, 0], s2[1, 1]))))
        x0, x1 = max(0, int(u[i]) - r), min(W, int(u[i]) + r + 1)
        y0, y1 = max(0, int(v[i]) - r), min(H, int(v[i]) + r + 1)
        if x1 <= x0 or y1 <= y0:
            continue
        xx = np.arange(x0, x1) - u[i]
        yy = np.arange(y0, y1) - v[i]
        gx, gy = np.meshgrid(xx, yy)
        iv = inv[k]
        e = iv[0, 0] * gx * gx + 2 * iv[0, 1] * gx * gy + iv[1, 1] * gy * gy
        w = np.exp(-0.5 * e)
        al = np.clip(alpha[i] * w, 0, 0.99)
        Tt = T[y0:y1, x0:x1]
        contrib = (al * Tt)[:, :, None] * rgb[i][None, None, :]
        img[y0:y1, x0:x1] += contrib
        T[y0:y1, x0:x1] = Tt * (1 - al)
    out = np.clip(img * 255, 0, 255).astype(np.uint8)
    Image.fromarray(out).save(a.out)
    print(f'{a.out}  用时 {_time.time() - t0:.1f}s   未覆盖(黑)像素占比 {(np.clip(img.sum(axis=2),0,0.05)<0.02).mean()*100:.1f}%')


if __name__ == '__main__':
    main()
