# -*- coding: utf-8 -*-
"""Kit 写的 PLY 侧车 → 模型几何/颜色诊断（离线·球心对齐版）

为什么不是"以原点为球心"的简化版：Kit 的输出坐标系相对**声明坐标系**多一个刚体变换
（旋转+平移+尺度）——实测地球模型的 |p| 直方图峰在 0.325 而不是 1.0，就是球心不在原点的
直接证据。"黑区是缺几何还是黑点挡前"必须在**球心系**里问，否则覆盖/亮度图整张都是错的。

本脚本流程：
  ① 解析 PLY（binary_little_endian；属性偏移按类型推，x/y/z/f_dc_0..2/opacity/scale/rot）
  ② **RANSAC 球拟合**（4 点定球 + 内点计数 + Kasa 精修）→ 球心 c*、球半径 R*
  ③ 可选 `--glb`：读导出 GLB 的 `Initial Camera` 与 CameraPath 关键帧，与**声明对角弧**
     （manifest：radius / orbitArcDeg / 帧数）做 Horn 相似变换拟合 → 尺度 s、旋转 R、平移 t
     ⇒ 得到"球在声明坐标系里的半径 = R*/s"（应 ≈1.0）与 **camR/R_ball***（应 ≈2.35）
  ④ 球心系经纬图（24×12）：每格给 点数 / **最小半径比 r/R\***（<1 = 有东西挡在球前）/
     平均亮度 ⇒ 三张图并排看，直接判"缺几何 vs 黑点挡前"
  ⑤ 黑点专项：亮度<阈值的点占多少、它们的半径比分布（贴面 or 前置）、集中在哪个经纬格

用法：
  python earth_ply_probe.py <model.ply> [--glb <导出模型.glb>] [--grid 24x12]
                            [--dark 20] [--prefix out/x] [--declared-radius 2.35]
"""
import argparse
import json
import math
import os
import struct
import sys

import numpy as np

PROP_TYPES = {
    'float': ('f', 4), 'float32': ('f', 4), 'double': ('d', 8),
    'uchar': ('B', 1), 'uint8': ('B', 1), 'char': ('b', 1), 'int8': ('b', 1),
    'short': ('h', 2), 'int16': ('h', 2), 'ushort': ('H', 2), 'uint16': ('H', 2),
    'int': ('i', 4), 'int32': ('i', 4), 'uint': ('I', 4), 'uint32': ('I', 4),
}


# ---------------- PLY ----------------
def read_ply(path):
    with open(path, 'rb') as f:
        head = b''
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError('PLY 头未结束')
            head += line
            if line.strip() == b'end_header':
                break
        blob = f.read()
    names, types = [], []
    count = 0
    in_vertex = False
    for ln in head.decode('ascii', errors='replace').split('\n'):
        s = ln.strip()
        if s.startswith('format') and 'binary_little_endian' not in s:
            raise RuntimeError(f'仅支持 binary_little_endian：{s}')
        if s.startswith('element '):
            in_vertex = s.startswith('element vertex ')
            if in_vertex:
                count = int(s.split()[2])
        elif in_vertex and s.startswith('property '):
            p = s.split()
            names.append(p[2])
            types.append(p[1])
    stride = sum(PROP_TYPES[t][1] for t in types)
    if len(blob) < count * stride:
        raise RuntimeError(f'数据不足 {len(blob)} < {count * stride}')
    offs, o = {}, 0
    for n, t in zip(names, types):
        offs[n] = (o, PROP_TYPES[t][0])
        o += PROP_TYPES[t][1]
    buf = np.frombuffer(blob, dtype=np.uint8, count=count * stride)
    return names, offs, stride, count, buf


def col(buf, offs, name, n, stride, dtype=None):
    """取一列属性值。

    ⚠️ 必须按**记录跨距**切片：直接 `frombuffer(buf, dtype, count=n, offset=o)` 会把
    `buf[o], buf[o+4], buf[o+8]…` 当成同一列（跨距 4B），而真记录跨距是 stride（实测 68B）
    —— 只有第 0 条读对，其余全错（本轮踩过：合成模型自检时才发现）。
    """
    if name not in offs:
        return None
    o, code = offs[name]
    dt = np.dtype(dtype or code).newbyteorder('<')
    it = dt.itemsize
    if stride % it != 0 or o % it != 0:
        raise RuntimeError(f'属性 {name} 偏移/跨距与 {dt} 不对齐（o={o} stride={stride}）')
    arr = np.frombuffer(buf, dtype=dt)
    return arr[o // it::stride // it][:n].astype(np.float64)


# ---------------- 球拟合 ----------------
def sphere_from4(p):
    """4 点定球（不共面时唯一）。返回 (c, r) 或 None。"""
    A = 2 * (p[1:] - p[0])
    b = np.sum(p[1:] ** 2, axis=1) - np.sum(p[0] ** 2)
    try:
        c = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return None
    r = float(np.linalg.norm(p[0] - c))
    if not np.isfinite(r) or r <= 1e-9:
        return None
    return c, r


def kasa(P):
    A = np.hstack([2 * P, np.ones((P.shape[0], 1))])
    b = np.sum(P * P, axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:3]
    r = float(np.sqrt(max(sol[3] + c @ c, 0.0)))
    return c, r


def ransac_sphere(P, scale0, rel_eps=0.02, iters=8000, seed=20261010):
    """4 点定球 + **相对**内点容差（|d| < rel_eps·r）。

    为什么必须相对容差：点的整体尺度由远景板主导（|p| 到 9+），若用绝对容差，
    一个 R≈6 的"巨球"能把全部点判成内点（实测踩过）——必须是"距球面 r 的 2% 以内"。
    另外拒绝 r > 0.5·scale0 的候选（球不可能比整个场景还大）。
    """
    rng = np.random.default_rng(seed)
    n = P.shape[0]
    best = (0, None, None)
    for _ in range(iters):
        idx = rng.choice(n, 4, replace=False)
        got = sphere_from4(P[idx])
        if got is None:
            continue
        c, r = got
        if r > 0.5 * scale0 or r < 1e-6:
            continue
        d = np.abs(np.linalg.norm(P - c, axis=1) - r)
        k = int((d < rel_eps * r).sum())
        if k > best[0]:
            best = (k, c, r)
    if best[1] is None:
        return None, None, 0.0
    c, r = best[1], best[2]
    d = np.abs(np.linalg.norm(P - c, axis=1) - r)
    inl = d < rel_eps * r * 1.5
    if inl.sum() > 32:
        c, r = kasa(P[inl])
        d = np.abs(np.linalg.norm(P - c, axis=1) - r)
        inl = d < rel_eps * r * 1.5
    return c, float(r), float(inl.mean())


# ---------------- 声明路径 / GLB 动画 ----------------
def declared_path(radius, arc_deg, n):
    out = []
    for i in range(n):
        th = math.radians(-arc_deg / 2 + arc_deg * i / (n - 1))
        out.append([radius * math.sin(th) * math.cos(th), radius * math.sin(th),
                    radius * math.cos(th) * math.cos(th)])
    return np.asarray(out, dtype=np.float64)


def glb_anim(path):
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

    node = [x for x in g.get('nodes', []) if x.get('name') == 'Initial Camera']
    trans = None
    for an in g.get('animations', []):
        for ch in an['channels']:
            if ch['target'].get('path') == 'translation':
                trans = acc(an['samplers'][ch['sampler']]['output'])
    cam = g.get('cameras', [{}])[0].get('perspective', {})
    return dict(trans=trans, node_trans=node[0].get('translation') if node else None,
                yfov=cam.get('yfov'), aspect=cam.get('aspectRatio'),
                count=g.get('accessors', [{}])[0].get('count'))


def similarity_fit(A, B):
    """求 s,R,t 使 s·R·A + t ≈ B（Horn/Umeyama，行=点）。返回 s,R,t 与残差。"""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    A0, B0 = A - ca, B - cb
    H = A0.T @ B0 / A.shape[0]
    U, S, Vt = np.linalg.svd(H)
    D = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[2, 2] = -1
    R = Vt.T @ D @ U.T                       # B ≈ R·A
    s = float((S * np.diag(D)).sum() / (A0 ** 2).sum() * A.shape[0])
    t = cb - s * (R @ ca)
    res = np.linalg.norm((s * (R @ A.T)).T + t - B, axis=1)
    return s, R, t, res


def maps(cnt, vals, k_lon, k_lat, fmt):
    """每格 1 字符；cnt==0 的格印 '.'。"""
    rows = []
    for r in range(k_lat):
        row = ''
        for c in range(k_lon):
            i = r * k_lon + c
            row += '.' if cnt[i] == 0 else fmt(vals[i])
        rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('ply')
    ap.add_argument('--glb', default='')
    ap.add_argument('--grid', default='24x12')
    ap.add_argument('--dark', type=float, default=20.0)
    ap.add_argument('--prefix', default='')
    ap.add_argument('--declared-radius', type=float, default=2.35)
    ap.add_argument('--max-points', type=int, default=300000)
    a = ap.parse_args()
    k_lon, k_lat = (int(x) for x in a.grid.lower().split('x'))

    names, offs, stride, count, buf = read_ply(a.ply)
    print(f'PLY {os.path.basename(a.ply)}  n={count} stride={stride}B props={len(names)}'
          f'  （属性：{",".join(names[:8])}…）')
    P3 = np.stack([col(buf, offs, 'x', count, stride), col(buf, offs, 'y', count, stride),
                   col(buf, offs, 'z', count, stride)], axis=1)
    op = col(buf, offs, 'opacity', count, stride)
    dc = [col(buf, offs, f'f_dc_{i}', count, stride) for i in range(3)]
    have_dc = all(d is not None for d in dc)
    lum_all = None
    if have_dc:
        lum_all = np.clip(255 * (0.5 + 0.2820948 * (dc[0] + dc[1] + dc[2]) / 3), 0, 255)
    # 抽样（拟合与统计都在抽样上做，量级 20 万点足够）
    if count > a.max_points:
        step = int(math.ceil(count / a.max_points))
        P, lum = P3[::step], (lum_all[::step] if lum_all is not None else None)
    else:
        P, lum = P3, lum_all
    print(f'抽样 {P.shape[0]} 点   |p| 分位: '
          + '  '.join(f'{q}%={np.percentile(np.linalg.norm(P, axis=1), q):.3f}'
                      for q in (50, 75, 90, 99)))

    # ② RANSAC 球拟合
    # ⚠️ 先剔"远景墙"：Kit 的模型尾部有 10000 个 |p|≈200 的黑点（canonical 远平面填充；两套
    # 素材实测都恰好 10000 个），它们会把整体尺度与 RANSAC 容差带跑偏（实测：scale0 取到 200，
    # 于是 4 点定球只能找出 R≈4 的巨球）。
    Rn = np.linalg.norm(P, axis=1)
    lim = 20.0   # 远景墙在 |p|≈200；近场（球+板+碎块）实测都在 20 以内
    near = Rn < lim
    if near.sum() < P.shape[0] * 0.5:
        near = np.ones(P.shape[0], dtype=bool)
    print(f'（远景剔除：|p|≥{lim:.1f} 的 {int((~near).sum())} 点不参与拟合）')
    Pn = P[near]
    scale0 = float(np.percentile(np.linalg.norm(Pn, axis=1), 95))
    c, r, frac = ransac_sphere(Pn, scale0)
    if c is None:
        print('!! 球拟合失败（点云里没有可辨识的球面）')
        return 2
    d = np.abs(np.linalg.norm(P - c, axis=1) - r)
    frac = float((d[near] < 0.02 * r).mean())
    print(f'\nRANSAC 球：中心=({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f})  R*={r:.4f}  '
          f'近场子集内点占比={frac * 100:.1f}%（|d| 中位={np.median(d[near]):.4f}，'
          f'P90={np.percentile(d[near], 90):.4f}）')

    # ③ 与声明路径对账（有 GLB 才有）
    gauge = None
    if a.glb:
        g = glb_anim(a.glb)
        trans = g['trans']
        print(f'GLB：高斯数={g["count"]} yfov={math.degrees(g["yfov"]):.2f}° '
              f'aspectRatio={g["aspect"]} 动画关键帧={0 if trans is None else trans.shape[0]}')
        if trans is not None and trans.shape[0] >= 3:
            decl = declared_path(a.declared_radius, 10.0, trans.shape[0])
            s, R, t, res = similarity_fit(decl, trans)
            r_decl = r / s
            camc = np.linalg.norm((trans[0] - t) / s)           # 相机到球心（声明单位）
            print(f'  相似变换拟合：尺度 s={s:.4f}（1 声明单位 = {s:.4f} 模型单位）'
                  f'  残差均={res.mean():.4f}')
            print(f'  ⇒ 球半径（换算到声明单位）= R*/s = {r_decl:.4f}   （声明应为 1.0）')
            print(f'  ⇒ 相机到球心 = {camc:.4f} 声明单位 ⇒ **camR/R球 = {camc / r_decl:.3f}**'
                  f'   （声明 {a.declared_radius}）')
            gauge = dict(s=s, R=R, t=t, r_decl=r_decl, camR=camc,
                         camR_over_R=camc / r_decl)
        else:
            print('  ⚠️ 动画关键帧不足，跳过相似变换')

    # ④ 球心系经纬图：点数 / 最小半径比 / 平均亮度
    rel = P - c
    rad = np.linalg.norm(rel, axis=1)
    rr = rad / r
    lon = np.arctan2(rel[:, 0], rel[:, 2])
    lat = np.arcsin(np.clip(rel[:, 1] / np.maximum(rad, 1e-12), -1, 1))
    cl = np.clip(((lon / (2 * math.pi) + 0.5) * k_lon).astype(int), 0, k_lon - 1)
    cr = np.clip(((0.5 - lat / math.pi) * k_lat).astype(int), 0, k_lat - 1)
    ci = cr * k_lon + cl
    ncell = k_lon * k_lat
    cnt = np.bincount(ci, minlength=ncell)
    rmin = np.full(ncell, np.inf)
    np.minimum.at(rmin, ci, rr)
    rmean = np.bincount(ci, weights=rr, minlength=ncell) / np.maximum(cnt, 1)
    if lum is not None:
        lmean = np.bincount(ci, weights=lum, minlength=ncell) / np.maximum(cnt, 1)
    else:
        lmean = np.zeros(ncell)
    empty = int((cnt == 0).sum())
    print(f'\n球心系经纬图 {k_lon}x{k_lat}（lon −180→180，lat +90→−90）')
    print('  ① 覆盖（点数对数档 1-9；.=0，该方向球面缺几何）：')
    for row in maps(cnt, cnt, k_lon, k_lat,
                    lambda v: str(min(9, 1 + int(math.floor(math.log(max(v, 1), 4)))))):
        print(f'    |{row}|')
    print('  ② 最小半径比 r/R* 分档（0 = ≤0.80R（有东西明显挡在球前），9 = ≥0.98R（贴面））：')
    for row in maps(cnt, rmin, k_lon, k_lat,
                    lambda v: str(min(9, max(0, int(round((v - 0.8) * 50)))))):
        print(f'    |{row}|')
    if lum is not None:
        print('  ③ 平均亮度（SH-DC/26 → 0-9；0-1 = 近黑）：')
        for row in maps(cnt, lmean / 26.0, k_lon, k_lat,
                        lambda v: str(min(9, max(0, int(round(v)))))):
            print(f'    |{row}|')

    # ⑤ 半径带汇总（诊断主结论）：前挡 / 球面 / 球后 / 远景
    print('')
    bands = [('<0.95R（挡在球前）', 0.0, 0.95), ('0.95~1.05R（球面）', 0.95, 1.05),
             ('1.05~1.5R（球后近）', 1.05, 1.5), ('>1.5R（远景板等）', 1.5, 1e9)]
    for nm, lo, hi in bands:
        m = (rr >= lo) & (rr < hi)
        if lum is not None:
            dk = int((m & (lum < a.dark)).sum())
            print(f'  {nm:22s} {int(m.sum()):7d} 点（{m.mean() * 100:5.2f}%），其中暗点 {dk}'
                  f'（{dk / max(int(m.sum()), 1) * 100:5.1f}%）')
        else:
            print(f'  {nm:22s} {int(m.sum()):7d} 点（{m.mean() * 100:5.2f}%）')
    if lum is not None:
        dark = lum < a.dark
        near = rr < 1.3
        print(f'\n暗点（亮度<{a.dark:.0f}/255）共 {int(dark.sum())}（{dark.mean() * 100:.2f}%）；'
              f'其中"球附近（r/R*<1.3）"的 {int((dark & near).sum())}')
        if (dark & near).sum() > 0:
            dcnt = np.bincount(ci[dark & near], minlength=ncell)
            nz = np.flatnonzero(dcnt)
            order = nz[np.argsort(dcnt[nz])[::-1]]
            for i in order[:5]:
                lat0 = 90 - (i // k_lon) * (180 / k_lat)
                lon0 = -180 + (i % k_lon) * (360 / k_lon)
                print(f'    球附近暗点最多格：lon≈{lon0 + 180 / k_lon:.0f}° lat≈{lat0 - 90 / k_lat:.0f}°'
                      f'  暗点 {dcnt[i]}/{cnt[i]}')
    if op is not None:
        o = op[::max(1, count // 100000)]
        print(f'不透明度（logit）分位：50%={np.percentile(o, 50):+.2f}  '
              f'90%={np.percentile(o, 90):+.2f}')

    print(f'\n汇总：空档 {empty}/{ncell} 格（>0 = 该经纬方向球面缺几何）')
    if a.prefix:
        with open(a.prefix + '_rmin.txt', 'w', encoding='utf-8') as f:
            f.write('\n'.join(maps(cnt, np.minimum(rmin, 9.99) * 100, k_lon, k_lat,
                                   lambda v: format(int(v) if np.isfinite(v) else 999, '3d')[1:])))
        print(f'已写出 {a.prefix}_rmin.txt')
    return 0


if __name__ == '__main__':
    sys.exit(main())
