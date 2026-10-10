# -*- coding: utf-8 -*-
"""从真机产物 mp4 里取出内嵌 GLB（Kit 的 3DGS 载荷），可选导出成同布局 PLY 侧车。

为什么需要：真机 PLY 侧车在应用沙箱里，主机 `hdc file recv/ls` 对该目录 permission denied；
而**相册里的成片 mp4 就是我们能拿到的交付载荷本体**（含 GLB 的 JSON + BIN 两个 chunk）。
把 BIN 按 accessor 解出来写成语义名与 Kit PLY 一致的 PLY，即可直接喂既有工具：
  · earth_ply_probe.py     —— 球心拟合 / 覆盖 / 暗点专项
  · earth_splat_render.py  —— 用 GLB 相机把它渲染出来，与成片逐帧对照

用法：
  python earth_glb_extract.py <model.mp4> --out-dir out/_x [--scan-from 0]
      → 写 <name>.glb（逐字节）+ <name>.ply（转换后）；--dump 打印 GLB JSON 结构摘要
"""
import argparse
import json
import os
import struct
import sys

import numpy as np

# Kit PLY 侧车属性顺序（见 earth_ply_probe.py / native sr 写出的 68B stride：
#   x y z f_dc_0 f_dc_1 f_dc_2 opacity scale_0 scale_1 scale_2 rot_0..3  → 3*4 + 3*4 + 4 + 3*4 + 4*4 = 68）
PLY_FLOAT_PROPS = ['x', 'y', 'z', 'f_dc_0', 'f_dc_1', 'f_dc_2', 'opacity',
                   'scale_0', 'scale_1', 'scale_2', 'rot_0', 'rot_1', 'rot_2', 'rot_3']


def find_glb(data):
    """扫 'glTF' 魔数（version=2 + length 自洽）——与 ArkTS resolveGsPayloadOffset 同口径。"""
    magic = b'glTF'
    i = data.find(magic)
    while i >= 0:
        if i + 12 <= len(data):
            ver, ln = struct.unpack_from('<II', data, i + 4)
            if ver == 2 and i + ln <= len(data) + 0:
                return i, ln
        i = data.find(magic, i + 1)
    raise RuntimeError('no glTF magic found')


def parse_glb(data, off, ln):
    magic, ver, length = struct.unpack_from('<III', data, off)
    assert magic == 0x46546C67 and ver == 2, (hex(magic), ver)
    p = off + 12
    chunks = []
    while p < off + length:
        clen, ctype = struct.unpack_from('<II', data, p)
        chunks.append((ctype, p + 8, clen))
        p += 8 + clen
    js = None
    bn = None
    for ctype, cstart, clen in chunks:
        if ctype == 0x4E4F534A:
            js = (cstart, clen)
        elif ctype == 0x004E4942:
            bn = (cstart, clen)
    assert js and bn, 'missing JSON/BIN chunk'
    g = json.loads(data[js[0]:js[0] + js[1]].decode('utf-8'))
    return g, js, bn


CTYPE = {5120: ('b', 1), 5121: ('B', 1), 5122: ('h', 2), 5123: ('H', 2),
         5125: ('I', 4), 5126: ('f', 4)}
NCOMP = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT4': 16}


def read_accessor(g, data, bin_start, bin_len, ai):
    acc = g['accessors'][ai]
    bv = g['bufferViews'][acc['bufferView']]
    fmt, sz = CTYPE[acc['componentType']]
    nc = NCOMP[acc['type']]
    stride = bv.get('byteStride') or (sz * nc)
    base = bin_start + bv.get('byteOffset', 0) + acc.get('byteOffset', 0)
    out = np.zeros((acc['count'], nc), dtype=np.float64)
    for k in range(nc):
        out[:, k] = np.frombuffer(data, dtype=np.dtype('<' + fmt),
                                  count=acc['count'], offset=base + k * sz, buffer=None)
    # 上面这行只对紧密连续 (stride == sz*nc) 有效；有跨距时逐行读
    if stride != sz * nc:
        for r in range(acc['count']):
            o = base + r * stride
            out[r] = np.frombuffer(data, dtype=np.dtype('<' + fmt), count=nc, offset=o)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src', help='真机成片 mp4（内含 GLB 载荷）')
    ap.add_argument('--out-dir', default=None)
    ap.add_argument('--dump', action='store_true')
    a = ap.parse_args()
    data = open(a.src, 'rb').read()
    off, ln = find_glb(data)
    print(f'GLB @ {off} len={ln} (file {len(data)})')
    g, (js_start, js_len), (bin_start, bin_len) = parse_glb(data, off, ln)
    if a.dump:
        print(json.dumps({k: (v if not isinstance(v, list) or len(v) < 12 else
                              f'<{len(v)} items>') for k, v in g.items()},
                         ensure_ascii=False, indent=1)[:4000])
        for i, acc in enumerate(g.get('accessors', [])):
            print(f'  accessor[{i}] type={acc["type"]} n={acc["count"]} ct={acc["componentType"]}'
                  f' bv={acc.get("bufferView")} off={acc.get("byteOffset")}'
                  f' min={acc.get("min")} max={acc.get("max")}')
        for i, m in enumerate(g.get('meshes', [])):
            for pr in m.get('primitives', []):
                print(f'  mesh[{i}] mode={pr.get("mode")} attrs={list(pr.get("attributes", {}).keys())}'
                      f' idx={pr.get("indices")}')
        for i, n in enumerate(g.get('nodes', [])):
            print(f'  node[{i}] name={n.get("name")} mesh={n.get("mesh")} cam={n.get("camera")}'
                  f' trans={n.get("translation")} rot={n.get("rotation")}')
    out_dir = a.out_dir or os.path.join('out', '_glb')
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.splitext(os.path.basename(a.src))[0]
    with open(os.path.join(out_dir, base + '.glb'), 'wb') as f:
        f.write(data[off:off + ln])
    # --- GLB → PLY（语义名对齐 Kit PLY 侧车）---
    mesh = g['meshes'][0]
    pr = mesh['primitives'][0]
    attrs = pr['attributes']
    name_map = {}
    for k in attrs:
        kl = k.upper()
        if kl == 'POSITION':
            name_map[k] = ['x', 'y', 'z']
        elif kl in ('F_DC_0', 'F_DC_1', 'F_DC_2', 'OPACITY', 'SCALE_0', 'SCALE_1',
                    'SCALE_2', 'ROT_0', 'ROT_1', 'ROT_2', 'ROT_3'):
            name_map[k] = [kl.lower()]
        else:
            name_map[k] = None
    grid = {}
    n = None
    for k, names in name_map.items():
        if names is None:
            continue
        v = read_accessor(g, data, bin_start, bin_len, attrs[k])
        n = v.shape[0] if n is None else n
        for j, nm in enumerate(names):
            grid[nm] = v[:, j]
    missing = [p for p in PLY_FLOAT_PROPS if p not in grid]
    print(f'points={n} missing_ply_props={missing} bbox_pos='
          f'{[(float(grid["x"].min()), float(grid["x"].max())), (float(grid["y"].min()), float(grid["y"].max())), (float(grid["z"].min()), float(grid["z"].max()))]}')
    arr = np.zeros((n, len(PLY_FLOAT_PROPS)), dtype='<f4')
    for j, nm in enumerate(PLY_FLOAT_PROPS):
        arr[:, j] = grid[nm] if nm in grid else 0.0
    ply_path = os.path.join(out_dir, base + '.ply')
    with open(ply_path, 'wb') as f:
        head = ('ply\nformat binary_little_endian 1.0\nelement vertex %d\n' % n)
        for nm in PLY_FLOAT_PROPS:
            head += f'property float {nm}\n'
        head += 'end_header\n'
        f.write(head.encode('ascii'))
        f.write(arr.astype('<f4').tobytes())
    print(f'-> {ply_path}')


if __name__ == '__main__':
    main()
