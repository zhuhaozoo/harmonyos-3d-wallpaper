# -*- coding: utf-8 -*-
"""量「成片里的地球盘」——横向拉宽（盘纵横比）与黑区穿前（盘内近黑像素）。

为什么要有它：本轮真机成片的两个已知缺陷是**观感量**（"横向有点拉宽"、"黑背景突出到地球
前方"），日志里的标量（refined aspect / plydiag axisStd）是**间接量**，两者对不上号时无法定案。
本工具直接从成片像素量两个数：

  ① 盘纵横比 rx/ry —— 物理场景是球 + 各向同性相机，**理想值恒 = 1.000**；>1 即横向拉宽。
     口径：亮度阈值 → 最大连通域（甩掉星星）→ 填洞 → 逐行/逐列弦长最小二乘拟合椭圆
     （用弦长而非 bbox，避免盘缘毛刺被当成半径）。
  ② 盘内近黑占比与位置 —— 判"黑背景穿到地球前方"：盘内（已向内缩边）近黑像素的占比、
     质心相对盘心的偏移（以 rx/ry 为单位）、以及距盘心最远的一个黑像素（黑区伸到多外圈）。

数值口径与 tools/depth_mesh/earth_splat_render.py 无关（那是渲染器）；本工具只量**已有像素**，
可用于：真机成片 mp4、离线渲染 png、以及素材帧（数据集目录 f00xx.jpg）。

用法：
  python earth_out_measure.py <mp4|png|jpg|dir> [...] [--n 8] [--thr 22] [--dark 12]
  # 默认每个输入抽 n 帧（首/中/末含）；--ash 打印 ASCII 轮廓图便于肉眼对位
  # 视频输入需要 ffmpeg：优先用 PATH 里的 ffmpeg，其次环境变量 FFMPEG 指定，再次 imageio_ffmpeg 包
"""
import argparse
import glob
import os
import shutil
import subprocess
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))


def find_ffmpeg():
    """视频抽帧用 ffmpeg：PATH → 环境变量 FFMPEG → imageio_ffmpeg 附带（可选依赖）。"""
    p = shutil.which('ffmpeg')
    if p:
        return p
    p = os.environ.get('FFMPEG', '')
    if p and os.path.exists(p):
        return p
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except Exception:
        return 'ffmpeg'


FFMPEG = find_ffmpeg()


def read_any(path, n):
    """返回 [(标签, HxW float 灰度, HxWx3 uint8 RGB)]：图片直接读，视频抽 n 帧。"""
    ext = os.path.splitext(path)[1].lower()
    if ext in ('.png', '.jpg', '.jpeg', '.bmp'):
        im = Image.open(path).convert('RGB')
        a = np.asarray(im)
        return [(os.path.basename(path), a)]
    if ext in ('.mp4', '.mov', '.mkv'):
        return read_video(path, n)
    raise ValueError('unsupported input: ' + path)


def read_video(path, n):
    import json
    import tempfile
    # 用 ffprobe 同源（ffmpeg -i）取总帧数：解析 stderr 的 Duration/fps 不靠谱，直接抽全帧再选
    tmpd = tempfile.mkdtemp(prefix='earth_out_measure_')
    cmd = [FFMPEG, '-hide_banner', '-loglevel', 'error', '-i', path,
           '-vsync', '0', '-f', 'image2', os.path.join(tmpd, 'f_%05d.png')]
    subprocess.run(cmd, check=True)
    files = sorted(glob.glob(os.path.join(tmpd, 'f_*.png')))
    if not files:
        raise RuntimeError('no frames extracted: ' + path)
    tot = len(files)
    idxs = sorted(set(int(round(i * (tot - 1) / max(n - 1, 1))) for i in range(n)))
    out = []
    for i in idxs:
        a = np.asarray(Image.open(files[i]).convert('RGB'))
        out.append((f'{os.path.basename(path)}#f{i}/{tot - 1}', a))
    return out


def largest_component(mask):
    from scipy import ndimage  # noqa: F401  (可选依赖：无 scipy 时用手写 BFS)
    lbl, num = ndimage.label(mask)
    if num <= 1:
        return mask
    sizes = ndimage.sum(mask, lbl, index=np.arange(1, num + 1))
    return lbl == (int(np.argmax(sizes)) + 1)


def flood_fill_holes(mask):
    """把最大连通域的洞填上（地球盘内部的黑区/暗纹不该算成背景）。"""
    from scipy import ndimage
    return ndimage.binary_fill_holes(mask)


def fit_disc(mask):
    """逐行/逐列弦长最小二乘拟合圆盘椭圆参数（cx, cy, rx, ry）。

    弦长公式：行 y 上 |x−cx| 的最大值满足 ((x−cx)/rx)^2 + ((y−cy)/ry)^2 = 1
    即 chu(y) = rx * sqrt(1 − ((y−cy)/ry)^2)。对 (chu, y) 做非线性最小二乘过重（本工具宁可简单）：
    改为对 4 个极值组合直接解——左右极值给 cx/rx，上下极值给 cy/ry，再用"弦长回归"精修。
    """
    ys, xs = np.nonzero(mask)
    y0, y1 = ys.min(), ys.max()
    x0, x1 = xs.min(), xs.max()
    cy = (y0 + y1) / 2.0
    cx = (x0 + x1) / 2.0
    ry = (y1 - y0) / 2.0
    rx = (x1 - x0) / 2.0
    # 用弦长做一次一致的再估计（对毛刺/缺口更稳）：r_est = chu / sqrt(1 − t^2)，取中位数
    for _ in range(3):
        rows = []
        for y in range(y0, y1 + 1):
            r = mask[y]
            nz = np.nonzero(r)[0]
            if nz.size == 0:
                continue
            t = (y - cy) / ry
            if abs(t) > 0.985:
                continue
            rows.append((nz[-1] - nz[0] + 1) / 2.0 / np.sqrt(1.0 - t * t))
        if rows:
            rx = float(np.median(rows))
        cols = []
        for x in range(x0, x1 + 1):
            c = mask[:, x]
            nz = np.nonzero(c)[0]
            if nz.size == 0:
                continue
            t = (x - cx) / rx
            if abs(t) > 0.985:
                continue
            cols.append((nz[-1] - nz[0] + 1) / 2.0 / np.sqrt(1.0 - t * t))
        if cols:
            ry = float(np.median(cols))
    return cx, cy, rx, ry


def ascii_view(gray, inner, step=48, thr=12):
    """粗 ASCII：'#'=盘内有内容，'.'=盘内近黑，' '=盘外。"""
    h, w = gray.shape
    lines = []
    for y in range(0, h, step):
        row = []
        for x in range(0, w, step):
            blk = gray[y:y + step, x:x + step]
            m = inner[y:y + step, x:x + step]
            if m.mean() < 0.5:
                row.append(' ')
            elif (blk < thr).mean() > 0.5:
                row.append('.')
            else:
                row.append('#')
        lines.append(''.join(row))
    return lines


def measure(path, n, thr, dark, want_ascii, shrink):
    print(f'\n===== {path}')
    for tag, rgb in read_any(path, n):
        h, w = rgb.shape[0], rgb.shape[1]
        gray = (0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]).astype(np.float64)
        mask = gray > thr
        mask = largest_component(mask)
        filled = flood_fill_holes(mask)
        cx, cy, rx, ry = fit_disc(filled)
        # 盘内近黑（向内缩边 shrink 像素，避免盘缘 3DGS 羽化/背景半影被当成"黑区"）
        inner = np.zeros_like(filled)
        yy, xx = np.mgrid[0:h, 0:w]
        rr = np.sqrt(((xx - cx) / rx) ** 2 + ((yy - cy) / ry) ** 2)
        inner = rr < (1.0 - shrink / min(rx, ry))
        din = inner & (gray < dark)
        frac = din.sum() / max(inner.sum(), 1)
        if din.sum() > 0:
            dy, dx = np.nonzero(din)
            dcx, dcy = dx.mean(), dy.mean()
            rmax = np.sqrt(((dx - cx) / rx) ** 2 + ((dy - cy) / ry) ** 2).max()
        else:
            dcx = dcy = rmax = float('nan')
        print(f'{tag}: {w}x{h} 盘 cx={cx:.1f} cy={cy:.1f} rx={rx:.2f} ry={ry:.2f} '
              f'**纵横比 rx/ry={rx / ry:.4f}**  盘内近黑 {frac * 100:.3f}%'
              + (f' 质心偏移=({(dcx - cx) / rx:+.3f},{(dcy - cy) / ry:+.3f})·r 最外=({rmax:.3f}·r)'
                 if din.sum() > 0 else ''))
        if want_ascii:
            print(f'  ASCII（每格 {48}px；#=盘内内容 .=盘内近黑 空格=盘外）:')
            for ln in ascii_view(gray, inner, 48, dark):
                print('  |' + ln + '|')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inputs', nargs='+')
    ap.add_argument('--n', type=int, default=8, help='每个输入抽帧数（视频）')
    ap.add_argument('--thr', type=float, default=22.0, help='盘亮阈值（灰度 > thr 视为非背景）')
    ap.add_argument('--dark', type=float, default=12.0, help='盘内"近黑"阈值')
    ap.add_argument('--shrink', type=int, default=10, help='盘内缩边像素（排除盘缘半影）')
    ap.add_argument('--ash', action='store_true', help='打印 ASCII 轮廓')
    a = ap.parse_args()
    for p in a.inputs:
        if os.path.isdir(p):
            for f in sorted(glob.glob(os.path.join(p, '*.png'))) + sorted(glob.glob(os.path.join(p, '*.jpg'))):
                measure(f, a.n, a.thr, a.dark, a.ash, a.shrink)
        else:
            measure(p, a.n, a.thr, a.dark, a.ash, a.shrink)


if __name__ == '__main__':
    main()
