#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段0 · 参考深度推理（onnxruntime）→ 产出交换格式，供 depth_mesh_check.js 消费

设计文档：docs/3d-wallpaper-route-b-implementation.md §2.1/§2.2/§7.1
输出（--prefix 前缀）：
  <prefix>.depth.f32.bin   w*h float32 LE，**原始相对逆深度**（越大越近；归一化交给判据脚本，单一来源）
  <prefix>.rgba.bin        w*h*4 uint8（3:4 裁剪后的源图，尺寸 = --size，默认 1080x1440）
  <prefix>.json            meta（含 onnx SHA-256 / 输入 shape / 源图 / 预处理说明）
  <prefix>.depth16.png     16bit 灰度可视化；<prefix>.depth_color.png 8bit 伪彩

用法：
  python ref_depth.py --onnx tools/depth_mesh/out/da_v2_small_728x546.onnx \
      --image tools/p1.jpeg --prefix tools/depth_mesh/out/p1

依赖：onnxruntime numpy pillow
"""
import argparse, hashlib, json, os, sys, time
import numpy as np

MEAN = np.array([0.485, 0.456, 0.406], np.float32)
STD = np.array([0.229, 0.224, 0.225], np.float32)


def crop_3to4(im):
    """居中裁剪到 3:4（Kit 只认 1080x1440；此处与文档 §3.5 的"智能裁剪"对应，
    阶段0 先用居中裁剪，主体引导裁剪在真机阶段用现有主体分割补）"""
    w, h = im.size
    if w * 4 == h * 3:
        return im
    if w * 4 < h * 3:                      # 太窄 → 裁高
        nh = int(round(w * 4 / 3))
        top = (h - nh) // 2
        return im.crop((0, top, w, top + nh))
    nw = int(round(h * 3 / 4))             # 太宽 → 裁宽
    left = (w - nw) // 2
    return im.crop((left, 0, left + nw, h))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--image', required=True)
    ap.add_argument('--prefix', required=True)
    ap.add_argument('--size', default='1080x1440', help='输出帧尺寸（= Kit 约束）')
    ap.add_argument('--fit', choices=['stretch', 'letterbox'], default='stretch')
    ap.add_argument('--nocrop', action='store_true',
                    help='不做 3:4 裁剪：复刻端侧"深度自检"的路径（整帧压缩到 3:4 输入），用于与设备结果比对')
    ap.add_argument('--filter', choices=['bicubic', 'bilinear', 'lanczos'], default='bicubic',
                    help='缩放到模型输入用的滤波器；与设备比对时用 bilinear（端侧 ResizeNormalize 即双线性）')
    args = ap.parse_args()

    import onnxruntime as ort
    from PIL import Image

    ow, oh = (int(v) for v in args.size.lower().split('x'))
    sess = ort.InferenceSession(args.onnx, providers=['CPUExecutionProvider'])
    inp = sess.get_inputs()[0]
    shape = inp.shape                                    # 例：[1,3,728,546] 或含 'dynamic'
    ih = shape[2] if isinstance(shape[2], int) else 728
    iw = shape[3] if isinstance(shape[3], int) else 546
    if ih % 14 or iw % 14:
        print(f'⚠️ 模型输入 {iw}x{ih} 非 14 的倍数，Depth Anything V2 结果可能异常', file=sys.stderr)

    im = Image.open(args.image).convert('RGB')
    if not args.nocrop:
        im = crop_3to4(im)
    src_w, src_h = im.size
    FILT = {'bicubic': Image.BICUBIC, 'bilinear': Image.BILINEAR, 'lanczos': Image.LANCZOS}[args.filter]
    frame = im.resize((ow, oh), Image.LANCZOS)           # 送给判据脚本的源帧（3:4）

    # 预处理：等比缩放到模型输入；宽高比与模型输入不一致时按 --fit 处理
    if abs(iw / ih - ow / oh) < 1e-3:
        net = im.resize((iw, ih), FILT)                  # 同比例 → 直接缩放（默认路径，无 letterbox）
        pad = (0, 0, 0, 0)
    elif args.fit == 'letterbox':
        s = min(iw / src_w, ih / src_h)
        nw, nh = max(14, int(src_w * s) // 14 * 14), max(14, int(src_h * s) // 14 * 14)
        r = im.resize((nw, nh), Image.BICUBIC)
        net = Image.new('RGB', (iw, ih), (int(MEAN[0] * 255), int(MEAN[1] * 255), int(MEAN[2] * 255)))
        pad = ((iw - nw) // 2, (ih - nh) // 2, (iw - nw) // 2, (ih - nh) // 2)
        net.paste(r, (pad[0], pad[1]))
    else:
        net = im.resize((iw, ih), FILT)
        pad = (0, 0, 0, 0)

    x = np.asarray(net, dtype=np.float32) / 255.0
    x = (x - MEAN) / STD
    nchw = np.transpose(x, (2, 0, 1))[None]
    t0 = time.time()
    out = sess.run(None, {inp.name: nchw})[0]            # (1,h,w)
    ms = (time.time() - t0) * 1000
    d = np.asarray(out).reshape(out.shape[-2], out.shape[-1]).astype(np.float32)
    if pad[0] or pad[1]:                                  # 去掉 letterbox 边（两侧按比例裁）
        sx = iw / net.width; sy = ih / net.height
        x0 = int(round(pad[0] * sx)); y0 = int(round(pad[1] * sy))
        d = d[y0:d.shape[0] - y0, x0:d.shape[1] - x0]

    # → 输出尺寸（判据脚本按 1080x1440 口径消费）
    dimg = Image.fromarray(d)
    dimg = dimg.resize((ow, oh), Image.BILINEAR)
    dout = np.asarray(dimg, np.float32)

    p1, p50, p99 = (float(np.percentile(dout, q)) for q in (1, 50, 99))
    print(f'推理耗时 {ms:.0f} ms  raw: min={float(dout.min()):.3f} max={float(dout.max()):.3f} '
          f'p1={p1:.3f} p50={p50:.3f} p99={p99:.3f}  非线性量程={p99 - p1:.3f}')
    print('（若 p1/p99 跨度为 0 附近 → 该图深度无效，判据脚本会判 span 不足）')

    os.makedirs(os.path.dirname(os.path.abspath(args.prefix)), exist_ok=True)
    dout.astype('<f4').tofile(args.prefix + '.depth.f32.bin')
    rgba = np.dstack([np.asarray(frame, np.uint8), np.full((oh, ow, 1), 255, np.uint8)])
    rgba.tofile(args.prefix + '.rgba.bin')
    sha = hashlib.sha256(open(args.onnx, 'rb').read()).hexdigest()
    json.dump({'w': ow, 'h': oh, 'src': os.path.abspath(args.image), 'frame': '3:4 center crop',
               'onnx': os.path.abspath(args.onnx), 'onnx_sha256': sha,
               'input_shape': [1, 3, ih, iw], 'fit': args.fit,
               'preproc': 'RGB/255, mean=[0.485,0.456,0.406], std=[0.229,0.224,0.225], CHW, 14-multiple',
               'output': 'raw relative INVERSE depth (larger = closer)', 'elapsed_ms': round(ms)},
              open(args.prefix + '.json', 'w'), ensure_ascii=False, indent=2)

    vis = np.clip((dout - p1) / max(1e-9, p99 - p1), 0, 1)
    Image.fromarray((vis * 255).astype(np.uint8)).save(args.prefix + '.depth16.png')
    Image.fromarray((vis * 255).astype(np.uint8)).convert('RGB').save(args.prefix + '.depth_color.png')
    print(f'已写出：{args.prefix}.depth.f32.bin / .rgba.bin / .json / .depth16.png（近=亮）')
    print(f'下一步：node tools/depth_mesh/depth_mesh_check.js --input={args.prefix}')


if __name__ == '__main__':
    main()
