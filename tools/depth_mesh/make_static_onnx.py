#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段0 · 把动态 shape 的 ONNX 静态化（onnxsim：固定输入尺寸 + 常量折叠）

为什么需要它（docs/3d-wallpaper-route-b-implementation.md §2.3 纪律 0）：
  社区 ONNX（onnx-community/depth-anything-v2-small）输入是动态的（height/width），
  图里含 floor(h/14) 一类"由图算形状"的子图；MindSpore Lite converter 一旦被强制静态
  （--inputShape）就在 /depth_head/projects.0/Conv 处 InferShape 失败。
  onnxsim 会把这类形状子图**折叠成常量**并固定输入尺寸 → 转换器拿到全静态图。
  本脚本同时做**静态版 vs 动态版数值一致性校验**（同一输入，输出必须一致）。

用法：
  python make_static_onnx.py --onnx <动态onnx> --out <静态onnx> [--shape 1,3,728,546] [--image <照片做一致性校验>]

依赖：onnx onnxsim onnxruntime numpy pillow
"""
import argparse, hashlib, os, sys
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--shape', default='1,3,728,546', help='固定输入尺寸 N,C,H,W（H/W 必须是 14 的倍数）')
    ap.add_argument('--image', default='', help='用于"静态版 vs 动态版"输出一致性校验的照片')
    args = ap.parse_args()

    import onnx
    import onnxsim
    import onnxruntime as ort

    shape = [int(v) for v in args.shape.split(',')]
    assert len(shape) == 4 and shape[2] % 14 == 0 and shape[3] % 14 == 0, 'H/W 必须是 14 的倍数'

    m = onnx.load(args.onnx)
    in_name = m.graph.input[0].name
    print(f'输入名：{in_name}  原 shape：{[d.dim_value or d.dim_param for d in m.graph.input[0].type.tensor_type.shape.dim]}')

    m2, ok = onnxsim.simplify(m, overwrite_input_shapes={in_name: shape}, dynamic_input_shape=False)
    if not ok:
        print('onnxsim 自检失败（simplify 返回 ok=False）', file=sys.stderr)
        sys.exit(1)
    onnx.checker.check_model(m2)
    onnx.save(m2, args.out)
    sha = hashlib.sha256(open(args.out, 'rb').read()).hexdigest()
    print(f'静态化完成：{args.out}  {os.path.getsize(args.out)/1048576:.1f} MB')
    print(f'新输入 shape：{[d.dim_value for d in m2.graph.input[0].type.tensor_type.shape.dim]}')
    print(f'SHA-256: {sha}')
    open(args.out + '.sha256', 'w').write(sha + '  ' + os.path.basename(args.out) + '\n')

    # —— 一致性校验：同一输入，动态版 vs 静态版 ——
    rng = np.random.default_rng(0)
    if args.image:
        from PIL import Image
        MEAN = np.array([0.485, 0.456, 0.406], np.float32)
        STD = np.array([0.229, 0.224, 0.225], np.float32)
        im = Image.open(args.image).convert('RGB').resize((shape[3], shape[2]), Image.BICUBIC)
        x = np.asarray(im, np.float32) / 255.0
        x = (x - MEAN) / STD
        x = np.transpose(x, (2, 0, 1))[None]
    else:
        x = rng.standard_normal(shape).astype(np.float32)

    def run(model_path, input_name):
        s = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
        return s.run(None, {input_name: x})[0]

    y_dyn = run(args.onnx, in_name)
    y_sta = run(args.out, in_name)
    print(f'输出 shape：动态版 {y_dyn.shape} / 静态版 {y_sta.shape}')
    d = float(np.abs(y_dyn - y_sta).max())
    rng_t = float(y_dyn.max() - y_dyn.min()) + 1e-9
    print(f'静态版 vs 动态版：max|Δ|={d:.6g}  相对量程={d/rng_t:.3g}')
    if d / rng_t > 1e-4:
        print('❌ 两者不一致，静态化有问题，禁止使用', file=sys.stderr)
        sys.exit(1)
    print('✅ 一致（静态化无损）')
    print(f'\n下一步：把 {args.out} 传到 Linux 机器（本项目 = 服务器 ~/mslite_convert/），')
    print('  converter_lite --fmk=ONNX --modelFile=... --outputFile=... --inputDataFormat=NCHW   （静态图无需 --inputShape）')


if __name__ == '__main__':
    main()
