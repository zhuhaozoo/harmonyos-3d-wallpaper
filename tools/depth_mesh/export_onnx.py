#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阶段0 · 导出 Depth Anything V2 Small 固定 shape ONNX（1x3x728x546，NCHW，opset 17）

设计文档：docs/3d-wallpaper-route-b-implementation.md §2.3
本机网络实况：github.com 不可达 → 走 HF 镜像（hf-mirror.com）；权重来自
  depth-anything/Depth-Anything-V2-Small-hf（Apache-2.0，可商用；Base/Large/Giant 禁商用）

依赖（建议在 venv 里装，走清华源）：
  python -m venv .venv && .venv\\Scripts\\activate
  pip install -i https://pypi.tuna.tsinghua.edu.cn/simple torch --index-url https://pypi.tuna.tsinghua.edu.cn/simple
  pip install -i https://pypi.tuna.tsinghua.edu.cn/simple transformers onnx onnxruntime numpy pillow

用法：
  python export_onnx.py --out tools/depth_mesh/out/da_v2_small_728x546.onnx
  python export_onnx.py --image tools/p1.jpeg        # 附带 torch↔onnxruntime 数值一致性比对

产出：<out>.onnx + <out>.onnx.sha256 + 控制台打印"下一步 converter_lite 命令"
"""
import argparse, hashlib, os, sys

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='tools/depth_mesh/out/da_v2_small_728x546.onnx')
    ap.add_argument('--hf-endpoint', default='https://hf-mirror.com')
    ap.add_argument('--image', default='', help='用于 torch↔onnxruntime 数值一致性比对的测试图')
    ap.add_argument('--opset', type=int, default=17)
    args = ap.parse_args()

    os.environ.setdefault('HF_ENDPOINT', args.hf_endpoint)   # github/HF 直连不稳 → 镜像
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)

    import torch
    from transformers import AutoModelForDepthEstimation

    model_id = 'depth-anything/Depth-Anything-V2-Small-hf'
    print(f'[1/4] 加载 {model_id}（HF_ENDPOINT={os.environ["HF_ENDPOINT"]}）…')
    model = AutoModelForDepthEstimation.from_pretrained(model_id)
    model.eval()

    # 固定 shape：1x3x728x546（宽 546=39*14、高 728=52*14，恰 3:4；DA-V2 要求输入边长为 14 的倍数）
    dummy = torch.randn(1, 3, 728, 546)

    class Wrap(torch.nn.Module):      # HF 端口 forward 返回 dict → 只导出 depth 张量
        def __init__(self, m): super().__init__(); self.m = m
        def forward(self, pixel_values): return self.m(pixel_values=pixel_values).predicted_depth

    print(f'[2/4] 导出 ONNX（opset={args.opset}, 固定 1x3x728x546, NCHW）…')
    with torch.no_grad():
        torch.onnx.export(
            Wrap(model), dummy, args.out,
            input_names=['pixel_values'], output_names=['predicted_depth'],
            opset_version=args.opset, do_constant_folding=True,
            dynamic_axes=None,          # 固定 shape：便于 converter_lite 与端侧静态图
        )

    sha = hashlib.sha256(open(args.out, 'rb').read()).hexdigest()
    open(args.out + '.sha256', 'w').write(sha + '  ' + os.path.basename(args.out) + '\n')
    size_mb = os.path.getsize(args.out) / 1048576
    print(f'[3/4] 产出：{args.out}（{size_mb:.1f} MB）')
    print(f'      SHA-256: {sha}   → 已写入 {args.out}.sha256（进版本库/记录）')

    if args.image:
        print('[4/4] 数值一致性比对（torch vs onnxruntime，同一张图）…')
        import numpy as np, onnxruntime as ort
        from PIL import Image
        im = Image.open(args.image).convert('RGB').resize((546, 728), Image.BICUBIC)
        x = np.asarray(im, dtype=np.float32) / 255.0
        mean = np.array([0.485, 0.456, 0.406], np.float32)
        std = np.array([0.229, 0.224, 0.225], np.float32)
        x = (x - mean) / std
        nchw = np.transpose(x, (2, 0, 1))[None]                      # 1x3x728x546
        with torch.no_grad():
            t = Wrap(model)(torch.from_numpy(nchw)).numpy()
        sess = ort.InferenceSession(args.out, providers=['CPUExecutionProvider'])
        o = sess.run(None, {'pixel_values': nchw})[0][:, None]
        diff = np.abs(t[:, None] - o)
        rng = float(t.max() - t.min()) + 1e-9
        rel = float(diff.max()) / rng
        print(f'      max|Δ|={float(diff.max()):.5f}  相对量程={rel:.5f}（阈值 0.02）→',
              'PASS' if rel <= 0.02 else 'FAIL（检查导出/归一化）')
        if rel > 0.02:
            sys.exit(1)

    print('\n下一步（在 Linux/WSL 里，官方 converter 只提供 Linux x86_64）：')
    print('  tools/depth_mesh/convert_ms.sh <converter包解压目录> ' + args.out + ' tools/depth_mesh/out/da_v2_small_728x546')
    print('  转换前先逐算子核对官方《MindSpore Lite Kit 算子支持列表》；转换后跑：')
    print('  python tools/depth_mesh/ref_depth.py --onnx tools/depth_mesh/out/da_v2_small_728x546.onnx --image <照片> --prefix tools/depth_mesh/out/p1')
    print('  node  tools/depth_mesh/depth_mesh_check.js --input=tools/depth_mesh/out/p1')

if __name__ == '__main__':
    main()
