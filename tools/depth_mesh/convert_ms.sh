#!/usr/bin/env bash
# 阶段0 · ONNX → MindSpore Lite .ms（官方 converter 仅提供 Linux x86_64；本机 Windows 用 WSL2）
#
# 用法（在 Linux/WSL 内，仓库根目录）：
#   tools/depth_mesh/convert_ms.sh <converter包解压目录> <onnx路径> <输出前缀>
# 例：
#   tools/depth_mesh/convert_ms.sh ~/mslite/mindspore-lite-2.7.0-linux-x64 \
#       tools/depth_mesh/out/da_v2_small_728x546.onnx tools/depth_mesh/out/da_v2_small_728x546
#
# 转换包来源（官方《使用 MindSpore Lite 进行模型转换》下载表，Linux-x86_64）：
#   https://ms-release.obs.cn-north-4.myhuaweicloud.com/2.7.0/MindSporeLite/lite/release/linux/x86_64/mindspore-lite-2.7.0-linux-x64.tar.gz
#   SHA-256: 8bb1097100c9fec12675670ba2d4264a2cd6da3a9be093eb56631d00fc0c455b
#
# ⚠️ 关键纪律（官方文档口径）：
#   ① --inputDataFormat 默认 NHWC，而 ONNX 是 NCHW → **必须显式 NCHW**，否则数据布局错、结果全错且不报错；
#   ② 转换前逐算子核对《MindSpore Lite Kit 算子支持列表》（Resize/Gelu/LayerNorm/MatMul/Gather/Slice/Pad 重点看）；
#   ③ fp32 与 fp16 各转一份，端侧用 ref_depth 的比对结果决定用哪份；
#   ④ NPU(NNRt) 后端若要关 clip 融合，需要源码编译的 converter（下载版不具备）——本次 CPU 优先。
set -euo pipefail

PKG="${1:?用法: convert_ms.sh <converter包解压目录> <onnx路径> <输出前缀>}"
ONNX="${2:?缺少 onnx 路径}"
PREFIX="${3:?缺少输出前缀}"

CNV="$PKG/tools/converter/converter/converter_lite"
[ -x "$CNV" ] || { echo "找不到 converter_lite：$CNV"; exit 1; }
export LD_LIBRARY_PATH="$PKG/tools/converter/lib:${LD_LIBRARY_PATH:-}"

echo "== fp32 =="
"$CNV" --fmk=ONNX --modelFile="$ONNX" --outputFile="${PREFIX}_fp32" \
       --inputShape="pixel_values:1,3,728,546" --inputDataFormat=NCHW

echo "== fp16（权重存半精度；精度需肉眼核验后才允许启用）=="
"$CNV" --fmk=ONNX --modelFile="$ONNX" --outputFile="${PREFIX}_fp16" \
       --inputShape="pixel_values:1,3,728,546" --inputDataFormat=NCHW --fp16=on

echo "== SHA-256（记录进版本库/开源说明）=="
sha256sum "${PREFIX}_fp32.ms" "${PREFIX}_fp16.ms" | tee "${PREFIX}.ms.sha256"

cat <<'EOF'

下一步：
1) 把选定的一份改名 da_v2_small_728x546.ms 放入工程（按最新决策：**不随包**，放服务器供"用时下载"）：
   下载校验：服务端与端侧都校验 SHA-256；端侧落到 filesDir 后用 OH_AI_ModelBuild 加载。
2) 端侧最小自检入口（阶段1）：depthProbe / depthEstimateTest → 出 .depth.f32.bin 回传，
   与 PC 端 ref_depth.py 结果比对（相对误差 ≤ 0.02 判 PASS）。
EOF
