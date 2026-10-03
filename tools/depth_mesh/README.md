# tools/depth_mesh —— 3D壁纸·路线B 阶段0（离线，零设备）运行手册

设计文档：`docs/3d-wallpaper-route-b-implementation.md`。本目录的东西全部**可离线复现**，
目标是在写任何 C++/ArkTS 之前，把"深度 → 深度网格 → 多视角帧 → 喂 Kit"这条链路的**数学与观感**证清楚。

## 1. 本机实况（2026-10-01 探测，改动前先复核）

| 项 | 实况 | 影响 |
|---|---|---|
| Node | v24.17.0 | 判据脚本可直接跑 |
| Python | **未安装**（只有 Windows Store 占位符） | 导出/参考推理前需装（`winget install -e --id Python.Python.3.12 --scope user`） |
| WSL | **只是"桩"**（功能未安装，`--import` 不可用；真装需管理员+重启+占 C 盘） | **转换改在项目服务器执行**（Ubuntu 22.04 x86_64，见 `docs/server-ops.md`；本地不装 Linux） |
| github.com | **不可达**（000） | 权重走 HF 镜像 `hf-mirror.com`（可达 200）；converter 包走华为 OBS（可达 206） |
| MindSpore converter | 官方**只有 Linux-x86_64** 包（Windows 包不存在，实测 403） | 转换只能在 Linux/WSL 做 |
| PyPI | 清华源可达（200） | pip 加 `-i https://pypi.tuna.tsinghua.edu.cn/simple` |

## 2. 三步走

```bash
# ① 判据先跑（零依赖、零素材，合成场景自证数学）——任何时候都可跑
node tools/depth_mesh/depth_mesh_check.js
#   → 期望：PASS 18 / FAIL 0（[7] 在 C++ 未实装时 SKIP）
#   → 出图：tools/ui_shots/depth_mesh/{dm_ref,dm_depth,dm_bg_fill,dm_end_pos,dm_end_neg}.png

# ② 导出 ONNX（需 Python + torch；权重走 HF 镜像，github 不可达）
python tools/depth_mesh/export_onnx.py --out tools/depth_mesh/out/da_v2_small_728x546.onnx --image tools/p1.jpeg
#   → 固定 1x3x728x546(NCHW) opset17 + SHA-256 + torch↔onnxruntime 数值一致性比对

# ③ 参考推理 → 真实深度 → 判据（真实模式）
python tools/depth_mesh/ref_depth.py --onnx tools/depth_mesh/out/da_v2_small_728x546.onnx \
    --image tools/p1.jpeg --prefix tools/depth_mesh/out/p1
node tools/depth_mesh/depth_mesh_check.js --input=tools/depth_mesh/out/p1      # 无天空照片加 --skyskip=1
```

`.ms` 转换（Linux/WSL）：见 `convert_ms.sh` 头部注释（含 converter 下载地址与 SHA-256、NHWC 陷阱、算子核对纪律）。

## 2.5 转换结论（2026-10-01 实测定案，先读这条再动手）

| 项 | 结论 |
|---|---|
| 在哪转 | **项目服务器**（Ubuntu 22.04 x86_64，见 `docs/server-ops.md`）；本机 WSL 只是"桩"、Windows 包不存在 |
| 能否静态 | ❌ **不能**：强制 `--inputShape` 会卡在 `/depth_head/projects.0/Conv` 的 `Unexpected input format 0`（`Transpose→Reshape→Conv` 格式传播断链）。已穷举失败：自导出静态 ONNX（`make_static_onnx.py`，onnxsim 固定+折叠，数值无损）、`--inputDataFormat=NHWC`、关融合/黑名单融合 |
| 能怎么转 | ✅ **动态 shape**：`converter_lite --fmk=ONNX --modelFile=<onnx> --outputFile=<out> --inputDataFormat=NCHW`（**不加 `--inputShape`**） |
| 产物 | `out/ms/da_v2_small_728x546_dyn_fp32.ms`（94MB）/ `da_v2_small_fp16_dyn.ms`（47MB），SHA-256 见实现文档附录 C；服务器 `~/mslite_convert/` 留一份（后续"用时下载"的服务端源） |
| 端侧怎么用 | **必须先 `OH_AI_ModelResize(model, inputs, dims=[[1,3,728,546]])` 再 predict**（Native `OH_AI_ModelResize` / ArkTS `Model.resize(inputs, dims)`，本机 SDK 已确认）——这是阶段1 的第一验证项 |
| 生产机纪律 | 服务器上**严禁**跑 benchmark 类重型验证（实测动态 .ms 让 benchmark 吃 1.27GB 内存、11 分钟未收敛）；运行时验证一律放真机 |

## 3. 交换格式（ref_depth.py ↔ depth_mesh_check.js）

| 文件 | 内容 |
|---|---|
| `<prefix>.depth.f32.bin` | `w*h` float32 LE，**原始相对逆深度（越大越近）**——归一化只在判据脚本里做（单一来源） |
| `<prefix>.rgba.bin` | `w*h*4` uint8，3:4 裁剪后的源帧（默认 1080×1440，与 Kit 约束一致） |
| `<prefix>.json` | meta：尺寸 / 源图 / onnx SHA-256 / 输入 shape / 预处理 / 输出语义 |

## 4. 判据口径要点（为什么这么定，见文档 §3.3/§3.6/§7.1）

- **位移口径**：历史文档（坑 126 表格）的位移数字 ≈ **精确投影值的 2 倍**（复核发现）。
  本工具一律用精确针孔投影，守卫 = **最近层相邻关键帧位移 ≤ 80px**
  （同一口径下历史实测：成功轮 ≈70px、失败轮 ≈100px）。
- **参考位姿恒等**：θ=0 的网格渲染必须等于源帧；深度台阶带（0.58%）单独统计，**带外要求 100% 恒等**。
  台阶带由"1px 正面平行小片"把带宽压到最小——**C++ 实装必须同样细分，且绝不跨台阶连接**（否则新视角下拉成长条纹）。
- **三层构件**：照片网格（前景）+ 背景层（近区 push-pull 补洞，承接轮廓消隐）+ 远景板（兜底）；
  照片网格带 **80px 裙边**（边界像素按深度外延）承接轨道端点的画幅边缘空隙。
- 参数扫描表在判据输出末尾（Θ=3°/4°/5°/6°/8° 的 stepNear/slip/端点位移）；产品默认 **Θ=±5°（对角弧 Φ=Θ）**、72 个候选帧（2026-10-02 起；历史 fx=1500 纯水平弧口径停用）。

## 5. 阶段0 验收标准

1. `depth_mesh_check.js` 合成模式 **PASS 18 / FAIL 0**（含结构守卫、位移公式实测误差 ≤3px、三层占比）；
2. 出图 5 张肉眼核验：无条纹、无鬼影、端点帧照片占比 ≥ 70%、消隐带为背景层内容；
3. ONNX 导出后 torch↔onnxruntime 相对误差 ≤ 0.02；
4. 真实照片跑真实模式判据：深度方向（天空 < 地面）与位移守卫通过；
5. `.ms` 转换成功并记录 SHA-256（转换失败的算子，登记到文档 §8 风险表）。

## 6. 阶段1（真机深度推理）验证流程——代码已落地，待构建

模型随包：`entry/src/main/resources/rawfile/models/da_v2_small_fp16_dyn.ms`（49.5MB，SHA-256 `3a72a9f2…`）。
代码：`mslite_depth_runner.cpp`（加载/resize/推理/统计/ASCII 缩略图）+ napi `depthProbe`/`depthEstimateTest`
+ `SpatialReconService.depthSelfTest` + 「3D 壁纸」页「深度自检」按钮。

```bash
# 1) 构建安装（用户执行；本工程默认不自行构建）
# 2) 页面操作：3D 壁纸 → 选照片 → 「深度自检（路线B·阶段1）」
# 3) 日志取证（沙箱取不出文件 → 只走日志；native 打 32 行 48 列 ASCII 深度缩略图，近=亮）
hdc shell hilog -x | grep -E "SRD (probe|model|selftest|depth)" > srd.log
# 4) 判据比对（本地已自测：r=0.9797 / mad=0.0356 → PASS）
python tools/depth_mesh/compare_device_depth.py --dev-ascii srd.log --ref tools/depth_mesh/out/p1
```

期望：`code=0 … layout=NCHW … inferMs=…`，且 `[2] r ≥ 0.95`、`[3] 平均绝对差 ≤ 0.08`。
失败签名与下一步见实现文档 §9「阶段1 真机验证清单」。

## 7. 阶段2（深度网格出片）——✅ 真机已跑通（2026-10-01 复测，pid 48868）

- 入口：3D 壁纸页「场景」第 6 档「**深度网格**」（scene=5）→ 按当前选项生成。
- 链路：Service 裁 3:4 → `depthEstimateTest`（端侧深度）→ 回读 `.bin` → `spatialReconTestGenerate(scene=5, depthBuffer…)`
  → bridge `SrBakeDepthMeshWorld`（世界点一次性固化）+ `SrRasterizeDepthMesh`（远景板→前景层→背景层三层光栅化，72 帧）
  → Kit 重建 → MP4。
- 复测实测：`SRCHK depth mesh ok depth=546x728 **canvas=1080x1440** … plane=68x90` → 72 帧 `miss=0` →
  Kit `STAGE01→04` 全 completed → `[ENCODER] Video Encoding Completed.` → `onFinished status=0` → `done …/model_*.mp4`；
  **无 `SR3D fallback` 行**（若出现 = scene=5 又失败、出片是 scene=1 降级产物）；会话 ≈87 s。
- 几何级核验（**唯一取证手段**，设备 shell 取不到沙箱）：`tools/depth_mesh/scene5_render_check.js` 一次运行给三件结论——
  `[C]` 设备 `f000/fmid` 与离线两种语义（`cpp`=首测实装语义（回归用）/ `fix`=判据口径）逐格比对；
  `[D]` **设备内部双检**（① `fmid` vs `src` = 参考位姿恒等 ② `f000` vs `fmid` = 视差存在）；③ 位移守卫。
  ```bash
  node tools/depth_mesh/scene5_render_check.js \
      --depth=tools/depth_mesh/out/dev5.depth.f32.bin --rgba=tools/depth_mesh/out/dev5.rgba.bin \
      --frame=tools/depth_mesh/out/dev5fb.rgba.bin --log=s2.log
  ```
  留档：首测（零视差 + 不恒等）`out/scene5_repro_dev5.log`；复测（恒等 98.0% / 视差 58.9%）`out/scene5_repro_dev5_fixed.log`。
  重建照片输入：`ref_depth.py --image out/dev5_photo_crop.png --size 546x728 --nocrop --filter bilinear`。
- 首测照片的一个非阻塞 FAIL：`depth_mesh_check.js --input=out/dev5f` 判「台阶带占比 ≤ 3%」**FAIL（4.60%）**——观感级
  （1px 平片的视差量化），真机已通过；待观感确认后再决定是否调 `REFINE_RATIO`；其余 22 项 PASS。
- 坑 129（**必读**）：`.trae/documents/arkts-arkui-common-pitfalls.md` —— 世界点逐帧重建 → 零视差；
  深度场坐标未映射画布 → 参考位姿不恒等。已修 + 判据 [7] 3 条源码守卫防回归。
