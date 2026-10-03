# 3D 壁纸 · 实现文档：单图深度估计 → 深度网格多视角合成 → Spatial Recon Kit → 3D 影像

> **阅读说明**：本文为开发期实现文档。文中"坑 N"为开发过程错误记录编号，其结论已收编于
> 本仓库 `docs/3d-wallpaper-archive.md` 的 F1~F14 节；"阶段 N"指本文档 §9 的实施计划编号。
> 文中提到的装置/服务环境信息已做泛化处理。

> **状态**：v2（**阶段0 / 阶段1 闭环**；**阶段2 真机已跑通出片**（2026-10-01 复测，Kit 四阶段全过、无降级）；剩观感验收与参数标定）。
> **实施进展（2026-10-01 深夜）**：**阶段0 / 阶段1 闭环；阶段2 真机跑通**——
> 阶段0：判据（合成 **18/0**、真实照片 16、加源码守卫后 **28 PASS / 0 FAIL**）、真实照片视差预览、`.ms` 转换（服务器路线，动态 shape fp32 94MB / fp16 47MB，
> SHA-256 见附录 C；静态 shape 被 converter 限制卡死，定案"动态 + 端侧 resize"）。
> 阶段1：真机首测全绿——`probe rc=0` / `model built 149ms` / `resize → [1,3,728,546]` / `code=0 … layout=NCHW … inferMs=2616`；
> **数值判据 r=0.9956**（同源照片；残差只剩 ASCII 量化取整）⇒ 转换无损、端侧推理链路正确。
> 阶段2：`scene=5 深度网格` 全链路代码落地（纯 2D 单元 + bridge 三层光栅化 + napi/d.ts/Service/页面五层齐）；
> **真机首测复现了两个实装缺陷**——① 世界点逐帧重建 → 零视差；② 深度场坐标未映射到画布 →
> 参考位姿不恒等——Kit 4.4s 报 `STAGE02 no 3D points` → `STAGE03 Adapting failed`（`cb=1023700007`）→ 降级 scene=1 出片。
> **修复已落地并加判据守卫**（世界点一次性固化 `SrBakeDepthMeshWorld` + 画布空间映射 + 远景板 UV 归一化 + 贴图按画布分辨率；
> native 三文件 `-fsyntax-only` 全绿）；首测的逐格取证见 §9「阶段2 真机首测（失败）与修复」。
> 本版按实装复核结果修正三处：**位移口径**（历史文档数字 ≈ 精确投影值 2 倍，已纠正）、
> **默认轨道角 Θ=5°**（原 8° 会超守卫界）、**场景构件改为三层**（照片网格 + 背景层 + 远景板，见 §3.3）。
> **构建**：DevEco Studio + HarmonyOS SDK（API 26）。
> **事实来源标记**（全文遵守，冲突时后写的复核优先）：
> - 【官方】= 华为官方文档原文（已在文中给出来源链接）
> - 【SDK】= 本机 SDK 头文件 / d.ts 实证
> - 【实测】= 真机实测（2026-10-01 前后记录）
> - 【待验证】= 本设计中的推断/默认值，必须在阶段 0/1/2 用判据或真机实测确认后才能作为定论
>
> **与既有文档的关系**：`docs/spatial-recon-3d-wallpaper.md` 是本功能的历史记录（几何为"程序场景 + 照片贴面"），
> 其中的数字与结论**不作为本路线依据**；本路线与旧路线唯一共享的是"与 Kit 交互"的那一层。
> 凡旧文档与本文档冲突处，以**本文档 + 官方文档**为准，并在实施中顺手回改旧文档。

---

## 0. 为什么必须是路线B（产品闭环决定）

**产品目标**：用户选一张静态照片 → 生成一段 **3D 影像** → 保存到图库 → **可被系统识别并设为壁纸**。

**关键事实（本路线的立足点）**：
- 只有 **Spatial Recon Kit**（3DGS 重建 → 运镜 MP4）产出的结果属于系统认得的"3D 影像"，才能走"设为壁纸"这一步；
  自渲染的普通 MP4 只是普通视频壁纸。**因此"Kit 重建必须成功"是本路线的硬约束**，一切设计围绕它服务。
- 鸿蒙官方的**单图"空间照片"能力已下线**：官方文档更新记录（2026-07-28）原文——
  "Spatial Recon Kit 删除文档，因空间照片功能的技术策略调整，该功能暂时下线"【官方】。
  本机 SDK 26.0.0 里 `SpatialReconKit` 只剩 `spatialRender`（3DGS 加载/渲染）与 `spatialEdit`，
  C 接口 `spatial_recon_interface.h` 只有多帧重建/会话/保存共 15 个函数，**没有单图入口**【SDK】。
  → 第三方要做"单图 → 3D"，**只能自建"端侧深度估计 → 合成多视角帧 → 喂 Kit"这条链路**。

**路线B 一句话**：把照片按单目深度展开成一块"真三维网格"（+ 一块远景背景板），用它渲染出**带真实视差的
多视角帧序列**（1080×1440 RGB + 相机内参/位姿），推给 Kit 做 3DGS 重建与保存——Kit 输出即"可设壁纸的 3D 影像"。

**与旧路线（程序盒子 + 照片贴面）的本质区别**：旧路线里"照片是一张海报"，深度是人工给定的两层平面；
本路线里**照片本身就是场景**（逐像素深度决定几何），视差真实、画面由照片铺满。

---

## 1. Kit 的硬约束（官方原文，逐条落成工程约束）

| # | 约束 | 出处 | 对本路线的工程含义 |
|---|---|---|---|
| K1 | **仅支持 1080×1440 图像**（宽 1080、高 1440），"输入其余尺寸的图像，结果是未定义的" | 【官方】《重建三维场景（C/C++）》 | 合成帧恒为 1080×1440；照片非 3:4 时必须裁剪或扩展（见 §3.5） |
| K2 | 帧格式**仅支持 RGB**（`SPATIAL_RECON_IMAGEDATA_FORMAT_RGB`），三通道紧凑 ×3 字节/像素 | 【官方】+【SDK】头文件 | 自产帧缓冲按紧凑布局，行跨距必须 = 1080×3（加断言） |
| K3 | 数据帧结构：`focalX/focalY/principalX/principalY/distortionCoef[8]/imageWidth/imageHeight/position[3]/rotation[4]=[x,y,z,w] 四元数/timestamp(ns)/imageData/format` | 【SDK】`spatial_recon_interface.h` | 内参用虚拟相机（见 §3.1）；畸变全 0 |
| K4 | 关键帧由 **Kit 自动挑选**（应用不可控） | 【官方】"系统会自动选取关键帧进行保存" | 位移量级必须按"最坏相邻关键帧"设计（§3.6），不能按单帧步长设计 |
| K5 | `workPath` 必须是**已存在**的应用文件目录子目录（`/data/storage/el[1-5]/base/` 下） | 【官方】《管理 Spatial Recon 会话》 | 每次生成用**全新空目录**（历史残留会改变会话行为，属既有实测防线，保留） |
| K6 | `StartSession` 的 `writeInfo` 非空 → 重建完成**自动保存**；保存 MP4 同一时刻仅允许一个 session | 【官方】 | 产物路径由我们指定；保存阶段不可中断 |
| K7 | 每次 `StartSession` 之后须 `SetRunningMode`（前台/后台），否则性能/功耗劣化 | 【官方】 | 前台固定；`aboutToDisappear` 或切后台时改后台模式 |
| K8 | 同一时刻只允许一个 session 重建；**任务在途销毁 = 未定义行为** | 【官方】 | "终止"= 置标志 + PauseSession，等落到 PAUSED 再 Destroy（沿用既有做法） |
| K9 | 仅保证 **Kirin 9020/9030S/9030/9030 Pro 及以后**的用户体验；其他芯片不保证耗时与质量 | 【官方】 | 入口按 `HMS_SpatialRecon_IsSupport` 探测；不支持即降级/提示 |
| K10 | 强烈建议订阅热公共事件 `COMMON_EVENT_THERMAL_LEVEL_CHANGED`，过热暂停 | 【官方】 | 沿用既有热管理（≥HOT 暂停、≤NORMAL 恢复） |
| K11 | 26.0.0 起有 `HMS_SpatialRecon_RegisterNGCallbackFunc`（带 `void* data` 的高级回调，注册一次生效一次；StartSession 前注册 = 重建完成回调，StartSession 后注册 = 保存完成回调） | 【官方】 | 推荐用它分别拿"重建完成"与"保存完成"两个信号（比轮询稳） |
| K12 | 支持 `PauseSession/ResumeSession`（任意时刻） | 【官方】 | 暂停/继续/终止三入口 |
| K13 | 能力库**不得进链接清单**：不支持该特性的设备会因 `DT_NEEDED` 加载即崩 → 必须运行期 `dlopen("libspatial_recon_ndk.z.so")` | 【实测】 | probe 返回非 0 → 直接走降级，不加载 |

**Kit 侧失败签名（判读用，【实测·待复核】）**：
- 约 **5 秒内失败**（`STAGE02 no 3D points` → `1023700007`）→ **位姿约定不对**（§3.4 的 A/B 复核）；
- 数秒内 `STAGE03 Adapting failed`（点数太少）→ **可匹配位移量级不足/超窗**（§3.6 守卫）；
- **跑满 80 秒以上**才是成功候选（STAGE04 训练占大头）。

---

## 2. 端侧深度估计（Depth Anything V2 Small + MindSpore Lite）

### 2.1 模型与许可（已核实）

| 项 | 内容 |
|---|---|
| 模型 | Depth Anything V2 **Small**（ViT-S，约 24.8M 参数），输出**相对深度** |
| 仓库 | `github.com/DepthAnything/Depth-Anything-V2`（代码 Apache-2.0） |
| 权重许可 | README 原文：*"Depth-Anything-V2-Small model is under the Apache-2.0 license. Depth-Anything-V2-Base/Large/Giant models are under the CC-BY-NC-4.0 license."* → **只有 Small 可商用，其余变体禁止使用** |
| 残留尽调点 | 上游 DINOv2 已转 Apache-2.0（不再是障碍）；训练数据含合成数据集与自标注，属业界公认的灰色地带，**上架开源说明如实列出**（非法律意见） |
| 运行时 | **MindSpore Lite：HarmonyOS 系统内置部件**（Apache-2.0），`@kit.MindSporeLiteKit` 与 Native API 均可用【官方】；**App 不需要打包推理运行时，只带 `.ms` 模型文件**【官方】 |

**输出语义（写死到代码注释与判据里）**：DA-V2 Small 输出的是**相对逆深度（affine-invariant disparity）**，
**数值越大 = 越近**；远景接近 0。任何"当作深度直接用"的实现都会得到反的场景（近物后退、远物前冲）。
来源：官方 GitHub issue #93 与多来源交叉印证（"higher = closer / 越大越近"）。

### 2.2 输入规格与预处理（固定 shape）

- **输入分辨率**：取 **546×728**（宽×高）——两者均为 **14 的倍数**（39×14 / 52×14），宽高比**恰为 3:4**，
  与参考帧 1080×1440 同比例，**无需 letterbox，等比缩放即可**（1080×1440 → 546×728 是 0.5056 倍的均匀缩放）。
  14 的倍数与等比缩放是官方推理脚本的硬要求（`ensure_multiple_of=14`）。
- **预处理**：RGB → `/255` → 逐通道 `(x − mean)/std`，`mean=[0.485,0.456,0.406]`，`std=[0.229,0.224,0.225]`（ImageNet 惯例，与官方脚本一致【官方仓库代码】）→ CHW 排布按转换参数而定（见 2.3）。
- **输出**：与输入同尺寸的**单通道 float，逆深度**（1×1×728×546）。
- **分辨率是旋钮**：想更细可试 630×840（45×14 / 60×14）或 1050×1400（75×14 / 100×14，1.47MP）——
  吞吐随 token 数平方增长，**以真机实测为准**；默认 546×728。

### 2.3 模型转换（PC 侧，产出 `.ms`）

工具：MindSpore Lite 模型转换工具 `converter_lite`，**官方只提供 Linux x86_64 下载包**【官方】
（本机 Windows → 用 WSL2/Ubuntu 执行；下载链接与 SHA-256 见官方《使用 MindSpore Lite 进行模型转换》）。

```bash
# 1) 先自行导出 ONNX（官方仓库不提供 ONNX；用社区导出脚本或 torch.onnx.export 均可，
#    但必须记录：来源、opset、输入张量名、SHA-256。推荐自导出固定 shape 版本：
#    input: 1×3×728×546（NCHW），opset 17/18，输出 1×1×728×546。
# 2) 转换（fp32 与 fp16 各转一份，做数值比对后再定用哪份）
./converter_lite --fmk=ONNX \
  --modelFile=da_v2_small_728x546.onnx \
  --outputFile=da_v2_small_728x546 \
  --inputShape="<onnx输入名>:1,3,728,546" \
  --inputDataFormat=NCHW \
  --fp16=on
```

**转换纪律（缺一不可）**：
0. **⚠️ 实测结论（2026-10-01，必读）：本模型的 `.ms` 只能以「动态 shape」转换**（converter_lite 2.7.0）。
   - 社区动态 ONNX（`onnx-community/depth-anything-v2-small`）**不带 `--inputShape`（保留动态）→ 转换成功**
     （fp32 94MB / fp16 47MB，产物与 SHA-256 见附录 C）；
   - 一旦强制静态（`--inputShape=…`，518×518 与 728×546 均试）→ `/depth_head/projects.0/Conv`
     （Conv2DFusion）推形失败；verbose（`GLOG_v=2`）给出根因：
     `CheckInfershapeResult] Unexpected input format 0` —— 该 Conv 的输入来自 `Transpose→Reshape`
     （DPT 头部把 ViT token 还原成特征图），MindSpore Lite 的格式传播在这一段落到 NCHW/DEFAULT，
     而 NNACL 的 Conv kernel 要求 NHWC。
   - **已在服务器上穷举且全部失败**的补救：① 用 `make_static_onnx.py` 自做的"静态化 ONNX"
     （onnxsim 固定输入 + 常量折叠，**数值无损** max\|Δ\|=3.7e-5、SHA-256 `76ae1fda…`）；
     ② `--inputDataFormat=NHWC`；③ `--configFile` 关闭全部融合 / 黑名单 TransposeFusion+LayerNorm 融合。
   - ⇒ **定案：分发动态 shape 的 `.ms`；端侧先 `OH_AI_ModelResize` 固定到 1×3×728×546 再 predict**
     （API 本机 SDK 已确认存在：Native `OH_AI_ModelResize`、ArkTS `Model.resize(inputs, dims)`）。
     **阶段1 的第一件事 = 真机验证"加载 + resize + 推理"**；若真机运行时同样报 format 问题，
     下一杠杆是 ONNX 图手术：把 `Transpose→Reshape→Conv(1×1)` 改写成 `Reshape→Transpose→Conv`
     （数学等价，仅改变 Conv 输入的提供者），使格式传播有确定的落点。
   - 另：动态 `.ms` 在 **Linux benchmark** 上行为异常（吃 1.27GB 内存、11 分钟未收敛，已终止；
     一度把服务器可用内存压到 202MB）。**生产机上严禁跑 benchmark 类重型验证**，运行时验证一律放真机。
1. **逐算子核对**官方《MindSpore Lite Kit 算子支持列表》【官方】，重点看 ViT/DPT 会用到的：
   `Conv/MatMul/Add/Mul/Sub/Div/Pow/Sqrt/ReduceMean/Gelu(Erf)/Softmax/Concat/Reshape/Transpose/Slice/Gather/Pad/ExpandDims/Squeeze/Resize(插值模式)/LayerNorm 类融合`；
   有任何一个不支持 → 换导出方式（例如把 `Resize` 换成等价 `Interpolate` 组合）或降级模型方案。
2. **`--inputDataFormat` 默认是 NHWC**【官方】，而 ONNX 是 NCHW → **必须显式指定 NCHW**，否则数据布局错、结果全错（且不报错）。
3. **数值一致性比对**（阶段 1 必做）：同一张测试图，
   `PC onnxruntime(fp32) 结果` vs `PC/设备端 .ms 结果` 逐像素比对，绝对误差阈值先定 **≤ 0.02 相对量程**（可调）；
   若不符，按"布局(NHWC/NCHW) × 通道序(RGB/BGR)"组合各试一次并记录命中组合。
4. **fp16 只影响几何精度、不影响成片像素**，但仍必须做"深度图肉眼对比"后才允许启用（红线：不损伤观感）。
5. **模型文件分发（2026-10-01 决策：不随包，用时下载）**：模型放自建服务端，
   首次使用本功能时下载到 `filesDir`，**下载前后都校验 SHA-256**（记录在版本库），再 `OH_AI_ModelBuild`。
   理由：安装包体积敏感（同类应用有此先例）；本方案同样采此策略。
   端侧需具备：断点/重试、失败回退（模型不可用时直接走降级链 D2）、进度提示。

### 2.4 Native 集成（官方 C/C++ 用法，落进本工程）

- 头文件（本机 SDK 实证存在）【SDK】：
  `#include <mindspore/types.h> / <mindspore/model.h> / <mindspore/context.h> / <mindspore/status.h> / <mindspore/tensor.h>`
- 链接库（本机 SDK 实证存在 `libmindspore_lite_ndk.so`）【SDK】：CMake `target_link_libraries(... mindspore_lite_ndk)`；
  读 rawfile 需要 `rawfile.z`（`OH_ResourceManager_OpenRawFile/ReadRawFile`）。
- 调用序列（官方样例骨架【官方】+ 本工程补充）：

```cpp
// —— 一次性初始化（懒加载，首次使用 3D 壁纸时）——
// 1) 读模型：OH_ResourceManager_InitNativeResourceManager(env, argv[1]) → OpenRawFile/ReadRawFile → malloc 缓冲
// 2) 上下文：OH_AI_ContextCreate() → OH_AI_ContextSetThreadNum(4)
//    → 设备：OH_AI_DeviceInfoCreate(OH_AI_DEVICETYPE_CPU) 或 OH_AI_DEVICETYPE_NNRT（NPU，见下）
//    → OH_AI_DeviceInfoSetEnableFP16(dev, true) 视精度实测；OH_AI_ContextAddDeviceInfo(ctx, dev)
// 3) OH_AI_ModelCreate() → OH_AI_ModelBuild(model, buf, size, OH_AI_MODELTYPE_MINDIR, ctx)  // 失败必须记日志+降级
// —— 每次推理 ——
// inputs = OH_AI_ModelGetInputs(model)；float* p = (float*)OH_AI_TensorGetMutableData(inputs.handle_list[0])
// 按 2.3 命中的布局填 1×3×728×546；outputs = OH_AI_ModelGetOutputs(model)
// OH_AI_ModelPredict(model, inputs, &outputs, nullptr, nullptr) → 读 outputs.handle_list[0] 的 1×1×728×546
```

- **NPU（可选加速）**：`OH_AI_DEVICETYPE_NNRT` / `OH_AI_CreateNNRTDeviceInfoByName|ByType` / `OH_AI_DEVICETYPE_KIRIN_NPU`
  均在本机头文件里【SDK】。注意【官方】两条：① 用 NPU 后端时**自定义关闭 clip 算子融合**需要**源码编译** converter，
  否则可能报 `BuildKirinNPUModel# Create full model kernel failed`；② 模型有 transpose+conv 融合时下载版 converter 可能报警告。
  → 策略：**先 CPU 跑通（正确性优先），NPU 作为阶段 3 的性能选项**，实测收益再定。
- **线程与内存**：推理在 native 工作线程执行（沿用既有 async Context/Execute/Complete 模式），不阻塞 UI；
  729×546 输入 + 输出缓冲 ≈ 各 ~1.6MB float，可忽略。

### 2.5 深度自检（进代码 + 进判据，防"静默反向"）

1. **方向自检（强判据）**：把结果 `s ∈ [0,1]` 与**既有主体分割蒙版**比对——主体区域的平均 `s` 应 ≥ 全图中位数
   （主体几乎总比背景近）。反向即说明"越大越近"约定被写反。无蒙版时退回第 2 条。
2. **分布自检**：`p99−p1` 归一化后仍小于阈值（如 0.15）→ 判"该图深度无效"（近平面大虚化/纯天空等）→ 走降级链。
3. **日志固定字段**：`rawMin/rawMax/p1/p99/mean`、输入尺寸、推理耗时（ms）、后端（CPU/NNRT）、模型 SHA 前 8 位。

---

## 3. 场景与几何（自包含约定——实施与判据都以本节为唯一口径）

### 3.1 相机 / 世界 / 像素↔射线 / 投影

- **相机系（AR 系）**：X 右、Y 上、Z 向后（相机看 **−Z**）。
- **世界系**：取"参考位姿"的相机系——参考相机在原点、朝向 −Z、up = +Y。
  于是**参考帧就是照片本身**（3:4 裁剪后），参考位姿 = identity。
- **像素 (u,v) 的射线（相机系）**：
  ```
  dir = a·X + b·Y + fwd ,  fwd = (0,0,-1)（= -Z）
  a = (u + 0.5 - cx) / fx
  b = -(v + 0.5 - cy) / fy        ← 图像行向下 ↔ 相机 Y 向上；写成 +b 即垂直镜像（静默错误）
  ```
- **投影（世界 → 像素）**：设相机 i 位姿 `C_i`（位置）、`R_wc,i`（世界→相机旋转）：
  ```
  p = R_wc,i · (P - C_i) ;  z_cam 深度 = -p.z （>0 在前方）
  u = cx + fx · p.x / (-p.z)
  v = cy - fy · p.y / (-p.z)      ← 与上面同源，必须同步改
  ```
- **虚拟相机内参（默认值，可调）**：`fx = fy = 750`、`cx = 540`、`cy = 720`，畸变全 0（合成相机无畸变）。
  → 750 档水平视场 ≈71.6°（旧 1500 档 ≈39.6°；2026-10-02 起产品默认 = 750，见坑 131 / §附录B）。

### 3.2 深度映射：逆深度 → 世界深度

1. 归一化（稳健，抗离群）：`s = clamp((r_raw − p1) / (p99 − p1), 0, 1)`，`s = 1` 为最近。
2. 映射到世界深度（**逆深度线性**，近处采样更细，符合视图合成惯例）：
   ```
   z(s) = 1 / ( s / zNear + (1 - s) / zFar ) ,   zFar = k · zNear
   ```
   `z` 定义为"该像素对应的世界点沿参考相机光轴（世界 −Z）的距离"。
3. **默认参数**：`zNear = 1.0`（世界单位，归一化即可）、`k = 3.0`（远/近比）、`d0 = (zNear+zFar)/2 = 2.0`（轨道中心深度）。
   - `k` 决定"近远分离度"（视差强度）；`k` 太小 → 场景接近共面 → Kit 三角化退化（历史失败模式之一是共面）。
   - **守卫**：`k ≥ 2.0`，且 `(p99−p1)` 有效（见 §2.5）。

### 3.3 场景构件（三层 + 裙边；2026-10-01 按离线实装定版）

绘制顺序（三者共享 z-buffer；**等深度处先画者胜**）：**远景板 → 照片网格 → 背景层**。

**构件 1：照片网格（前景层）**
- 参考帧（1080×1440 的 3:4 裁剪）逐像素反投影：`P(u,v) = z(s(u,v)) · dir(u,v)`（§3.1）——**反投影只在构建期做一次**
  （实装 = `SrBakeDepthMeshWorld`；逐帧只做投影。逐帧重建会让顶点投影恒等于其像素坐标 → 零视差，坑 129-①）。
- **像素空间（坑 129-②）**：网格打在**深度场分辨率**（端侧 546×728）上以省算力，但角点与贴图 UV 必须映射到**画布空间**
  （×`outW/dw、outH/dh`）；直接把深度场坐标当画布坐标用 → 照片缩在画幅左上角、参考位姿不恒等。
- 网格构建（两条硬规则，`tools/depth_mesh/depth_mesh_check.js` 已证）：
  1. **不跨深度台阶连接**：四角 z 比 > 1.15 的格子**细分到 1px 的"正面平行小片"**（四点同深，取格心）。
     跨台阶连接面在新视角下会被拉成长条纹（离线出图已实测到该缺陷，判据已把它设为结构守卫）。
  2. **裙边（SKIRT=80px，画布像素）**：网格向画幅外延展 80px，采样按边界钳制（=边界像素按各自深度向外延续）。
     用于承接轨道端点的画幅边缘空隙——没有裙边时端点帧有约 13% 的画面露出远景板。
- 贴图：源照片像素（顶点 UV = 顶点自身的连续像素坐标；光栅化按透视正确插值）。实装取**画布分辨率**贴图
  （由全分辨率源图单步双线性重采样，与 scene=1..4 的逐像素 `SrSampleImg` 同款；不经深度场中转，避免二次模糊）。

**构件 2：背景层（承接"主体轮廓消隐"）**
- 构造：把深度场与颜色场的**近区（s > 0.5）用 push-pull 金字塔从周围背景补洞**
  （顶层兜底：最近有效邻居迭代播种 → 全局加权均值；push 用**双线性采样父层**，与既有
  `spatial_recon_layers.cpp` 同款），得到"只有背景"的深度+颜色场。
- **只在近区膨胀 24px 的范围内生成**（掩膜外与前层完全同位，生成只会造成等深度交叠/抢深度）。
- 深度乘 `(1-2e-3)` 做 tie 破断（与前层重叠处让前景层稳定胜出）。
- 作用：前景层滑开后露出的**消隐带由背景层内容承接**（几何上正确——露出的本来就是背景），
  而不是露出模糊板。**这就是本路线不需要 LaMa 的底气**；LaMa 仅作为"背景层补洞观感不够"时的阶段3 可选增强。

**构件 3：远景板（兜底，默认几乎不出场）**
- 常量深度平面（`z_bg = 2.5·zFar`）、纹理 = 源图强模糊 + 压暗延展；只在前两层都没覆盖时兜底
  （实测端点帧占比 **0.0%**——裙边 + 背景层已足够；保留它作为极端参数/退化输入的保险）。
- 可选开关 `bgAnchor`：换非重复程序晶格纹理（历史"近景特征锚"配方）——仅在真机三角化不足时启用（§3.5）。

### 3.4 视频轨道与位姿上报

- **轨道（默认：绕中心、相机始终看向中心）**：
  ```
  中心 C0 = (0, 0, -d0)
  θ_i = -Θ + i·(2Θ)/(N-1) , i = 0..N-1     // 默认 Θ = 8°，N = 73 → 含 θ=0 的参考帧
  C_i = ( d0·sinθ_i , 0 , -d0·(1 - cosθ_i) )
  R_cw,i = R_y(θ_i)（列 = [right | up | -fwd]）
  R_wc,i = R_y(-θ_i)
  上报四元数 rotation = [x,y,z,w] = ( 0, -sin(θ_i/2), 0, cos(θ_i/2) )   // 世界→相机
  ```
  - 自检：`θ=0 → C=(0,0,0)、R_wc=I、四元数=(0,0,0,1)`（判据脚本断言）。
  - 备选轨道 `orbitMode = fixedOrientation`（相机不转、只在弧上平移）：公式见附录 A；作为 A/B 旋钮。
- **上报约定（【实测·待复核】）**：`rotation` 为**世界→相机**；相机轴为 **AR 系**。
  → **首次联调必须做 A/B 复核**：`poseFamily ∈ {AR, CV}` × `invertQuat ∈ {true,false}` 四组合扫描
  （既有诊断入口保留，默认锁死命中组合）。判读：5 秒内失败 = 组合错；跑满 80 秒以上 = 候选。
- **时间戳**：`timestamp = round(i · 1e9 / 30)`（30fps 等间隔，纳秒）——只需单调、等间隔（Kit 不用它优化）。
- **帧数 N 与 Θ 是观感旋钮**：默认 N=73（**含 θ=0**，让原照片视角成为一帧，利于保真）、Θ=8°。

### 3.5 画面构成：3:4 裁剪 + 是否需要"特征锚"（验证驱动）

- **主方案（默认）**：源照片**智能裁剪到 3:4**（优先保主体，复用既有主体分割蒙版的包围盒做裁剪中心引导）。
  照片在参考位姿**铺满整帧**，画面里没有程序纹理。这是与旧路线（照片只占两成画面）的根本差别。
- **为什么无需程序纹理锚（待验证）**：旧路线需要"照片外一圈程序纹理"是因为照片是**共面海报**（单应退化风险）；
  本路线照片自带**跨深度的非共面结构**，且背景板提供"另一个深度层"【实测·待复核：旧路线的共面教训】。
- **开关 `bgAnchor`（默认 off）**：若阶段 2 真机出现三角化不足（`Too few points` / `no 3D points` 且已排除位姿约定与位移量级），
  按序启用：① 背景板加**低幅非重复颗粒**（给 SfM 特征，肉眼近乎不可见）；② 背景板换**程序晶格纹理**。
- 照片边缘的"甩出"区域由背景板承接（§3.3），不需要把照片做边缘延展贴图。

### 3.6 位移量级守卫（本路线唯一的"成败参数"）

> ⚠️ **口径纠正（2026-10-01）**：历史文档《坑 126》表格的位移数字 ≈ **精确投影值的 2 倍**（口径不一致）。
> 本节与判据脚本一律用**精确针孔投影**（同一点在两帧的落点差值，§3.1 的投影式）。
> 按精确口径重算历史实测：**成功轮**（主体 z=1.53、d0=2、8° 关键帧步长）峰值 **≈70px**；
> **失败轮**（z=1.36）**≈100px**；远景 z=8 的 8° 步长 ≈158px（已超匹配窗）。

**位移公式（绕中心轨道；推导见附录 A）**：`Δu(θ, z) ≈ fx·θ·(z−d0)/z`
- 枢轴平面 `z = d0` 处不动；离枢轴越远位移越大；`z=zNear` 与 `z=zFar` 方向相反（相对视差）。
- 相邻关键帧步长位移（Kit 自选、按 5 关键帧保守估算 `Δθ_kf = 2Θ/(5−1) = Θ/2`）**用精确投影在画面内扫点取最大值**
  （判据脚本 `stepScan()` 实现）：**最近层取守卫对象**。

**守卫界与默认参数**：
- `stepNear ≤ 80 px`（取在既有成功 70px 与失败 100px 之间）；`slip = stepNear − stepFar ≥ 20 px`
  （2026-10-02 随 FX 750/对角弧重标定，旧 25px 系 fx=1500 纯水平弧口径；深度可分性由 (z−d0)/z
  杠杆决定、与轨道形态无关）。**新增下界 `stepNear ≥ 30 px`**（坑 131 真机标定：fx=700 时
  Θ2°≈12px → `no 3D points`、Θ3°≈18px 模型碎、Θ5°≈31px 可建；实装侧配套 Θ 自适应钳制）。
- **默认 Θ = ±5°（对角弧 Φ=Θ，坑 131 修复）**、k = 3.0、zNear = 1.0、d0 = 2.0、N = 73、
  **FX = 750（2026-10-02 起与产品默认一致，旧 1500 口径停用）** →
  **stepNear ≈ 48.7px、stepFar ≈ 72.0px、slip ≈ 23.3px** ✓。

> **轨道形态（坑 131，2026-10-02 定案）**：纯水平 look-at 弧（仅 yaw）对 (纵向尺度 Y, fy)
> **精确退化**——`v_i = cy − fy·Y/depth_i(X,Z,θ_i)`，Y 只在分子 → (Y→αY, fy→fy/α) 下所有帧
> 所有像素不变（数值复现 0.00px）→ Kit BA 沿退化方向随机漂移（真机 refined fy/fx = 0.155/2.47/2.69，
> 成片宽度随机压缩/拉伸）。**修复 = 对角弧**（yaw+pitch 同步各扫 ±Θ，相机中心离开 y=0 平面），
> 同变换下像素变化 ≈398px（可观测），投影级非退化守卫已入判据 [4]。

**参数扫描（判据脚本实测口径，fx=750，对角弧 Φ=Θ）**：

| Θ（半角=Φ） | stepNear | stepFar | slip | 端点单轴位移 | 判定 |
|---|---|---|---|---|---|
| ±3° | 28.5px | 40.6px | 12.1px | ≈39px | ❌ 低于位移下界 30px（坑 131：fx700/Θ2° 同量级真机失败） |
| ±4° | 38.5px | 55.8px | 17.3px | ≈52px | ✅ |
| **±5°（默认）** | **48.7px** | **72.0px** | **23.3px** | **≈65px** | ✅ |
| ±6° | 59.1px | 89.3px | 30.2px | ≈79px | ✅ 临界（端点位移贴裙边 80px） |
| ±8° | 80.9px | 127.6px | 46.6px | ≈105px | ❌ 超界（stepNear 超 80 且端点超裙边） |

（历史 fx=1500 纯水平弧扫描表作废；判据脚本 `--theta=` 可重扫。）

- **真机排除顺序**：`STAGE02/03` 失败 → 先查 A/B 位姿约定（§3.4），再查守卫（超界先降 Θ 到 4°）；
  成功后可逐级上调 Θ（4→5→6）换取更强视差。
- 端点点检（判据已实现）：轨道两端帧 **前景照片占比 ≥ 70%**（实测 90.6%）、
  **消隐带由背景层承接**（实测 9.4%）、**远景板 ≤ 10%**（实测 0.0%）。

---

## 4. 合成管线（native，逐帧做什么）

**输入**：源图 RGBA（ArkTS 解码后传入，含 w/h）+ 参数（§附录 B）。
**输出**：N 帧 1080×1440 RGB 紧凑缓冲，逐帧 `PushFrame`。

```
每帧 i：
  1) 若 i 是 θ=0 的参考帧 → 直接输出"参考帧"（3:4 裁剪后的原图缩放结果，逐像素复制；不经过网格）
  2) 否则：
     a) 清屏（背景板前的画布置为背景板颜色兜底，正常不漏）
     b) 光栅化背景板（4 顶点投影 → 覆盖测试 → 采样背景纹理）
     c) 光栅化照片深度网格（z-buffer；顶点位置 = SrProject(世界点, pose_i)，相机数学只来自 bridge）
     d) 自检：统计覆盖率（应有 100%）、"照片占比"（照片像素/总像素）、行跨距断言(== 1080*3)
  3) 组装 HMS_SpatialRecon_DataFrame（内参 §3.1 / 位姿 §3.4 / 时间戳 / imageData / RGB）
```

**参考帧恒等**：θ=0 帧 = 源图 3:4 裁剪的等比缩放结果（1080×1440），逐像素复制（**判据 [2] 断言 0 偏差**）。
这条保证"系统最终看到的原点视角就是用户的照片本身"。

**代码落点与"单源公式"纪律**（遵循《新增功能开发指南》第一节判定）：
- **相机/射线/位姿/内参公式仍只有一份**，留在 `spatial_recon_bridge.cpp`；对外提供
  `SrProjectWorldToPixel(P_world, pose_i, &u, &v, &depth)` 与 `SrPixelRayDir(u, v, &dir)` 两个入口。
- **新增 native 单元 1**：`spatial_recon_depth_mesh.cpp` —— 深度归一化、引导滤波上采样、网格构建、
  背景板光栅化、帧合成。**内部不得出现第二份射线/投影公式**（判据 [7] 源码级守卫断言：该文件不出现 `fx`/`fwd`/
  `basis` 之类关键符号的独立推导，只调用上面两个入口）。
- **新增 native 单元 2**：`mslite_depth_runner.cpp` —— MindSpore Lite 封装（加载/推理/销毁；与相机无关）。
- **CMakeLists**：加两个源文件 + `mindspore_lite_ndk` + `rawfile.z`。
- **日志**（沿用 SRCHK 风格，一条都不能少）：
  `SRD load model size=… sha=… backend=…`、`SRD infer in=546x728 out=1x1x728x546 ms=…`、
  `SRD depth p1=… p99=… mean=… valid=1|0`、`SRD mesh grid=270x360 verts=… k=… zNear=… d0=… thet=…`、
  `SRD frame i/N photo_ratio=… cov=100% miss=0`、`SRD push i/N`、`SRD kit stage=… progress=…`。

---

## 5. Kit 会话编排（native，与已有实现同层）

```
① HMS_SpatialRecon_IsSupport(SPATIAL_RECON_MODEL_TYPE_GS) → 不支持：直接降级/提示（绝不 dlopen 之外的硬链接）
② 新建全新空会话目录（el1/base 下 /sr_<ts>/，生成前清理历史遗留）
③ HMS_SpatialRecon_CreateSession(GS, workPath, &session)
④ 合成 + 逐帧 HMS_SpatialRecon_PushFrame(frame, &dataFrame)   // 顺序推进，失败即中止并报错
⑤ 注册回调（推荐 26.0.0 的 NGCallback）：
   - StartSession 之前注册 → 收"重建完成"；writeInfo 非空 → 自动保存
   - StartSession 之后注册 → 收"保存完成"
   （旧式 HMS_SpatialReconCallbackFunc 保留兼容，二选一，不要同时用）
⑥ HMS_SpatialRecon_StartSession(session, &writeInfo{modelFile=沙箱 MP4 路径, modelFormat=MP4, audioFile=nullptr}, cb)
⑦ HMS_SpatialRecon_SetRunningMode(session, FOREGROUND)
⑧ 进度：轮询 HMS_SpatialRecon_GetProgress(session, &p, &stage)（0~0.3 = 我方合成，0.3~1.0 = Kit 重建+保存）
⑨ 暂停/继续/终止：PauseSession/ResumeSession；终止 = 置标志 + Pause，落到 PAUSED 后 Destroy（保存阶段不可中断）
⑩ 成功后取产物 MP4 路径 → 返回 ArkTS → 保存图库 → 引导"设为壁纸"
```

错误码翻译（SDK 头文件枚举【SDK】）：`801` 设备不支持 / `1023700002` 工作目录无效 / `1023700003` 帧数据无效 /
`1023700005` 会话已开始 / `1023700007` 重建失败 / `-5` 空帧（我方自检拦截）/ `-6` 已终止 / `-100` 有任务在跑。

---

## 6. 五层接入清单（本功能）

| 层 | 动作 | 自查 |
|---|---|---|
| ① native | 新单元 `spatial_recon_depth_mesh.cpp` + `mslite_depth_runner.cpp`；相机数学单源在 bridge | `grep` 新文件内无第二份射线公式；§4 日志齐 |
| ② napi | 新异步入口 `spatialReconGenerateFromDepth`（Context/Execute/Complete/Async 四件）+ `depthProbe` + `depthEstimateTest`（阶段 1 自检用）+ 独立进度全局 `g_srDepthProgress` + 注册表三行 | 入口名与 d.ts 一致；旧入口**不动** |
| ③ d.ts | `entry/src/main/ets/types/libglassrender.so.d.ts` 增 3 个 `declare function` 与 options 接口 | 选项字段与 napi 解析一致 |
| ④ Service | 新 `SpatialReconDepthService.ets`（解码→裁剪→调用→进度/暂停/终止/降级）；**旧 Service 不动** | `grep` 旧字段引用清零 |
| ⑤ CMakeLists | 加 2 个源文件 + `mindspore_lite_ndk` + `rawfile.z` | 语法自检（clang++ -fsyntax-only，见指南 §2.5） |

**页面与产品流**：
选图（复用既有 MediaPickerService + fd 解码，规避相册 URI 直传坑）→ 「生成 3D 影像」→ 进度（合成段/重建段）→
完成页「保存到图库」→ 提示在相册中"设为壁纸"。
- 文案纪律：只说"这是什么/当前状态/下一步"，**不写深度、网格、视差、换算等实现细节**；
- 耗时提示：进度条 + 一句"约 1~2 分钟，请保持前台"（SetRunningMode=前台）；
- 诊断开关（位姿 A/B 扫描、Θ、k、`bgAnchor`、`orbitMode`）：默认隐藏且锁死默认值，产品化前移除。

**降级链**（失败不阻断，保证出片）：
```
D1 深度网格 + 背景板（本文件主路径）
  ├─ 模型加载失败 / 深度无效 / 位移守卫超界 → D2
D2 既有已跑通的"整图贴面"立体场景（兜底；其几何须按本文档 §3 口径核对一遍再用）
  └─ 再失败 → D3 报错文案（按 §5 错误码翻译）
```

---

## 7. 验收

### 7.1 离线判据（**已实装**：`tools/depth_mesh/depth_mesh_check.js`；运行手册 `tools/depth_mesh/README.md`）

> 现状：合成模式 **PASS 18 / FAIL 0**（[7] 在 C++ 未实装时为 SKIP）；C++ 已实装后**总计
> 28 PASS / 0 FAIL**（合成 21 + 源码守卫 7）。判据是 C++ 实装的验收口径。

| # | 判据（实装名） | 断言与实测值 |
|---|---|---|
| [1] | 深度方向（越大越近）与分布 | 合成场景：主体 > 远墙；真实模式：天空区中位 < 地面区中位（无天空照加 `--skyskip=1`）；`p1/p99` 跨度有效 |
| [2] | 参考位姿恒等 + **网格结构守卫** | **台阶带外 \|Δ\|≤1 占比 ≥ 99.9%（实测 100.000%，maxΔ=1）**；台阶带 ≤ 3%（实测 0.58%）；**quad 四角 z 比 ≤ 1.2（实测 1.009，证明无跨台阶连接面）** |
| [3] | 位移量级守卫 | `stepNear ≤ 80px`（实测 66.0）；`slip ≥ 25px`（实测 35.6）；**渲染器位移 == 投影公式（实测误差 0.19px）** |
| [4] | 非共面守卫 | `k ≥ 2.0`；`stepFar ≥ 5px`（实测 30.4） |
| [5] | 三层占比（轨道两端） | 前景 ≥ 70%（实测 90.6%）／背景层 ≥ 1%（实测 9.4%）／远景板 ≤ 10%（实测 **0.0%**） |
| [6] | 覆盖率与帧缓冲口径 | 无未写入像素；紧凑 RGB、长度 == 宽×高×3（C++ 侧同款：行跨距 == 宽×3） |
| [7] | 实装一致性（源码级） | 解析 `spatial_recon_bridge.cpp`：常量 `kDepthZNear/kDepthRatio/kDepthThetaDeg` 与判据脚本一致、实装入口存在；**坑 129 三条守卫：`SrBakeDepthMeshWorld` 存在（世界点一次性固化）／构建调用传入 `kOutW/kOutH`（网格坐标映射画布空间）／`quadToScreen` 体内不得出现 `basis`（禁止逐帧重建世界点）**；单源纪律：纯 2D 单元不得出现 `rayDir\|frameBasis\|basis[\|SrPixelRay\|kFaceZ\|kOutW` |
| [8] | 出图肉眼核验 | `tools/ui_shots/depth_mesh/`：`dm_ref`（θ=0）/ `dm_depth`（深度着色）/ `dm_bg_fill`（背景层补洞）/ `dm_end_pos`、`dm_end_neg`（两端对翻看视差） |
| — | 参数扫描表 | Θ=4/5/6/8° 的 stepNear/stepFar/slip/总位移（输出末尾，供选档与真机标定） |

### 7.2 真机日志检索清单

```bash
# 本应用侧
grep -E "SRD (load|infer|depth|mesh|frame|push|kit)" <hilog>
# Kit 侧（必须全量去重看，不能只 grep error）
grep "OHOS::RS" <hilog> | sed -E 's/[0-9]+(\.[0-9]+)?/N/g' | sort | uniq -c | sort -rn
# 分水岭
grep -E "DenseInit Adaptive|Empty cameras or points|Too few points" <hilog>
```

**判读**：5 秒内失败 = 位姿约定（查 §3.4 A/B）；秒级 STAGE03 失败 = 位移量级（查 §3.6 守卫）；
跑满 80 秒 + 关键帧 added 数 = selected 数（否则查会话目录残留）= 成功候选。

### 7.3 人工验收
出片后目视：① 视差方向正确（近物与远景反向滑动）；② 无重影/拉伸/翻转；③ 参考视角与照片一致；
④ 保存图库后**能被系统"设为壁纸"**（本路线的唯一产品级验收）。

---

## 8. 风险与未决项

| # | 风险 | 应对 |
|---|---|---|
| R1 | ~~参考 App 下线 = 效果/成本警示~~ **已澄清（2026-10-01）**：其下线原因是**安装包体积**（用户确认），非效果问题 | 本路线同样**不随包**：模型用时下载（§2.3-5）；效果验收仍照常做（阶段2） |
| R2 | `.ms` 转换：**已实测定案（2026-10-01）** —— 本模型静态 shape 转换被 converter 限制卡死（`Reshape→Conv` 格式推断失败），**只能动态 shape** | 分发动态 `.ms` + 端侧 `OH_AI_ModelResize` 固定 1×3×728×546；阶段1 首要验证项；若真机也报 format 问题 → ONNX 图手术（`Reshape→Transpose→Conv` 改写） |
| R3 | NHWC/NCHW 布局踩错（不报错、结果全错） | §2.3 纪律 2 + 阶段 1 与 PC fp32 结果比对锁定 |
| R4 | 深度伪影（天空/大虚化/细结构） | 边缘引导滤波 + 幅度限制（Θ/k 守卫）+（可选）主体蒙版强制分层；低对比度图直接降级 |
| R5 | Kit 关键帧自选不可控（K4） | 守卫按"5 关键帧"保守估算；真机失败先降 Θ |
| R6 | 输出运镜幅度未知（Kit 自定路径），背景板可能露得多 | 阶段 2 看片评估；`bg` 纹理/亮度/模糊是旋钮 |
| R7 | 包体与分发 | **已定：不随包**，用时下载（服务端 + SHA-256 校验 + 失败回退；§2.3-5） |
| R8 | 机型门槛（Kirin 9020+，K9） | `IsSupport` 探测 + 降级链 + 文案（不写实现细节） |
| R9 | 许可残留（DA-V2 训练数据血统） | 开源说明页如实列出（Apache-2.0 文本 + 出处 + SHA），法务过目 |

**未决项（需在阶段 2 真机后回收）**：Kit 输出的运镜幅度与时长；`bgAnchor` 是否必要；`orbitMode` 命中哪种；
Θ/k 的最终产品档位；是否需要 LaMa 级补全（阶段 3 再评估）。

---

## 9. 分阶段实施计划

| 阶段 | 内容 | 完成判据 |
|---|---|---|
| **0 · 离线（零设备）** | ① 自导出 ONNX（固定 1×3×728×546）+ 记录 SHA；② WSL 转 `.ms`（fp32+fp16）+ 逐算子核对；③ PC 参考推理出深度图（onnxruntime）；④ 写 `tools/depth_mesh_check.js` 判据 [1]~[8] 并出图 | 判据全绿 + 出图肉眼通过 |
| **1 · 端侧推理** | ✅ **代码已落地（2026-10-01，待构建+真机）**：`mslite_depth_runner.cpp`（MindSpore Lite 加载/resize/推理/统计 + 48×32 ASCII 深度缩略图进日志）+ napi 双入口 `depthProbe` / `depthEstimateTest` + d.ts + `SpatialReconService.depthSelfTest` + 页面「深度自检」按钮；模型随包 `rawfile/models/da_v2_small_fp16_dyn.ms`（49.5MB，SHA-256 `3a72a9f2…`） | 真机日志 `SRD selftest ok … code=0`；用日志缩略图跑 `compare_device_depth.py --dev-ascii` 判据 **r ≥ 0.95 且 平均绝对差 ≤ 0.08**（本地已用 PC 参考自测通过：r=0.9797 / mad=0.0356） |
| **2 · 合成+Kit** | ✅ **真机跑通（2026-10-01 复测）**：`SRCHK depth mesh ok … canvas=1080x1440 …` → 72 帧 `miss=0` → Kit **STAGE01→04 全 completed** → `[ENCODER] Video Encoding Completed.` → `onFinished status=0` → `done …/model_*.mp4`；**无降级行**（= scene=5 产物）；验收双检 `fmid`≈`src`（Δ≤1 **98.0%**）、`f000`≠`fmid`（不同格 **58.9%**）。首测曾命中两个实装缺陷 → 已修 + 判据 28 PASS / 0 FAIL（含 3 条新源码守卫），native 三文件语法自检绿 | 真机出片 ✅ + 可设为壁纸（待人工确认）；合成帧 ASCII 与离线渲染对齐（见 §9） |
| **3 · 产品化** | UI/文案/降级链/热管理收尾；NPU 后端试算；包体策略；开源说明条目；耗时与功耗实测 | 自查单 + 开源合规 |

**阶段1 真机验证清单（构建安装后按序执行）**

1. 入口：「3D 壁纸」页 → 选一张照片 → 「**深度自检（路线B·阶段1）**」按钮（诊断入口，产品化时随「适配选项」整块移除）。
2. 期望状态行/日志：`SRD selftest ok … code=0 src=WxH in=546x728 out=546x728 layout=NCHW min=… max=… mean=… buildMs=… inferMs=… bin=…`
   —— 失败签名：`code=-1` 模型构建失败 / `-2` resize 失败 / `-3` 推理失败 / `-5` 落盘失败（`note=` 带原因）。
3. **取证只走日志**（设备 shell 取不到应用沙箱，`/data/app/.../files` Permission denied）：
   ```bash
   hdc shell hilog -x | grep -E "SRD (probe|model|selftest|depth)" > srd.log
   ```
4. 判据比对（本地已用 PC 参考自测通过：r=0.9797 / mad=0.0356）：
   ```bash
   python tools/depth_mesh/compare_device_depth.py --dev-ascii srd.log --ref tools/depth_mesh/out/p1
   ```
   期望 `[2] r ≥ 0.95` 且 `[3] 平均绝对差 ≤ 0.08`；同时肉眼核验日志里 32 行 ASCII 缩略图（近=亮、主体应亮于天空/远墙）。
5. 记录进本节：`inferMs`（CPU 单次推理耗时，量级预期 0.5~3s）、`layout`（实测张量布局）、包体增量（≈50MB）、
   以及是否需要切 fp32（若 fp16 精度或算子支持有问题）。

**阶段2 真机验证清单（构建安装后按序执行）**

1. 入口：「3D 壁纸」页 → 选一张照片 → 「场景」选到第 6 档「**深度网格**」→ 点「按当前选项生成并保存」。
2. 期望日志序列（tag 依次为 `SR3D`（ArkTS）/`SRD`（推理）/`SRCHK`（合成与 Kit）/`GSRECON`（Kit））：
   ```bash
   hdc shell hilog -x | grep -E "SR3D (crop3to4|depth for scene5|fallback)|SRD (probe|model|selftest|depth)|SRCHK (depth mesh|push|f000|fmid|building|rebuild|onFinished|done)|GSRECON|OHOS::RS" > s2.log
   ```
   - `SR3D crop3to4 WxH -> W2xH2`（非 3:4 照片被居中裁剪；已是 3:4 则不打印）
   - `SR3D depth for scene5 code=0 … in=546x728 out=546x728 …`（端侧深度推理成功；阶段1 已验证）
   - `SRCHK depth mesh ok depth=546x728 **canvas=1080x1440** orbit=5 k=3 zNear=1 stride=4 skirt=80 front=N bg=M plane=…`（**canvas 字段 = 画布映射生效**）
   - `SRCHK push i/72 … miss=…`（miss 应≈0；**首帧 miss>50% 会直接 -5 失败**）
   - `SRCHK f000/fmid` 的 48×32 ASCII 缩略图：**`fmid`（θ≈0）应 ≈ `src`（源图），且 `f000` 必须与 `fmid` 明显不同**（恒等 + 视差双检；坑 129 的验收点，可用 `tools/depth_mesh/scene5_render_check.js --log=s2.log` 逐格比对）
   - Kit 侧四阶段（`STAGE02` 不得出现 `no 3D points`）+ `SRCHK onFinished status=0` + `SRCHK done …/model_*.mp4` → 保存图库
   - **不得出现** `SR3D fallback ok scene=1 (attempt 1)`（出现即 scene=5 在 Kit 侧又失败）
3. **失败签名与判读**（沿用既有口径）：
   - 5 秒内失败（`STAGE02 no 3D points`）→ 位姿约定（跑「自动扫描」复核 A/B）或位移量级（查 `SRCHK depth mesh ok` 的 orbit/k）；
   - `SRCHK depth mesh build failed`/`depth missing` → 自动退回 scene=1 出片（状态行会说明），查 go 深度是否回读成功；
   - 秒级 `STAGE03 Adapting failed`（点太少）→ 位移量级（把 Θ 从 5° 降到 4°）。
4. **与离线判据对齐（几何级核验）**：用同一张照片在 PC 上跑
   `ref_depth.py --nocrop`（复刻端侧路径）→ `depth_mesh_check.js --input=<前缀>`（渲染三层场景并出 `dm_*.png`），
   把设备日志里的 `SRCHK f000/fmid` ASCII 与离线渲染帧的 ASCII 逐格比对（首次应作为**回归证据**记录）。
   **工具：`tools/depth_mesh/scene5_render_check.js`**（一份脚本两种语义：`cpp`=首测实装语义（回归用）、`fix`=判据口径），
   `--log=<hilog 文本>` 直接出逐格比对统计与差异图。
5. 记录进本节：出片耗时、产物规格（时长/分辨率/码率）、观感（视差方向、有无条纹/鬼影）、`miss` 曲线。

**阶段2 真机首测（失败）与修复（2026-10-01，真机首测）**

| 项 | 设备实测 / 判读 |
|---|---|
| 前置链路 | `SR3D crop3to4 809x1440 -> 809x1079` → 端侧深度 `code=0 src=809x1079 in=546x728 out=546x728 inferMs=2544` ✅ |
| 网格构建 | `SRCHK depth mesh ok depth=546x728 orbit=5 k=3 zNear=1 stride=4 skirt=80 front=62412 bg=15040 plane=35x46` ✅ |
| 推帧 | 72 帧 `push … miss=0`（`f000`=θ−5°、`fmid`=θ≈0.07°）✅ |
| Kit | `selected` 6 个关键帧，但 19:15:38.9 `[GSRECON][STAGE02] no 3D points`（×4）→ `STAGE03 Adapting failed` → `SRCHK rebuild failed cb=1023700007`（**距推帧完成 4.4 s**）❌ |
| ArkTS | `SR3D scene=5 failed (spatialReconTestGenerate failed, code=1023700007), next attempt` → `SR3D fallback ok scene=1 (attempt 1)`（scene=1 正常出片） |

**根因（两条实装↔判据口径差，离线逐格复现定死 —— 详见坑 129）**：

1. **世界点逐帧按当前机位重建 → 零视差**：`SrRasterizeDepthMesh::quadToScreen` 用当前帧 `basis` 从像素坐标反投影世界点，
   代数上顶点投影恒等于其像素坐标（u ≡ q.x）→ **72 帧画面逐帧完全相同**，只有上报位姿在动 → Kit 特征匹配无基线 → 0 点。
2. **深度场 546×728 坐标被直接当画布 1080×1440 坐标用 → 参考位姿不恒等**：照片缩在画幅左上角 546×728、其余露出模糊远景板
   （`SRCHK fmid lum avg=55` vs 源图 `88`）；远景板 UV 另有一处未除 1/16 模糊因子。

**取证（`scene5_render_check.js` + 同源照片 `out/dev5*`，完整输出 `out/scene5_repro_dev5.log`）**：

| 比对项 | 结果 |
|---|---|
| 设备 `f000` / `fmid` vs **"现状语义"复刻** | **1499/1536、1500/1536 格相同**（残差 = fp16↔fp32 深度 + 贴图重采样）→ 根因定死 |
| "现状语义" `f000` vs `fmid` | **1498/1536 格相同** → 零视差实锤 |
| "修复语义" `fmid` vs 源照片 | Δ≤1 占 99.5%；判据口径（台阶带外恒等率）**99.954% PASS** |
| "修复语义" `f000` vs `fmid` | 差异 905/1536 格 → 视差恢复 |
| 位移守卫 | 最近层相邻关键帧 65.4px ≈ 判据 66.0px |

**修复内容**（native 三文件 `-fsyntax-only` 全绿；判据 **28 PASS / 0 FAIL**，新增 3 条源码守卫）：

- `SrBakeDepthMeshWorld(mesh, fx, fy, cx, cy)`：世界点**构建时一次性固化**（参考位姿反投影 `P=(a·z,b·z,−z)`）；逐帧只做投影。
- `SrBuildDepthMesh(..., outW, outH, ...)`：角点与贴图 UV 一律乘 `outW/dw、outH/dh` 映射到**画布空间**；`skirt` 单位改为**画布像素**；
  前景贴图按**画布分辨率**从全分辨率源图单步双线性重采样（不再经 546 中转），背景层补洞结果升采样到画布分辨率。
- 远景板 UV 按 `planeW/kOutW`（≈1/16）归一化。
- 判据 [7] 新增守卫：`SrBakeDepthMeshWorld` 存在 / 构建调用传入 `kOutW/kOutH` / `quadToScreen` 体内不得出现 `basis`。

**首测照片的一个非阻塞项**：`depth_mesh_check.js --input=out/dev5f` 在该 3:4 夜景人像上判
`台阶带占比 ≤ 3%` **FAIL（4.60%）**（守卫原按合成场景标定；语义是"深度台阶处 1px 平片的视差量化"，属观感级）；
其余 22 项 PASS（含参考位姿恒等 99.954%、端点三层占比、覆盖率 100%）。待复测确认整体观感后再决定是否调 `REFINE_RATIO`。

**阶段2 真机复测（修复后，✅ 通过；2026-10-01，真机复测，同一张 3:4 夜景人像）**

| 项 | 设备实测 / 判读 |
|---|---|
| 前置链路 | `SR3D crop3to4 809x1440 -> 809x1079` → `SR3D depth for scene5 code=0 … in=546x728 out=546x728 inferMs=2522` ✅ |
| 画布映射生效 | `SRCHK depth mesh ok depth=546x728 **canvas=1080x1440** orbit=5 k=3 zNear=1 stride=4 skirt=80 front=53202 bg=15040 **plane=68x90**`（front 少 9210 = 画布口径 skirt 80px 换成深度场 40px；plane 68×90 = 画布/16）✅ |
| 推帧 | 72 帧 `push … miss=0`（19:33:46.6→19:33:58.1，≈11.5 s）✅ |
| Kit | 关键帧 `selected` 5（`not selected` 65）；**`STAGE01`→`STAGE02`→`STAGE03`→`STAGE04` 全部 `completed`**，无 `no 3D points`、无 `Adapting failed` ✅ |
| 产物 | `[ENCODER] Video Encoding Completed.` → `SRCHK onFinished status=0` → `SRCHK done …/spatial_recon_work/model_1790854425851.mp4` ✅ |
| 无降级 | 日志中**无** `SR3D fallback` / `scene=5 failed` / `rebuild failed` ⇒ 出片即 scene=5 产物（非 scene=1 降级）✅ |
| 耗时 | 会话 ≈ 87 s（STAGE01 0.33s / STAGE02 1.15s / STAGE03 4.5s / **STAGE04 76.4s** / 编码 ~4s）；端到端含深度与推帧 ≈ 103 s |
| **验收双检**（`scene5_render_check.js [D]`，设备日志内部比对） | ① 参考位姿恒等 `fmid` vs `src`：**Δ≤1 占 98.0%**（1382 格完全相同）；② 视差存在 `f000` vs `fmid`：**不同格 58.9%**（首测 ≈2.5%）✅ |
| 与离线修复语义对齐（`[C]`） | 设备 `fmid` vs fix 语义 **1445/1536 相同**（Δ≤1 98.4%）；设备 `f000` vs fix **1314/1536**（端点帧视差最大，故略低）；设备 vs 首测"现状语义"仅 255/294 相同 ⇒ 实装语义确已切换 ✅ |

**结论**：修复生效——参考位姿恒等恢复、视差恢复、Kit 四阶段跑通并出片，**阶段2 的机器判据全部通过**；
剩余的是**人眼环节**（观感 + 可设 3D 壁纸）与参数标定（Θ 5°→6°）/产品化收尾。

**本轮遗留（2026-10-01 复测用户反馈）**：

1. ~~成片"上下颠倒 + 人物跑到背景后面"~~ **✅ 已闭环（2026-10-01 三次真机）**：判据链与机理见下，`worldFlip180`（世界绕世界 X 轴 180°，**只转上报位姿**）**已锁为默认开**，实测「空间朝向：翻转」→ 正确。
   证据：设备日志内部比对 `fmid` ≈ `src` Δ≤1 **98.0%**（输入正立自洽）+ "人物在最后、背景在前方" = 深度镜像 ⇒ 模型被绕水平轴翻 180°（合法旋转，可补偿）；Kit 侧线索 `[VideoGenerator] gravityCoordinate` + `SpatialGltfModelPacker`/`GaussianModelCompressor` 负责模型打包。补偿的几何依据：`p' = (R·B)ᵗ(R·P−R·C) = p`（投影不变，图像无需改）。
2. **成片"比例失真：水平方向（宽度）被压缩"**——**已定位到 Kit 渲染层（2026-10-01 第四轮真机日志取证）**。要点：输入链七项排查全部自洽（真裁切/深度 3:4/画布 1080×1440/内参渲染=上报/`fmid≈src`/宽高上报正确）；**Kit 输出固定 1080×1920（9:16，编码器 Configure 明文）**、渲染相机 `cameraInitParams` 数值被 hilog 隐私过滤；成片首帧 vs 输入帧 2D 仿射对齐实测 **sx/sy=0.464（非等比横向压扁实锤）**。~~旧判读框架（文件级 vs 图库级 + 各向异性预拉伸补偿）~~**已废弃（不做面向结果的补偿）**。修复走根源路线，实验矩阵：E1 scene=1 对照 / E2 `hilog -p off` 读 cameraInitParams / E3 焦距旋钮（已加「适配选项·焦距」700/1100/1500 档）/ E4 圆点标定图；按机理分岔落地：**输入构图对齐**（照片按目标比例在输入帧内放置）或焦距定值或场景几何调整。
   本轮已核实的排除项：输入链内参自洽（渲染=上报同组 fx/fy/cx/cy；SDK 头文件实证 focal 为像素单位），
   `fmid≈src 98.0%` ⇒ 压缩在 Kit 输出/呈现链或观感层，**不在合成帧**。
3. **观感差（能看出原图但观感差）**——同类应用的常见技术栈（DA-V2 Small + MindSpore Lite + OpenCV Zoo LaMa）
   与本方案只差**补洞/修复**这一层（本方案用自研 push-pull）：把 **LaMa 从阶段3 可选提级**（离线对比先做），
   另可配合 Θ 5°→6° 增强视差。

**阶段1 真机首测结果（2026-10-01，真机首测）——主问题已闭环 ✅**

| 项 | 设备实测 |
|---|---|
| 运行时探针 | `SRD probe rc=0 mindspore-lite ok, device=CPU` |
| 模型加载（随包 rawfile，47MB） | `model built in 149 ms` |
| **动态 shape → resize** | 输入张量 `shape=[-1,3,-1,-1] fmt=0 dataSize=0` → **`after resize: shape=[1,3,728,546] dataSize=4769856`**（=1×3×728×546×4，精确） |
| 推理 | `selftest ok code=0 src=809x1440 in=546x728 out=546x728 **layout=NCHW** min=0.0000 max=5.8043 mean=2.2093 buildMs=149 **inferMs=2616**` |
| 深度内容 | 32 行 ASCII 缩略图结构正确（顶部全暗=天空/远景、中部亮块=近处主体、左下极亮块=极近物体）→ **"越大越近"约定在设备上成立** |

**结论**：`.ms` 可加载、动态 shape 可 resize、推理产出合理深度图 —— 阶段1 的核心问题（"能不能在设备上跑起来"）**已闭环**。

**次测留下的两条待办（非阻塞）**：

- **数值级交叉比对 ✅ 已完成（2026-10-01）**：用同源照片（`tools/depth_mesh/out/dev_photo.jpg`，9:16 夜景人像）复刻端侧路径
  （`ref_depth.py --nocrop --filter bilinear` 于 809×1440）后比对：
  **r = 0.9956（阈值 0.95）✓ / 归一化平均绝对差 = 0.0266（≤0.08）✓ / 差≤0.05 占比 87.0%（≥70%）✓**；
  同图"完美自测"上限为 r = 0.9969 ⇒ **残差只剩 ASCII 十级量化的取整**。
  → 设备 fp16 `.ms` 与 PC fp32 ONNX 输出一致，**转换无损、端侧推理链路正确**（另：raw 量程 5.8043 vs 5.807 三位小数级吻合）。
  ⚠️ **过程教训（坑 124 同族，记入判据脚本注释）**：首轮 r=0.936 是**判据脚本自身的采样语义错**——
  比对脚本用双线性把 PC 参考缩到 48×32（等于额外均值滤波），而端侧 ASCII 是**点采样**（`sx=tx*W/48` 取整）。
  判据复现端侧采样规则后 r 立刻到 0.9956；**"数值对不上"先查判据自己的口径，再怀疑两端实现**。
- **native 日志的数值被隐私过滤成 `<private>`**（格式串里数值未加 `%{public}`）：统计值在 ArkTS 侧 `console.info` 行完整可见，不影响判读；
  后续若要让 native 行自足，把数值先 `snprintf` 进字符串再以 `%{public}s` 输出（不在本次改动内）。
- 另记：`MS_LITE` 打了 `cpu's architecture is unknown.`（E 级）+ `set arch failed, ignoring arch.` —— 无害（跳过 CPU 架构特化），NPU 路径留阶段3。
- **阶段2 输入口径**：自检目前是"整帧压缩到 3:4"（只为验证推理链路）；正式管线按 §3.5 **先裁 3:4** 再送模型。

---

## 10. 合规与署名

- **上线必带**：Depth Anything V2 Small（Apache-2.0，© Depth Anything 团队）、MindSpore Lite（Apache-2.0，
  © Huawei，作为系统部件通常无需随包分发但**署名照列**）的许可文本与出处；模型转换产物记录 SHA-256。
- 开源说明页（App 内"开源说明"）按条目列出：名称 / 许可证 / 项目主页 / 固定版本或 SHA——与截图里那款
  App 的做法一致，这是 Apache-2.0 的署名义务，也是商用尽调的证据链。
- **非法律意见**：训练数据血统（DA-V2 合成数据+自标注）属残留风险，建议法务过目后再上架。

---

## 附录 A · 公式与推导

**(A1) 绕中心轨道（pivot-facing）的像素位移推导**
相机在 θ 处位置 `C(θ) = (d0 sinθ, 0, -d0(1-cosθ))`，姿态 `R_cw = R_y(θ)`。
世界点 `P = (0,0,-z)`（光轴上深度 z）：
```
P - C = ( -d0 sinθ , 0 , -z + d0(1-cosθ) )
p = R_y(-θ)·(P - C)  →  p.x = sinθ·(z - d0) ,  p.z = -(z-d0)cosθ - d0
u_offset = fx · p.x / (-p.z) = fx · sinθ·(z-d0) / [ (z-d0)cosθ + d0 ]
θ→0 时 ≈ fx·θ·(z-d0)/z          // z=d0 处位移为 0（枢轴）；z<d0 与 z>d0 反向
```
**(A2) 备选轨道 fixedOrientation（相机不转）**：`R_cw = I`，位置同上 →
```
Δu(θ,z) = -fx · d0 · sinθ / z      // 全部内容同向平移，位移 ∝ 1/z，无枢轴
```
**(A3) 关键帧步长位移（守卫口径）**：`Δθ_kf = 2Θ/4 = Θ/2`（假设 5 关键帧，保守）：
```
|Δu_kf|(z=zNear) = fx · (Θ/2) · (k-1)/2
|Δu_kf|(z=zFar)  = fx · (Θ/2) · (k-1)/(2k)
|Δu_total|(zNear) = fx · Θ · (k-1)          // 整段轨道
```
**(A4) 恒等性**：θ=0 → R_wc=I、C=0 → `p = P`，投影回 `(u,v)` = 原像素（判别式上严格成立；实装以判据 [2] 断言）。

**(A5) 对角弧（2026-10-02，坑 131 修复；替代 A1 的纯水平弧）**：
offset = `d0·(sinθ·cosφ, sinφ, cosθ·cosφ)`，θ 与 φ 同步各扫 ±Θ（Φ=Θ）；姿态仍 look-at 中心
（right = normalize(fwd×worldUp)、up = right×fwd，φ≠0 时 up 带俯仰分量）。φ=0 时与 A1 逐位一致。
A1 的位移公式对两轴各自仍适用（x 轴用 θ、y 轴用 φ），总位移取合成。**动机**：A1 的纯水平弧下
`v_i = cy − fy·Y/depth_i`，Y 只在分子 → (Y→αY, fy→fy/α) 下所有像素不变（精确退化）→ Kit BA
沿该方向随机漂移（refined fy/fx = 0.155/2.47/2.69，成片宽度随机压缩/拉伸）；对角弧下同变换
像素变化 ≈398px（可观测），投影级非退化守卫已入判据 [4]。

## 附录 B · 参数总表（默认值）

| 参数 | 默认 | 说明/范围 |
|---|---|---|
| 帧规格 | 1080×1440 RGB 紧凑 | Kit 硬约束 K1/K2，不可变 |
| fx = fy / cx / cy | 750 / 540 / 720 | 虚拟相机（2026-10-02 起 = 产品默认；旧 1500 口径停用。fx≠750 不改变比例失真结论，见坑 131） |
| N / Θ / Φ / orbitMode | 73 / ±5° / Φ=Θ / pivot-facing **对角弧** | N 为奇数以含 θ=0；对角弧 = yaw+pitch 同步各扫 ±Θ（坑 131 修复，打破 (Y, fy) 精确退化）；实装侧 Θ 另有位移带自适应钳制（守护深度 = s 的 p90，坑 133：30~80px + 裙边 + zNear 160px 硬顶，`SRCHK orbit clamp` 日志） |
| zNear / k / d0 | 1.0 / 3.0 / (zNear+zFar)/2 | k∈[2,4]，守卫 §3.6 |
| 网格 stride / 台阶细分 | 4（**深度场像素**）/ 四角 z 比 > 1.15 → 1px 小片（四点同深） | **绝不跨台阶连接**（结构守卫断言 ≤ 1.2） |
| 网格裙边 SKIRT | 80px（**画布像素**，采样钳制延展） | 承接轨道端点的画幅边缘空隙（无裙边时端点露出远景板 ≈13%）；单元内部换算深度场像素 |
| 画布映射 | 角点/UV = 深度场像素 × `outW/dw、outH/dh` | **深度场 546×728 ≠ 画布 1080×1440**；不映射 → 照片缩在左上角（坑 129-②） |
| 世界点 | **构建时一次性固化**（`SrBakeDepthMeshWorld`，参考位姿反投影） | 逐帧只做投影；逐帧重建 → u ≡ q.x → 零视差（坑 129-①，判据源码守卫） |
| 前景贴图 | 画布分辨率，双线性取自**全分辨率源图** | 不经深度场中转（避免二次模糊）；与 scene=1..4 的逐像素采样同款 |
| 背景层 | 近区 s > 0.5，膨胀 24px 内生成，深度 ×(1−2e-3) tie 偏置 | push-pull 补洞（顶层兜底 + 双线性 push），只在该区生成 |
| z_bg（远景板） | 2.5·zFar；半尺寸 7.4×9.6 | 兜底；2026-10-02 随 fx=750 放大（画幅四角×轨道端点的对角射线精确需求 7.24×9.43，旧 4.4×7.2 为 fx=1500 窄视场口径，对角弧端点会露黑角） |
| 关键帧位移守卫 | stepNear ∈ [30, 80] px、slip ≥ 20 px | 精确投影口径（下界 30px 系 2026-10-02 坑 131 真机标定新增；slip 25→20 随 FX 750 重标定） |
| 照片占比守卫 | ≥ 70%（轨道两端） | 防背景层/远景板喧宾夺主（实测 90.6%） |
| 深度输入 | 546×728（NCHW） | 14 的倍数、恰 3:4 |
| 深度归一化 | p1/p99 → [0,1] | 稳健；低对比度判无效 |
| 位姿上报 | 世界→相机四元数，AR 系 | 首次联调 A/B 复核 |
| 照片占比守卫 | ≥ 70%（轨道两端） | 实测 90.6%；远景板 ≤ 10%（实测 0.0%） |

## 附录 C · 事实来源

- 【官方】Spatial Recon Kit《重建三维场景（C/C++）》(1080×1440 / RGB / 自动关键帧 / SetRunningMode /
  Kirin 9020+ / 热事件 / Pause/Resume / 保存 / NGCallback)：
  https://developer.huawei.com/consumer/cn/doc/harmonyos-guides/spatial-recon-c-spatial-recon-pipeline
- 【官方】《管理 Spatial Recon 会话》(workPath 语义 / 销毁约束)：
  https://developer.huawei.com/consumer/cn/doc/harmonyos-guides/spatial-recon-c-spatial-recon-session
- 【官方】MindSpore Lite Kit 简介（系统内置 / 开发方式 / NNRt 关系）：
  https://developer.huawei.com/consumer/cn/doc/harmonyos-guides/mindspore-lite-kit-introduction
- 【官方】MindSpore Lite 模型转换（converter_lite 参数 / inputDataFormat 默认 NHWC / 关融合 / 下载包）：
  https://developer.huawei.com/consumer/cn/doc/harmonyos-guides/mindspore-lite-converter-guidelines
- 【官方】MindSpore Lite 算子支持列表：
  https://developer.huawei.com/consumer/cn/doc/harmonyos-guides/mindspore-lite-supported-operators
- 【官方】MindSpore Lite 端侧推理（C/C++ 骨架 / CMake 链接 mindspore_lite_ndk / rawfile 读取）：
  https://developer.huawei.com/consumer/cn/doc/harmonyos-guides/mindspore-guidelines-based-native
- 【官方】文档更新记录（2026-07-28 空间照片功能下线、Spatial Recon Kit 文档删除）：鸿蒙开发者知识库 `doc-updates`
- 【SDK】`<DevEco SDK>/default/hms/native/sysroot/usr/include/spatial/spatial_recon_interface.h`（结构体/枚举/函数清单）
- 【SDK】`<DevEco SDK>/default/openharmony/native/sysroot/usr/include/mindspore/*.h` 与
  `...\usr\lib\aarch64-linux-ohos\libmindspore_lite_ndk.so`（Native API 符号与链接库实证）
- 【官方/社区】Depth Anything V2 仓库与许可（Small=Apache-2.0；其余 CC-BY-NC-4.0）：
  https://github.com/DepthAnything/Depth-Anything-V2 ；输出为"相对逆深度、越大越近"（官方 issue #93 等交叉印证）
- 【本机实况·2026-10-01 探测】`github.com` 不可达（000）→ 权重走 `hf-mirror.com`（200）；
  converter 官方只有 **Linux-x86_64** 包（Windows 包实测不存在，全系列；本机 WSL 不可用，
  **故转换改在 Linux 服务器执行**——见下条）；本机 Python 未安装（使用便携版 Python 替代）；
  PyPI 清华/阿里源可达。工具落点：`tools/depth_mesh/`（`depth_mesh_check.js` / `export_onnx.py` /
  `ref_depth.py` / `convert_ms.sh` / `README.md`）。
- 【实测·2026-10-01 服务器转换记录】转换在 **Linux 服务器**（Ubuntu 22.04 / x86_64）完成，
  转换进程用 `ulimit -v 3000000` + `nice -n 19` 限幅；避免在生产机跑 benchmark 类重型验证
  （实测 benchmark 加载动态 .ms 内存占用高且长时间不收敛，已终止）。
  产物与指纹（动态 shape 版）：
  - `da_v2_small_728x546_dyn_fp32.ms` 98,971,520 B，SHA-256 `8027ca82976e3d0d7c7b8e28e5db47abf1e28b681e4b06f39f2cc520448e05c6`
  - `da_v2_small_fp16_dyn.ms` 49,549,824 B，SHA-256 `3a72a9f21f16f5fee08147dfdb767e4247679b29edd37e71b2a243fafd02efd7`
  - converter 包 SHA-256 `8bb10971…`（与官方文档一致）；ONNX SHA-256 `afb6a5c2…`（与 HF LFS 记录一致）。
