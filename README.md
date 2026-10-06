# 单图 → 3D 影像壁纸（HarmonyOS · Spatial Recon Kit）

把一张普通照片转成系统可识别的 **3D 影像**（可保存到图库并设为壁纸）。

**这是"证明链路可行"的归档开源**：核心管线已在真机完整出片（重建四阶段全部完成、
约 87 秒、无降级 fallback），同时存在明确的技术局限。请先读技术档案
[`docs/3d-wallpaper-archive.md`](docs/3d-wallpaper-archive.md) —— 它包含全部技术结论、
14 条避坑发现（F1~F14）、可复现参数与未竟事项清单。

## 技术路线

```
相册选图
  → 解码 + 居中裁 3:4
  → 端侧单目深度估计（Depth Anything V2 Small + MindSpore Lite，546×728 相对逆深度）
  → 三层深度网格场景（纯 2D 构建）:
       ① 照片网格：逐像素反投影为三维网格（不跨深度台阶连接 + 80px 裙边）
       ② 背景层：近区 push-pull 补洞（承接主体滑开后露出的消隐带）
       ③ 远景板：兜底平面
  → 72 帧多视角合成（自报内参与位姿；参考位姿帧逐像素等于原照片）
  → 逐帧 PushFrame → 端侧 3DGS 重建 → MP4（= 系统"3D 影像"）
  → 保存图库
```

**本方案存在的意义**：官方 SDK 只提供"多视角图像 + 位姿 → 3DGS"的重建接口，
没有"单图直接输入"的入口；本仓库用自建的"深度 → 网格 → 多视角帧"管线，
把单张照片构造为该 Kit 可接受的输入。

## 目录结构

| 路径 | 内容 |
|---|---|
| `docs/3d-wallpaper-archive.md` | **技术档案（先读）**：结论摘要 / 链路与真机证据 / 参数总表 / F1~F14 关键发现 / 已解改进方向 / 局限 / 未竟事项 / 许可 |
| `docs/3d-wallpaper-route-b-implementation.md` | 实现文档：逐段设计、公式推导（附录 A/B） |
| `docs/spatial-recon-kit-reference.md` | Spatial Recon Kit 参数/模式全表（含未文档化项清单） |
| `src/cpp/` | native 层：`spatial_recon_bridge`（Kit 桥 + 相机数学单源）/ `spatial_recon_depth_mesh`（纯 2D 网格）/ `spatial_recon_layers`（分层深度）/ `mslite_depth_runner`（端侧深度推理） |
| `src/ets/` | ArkTS 层：`SpatialReconService`（编排）/ `Wallpaper3DPage`（示例页面）/ `GallerySaver`（保存图库）/ `libglassrender-3d.d.ts`（接口声明摘录） |
| `src/cpp/CMakeLists.snippet.txt` | 编译接入片段（源文件与链接库） |
| `tools/depth_mesh/` | 离线工具链：判据脚本（零依赖 Node）+ PC 参考推理脚本 |
| `tools/sr_frames_check.js` | 场景合成帧判据（scenes 0-4） |

## 快速开始

### 1. 离线判据（无需设备）

```bash
node tools/depth_mesh/depth_mesh_check.js     # 合成场景几何/位移/输出判据
node tools/sr_frames_check.js                 # scenes 0-4 合成帧判据
```

判据脚本中包含**源码级守卫**：会解析 `src/cpp/` 下的实现文件核对常量与关键结构
（见档案 F14 的说明——这是本项目的重要方法论）。

### 2. PC 参考推理（复刻端侧深度链路）

需要 ONNX 格式的 Depth Anything V2 Small 模型（见下方"依赖"；本仓库不附带模型文件）：

```bash
python tools/depth_mesh/ref_depth.py --onnx <模型.onnx> --image <照片> --prefix out/p1
node tools/depth_mesh/depth_mesh_check.js --input=out/p1   # 用真实深度跑判据
```

### 3. 端侧移植（五层接入）

| 层 | 动作 | 参考 |
|---|---|---|
| ① native | 将 `src/cpp/*.cpp` 加入编译（见 `CMakeLists.snippet.txt`）；`vg_common.h` 为本仓库提供的最小日志头 | `src/cpp/` |
| ② napi | 按 `libglassrender-3d.d.ts` 声明注册入口（`spatialReconTestGenerate` / `depthEstimateTest` 等，异步四件套） | d.ts + `SpatialReconService` 的调用方式 |
| ③ d.ts | 将 `libglassrender-3d.d.ts` 内容并入你的 native 声明文件 | — |
| ④ ArkTS | `SpatialReconService` + 页面（示例页面依赖的系统资源引用需按你的工程调整） | `src/ets/` |
| ⑤ CMake | 链接 `libmindspore_lite_ndk.so` 与 `librawfile.z.so`；**Spatial Recon 能力库必须运行期 dlopen，不得进链接清单** | `CMakeLists.snippet.txt` |

## 核心实现要点（避坑速览）

> 说明：源码注释中的"坑 N"为开发期错误记录编号，其结论均已收编于技术档案的 F1~F14 节。

> 完整版见档案 F1~F14；以下是最容易踩的几条。

1. **图像与位姿必须自洽**（射线 `b = -(v+0.5-cy)/fy`，第三项取负）；场景必须有 ≥2 个深度层。
2. **位移量级守卫**：相邻关键帧总位移 30~80px（精确针孔投影口径），低于 30px 三角化点不足、
   高于 80px 超出匹配窗。
3. **禁止纯水平轨道**：存在"纵向尺度与 fy 精确不可观测"的退化 → 使用对角弧（yaw+pitch 同步扫）。
4. **世界点构建期一次性固化**：逐帧反投影会退化为恒等式（零视差）。
5. **每次会话用全新空工作目录**：历史残留会触发 resume 模式静默拒收关键帧。

## 已知局限

重建服务为黑盒（关键帧/训练/运镜不可控）、素材敏感、仅 Kirin 9020+ 与中国
境内、端到端约 100 秒且高功耗。**"设为壁纸"的端到端闭环未在归档前验证**——建议作为接手后
的第一项验证。详见档案 §6。

## 依赖的开源项目

| 项目 | 用途 | 许可 | 地址 |
|---|---|---|---|
| Depth Anything V2（**仅 Small**） | 端侧单目深度 | Apache-2.0（Base/Large/Giant 为 CC-BY-NC，勿商用） | https://github.com/DepthAnything/Depth-Anything-V2 |
| MindSpore Lite | 端侧推理运行时（系统内置，无需随包） | Apache-2.0 | https://github.com/mindspore-ai/mindspore-lite |
| OpenCV Zoo · LaMa（可选） | 遮挡区域结构化补全 | Apache-2.0 | https://github.com/opencv/opencv_zoo （`inpainting_lama`） |

**模型文件不随本仓库分发**（体积与许可原因）。模型获取、转换（ONNX → `.ms`）与端侧部署
的完整规程见实现文档 §2.3 与档案 F13。

## 许可

本仓库代码以 [Apache-2.0](LICENSE) 许可发布。文档中引用的官方接口与文档版权归华为所有；
使用 Spatial Recon Kit 须遵守华为开发者协议。
