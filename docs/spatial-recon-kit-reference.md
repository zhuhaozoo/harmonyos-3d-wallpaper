# Spatial Recon Kit 参数与模式全表（参考）

> **用途**：3D 壁纸方案所用空间建模服务的参数/模式清单。
> **来源标记**：
> - 【官方】= 华为开发者知识库原文（文中标注文档名，均为 2026-10-01 在线核对）
> - 【SDK】= SDK 头文件实证（`<DevEco SDK>/default/hms/native/sysroot/usr/include/spatial/spatial_recon_interface.h`）
> - 【实测】= 真机实测（HarmonyOS 6.1.0(23)+ / Kirin 9020 系列）
> - 【未文档化】= 官方文档**没有**规定的项——用到的场合均以实测/判据确定
> 最后更新：2026-10-01。

---

## 0. 服务的两大形态（先分清）

| 形态 | 能做什么 | 接口 | 起始版本 |
|---|---|---|---|
| **重建（Reconstruction）** | 多视角图像 + 相机内外参 → **3DGS 模型** → 保存 PLY / MP4（运镜视频） | **仅 C API**（`spatial_recon_interface.h`，`libspatial_recon_ndk.z.so`） | 6.1.0(23) |
| **渲染/加载（spatialRender）** | 加载 3DGS 模型（**MP4 / PLY / GLB**）并渲染；含 4 个预置滤镜效果 ID | ArkTS（`@kit.SpatialReconKit`） | 6.0.1(21) |
| **编辑（spatialEdit）** | 选择/变换/上色/删除高斯点、撤销、**提取 3D 主体**、另存 PLY | ArkTS | 26.0.0 |

【官方】《Spatial Recon Kit简介》《ArkTS API》《spatialRender》《spatialEdit》
**注意**：重建侧（本项目用的那半）**没有 ArkTS API**——ArkTS 只有渲染与编辑。渲染侧另有分块加载（`TiledGSNode`，26.0.0+，用于大规模场景按需加载瓦片）。

---

## 1. 重建侧·全部模式（枚举）

### 1.1 模型类型 `HMS_SpatialReconModelType`
| 枚举项 | 说明 |
|---|---|
| `SPATIAL_RECON_MODEL_TYPE_GS` | 3DGS 模型（**当前唯一支持**） |

### 1.2 输出格式 `HMS_SpatialReconOutputFormat`
| 枚举项 | 说明 |
|---|---|
| `SPATIAL_RECON_OUTPUT_FORMAT_PLY` | PLY 点云 |
| `SPATIAL_RECON_OUTPUT_FORMAT_MP4` | MPEG-4 运镜视频（本项目用这个） |

### 1.3 运行模式 `HMS_SpatialReconRunningMode`
| 枚举项 | 说明 |
|---|---|
| `SPATIAL_RECON_RUNNING_FOREGROUND_MODE` | 默认；前台执行分配更多资源、重建更快 |
| `SPATIAL_RECON_RUNNING_BACKGROUND_MODE` | 后台；系统优先处理前台应用，重建较慢 |

**规则**【官方】：每次 `StartSession` 之后必须调用 `SetRunningMode`；应用切前后台时也要更新。
**规则**【SDK】：必须在 `StartSession` **之后**、重建完成之前调用；非活动会话期间调用无效果。

### 1.4 阶段 `HMS_SpatialReconStage`
| 枚举项 | 说明 |
|---|---|
| `SPATIAL_RECON_STAGE_INIT` | 初始化（准备资源与环境） |
| `SPATIAL_RECON_STAGE_BUILDING` | 重建中（处理数据、构建 3D 模型） |
| `SPATIAL_RECON_STAGE_PAUSED` | 已暂停 |
| `SPATIAL_RECON_STAGE_FINISHED` | 重建成功完成（可保存） |
| `SPATIAL_RECON_STAGE_SAVING` | 保存中（写文件） |
| `SPATIAL_RECON_STAGE_UNKNOWN` | 未知/不确定 |

### 1.5 图像数据格式 `HMS_SpatialReconImageDataFormat`
| 枚举项 | 说明 |
|---|---|
| `SPATIAL_RECON_IMAGEDATA_FORMAT_RGB` | RGB 三通道（**当前唯一支持**；官方注明"未来可能支持其他色彩空间"） |

### 1.6 状态码 `HMS_SpatialReconStatus`（含错误码）
| 值 | 名称 | 含义 |
|---|---|---|
| 0 | `SUCCESS` | 成功 |
| 801 | `DEVICE_NOT_SUPPORT` | 设备不支持重建 |
| 1023700001 | `EXCEEDS_MAXIMUM` | **超过最大重建帧数**（上限数值官方未给） |
| 1023700002 | `INVALID_WORK_PATH` | 工作路径无效 |
| 1023700003 | `INVALID_FRAME_DATA` | 帧数据无效 |
| 1023700004 | `STAGE_NOT_INITIALIZED` | 会话未初始化 |
| 1023700005 | `STAGE_BUILDING` | 重建会话已开始 |
| 1023700006 | `STAGE_NOT_FINISHED` | 重建会话未完成（此时保存会返回此码） |
| 1023700007 | `FAILED` | 空间重建失败 |

---

## 2. 重建侧·全部输入参数

### 2.1 `HMS_SpatialRecon_DataFrame`（**每一帧**输入，11 个字段）【官方+SDK】

| 字段 | 类型 | 官方说明 | 本项目传入值【实测】 |
|---|---|---|---|
| `focalX` | float | X 轴焦距，单位 px | 750（宽度压缩修复值，见坑 16） |
| `focalY` | float | Y 轴焦距，单位 px | 750（= focalX） |
| `principalX` | float | 主点 X（光心），px | 540（= W/2） |
| `principalY` | float | 主点 Y（光心），px | 720（= H/2） |
| `distortionCoef[8]` | float[8] | 畸变 [k1,k2,p1,p2,k3,k4,k5,k6] | 全 0（合成相机无畸变） |
| `imageWidth` | int32 | 图像宽，px | **1080（硬约束）** |
| `imageHeight` | int32 | 图像高，px | **1440（硬约束）** |
| `position[3]` | float[3] | 相机在 3D 空间的位置 [x,y,z] | 轨道位姿（bridge 计算） |
| `rotation[4]` | float[4] | 相机旋转，四元数 **[x,y,z,w]**（顺序已明示） | 世界→相机四元数【实测命中】 |
| `timestamp` | int64 | 帧时间戳，ns | i×33333333（30fps 等间隔） |
| `imageData` | uint8_t* | 原始像素指针 | 1080×1440×3 紧凑 RGB |
| `format` | 枚举 | 图像格式 | `..._FORMAT_RGB` |

**官方硬约束**【官方《重建三维场景（C/C++）》原文】："当前仅支持输入宽度（width）为 1080 像素且高度（height）为 1440 像素的图像进行重建。**如果输入其余尺寸的图像，结果是未定义的**。"

### 2.2 `HMS_SpatialRecon_ModelWriteInfo`（**保存配置**，5 个字段）【官方+SDK】

| 字段 | 类型 | 官方说明 | 本项目传入值 |
|---|---|---|---|
| `longitude` | float | 地理定位经度，[-180,180]，东正西负 | 0（未用） |
| `latitude` | float | 地理定位纬度，[-90,90]，北正南负 | 0（未用） |
| `audioFile` | const char* | 可选音频路径，**须以 .mp3 结尾**；nullptr=无音频 | nullptr |
| `modelFile` | const char* | **必填**；输出文件名，**必须是应用文件目录的子目录** | 沙箱 `model_<ts>.mp4` |
| `modelFormat` | 枚举 | 输出格式（PLY/MP4） | `..._FORMAT_MP4` |

### 2.3 会话工作目录 `workPath`（CreateSession 参数）
- 【官方】必须是**已经存在的目录**，且必须是**应用文件目录的子目录**（示例 `/data/storage/el1/base/spatial_recon_files/`；"开发者可视情况选取不同的加密等级" el1~el5）。
- 【实测】历史残留会改变会话行为（Kit 进 resume 模式拒收关键帧）→ 每次用全新空目录。

---

## 3. 重建侧·全部函数（13 个）【官方 `spatial_recon_interface.h` 页 + SDK】

| # | 函数 | 作用 | 关键注意事项 |
|---|---|---|---|
| 1 | `IsSupport(type)` | 查设备是否支持 | 返回 SUCCESS / DEVICE_NOT_SUPPORT，**建议先查** |
| 2 | `CreateSession(type, workPath, &session)` | 建会话 | workPath 规则见 2.3 |
| 3 | `DestroySession(session)` | 销毁会话 | **任务在途销毁 = 未定义行为**；销毁后指针失效 |
| 4 | `PushFrame(session, &frame)` | 推一帧（可重复推） | **StartSession 后不能再推**；数据无效返回 FAILED；**系统自动选关键帧** |
| 5 | `PushARFrame(session, arSession, arFrame)` | 推 AR Engine 帧 | 推之前须先更新一次 AR 引擎结果 |
| 6 | `StartSession(session, writeInfo, cb)` | 启动重建（异步） | **writeInfo 非空 → 重建完成自动保存**；为空则之后手动 `SaveResultToFile` |
| 7 | `SetRunningMode(session, mode)` | 设运行模式 | 见 1.3 规则 |
| 8 | `PauseSession(session)` | 暂停 | 任意时刻可用 |
| 9 | `ResumeSession(session)` | 继续 | 会话未被暂停时返回错误 |
| 10 | `GetProgress(session, &progress, &stage)` | 查进度(0~1)+阶段 | 调 `SaveResultToFile` 后 progress 只反映保存进度 |
| 11 | `GetRefinedFrame(session, iFrame, &outFrame)` | 取**重建优化后**该帧的内外参 | **不返回图像像素**（imageData=null）；本方案**尚未使用**——可作"Kit 视角看上报位姿"的对账工具 |
| 12 | `SaveResultToFile(session, writeInfo, cb)` | 手动保存 | 重建未成功时返回 `STAGE_NOT_FINISHED` |
| 13 | `RegisterNGCallbackFunc(session, cb, data)` | 带 void* 数据的回调 | **26.0.0+**；StartSession 前注册=重建完成回调、之后注册=保存完成回调；旧式回调会被覆盖 |

**会话级硬约束**【官方《管理 Spatial Recon 会话》+《重建三维场景》】：
- 同一时刻**只允许一个 session 重建**；同一时刻**只允许一个 session 保存 MP4**；并发 = 未定义行为。
- session 为**裸指针语义**，不保证并发能力；任务完成后手动销毁。
- 关键帧由**系统自动选取**（应用不可控）。
- 仅保证 **Kirin 9020/9030S/9030/9030 Pro 及以后**的体验（其他芯片不保证耗时与质量）。
- 强烈建议订阅热公共事件 `COMMON_EVENT_THERMAL_LEVEL_CHANGED`（过热暂停）。
- 本 Kit 仅支持中国境内；支持 Phone/Tablet/PC-2in1/TV；**不支持模拟器**。

---

## 4. 渲染/编辑侧模式（ArkTS，供产品化参考）

### 4.1 spatialRender（6.0.1(21)+）
- `GSNode`：单个 3DGS 渲染对象（继承 Node，可设 position/scale/visible）。
- `GSPlugin.PLUGIN_ID`：**必须先 `renderContext.loadPlugin` 再调用加载接口**，否则未定义行为。
- `GSPlugin.loadGSNode(scene, {uri, offset}, parent?)`：加载 GS 模型（uri 支持 `OhosRawFile://` 与 `file://`，**空字符串=加载失败**；offset 默认 0）。
- `loadTiledGSNode(scene, {uri}, parent?)`（26.0.0+）：分块加载，清单为 JSON（`scene.json`）；配套 `setCamera` / `setTileRequestCallback` / `notifyTileReady`（瓦片按需加载）。
- **预置滤镜效果 ID**（作为 `createEffect` 的 effectId）：`RETRO_EFFECT_ID`（复古：colorNum/pixelSize/blendEnabled/curve 参数）、`COMIC_EFFECT_ID`（漫画）、`OBRA_DINN_EFFECT_ID`（黑白 bit）、`COLOR_EDITING_EFFECT_ID`（颜色编辑）。

### 4.2 spatialEdit（26.0.0+）
- `GSEdit.editGSNode(node)` → 编辑句柄；选择：`selectBy2DBox` / `selectBy3DBox` / `selectByIndex` / `selectBy2DMask`；`invertSelection` / `clearSelection`。
- 操作：`transform(Mat4x4)` / `paint(Color, PaintMode{REPLACE=0, MULTIPLY=1, ADD=2})` / `remove()` / `undo()`。
- `getRecommended3DBox(ori, dir)`：沿射线取客体 AABB。
- `saveToPLY(uri)`：另存 PLY。
- `extract3DMainBody(pressPoint)`：**按屏幕点提取 3D 主体**（结果覆盖内存中 GSNode）——与"主体分割"产品方向相关。

---

## 5. 官方未规定项（【未文档化】——必须实测，不许编造）

| 项 | 状态 | 已知实测 |
|---|---|---|
| 姿态坐标系约定（世界→相机 / 相机→世界）、旋转四元数手性 | 文档未规定 | 【实测】world→camera 命中（`invertQuat=true`）；相机轴 AR 系（X右/Y上/Z后） |
| position/rotation 的坐标系基准（AR 系 vs CV 系） | 文档未规定 | 【实测】AR 系命中（`poseFamily=0`） |
| 输出 MP4 的分辨率/帧率/码率/编码 | 文档未规定 | 【实测】1080×1920、30fps、6Mbps、HEVC；**封面附图为 720×960 3:4** |
| 关键帧选取策略与数量 | 文档未规定 | 【实测】selected 5~6 个（72 帧输入） |
| 最大重建帧数 | 仅知存在（错误码 1023700001），**数值未给** | 未探到上限 |
| 输出重建的渲染相机内参（focal/画幅适配） | 文档未规定 | 【实测】对输入 fx 有 **≈750** 的固定预期（`cameraInitParams` 日志被隐私过滤，数值不可读） |
| 重建内部算法参数（迭代数、阈值等） | **完全无暴露** | — |
| "3D 影像"（可设壁纸）的容器规范 | 文档未规定 | 【实测】保存的 MP4 即可被图库按 3D 影像处理 |

---

## 6. 本项目映射速查（改代码前看这张）

| 项 | 本方案传入 | 备注 |
|---|---|---|
| 帧尺寸 | 1080×1440 RGB 紧凑（行跨距 = 1080×3，native 断言） | Kit 硬约束 |
| 内参 | fx=fy=**750**、cx=540、cy=720、畸变 0 | 750 = 宽度压缩修复值；改动会同时影响模型形状与视差量级 |
| 位姿 | 世界→相机四元数 [x,y,z,w]；`worldFlip180` 默认开（补偿 Kit 模型 180° 翻转） | 均为【实测】命中配置 |
| 帧序/时间戳 | 72 帧、30fps 等间隔 ns、θ 从 −5° 到 +5°（含 θ=0） | 与产品档位常量一致（Θ=±5°、k=3、zNear=1） |
| 保存 | `writeInfo.modelFormat=MP4`、`modelFile` 沙箱路径 | writeInfo 非空 ⇒ 自动保存 |
| 会话 | 每次生成全新空目录（el2/base 下 `/sr_<ts>/`） | 防 resume 模式 |
| 未用的可用工具 | `GetRefinedFrame`（对账位姿）、`SaveResultToFile`（手动保存）、PLY 输出、spatialEdit 提取主体 | 见 §3/§4 |

---

## 7. 文档索引（本文所有【官方】来源）

- 《Spatial Recon Kit简介》：`harmonyos-guides/spatial-recon-introduction`
- 《重建三维场景（C/C++）》：`harmonyos-guides/spatial-recon-c-spatial-recon-pipeline`（输入约束 1080×1440、关键帧自选、RunningMode、热事件、保存、NGCallback）
- 《管理 Spatial Recon 会话》：`harmonyos-guides/spatial-recon-c-spatial-recon-session`（workPath、销毁规则）
- C API 接口页：`harmonyos-references/capi-spatial-recon-interface-h`（13 个函数、全部枚举）
- 结构体页：`capi-spatialrecon-hms-spatialrecon-dataframe` / `capi-spatialrecon-hms-spatialrecon-modelwriteinfo` / `capi-spatialrecon-hms-spatialrecon-session`
- ArkTS API：`harmonyos-references/spatial-recon-arkts` → `spatial-recon-spatialrender` / `spatial-recon-spatialedit`
- 术语表：`harmonyos-guides/spatial-recon-glossary`（Gaussian Point / Tiled 3DGS）

---

## 8. 代码实现 ↔ 官方文档对照审查（2026-10-01，逐项核过）

| # | 官方要求/能力 | 本方案实现（证据） | 判定 |
|---|---|---|---|
| 1 | 输入仅 1080×1440 | `kOutW=1080/kOutH=1440`（bridge:200-201）；`frame.imageWidth=kOutW/imageHeight=kOutH`（1106-1107）；日志 `canvas=1080x1440` | ✅ 合规（**1080×1920 是 Kit 输出分辨率，非合成帧分辨率**） |
| 2 | 仅 RGB（3B/px 紧凑） | `format=..._FORMAT_RGB`；行跨距断言 1080×3 | ✅（UI 的 RGBA 档本轮已移除；native 诊断参数保留、默认关） |
| 3 | focalX/focalY/principal 像素单位 | fx=fy（滑杆，默认 750）、主点=（W/2,H/2）、畸变全 0 | ✅ |
| 4 | rotation = 四元数 [x,y,z,w] | `SrMatToQuat` 输出即 [x,y,z,w]（bridge:192-194） | ✅ |
| 5 | timestamp ns | `i × 33333333` ns（30fps） | ✅ |
| 6 | 关键帧由 Kit 自选 | 不干预；只推帧 | ✅ |
| 7 | workPath 已存在、应用文件目录子目录 | 每次全新空目录（el2/base 下 `/sr_<ts>/`），防 resume 模式 | ✅ |
| 8 | ModelWriteInfo：modelFile 必填、audioFile .mp3 | MP4 沙箱路径；audioFile=nullptr | ✅ |
| 9 | StartSession 后调用 SetRunningMode | 紧随其后 `FOREGROUND`（bridge:1165） | ✅ |
| 10 | 同一时刻仅一个会话 | `g_srBusy` 门闸 | ✅ |
| 11 | 任务在途不得销毁 | 等完成回调 + 等保存落盘（stage≠SAVING）+ 文件存在校验后才 Destroy；终止路径等 PAUSED（bridge:1234-1255） | ✅ |
| 12 | IsSupport 先查 | 生成前与 probe 均查 | ✅ |
| 13 | 热事件强烈建议订阅 | 已订阅 `COMMON_EVENT_THERMAL_LEVEL_CHANGED`（页面 :131；≥HOT 暂停 / ≤NORMAL 恢复） | ✅ |
| 14 | Pause/Resume 任意时刻 | 已接（可选符号 dlopen，缺符号优雅降级） | ✅ |
| 15 | GetRefinedFrame（官方对账工具） | **本轮接入诊断探针**：保存落盘后读 i=0/mid/last 的优化后内外参 → `SRCHK refined i=… fx=… fy=… cx=… cy=… pos/quat` | ➕ 新增（此前未用） |
| 16 | SaveResultToFile（手动保存） | 未用（writeInfo 非空自动保存即够） | ○ 未用（可选） |
| 17 | PLY 输出 | 未用（产品要 MP4；PLY 可作调试手段） | ○ 未用（可选） |
| 18 | spatialEdit.extract3DMainBody / spatialRender 滤镜 | 未用（属渲染/编辑侧，产品化可评估） | ○ 未用（可选） |

**结论**：实现与官方文档无已知冲突；本轮修掉一处潜在违规（RGBA 输入档）并补上官方对账工具（GetRefinedFrame）。"宽度压缩"排查中一切【未文档化】项仍以真机实测为准。
