/**
 * libglassrender.so · 3D 壁纸接口声明（摘录）
 *
 * 本文件为 native 库中 3D 壁纸相关接口的 TypeScript 声明部分，供 ArkTS 侧
 * （SpatialReconService / Wallpaper3DPage）调用。接入方式见 README（五层接入说明）。
 */

/**
 * 3D壁纸 · Spatial Recon Kit 管线（spatialReconTestGenerate）。
 * 把"单张图片 + 已知虚拟相机位姿"合成的多视角帧序列喂给端侧 3DGS 重建，
 * 完成后保存 MP4 并返回产物路径；preset 模式下帧改由 spatialReconPresetPush 灌入。
 * 设备不支持（非旗舰芯片等）时 reject。
 */
export interface SpatialReconTestOptions {
  /** 会话工作目录（必须已存在，且在应用文件目录下） */
  workPath: string;
  /** 输出 mp4 沙箱绝对路径（同样须在应用文件目录下） */
  modelPath: string;
  /** 源图 RGBA_8888 像素（srcWidth*srcHeight*4 字节）。**preset 模式不传**（帧改由
   *  spatialReconPresetPush 逐帧灌入） */
  rgbaBuffer?: ArrayBuffer;
  /** 源图宽（px） */
  srcWidth: number;
  /** 源图高（px） */
  srcHeight: number;
  /** 虚拟相机轨道半角 ±Θ（度，钳 1~45）。scene=5 默认 5；诊断可覆盖 */
  orbitDeg?: number;
  /** 合成帧数（默认 72；native 侧钳 [2,240]）。更多帧 = 给 Kit 更多关键帧候选（重建稳定性），
   *  推帧耗时随帧数线性增长；默认值改动须带真机对比证据 */
  frameCount?: number;
  /** **D1/D4 深度边缘（默认 1=增强）**：1 = JBU RGB 引导上采样（深度边缘贴颜色边缘，
   *  网格角点密度不变）；2 = 保边预平滑 + JBU（再消细碎台阶）；0 = 关（旧行为）。
   *  日志 `SRDM edge mode=`；离线判据 tools/depth_mesh/depth_edge_check.js。
   *  ⚠️ 本档改喂给 Kit 的帧统计，真机记录过观感风险（局部拉伸带/拉丝），默认变更须同图 A/B。 */
  depthEdge?: number;
  /** **D3 自适应纵深（默认 1=自适应 k∈[2.2,3.8]）**：按深度直方图分离度取 k，
   *  d0 恒 2 公式（相机数学零改动）；0 = 固定 k=3（旧行为）。日志 `SRCHK adaptive k=` */
  adaptiveK?: number;
  /** **D2 主体对齐（默认 1=开）**：用 subjectMask 做蒙版门控保边平滑（主体内部抑噪）；
   *  需同时传 subjectMask（Service 在 scene=5 + subjectAlign>0 时自动构建，识别不到主体
   *  则静默跳过）；0 = 关（旧行为） */
  subjectAlign?: number;
  /** 虚拟相机焦距 px（默认 750，钳 200~6000；主点=画面中心，零畸变）。
   *  750 为典型广角主摄先验值（比例失真的根因是轨道退化而非焦距，见 docs/3d-wallpaper-archive.md F6） */
  focalPx?: number;
  /** **各向异性内参声明（默认 1.0）**：上报 focalY = focalYratio × focalX。
   *  只给**预置路线**用（照片路线恒各向同性）。背景：Kit 的 BA 无锁内参，refined 纵横比是
   *  内容相关漂移（跨素材实测 0.3~3.5），"模型纵横比 ≈ B·ρ_d/ρ_r"（见 docs/3d-wallpaper-archive.md）
   *  ⇒ 帧不动、只把声明按 1/B 反向偏置即可把纵横比拉回 1。
   *  native 钳 [0.3, 3.0]，越界按 1.0 处理。 */
  focalYratio?: number;
  /** 位姿约定：false=相机→世界；**true=世界→相机（默认 = 2026-10-01 真机验证命中组合）** */
  invertQuat?: boolean;
  /** 诊断 A/B：画面绕视轴滚 180°——默认 false（实测无法修正模型级翻转，保留作对照）。 */
  roll180?: boolean;
  /** **默认 true**（坑 132）：推帧前交换 R/B。该开关会改变成片的颜色表现，但成因**尚无定论**
   *  （见档案 F7）——默认值与 Service 层的"失败自动反交换重试"属工程折中，不代表结论；
   *  传 false 可手动回到直通 RGB。 */
  bgrSwap?: boolean;
  /** **诊断 A/B（默认 false）**：交换**位置**——true 时在"深度之后、贴图/网格构建之前"
   *  对源缓冲整体交换一次 R/B（native 内部用副本，`SRCHK src` 缩略图与 ArkTS 侧深度/判据
   *  仍看自然色），推帧口**不再**交换（与 bgrSwap 互斥；Service 保证，native 侧另有防呆）。
   *  用于验证"交换位置是否影响结果"这一 A/B 问题。 */
  swapRgbAtInput?: boolean;
  /** **同会话双产物（默认 false，Service 侧开启）**：MP4 自动保存完成且过比例门控后，
   *  用官方 SaveResultToFile 追加保存一份 PLY（modelPath 的 .mp4 后缀换成 .ply）。
   *  动机：spatialEdit.saveToPLY 对 MP4 容器加载的节点真机恒失败；PLY 是模型几何/颜色的
   *  唯一可解剖产物。失败只打日志（`SRCHK ply`），不影响 MP4 交付。 */
  alsoPly?: boolean;
  /** **默认 true**：世界绕世界 X 轴转 180°（up→−up、前后→后前）——只转上报位姿、合成帧不变
   *  （模型整体旋转、重建等价）。抵消 Kit 模型导出/输出环节的 180° 翻转（成片上下颠倒 +
   *  人物跑到背景后面 = 深度镜像）；2026-10-01 真机二次复测确认为正确朝向。
   *  显式传 false = 回到 Kit 原始朝向（对照用）。 */
  worldFlip180?: boolean;
  /** 上报相机系：0=AR 系（X右/Y上/Z后，**默认且已验证命中**，ARKit/AR Engine 同款）；
   *  1=CV 系（X右/Y下/Z前，OpenCV）。两者是同一点像素一致的两种坐标标签，可安全 A/B。 */
  poseFamily?: number;
  /** 合成场景：1=立体（主盒正面贴源图 + 副盒/侧面/远景用不可重复程序纹理，默认）；
   *  3=分层深度（需 subjectMask；蒙版缺失/退化时自动退回 scene=1）；
   *  4=盒中空间（几何与 3 同构、主盒正面放大到铺满画幅）；
   *  5=深度网格（单图深度 → 三层网格，本仓库主路径，需传 depthBuffer）。 */
  scene?: number;
  /** 主体蒙版（scene=3/4 用）：Uint8 一维数组，长度须 = srcWidth*srcHeight；
   *  0=背景、255=主体、中间值=主体概率（subjectSegmentation.mattingList 语义）。
   *  由 ArkTS 侧主体分割（Core Vision Kit）产出；缺失/尺寸不符自动退回 scene=1。 */
  subjectMask?: ArrayBuffer;
  /** 主体层深度比（scene=3/4 用）：主体平面深度 / 背景深度，<1 = 主体更近。
   *  默认 0.90（钳 0.60~0.95）。⚠️ 受"可匹配位移量级"约束：取值过小（如 0.80）会因
   *  位移超出已验证可匹配区间导致稠密初始化失败。 */
  subjectDepth?: number;
  /** 输入布局 A/B：true=按 4 字节/像素（RGBA，Alpha=255）上报，默认 RGB 3 字节/像素 */
  rgbaInput?: boolean;
  /** 深度网格（scene=5）：端侧深度估计的**原始相对逆深度**（float32 LE，越大越近；
   *  长度须 == depthWidth*depthHeight；由 depthEstimateTest 产出后再随参数传入） */
  depthBuffer?: ArrayBuffer;
  /** 深度网格宽（px，通常 546） */
  depthWidth?: number;
  /** 深度网格高（px，通常 728） */
  depthHeight?: number;
  /** **预置建模（预置数据集路线，默认 false）**：true 时不从源图合成帧，改为消费
   *  spatialReconPresetPush 灌入的预置帧（RGBA 1080×1440，队列容量 3 流式过界）；
   *  位姿由 presetTheta/presetRadius 闭环给出。rgbaBuffer/subjectMask/depthBuffer 等
   *  合成参数在本模式下全部忽略。 */
  preset?: boolean;
  /** 预置帧逐帧环绕角 θ（度）：帧数 = 数组长度（覆盖 frameCount，native 钳 ≤240）；
   *  θ=0 = 数据集参考帧。来源 = 数据集 manifest.json（离线生成器产出） */
  presetTheta?: number[];
  /** 环绕半径（世界单位；球半径归一 1，须 >1 且与帧内视半径/焦距自洽——由离线几何测量
   *  得出，非先验估值） */
  presetRadius?: number;
  /** 自转轴方向（世界系单位向量 3 值；缺省 (0,1,0) = 水平环绕）。声称位姿绕该轴整圈环绕：
   *  basis_i = R_A(θ_i)、pos_i = R_A(θ_i)·(0,0,radius)。轴倾角打破"相机 up 恒定 ⇒ (纵向尺度,
   *  fy) 精确退化"。⚠️ 必须与离线数据集同式同值。 */
  presetAxis?: number[];
  /** 参数域 ②：相机自身绕 right 轴的俯仰角（度，[-89, 89]；缺省 = 不用，走 presetAxis 轴式域；
   *  两域互斥，同传报错）。位姿 = 平环（绕 +Y 整圈环绕球心）+ 相机俯仰本角，公式与数据集
   *  manifest.poseConvention 逐句对应（SrPresetFlatPitchBasis）。 */
  presetPitchDeg?: number;
  /** **对角弧域（缺省 false = 关）**：true 时相机**位置**同时扫 yaw 与 pitch
   *  （pos.y ≠ 0，φ ≡ θ）——照片路线 scene=5 `SrFrameBasis`（Φ=Θ）的同口径移植，
   *  即"比例压缩"的破退化解。平环（含仅姿态俯仰）下 (纵向尺度, fy) 是**精确不可观测量**
   *  （Y 只出现在分子），Kit 无锁内参 API ⇒ 其 BA 沿该零梯度山谷随机漂。
   *  数据集配对字段 = manifest.poseDomain === 'diag-arc'。
   *  与 presetAxis / presetPitchDeg **互斥**（同传报错）。 */
  presetDiagArc?: boolean;
}

/**
 * 端侧重建管线验证。
 * @returns 成功 resolve 输出 MP4 路径；失败 reject 错误描述（含错误码）
 */
export declare function spatialReconTestGenerate(options: SpatialReconTestOptions): Promise<string>;

/** 尝试灌入一帧预置帧（preset 模式的帧来源；RGBA 1080×1440）。**非阻塞**——灌帧循环在
 *  UI 线程上，队列满时立即返回 1，调用方应 await 退让后重试（不要忙等）。
 *  @returns 0=已入队；1=队列满（稍后退让重试）；-1=参数空；-6=已终止/消费端离开（停止灌帧） */
export declare function spatialReconPresetPush(rgbaBuffer: ArrayBuffer): number;

/** 预置帧生产失败（解码错误等）：放行 native 消费端使其失败退出。@returns 0=已放行 */
export declare function spatialReconPresetFail(): number;

/** 探测设备是否支持空间重建：0=支持；-1=能力库绑定失败；801=设备不支持 */
export declare function spatialReconProbe(): number;

/** 获取最近一次管线进度（0.0~1.0），供 ArkTS 轮询 */
export declare function getSpatialReconProgress(): number;

/** 获取最近一次/进行中重建的 Kit 阶段（HMS_SpatialReconStage：0=INIT / 1=BUILDING /
 *  2=PAUSED / 3=FINISHED / 4=SAVING / 5=UNKNOWN；-1=本次任务尚未开始），供 UI 显示真实阶段 */
export declare function getSpatialReconStage(): number;

/**
 * 暂停进行中的重建（官方 PauseSession）。仅"已开始且未结束"的会话有效。
 * @returns 0=成功；-1=无活动会话/未绑定；其余=Kit 状态码
 */
export declare function spatialReconPause(): number;

/** 继续已暂停的重建（官方 ResumeSession）。@returns 同上 */
export declare function spatialReconResume(): number;

/**
 * 终止进行中的重建：请求终止 + 立即暂停重计算，由运行线程在阶段落定后销毁会话。
 * ⚠️ 保存阶段不可中断：该阶段调用会被忽略，产物照常保存并返回成功
 * （UI 须先用 spatialReconInSave 判定并禁用终止）。
 * @returns 0=已请求终止；-1=未绑定；其余=Kit Pause 状态码
 */
export declare function spatialReconAbort(): number;

/**
 * 是否处于"保存收尾"阶段（1=是；0=否）——UI 的暂停/终止按钮可用性判定依据。
 * ⚠️ 不能用 Kit 的 stage 值替代：保存期 Kit 可能报 FINISHED(3) 而非 SAVING(4)，
 * 会导致按钮看着可点、点下去无效（用户感知"点了没反应"）。
 */
export declare function spatialReconInSave(): number;

/**
 * 前后台切换 → 更新运行模式（官方规则：每次 StartSession 之后必须调用 SetRunningMode，
 * 应用切前后台时也要更新）。无会话/未绑定能力库时为 no-op。
 * @returns 0=no-op 或 Kit 状态码
 */
export declare function spatialReconSetForeground(foreground: boolean): number;

// ========== 端侧单图深度估计（MindSpore Lite + Depth Anything V2 Small）==========

/** 端侧深度自检入参 */
export interface DepthSelfTestOptions {
  /** 源图 RGBA8888 像素（紧凑缓冲，行跨距必须为 width*4） */
  rgbaBuffer: ArrayBuffer;
  width: number;
  height: number;
  /** 产物前缀：写 <outPrefix>.depth.f32.bin（原始相对逆深度 float32）与 <outPrefix>.meta.txt */
  outPrefix: string;
  /** 资源管理器（读随包 rawfile 模型用；缺省则回退 modelPath） */
  resMgr?: Object;
  /** rawfile 内模型相对路径（默认 models/da_v2_small_fp16_dyn.ms） */
  modelName?: string;
  /** 沙箱模型路径（可选，rawfile 不可用时的兜底） */
  modelPath?: string;
}

/**
 * MindSpore Lite 运行时探针（不加载模型）：验证设备上推理运行时可用。
 * @returns "rc=0 mindspore-lite ok, device=CPU" 形态的字符串（rc≠0 为失败）
 */
export declare function depthProbe(): string;

/**
 * 端侧深度估计：RGBA → 缩放 + ImageNet 归一化 → resize 到 1×3×728×546 → 推理 → 落盘统计。
 * @returns 成功 resolve 统计串（含 in/out 尺寸、layout、min/max/mean、耗时、bin 路径）；
 *          失败 reject 同款统计串（含 code 与 note，便于日志定位）
 */
export declare function depthEstimateTest(options: DepthSelfTestOptions): Promise<string>;
