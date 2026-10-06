/**
 * libglassrender.so · 3D 壁纸接口声明（摘录）
 *
 * 本文件为 native 库中 3D 壁纸相关接口的 TypeScript 声明部分，供 ArkTS 侧
 * （SpatialReconService / Wallpaper3DPage）调用。接入方式见 README（五层接入说明）。
 */

/**
 * 3D壁纸 · Spatial Recon Kit 管线（spatialReconTestGenerate）。
 * 把"单张图片 + 已知虚拟相机位姿"合成的多视角帧序列喂给端侧 3DGS 重建，
 * 完成后保存 MP4 并返回产物路径。设备不支持（非旗舰芯片等）时 reject。
 */
export interface SpatialReconTestOptions {
  /** 会话工作目录（必须已存在，且在应用文件目录下） */
  workPath: string;
  /** 输出 mp4 沙箱绝对路径（同样须在应用文件目录下） */
  modelPath: string;
  /** 源图 RGBA_8888 像素（srcWidth*srcHeight*4 字节） */
  rgbaBuffer: ArrayBuffer;
  /** 源图宽（px） */
  srcWidth: number;
  /** 源图高（px） */
  srcHeight: number;
  /** 虚拟相机轨道半角 ±Θ（度，钳 1~45）。scene=5 默认 5；诊断可覆盖 */
  orbitDeg?: number;
  /** 虚拟相机焦距 px（默认 750，钳 200~6000；主点=画面中心，零畸变）。
   *  750 为典型广角主摄先验值（比例失真的根因是轨道退化而非焦距，见 docs/3d-wallpaper-archive.md F6） */
  focalPx?: number;
  /** 位姿约定：false=相机→世界；**true=世界→相机（默认，已验证命中）** */
  invertQuat?: boolean;
  /** 诊断 A/B：画面绕视轴滚 180°——默认 false（实测无法修正模型级翻转，保留作对照）。 */
  roll180?: boolean;
  /** **默认 true**：推帧前交换 R/B。开发期的 A/B 试验中该开关会改变成片颜色表现，但成因
   *  **尚无定论**（见档案 F7）——默认值与"失败自动反交换重试"均属工程折中，不代表结论；
   *  传 false 可手动回到直通 RGB。 */
  bgrSwap?: boolean;
  /** **默认 true**：世界绕世界 X 轴转 180°（up→−up、前后→后前）——只转上报位姿、合成帧不变
   *  （模型整体旋转、重建等价），用于抵消该 Kit 模型导出/输出环节的 180° 翻转（成片上下颠倒 +
   *  人物跑到背景后面 = 深度镜像）。
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
}

/**
 * 端侧重建管线。
 * @returns 成功 resolve 输出 MP4 路径；失败 reject 错误描述（含错误码）
 */
export declare function spatialReconTestGenerate(options: SpatialReconTestOptions): Promise<string>;

/** 探测设备是否支持空间重建：0=支持；-1=能力库绑定失败；801=设备不支持 */
export declare function spatialReconProbe(): number;

/** 获取最近一次管线进度（0.0~1.0），供 ArkTS 轮询 */
export declare function getSpatialReconProgress(): number;

/**
 * 暂停进行中的重建（官方 PauseSession）。仅"已开始且未结束"的会话有效。
 * @returns 0=成功；-1=无活动会话/未绑定；其余=Kit 状态码
 */
export declare function spatialReconPause(): number;

/** 继续已暂停的重建（官方 ResumeSession）。@returns 同上 */
export declare function spatialReconResume(): number;

/**
 * 终止进行中的重建：请求终止 + 立即暂停重计算，由运行线程在阶段落定后销毁会话。
 * ⚠️ 保存阶段不可中断：该阶段调用会被忽略，产物照常保存并返回成功。
 * @returns 0=已请求终止；-1=未绑定；其余=Kit Pause 状态码
 */
export declare function spatialReconAbort(): number;

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
