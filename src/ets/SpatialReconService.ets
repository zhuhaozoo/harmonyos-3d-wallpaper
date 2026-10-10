/**
 * 3D壁纸 · Spatial Recon Kit 管线服务
 *
 * 职责：把相册图片解码为 RGBA（分层深度场景另做一次主体分割取蒙版）后交给 native 管线
 * （合成已知位姿帧序列 → 端侧 3DGS 重建 → MP4），产出沙箱 MP4 交由页面保存到图库。
 * 设备支持性探测与进度查询走同一条 native 桥。
 *
 * 两条生成路线同源同桥：
 * - **照片路线**（generateFromImage）：单张照片 → 端侧深度 → 深度网格多视角帧 → 重建；
 * - **预置路线**（generatePresetEarth）：离线合成数据集（tools/depth_mesh/earth_scene.py 产出）
 *   逐帧解码灌入 native 预置帧队列 → 重建，用于"地球"等预置 3D 壁纸成品。
 *
 * 注意：
 * - 会话工作目录/输出文件必须位于应用文件目录下（Kit 约束），且工作目录必须已存在；
 * - 每次生成必须用一个**全新的空会话目录**：Kit 会把目录里的历史数据当作"上次训练现场"
 *   进入 resume 模式，该模式下选中的关键帧被直接拒收（详情见坑 121）；
 * - 相册 URI 不能直传 createImageSource（promise 无声挂起，见坑 20），一律先转 fd；
 * - native 侧通过 dlopen 绑定能力库，不支持设备 probe 返回非 0，不会拖垮进程启动；
 * - 主体分割（Core Vision Kit）只服务分层深度场景（scene=3）：识别不到主体时**不阻断生成**，
 *   退回单层立体场景（scene=1）并在结果里说明。
 */
import { image } from '@kit.ImageKit';
import { media } from '@kit.MediaKit';
import { fileIo } from '@kit.CoreFileKit';
import { common } from '@kit.AbilityKit';
import { util } from '@kit.ArkTS';
import { subjectSegmentation } from '@kit.CoreVisionKit';
import { EarthPlyReport } from './EarthPlyReport';
import {
  spatialReconProbe, spatialReconTestGenerate, getSpatialReconProgress, getSpatialReconStage,
  spatialReconSetForeground,
  spatialReconPause, spatialReconResume, spatialReconAbort, spatialReconInSave,
  spatialReconPresetPush, spatialReconPresetFail,
  SpatialReconTestOptions,
  depthProbe, depthEstimateTest, DepthSelfTestOptions,
} from 'libglassrender.so';

/** 一次生成的产物 */
export class SpatialReconResult {
  /** 产物 MP4 沙箱绝对路径（保存到图库后可删） */
  modelPath: string = '';
  /** 解码后源图宽（px） */
  srcW: number = 0;
  /** 解码后源图高（px） */
  srcH: number = 0;
  /** 分层深度场景的说明（识别到主体 / 回退单层 / 未请求分层时为空串），供页面状态行显示 */
  layerNote: string = '';
}

/** 生成阶段（导出页据此显示真实阶段文案；进度百分比仍取自 native） */
export enum SpatialReconPhase {
  /** 空闲/未开始 */
  IDLE = 0,
  /** 读图/裁剪/预处理（进度尚无数据） */
  PREPARE = 1,
  /** 端侧深度估计（scene=5 前置；进度尚无数据） */
  DEPTH = 2,
  /** 已交给 native 合成+重建（进度由 native 给出） */
  RUN = 3,
}

/** Kit 重建阶段（镜像 HMS_SpatialReconStage，由 native 轮询 Kit 得到，供 UI 显示真实阶段） */
export enum SpatialReconStage {
  /** 本次任务尚未开始 */
  NONE = -1,
  /** 初始化（准备资源） */
  INIT = 0,
  /** 重建中（正在构建 3D 模型） */
  BUILDING = 1,
  /** 已暂停 */
  PAUSED = 2,
  /** 重建完成 */
  FINISHED = 3,
  /** 正在写文件（**此阶段不可中断**，终止请求会被忽略） */
  SAVING = 4,
  /** 未知/不确定 */
  UNKNOWN = 5,
}

/**
 * 位姿/场景适配选项（诊断开关）。
 * 默认值 = 2026-10-01 真机验证命中的组合（AR系 + 世界→相机），**勿随意改动**；
 * 四个开关只影响"上报给 Kit 的位姿/输入布局"，不改渲染，用于真机 A/B 定位约定。
 */
export interface SpatialReconAdaptOptions {
  /** true=四元数按"世界→相机"上报（默认，已验证命中）；false=相机→世界 */
  invertQuat?: boolean;
  /** 诊断 A/B：画面绕视轴滚 180°（图像+位姿同步滚，几何不变）——默认关（实测无效，保留作对照） */
  roll180?: boolean;
  /** **默认 true**（坑 132）：推帧前交换 R/B。该开关会改变成片的颜色表现，但成因**尚无定论**
   *  （坑 134 的"A/B 定案"已撤销，见档案 F7）——默认值与"失败自动反交换重试"属工程兜底，
   *  不代表结论；传 false 可手动直通 RGB */
  bgrSwap?: boolean;
  /** **默认 true**：世界绕世界 X 轴转 180°（只转上报位姿、合成帧不变；模型整体旋转、重建等价）
   *  ——抵消 Kit 模型导出/输出环节的 180° 翻转（成片上下颠倒 + 人物跑到背景后面）；
   *  2026-10-01 真机二次复测确认为正确朝向。显式传 false = 回到 Kit 原始朝向（对照用） */
  worldFlip180?: boolean;
  /** 0=AR 系（X右/Y上/Z后，默认，已验证命中）；1=CV 系（X右/Y下/Z前） */
  poseFamily?: number;
  /** 0=平面贴图；1=立体（主盒贴源图 + 程序纹理环境，默认）；2=立体但全程序纹理（不放源图）；
   *  3=分层深度（主体分割 → 主体/背景两层，需蒙版有效；识别不到主体时自动退回 1） */
  scene?: number;
  /** true=按 RGBA（4 字节/像素）上报；默认 RGB（3 字节/像素） */
  rgbaInput?: boolean;
  /** 主体层深度比（scene=3 用）：主体平面深度 / 背景深度，<1 = 主体更近（默认 0.90，钳 0.60~0.95）。
   *  ⚠️ 取值受"可匹配位移量级"约束：越小主体层位移越大（真机 0.80 曾致稠密初始化失败） */
  subjectDepth?: number;
  /** 虚拟相机焦距 px（默认 750，钳 200~6000）。⚠️ 2026-10-02 坑 131 定案：曾据此认为
   *  "Kit 渲染相机对输入焦距预期 ≈750、横向变形系数 a=750/fx"——已被三组同图 A/B 证伪
   *  （750→宽度压缩、700→宽度拉伸、700+小轨道→碎；真机 refined fy/fx = 0.155/2.47/2.69，
   *  与上报的 fx=fy 无确定函数）。比例失真真因 = 纯水平轨道对 (纵向尺度, fy) 精确退化 →
   *  Kit BA 沿退化方向随机漂移；修复在 native 侧（对角弧轨道）。750 仅作为"典型广角主摄"
   *  的先验默认值保留，不再是"修复值" */
  focalPx?: number;
  /** 合成轨道半角 ±Θ（度，钳 1~45，默认 5 = 判据口径）。scene=5 下为**对角弧**（yaw+pitch
   *  同步各扫 ±Θ，坑 131 修复：打破 fy/纵向尺度退化）；native 侧另有位移带自适应钳制
   *  （相邻关键帧总位移 30~80px、端点单轴位移 ≤ 裙边 80px，带外值会被钳回并打
   *  `SRCHK orbit clamp` 日志——真机日志对账时注意实际 Θ 可能与滑杆值不同） */
  orbitDeg?: number;
  /** 合成帧数（质量实验 R2，默认 72 = 判据口径；native 侧钳 [2,240]）。更多帧 = 给 Kit 更多
   *  关键帧候选（重建稳定性），推帧耗时随帧数线性增长；默认值改动须带真机对比证据 */
  frameCount?: number;
  /** **照片深度预估（默认 true = 现行为）**：false = 跳过端侧深度估计阶段（省推理耗时），
   *  深度缓冲不随参数传入 → native 侧走既有"深度缺失 → 退回 scene=1"降级链（立体场景、
   *  无深度网格，不阻断出片）。调试开关：编辑页「构图 → 深度预估」（关/开） */
  depthEstimate?: boolean;
  /** **阶梯回退开关（默认 true = 现行为）**：开 = 两档尝试序列 [交换@首选位 → 交换@另一位]
   *  （首选位 =「交换时机」选择，默认推帧口；自然 RGB 档已于 2026-10-04 移出阶梯——颜色对调
   *  的产物不如明确失败，需自然产物时手动关「通道交换」）。false = 仅按用户当前选择的方式
   *  执行一次，失败不再自动换档——测试用（坑 143 轮 9：回退会掩盖所选档位的真实结局；高危
   *  预判跳档同样不参与，显式开关优先）。本档自身的产物自检/比例门控交付不受影响（交付的
   *  仍是所选方式的产物）。调试开关：编辑页「适配 → 阶梯回退」（关/开） */
  attemptLadder?: boolean;
  /** **智能构图初档（层 1，默认 true = 开，2026-10-04 用户决定）**：开 = 按照片描述子
   *  （深度双峰/上远景/低纹理）自动调整构图初值（双峰→adaptiveK=1、弱纹理/上远景→帧数 96；
   *  仅用户保留默认值时生效，日志 `SR3D comp`。纵深默认已是自适应，故常驻效果 = 帧数 72→96）。
   *  背景：坑 151（曾因构图搜索两连败一并停用；描述子口径已验证与离线诊断一致）。
   *  调试开关：编辑页「构图 → 智能构图」 */
  smartCompose?: boolean;
  /** **高危档策略（坑 143，默认 true = 交换优先 + 产物自检兜底）**：高饱和照片（swapRisk 高于
   *  阈值）走交换档有"重建成功但模型垃圾"的风险（真机 0.617 实锤，比失败更恶劣：成功态阻断重试）。
   *  true = 先试交换档，成功后做**产物自检**，判"垃圾成功"自动退回自然 RGB（有机会拿回正确颜色）；
   *  false = 旧行为：直接跳过交换档（颜色对调，但重建干净）。
   *  调试开关：编辑页「适配 → 高危档位」（跳过交换 / 交换优先） */
  riskSwapFirst?: boolean;
  /** **交换位置（诊断 A/B，坑 143 轮 8，默认 false = 推帧口逐帧交换）**：true = "深度之后、
   *  贴图/网格构建之前"对源缓冲整体换一次（深度估计仍吃自然色；除交换点外全管线同款）。
   *  预期与现行等价（只验证位置差异），与上面的交换策略/产物自检正交。
   *  调试开关：编辑页「适配 → 交换时机」（推帧口 / 贴图前） */
  swapRgbAtInput?: boolean;
  /** **D1/D4 深度边缘（**默认 1=增强**，2026-10-04 用户决定：构图默认档整体上调为质量增强档）**：
   *  1 = JBU RGB 引导上采样（深度边缘贴颜色边缘）；2 = 保边预平滑 + JBU。0 = 旧行为（关）。
   *  ⚠️ 本档改喂给 Kit 的帧统计（坑 147 家族）——坑 154 真机记录过「增强→人物侧竖直拉伸带」
   *  「全开→逆光海边图左半背景纵向拉丝」，默认变更须随真机同图 A/B 复核。
   *  日志 `SRDM edge mode=` */
  depthEdge?: number;
  /** **D3 自适应纵深（**默认 1=自适应 k∈[2.2,3.8]**，2026-10-04 同上）**：按深度直方图分离度
   *  取 k（d0 恒 2 公式，相机数学零改动）；0 = 固定 k=3（旧行为）。日志 `SRCHK adaptive k=` */
  adaptiveK?: number;
  /** **D2 主体对齐（**默认 1=开**，2026-10-04 同上）**：主体蒙版门控的深度净化（主体内部抑噪）；
   *  本层会自动构建主体蒙版并随参数传入，识别不到主体则静默跳过（native 侧忽略）。 */
  subjectAlign?: number;
}

/** 生成尝试档（坑 134）：场景 + 通道交换组合（ArkTS 要求对象字面量对应显式声明的接口） */
interface SpatialReconAttempt {
  /** 合成场景（固定 5 = 深度网格） */
  sc: number;
  /** 推帧是否交换 R/B：首轮 = 用户/默认（开）；失败重试轮 = 反交换 */
  swap: boolean;
  /** 交换位置（坑 145 阶梯）：false = 推帧口逐帧交换；true = 贴图前对源缓冲整体交换。
   *  两位置推帧字节几乎相同，但真机行为系统性不同（详见 candidates 构造处注释） */
  atInput: boolean;
}

/** 照片构图描述子（智能构图初档用，2026-10-04）：由现成数据就地统计（RGBA 缓冲 + 深度场），
 *  零额外推理。口径与离线诊断一致（逆光海边样本：lowTex≈0.30、双峰 near/far≈0.39/0.49） */
interface PhotoCompProfile {
  /** 深度场强双峰（近/远两块准平面，如逆光水面人像）：s>0.5 与 s<0.2 的质量各 >25% */
  bimodal: boolean;
  /** 低纹理块占比（16×12 网格、块内梯度均值 <6，步长 4 采样） */
  lowTexFrac: number;
  /** 上部远景占比（顶部 45% 行内 s<0.05 的像素比例——大片天空/远山，无特征区） */
  farTopFrac: number;
  /** 近景占比（s>0.5） */
  nearFrac: number;
}

/** 产物自检结论（坑 143）：ok=false 表示"重建成功但模型垃圾"（自动退回下一档） */
interface ProductCheck {
  /** true = 视为可用；false = 判"垃圾成功"（画面全灰/结构崩塌） */
  ok: boolean;
  /** 判据摘要（进日志，供阈值校准） */
  detail: string;
}

/** 预置数据集 manifest（rawfile/preset/<id>/manifest.json；由 tools/depth_mesh/earth_scene.py
 *  离线生成，帧与位姿声明同源同参数） */
interface PresetManifest {
  /** 帧数（= frames.length；native 钳 ≤240） */
  count: number;
  /** 环绕半径（世界单位；球半径归一 1，相机必须 >1 = 在球外） */
  cameraRadius: number;
  /** 上报焦距 px（与数据集几何自洽；缺省 750） */
  focalPx: number;
  /** **各向异性内参声明（2026-10-09 加）**：上报 `focalY = focalYRatio × focalPx`。
   *  背景：Kit 的 BA **无锁内参 API**，其 refined 纵横比是**内容相关**漂移（跨素材实测 0.3~3.5）；
   *  退化分析给出 `模型纵横比 ≈ B · ρ_d/ρ_r`（B = 内容漂移、ρ_d = 声明比、ρ_r = 帧的真实比，
   *  见 docs/3d-wallpaper-archive.md）。帧保持各向同性（ρ_r = 1）时，**只改声明**即可把纵横比
   *  拉回 1：ρ_d = 1/B。
   *  ⚠️ 只允许 preset 路线使用（照片路线一律各向同性）；缺省 1.0 = 与历史口径逐位一致。 */
  focalYRatio?: number;
  /** 参数域 ①：自转轴方向（世界系单位向量；缺省 (0,1,0) = 水平环绕） */
  camAxis?: number[];
  /** 参数域 ②：平环 + 相机自身绕 right 轴俯仰（度）。与 camAxis 互斥——
   *  位姿公式域由 manifest 决定，native 侧同传防呆报错（防代码/数据版本错配） */
  camPitchDeg?: number;
  /** **位姿域显式选择（2026-10-09）**：'diag-arc' = 对角弧——相机**位置**同时扫 yaw 与 pitch
   *  （pos.y ≠ 0，φ ≡ θ），即照片路线 scene=5 的 `SrFrameBasis`（Φ=Θ）同口径移植；
   *  缺省（旧数据集）= 按 camAxis / camPitchDeg 自动择一。 */
  poseDomain?: string;
  /** **上报位姿的世界翻转（2026-10-09 加）**：只改上报位姿、不合成帧；缺省 true（历史口径）。
   *  用来抵消 Kit 导出时绕水平轴的 180° 翻转——但该翻转随**内容**变，故由数据集声明。 */
  worldFlip180?: boolean;
  /** 帧条目（file/thetaDeg 必用） */
  frames: PresetManifestFrame[];
}


interface PresetManifestFrame {
  /** 帧文件名（数据集目录下） */
  file: string;
  /** 逐帧环绕角 θ（度；0 = 参考帧） */
  thetaDeg: number;
}

export class SpatialReconService {
  /** 源图解码长边上限。⚠️ 2026-10-04 I1 实验（"裁剪后覆盖画布"，9:16 源贴图 810×1080→1080×1440）
   *  上机后用户真机反馈重建质量变差，当日回退到本验证口径——喂给 Kit 的帧统计变化会漂移黑盒
   *  行为（坑 147；候选机理：匹配/BA 落盆偏移、稀疏视图 3DGS 对高频过拟合）。重试须真机 A/B 证据。 */
  private static readonly MAX_DECODE_SIDE: number = 1440;
  /** 主体分割的默认层深度比（与 native 侧默认值一致，勿单边改）。
   *  0.90 = 主体层位移 ≈251px（±8° 轨道），落在"背景层 ≈145px 已验证可匹配"的邻近区间；
   *  更小的值（更强视差）位移量级会超出已验证的可匹配范围，真机稠密初始化会失败。 */
  private static readonly DEFAULT_SUBJECT_DEPTH: number = 0.90;
  /** 主体占比下限：低于此值视为"未识别到主体"（与 native 侧 SrBuildLayerAssets 的判据一致） */
  private static readonly MIN_SUBJECT_RATIO: number = 0.005;
  /** 主体占比上限：高于此值视为"蒙版把大半个画面都当主体"——分层会在照片前悬一块大幅浮空板，
   *  成片表现为内容拉伸/错位（真机笔记本照片 39.1% 即此），按"无主体"处理（判据同 native 侧） */
  private static readonly MAX_SUBJECT_RATIO: number = 0.35;
  /** 高危阈值（坑 143）：swapRisk（逐像素 |R−B|/max(R,G,B) 均值）高于此值 = 交换档有
   *  "垃圾成功"风险（真机 0.617 动漫实锤）。仅在 riskSwapFirst=false 时用于"跳过交换档"判定 */
  private static readonly SWAP_RISK_THRESHOLD: number = 0.5;
  /** 产物自检阈值（坑 143，保守；首轮真机数据后再校准）：饱和度比、结构强度比低于此值判"垃圾成功" */
  private static readonly CHECK_SAT_RATIO_MIN: number = 0.30;
  private static readonly CHECK_GRAD_RATIO_MIN: number = 0.20;

  /** 设备支持性：0=支持；-1=能力库绑定失败；801=设备不支持（Kit 状态码） */
  static probe(): number {
    return spatialReconProbe();
  }

  /** 当前生成阶段（模块级静态，供导出页轮询显示；见 SpatialReconPhase） */
  private static phaseVal: number = SpatialReconPhase.IDLE;

  /** 最近一次生成的阶段：0=空闲 / 1=准备 / 2=端侧深度估计 / 3=重建中 */
  static phase(): number {
    return SpatialReconService.phaseVal;
  }

  /** Kit 重建阶段（实时取自 native 对 Kit 的轮询；-1=未开始），见 SpatialReconStage */
  static stage(): number {
    return getSpatialReconStage();
  }

  /** 最近一次生成任务进度（0.0~1.0），页面定时轮询 */
  static progress(): number {
    return getSpatialReconProgress();
  }

  /** 暂停进行中的重建（Kit 官方 PauseSession；仅重建阶段有效，保存阶段不可中断） */
  static pause(): number {
    return spatialReconPause();
  }

  /** 继续已暂停的重建（Kit 官方 ResumeSession） */
  static resume(): number {
    return spatialReconResume();
  }

  /** 请求终止本次生成（保存收尾阶段会被忽略：该阶段编码落盘在途、不可中断，
   *  UI 须先用 inSave() 判定并禁用按钮） */
  static abort(): number {
    return spatialReconAbort();
  }

  /** 是否处于"保存收尾"阶段（true = 保存中）——暂停/终止按钮的可用性判定依据。
   *  ⚠️ 不可用 Kit 的 stage 值替代：保存期 Kit 可能报 FINISHED 而非 SAVING，
   *  会让按钮看着可点、点下去无效（用户感知"点了没反应"，2026-10-05 实测反馈）。 */
  static inSave(): boolean {
    return spatialReconInSave() === 1;
  }

  /** 换档重试提示（非空时应展示给用户；生成开始时清空）。
   *  语义：候选产物被质量检验判为不可用、正在换下一个候选（阶梯回退或智能构图重试时才有）。 */
  private static retryNoteVal: string = '';

  static retryNote(): string {
    return SpatialReconService.retryNoteVal;
  }

  /** 前后台切换 → 更新系统运行模式（官方规则：每次 StartSession 后须调用 SetRunningMode、
   *  应用切前后台时也要更新；无会话时零成本 no-op） */
  static setForeground(fg: boolean): number {
    return spatialReconSetForeground(fg);
  }

  /** 会话工作目录（应用文件目录子目录；只放产物 MP4） */
  static workDir(ctx: common.UIAbilityContext): string {
    return ctx.filesDir + '/spatial_recon_work';
  }

  /** 本次会话的 Kit 工作目录（每次全新的一级子目录，路径深度与已验证可用的目录同级） */
  static freshKitDir(ctx: common.UIAbilityContext): string {
    return ctx.filesDir + `/spatial_recon_kit_${Date.now()}`;
  }

  /**
   * 清理历史遗留（每次生成前调用，幂等）：
   * - 早期版本把 Kit 会话数据直接写在 workDir 里（现已改为一次性会话目录），这些残留一直占空间；
   * - 进程被杀等异常会遗留 spatial_recon_kit_* 会话目录。
   * 保留 workDir 下的产物 model_*.mp4 与**最新 4 份** model_*.ply（PLY 是模型侧唯一可解剖产物：
   * 几何自检 `SR3D plydiag`、PLY 导出都吃它；留 4 份是为了让"前后两轮对照"这类跨轮比较
   * 在下一轮生成后仍然拿得到——只留最新一份时，跑一轮就会把上一轮的对照删掉）。
   * 返回清理掉的条目数（>0 时打日志）。
   */
  static cleanLegacy(ctx: common.UIAbilityContext): number {
    let removed: number = 0;
    const legacy: string = SpatialReconService.workDir(ctx);
    if (fileIo.accessSync(legacy)) {
      const names: string[] = fileIo.listFileSync(legacy);
      // 按 mtime 取最新 kKeepPly 份 PLY 侧车（mtime 秒级，同秒时按名字兜底排序）
      const kKeepPly: number = 4;
      const plys: string[] = [];
      const plyAt: number[] = [];
      for (const name of names) {
        if (!name.endsWith('.ply')) {
          continue;
        }
        let t: number = -1;
        try {
          t = fileIo.statSync(legacy + '/' + name).mtime;
        } catch (e) {
          t = -1;
        }
        let ins: number = plys.length;
        while (ins > 0 && plyAt[ins - 1] < t) {
          ins--;
        }
        plys.splice(ins, 0, name);
        plyAt.splice(ins, 0, t);
      }
      const keepPly: string[] = plys.slice(0, kKeepPly);
      for (const name of names) {
        if (name.startsWith('model_') && name.endsWith('.mp4')) {
          continue;   // 保留产物
        }
        if (keepPly.indexOf(name) >= 0) {
          continue;   // 保留最新的若干份 PLY 侧车（几何自检 / 导出用）
        }
        if (SpatialReconService.removePath(legacy + '/' + name)) {
          removed++;
        }
      }
    }
    for (const name of fileIo.listFileSync(ctx.filesDir)) {
      if (name.startsWith('spatial_recon_kit_')
        && SpatialReconService.removePath(ctx.filesDir + '/' + name)) {
        removed++;
      }
    }
    // 端侧深度自检产物（每次 scene=5 生成写一份，约 1.6MB，从不清理会无界增长）：只留最新 4 份
    removed += SpatialReconService.pruneDepthArtifacts(ctx);
    return removed;
  }

  /**
   * 深度自检产物只保留最新 4 组（`<filesDir>/depth_selftest/depth_<ts>.depth.f32.bin` + `.meta.txt`）。
   * 保留几份是为了最近的诊断数据还能取回比对；更早的删除，避免无界增长。返回删除的文件数。
   */
  private static pruneDepthArtifacts(ctx: common.UIAbilityContext): number {
    const dir: string = ctx.filesDir + '/depth_selftest';
    if (!fileIo.accessSync(dir)) {
      return 0;
    }
    let names: string[] = [];
    try {
      names = fileIo.listFileSync(dir);
    } catch (e) {
      return 0;
    }
    const stamps: number[] = [];
    for (let i = 0; i < names.length; i++) {
      const n: string = names[i];
      if (!n.startsWith('depth_')) {
        continue;
      }
      const dot: number = n.indexOf('.');
      const ts: number = Number.parseInt(dot > 6 ? n.substring(6, dot) : n.substring(6), 10);
      if (Number.isFinite(ts) && stamps.indexOf(ts) < 0) {
        stamps.push(ts);
      }
    }
    stamps.sort((a: number, b: number): number => b - a);   // 时间戳降序，前面是最新
    let removed: number = 0;
    for (let s = 4; s < stamps.length; s++) {
      const prefix: string = `depth_${stamps[s]}`;
      for (let i = 0; i < names.length; i++) {
        if (names[i].startsWith(prefix)
          && SpatialReconService.removePath(dir + '/' + names[i])) {
          removed++;
        }
      }
    }
    return removed;
  }

  /** 删除文件或目录（目录递归删除）；失败返回 false，不抛 */
  private static removePath(path: string): boolean {
    try {
      if (fileIo.statSync(path).isDirectory()) {
        fileIo.rmdirSync(path);
      } else {
        fileIo.unlinkSync(path);
      }
      return true;
    } catch (e) {
      return false;
    }
  }

  /**
   * 主体分割 → 主体蒙版（0=背景 / 255=主体 / 中间值=主体概率，长度 = srcW*srcH）。
   *
   * 优先取官方 `mattingList`（"基于原图大小的一维数组"，无需读像素缓冲，天然规避行跨距问题）；
   * 长度不可信时回退读前景图 alpha（并做 getPixelBytesNumber 校验防 DMA 行跨距，坑 65）。
   *
   * @returns null = 未识别到主体（初始化失败/结果缺失/尺寸不符/主体占比过小）——
   *          调用方按单层立体场景生成，不阻断流程。
   */
  /** 居中裁剪到 3:4（原地）。Kit 只认 1080×1440；非 3:4 照片先居中裁剪，
   *  保证"深度估计输入 == 贴图"是同一张（1:1 对齐）。主体引导裁剪留待后续（现为居中裁剪）。 */
  private static async cropTo3to4(pm: image.PixelMap): Promise<void> {
    const info: image.ImageInfo = pm.getImageInfoSync();
    const w: number = info.size.width;
    const h: number = info.size.height;
    if (w * 4 === h * 3) {
      return;                                        // 已是 3:4
    }
    let cw: number = w;
    let ch: number = h;
    let x: number = 0;
    let y: number = 0;
    if (w * 4 < h * 3) {
      ch = Math.round(w * 4 / 3);
      y = Math.floor((h - ch) / 2);
    } else {
      cw = Math.round(h * 3 / 4);
      x = Math.floor((w - cw) / 2);
    }
    await pm.crop({ x: x, y: y, size: { width: cw, height: ch } });
    const after: image.ImageInfo = pm.getImageInfoSync();
    console.info(`SR3D crop3to4 ${w}x${h} -> ${after.size.width}x${after.size.height}`);
  }

  /** 读沙箱文件为 ArrayBuffer（scene=5 回读深度 .bin 用；失败返回 null） */
  private static readFileBuffer(path: string): ArrayBuffer | null {
    try {
      const st: fileIo.Stat = fileIo.statSync(path);
      const buf: ArrayBuffer = new ArrayBuffer(st.size);
      const f: fileIo.File = fileIo.openSync(path, fileIo.OpenMode.READ_ONLY);
      const n: number = fileIo.readSync(f.fd, buf);
      fileIo.closeSync(f);
      return n === st.size ? buf : buf.slice(0, n);
    } catch (e) {
      console.warn(`SR3D readFileBuffer failed ${path}`);
      return null;
    }
  }

  /**
   * 产物自检（诊断）：打印 Kit 成片的**视频轨分辨率/旋转角/体积**，并把**首帧**采样成 48×32 ASCII
   * 打进日志（与 native 侧 `SRCHK fmid`/`src` 同口径：点采样 48×32 + 10 档 ramp）。
   * 用途：排查成片"比例失真（宽度被压缩）"——① `videoWidth/Height/orientation` 不对 = 文件本身
   * 被转置/旋转；② ASCII 静立内容与源图逐格比对可**量出水平压缩比例**（某列起内容错位 = 被压）。
   * 失败不影响主流程。
   */
  private static async logOutputProbe(path: string): Promise<void> {
    let size: number = 0;
    try {
      size = fileIo.statSync(path).size;
    } catch (e) {
      size = -1;
    }
    let f: fileIo.File | null = null;
    let md: media.AVMetadataExtractor | null = null;
    try {
      f = fileIo.openSync(path, fileIo.OpenMode.READ_ONLY);
      md = await media.createAVMetadataExtractor();
      md.fdSrc = { fd: f.fd };
      const meta: media.AVMetadata = await md.fetchMetadata();
      const fw: number = Number.parseInt(meta.videoWidth ?? '0', 10);
      const fh: number = Number.parseInt(meta.videoHeight ?? '0', 10);
      console.info(`SR3D out video ${meta.videoWidth}x${meta.videoHeight} ` +
        `orientation=${meta.videoOrientation} duration=${meta.duration}ms bytes=${size}`);
      if (fw > 0 && fh > 0) {
        const pm: image.PixelMap = await md.fetchFrameByTime(0,
          media.AVImageQueryOptions.AV_IMAGE_QUERY_CLOSEST_SYNC, { width: fw, height: fh });
        const bytes: number = pm.getPixelBytesNumber();
        const buf: ArrayBuffer = new ArrayBuffer(bytes);
        await pm.readPixelsToBuffer(buf);
        SpatialReconService.logAsciiThumb('out', new Uint8Array(buf), fw, fh, bytes / (fw * fh));
        await pm.release();
      }
    } catch (e) {
      const msg: string = typeof e === 'string' ? e : ((e as Error).message || String(e));
      console.info(`SR3D out video probe failed (${msg}) bytes=${size}`);
    } finally {
      if (md !== null) {
        try {
          await md.release();
        } catch (e) {
          // 释放失败不影响主流程
        }
      }
      if (f !== null) {
        try {
          fileIo.closeSync(f);
        } catch (e) {
          // 关闭失败不影响主流程
        }
      }
    }
  }

  /** 48×32 ASCII 缩略图（与 native SrLogAscii 同口径：点采样 + 10 档 ramp + 亮度加权） */
  private static logAsciiThumb(tag: string, px: Uint8Array, w: number, h: number,
    ch: number): void {
    const ramp: string = ' .:-=+*#%@';
    const cols: number = 48;
    const rows: number = 32;
    for (let r = 0; r < rows; r++) {
      let line: string = '';
      for (let c = 0; c < cols; c++) {
        const x: number = Math.floor(c * w / cols);
        const y: number = Math.floor(r * h / rows);
        const o: number = (y * w + x) * ch;
        const lum: number = Math.floor((px[o] * 30 + px[o + 1] * 59 + px[o + 2] * 11) / 100);
        line += ramp[Math.floor(lum * 9 / 255)];
      }
      console.info(`SR3D ${tag} r${r < 10 ? '0' : ''}${r} |${line}|`);
    }
  }

  /**
   * 产物自检（坑 143）：识别"交换档重建成功但模型垃圾"（初始视角全灰/旋转后混乱，真机 0.617 实锤）。
   *
   * 判据只用**对 R/B 交换不变**的两个指标（关键：自检必须与"颜色是否正确"正交——不得用
   * 直方图/逐像素相关度这类会被 R↔B 交换改变的量，否则会把"颜色对调"本身误判成垃圾）：
   *   ① 饱和度比 satF/satS（交换不变：max/min 不变；模型全灰 → 趋 0）
   *   ② 结构强度比 gradF/gradS（48 列粗网格上相邻格亮度差均值；模糊/塌陷 → 显著低于源图）
   * 保守门槛（0.30 / 0.20）：只有明显崩塌才判 junk——误杀会把本可用的交换档踢回自然 RGB（颜色变错）。
   * 取成片首帧（1/4 降采样）；读像素失败/布局异常时**不阻断**（按可用处理）。数值一律进日志
   * （`SR3D check …`）供首轮真机数据校准阈值。
   */
  private static async checkProduct(path: string, srcBuf: ArrayBuffer, srcW: number,
    srcH: number): Promise<ProductCheck> {
    let f: fileIo.File | null = null;
    let md: media.AVMetadataExtractor | null = null;
    try {
      f = fileIo.openSync(path, fileIo.OpenMode.READ_ONLY);
      md = await media.createAVMetadataExtractor();
      md.fdSrc = { fd: f.fd };
      const meta: media.AVMetadata = await md.fetchMetadata();
      const fw: number = Number.parseInt(meta.videoWidth ?? '0', 10);
      const fh: number = Number.parseInt(meta.videoHeight ?? '0', 10);
      if (!(fw > 0 && fh > 0)) {
        console.info('SR3D check: no video size, skip (assume ok)');
        return { ok: true, detail: 'no-size' };
      }
      const pw: number = Math.max(16, Math.round(fw / 4));
      const ph: number = Math.max(16, Math.round(fh / 4));
      const pm: image.PixelMap = await md.fetchFrameByTime(0,
        media.AVImageQueryOptions.AV_IMAGE_QUERY_CLOSEST_SYNC, { width: pw, height: ph });
      const bytes: number = pm.getPixelBytesNumber();
      const buf: ArrayBuffer = new ArrayBuffer(bytes);
      await pm.readPixelsToBuffer(buf);
      const ch: number = pw * ph > 0 ? Math.round(bytes / (pw * ph)) : 0;
      if (ch !== 4 && ch !== 3) {
        void pm.release();
        console.info(`SR3D check: unexpected pixel layout ch=${ch}, skip (assume ok)`);
        return { ok: true, detail: 'layout' };
      }
      const fs: number[] = SpatialReconService.imageStats(new Uint8Array(buf), pw, ph, ch);
      void pm.release();
      const ss: number[] = SpatialReconService.imageStats(new Uint8Array(srcBuf), srcW, srcH, 4);
      const satRatio: number = ss[0] > 1e-6 ? fs[0] / ss[0] : 1;
      const gradRatio: number = ss[1] > 1e-6 ? fs[1] / ss[1] : 1;
      const ok: boolean = satRatio >= SpatialReconService.CHECK_SAT_RATIO_MIN
        && gradRatio >= SpatialReconService.CHECK_GRAD_RATIO_MIN;
      const detail: string = `sat ${fs[0].toFixed(3)}/${ss[0].toFixed(3)}`
        + ` grad ${fs[1].toFixed(2)}/${ss[1].toFixed(2)}`
        + ` ratio ${satRatio.toFixed(2)}/${gradRatio.toFixed(2)}`;
      console.info(`SR3D check ${ok ? 'ok' : 'junk'} ${detail}`);
      return { ok: ok, detail: detail };
    } catch (e) {
      const msg: string = typeof e === 'string' ? e : ((e as Error).message || String(e));
      console.warn(`SR3D check failed (${msg}), assume ok`);
      return { ok: true, detail: 'error' };
    } finally {
      if (md !== null) {
        try {
          await md.release();
        } catch (e) {
          // 释放失败不影响主流程
        }
      }
      if (f !== null) {
        try {
          fileIo.closeSync(f);
        } catch (e) {
          // 关闭失败不影响主流程
        }
      }
    }
  }

  /** 图像统计（源图/成片首帧共用）：48 列粗网格上的 [饱和度均值, 相邻格亮度差均值]（见 checkProduct） */
  private static imageStats(u8: Uint8Array, w: number, h: number, ch: number): number[] {
    const cols: number = 48;
    const rows: number = Math.max(8, Math.round(cols * h / Math.max(1, w)));
    const lum: number[] = [];
    let satSum: number = 0;
    let satCnt: number = 0;
    for (let r = 0; r < rows; r++) {
      const y: number = Math.min(h - 1, Math.floor((r + 0.5) * h / rows));
      for (let c = 0; c < cols; c++) {
        const x: number = Math.min(w - 1, Math.floor((c + 0.5) * w / cols));
        const o: number = (y * w + x) * ch;
        const rv: number = u8[o];
        const gv: number = u8[o + 1];
        const bv: number = u8[o + 2];
        const mx: number = Math.max(rv, Math.max(gv, bv));
        const mn: number = Math.min(rv, Math.min(gv, bv));
        if (mx >= 8) {
          satSum += (mx - mn) / mx;
          satCnt++;
        }
        lum.push((rv * 30 + gv * 59 + bv * 11) / 100);
      }
    }
    let gradSum: number = 0;
    let gradCnt: number = 0;
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < cols; c++) {
        const i: number = r * cols + c;
        if (c + 1 < cols) {
          gradSum += Math.abs(lum[i + 1] - lum[i]);
          gradCnt++;
        }
        if (r + 1 < rows) {
          gradSum += Math.abs(lum[i + cols] - lum[i]);
          gradCnt++;
        }
      }
    }
    const sat: number = satCnt > 0 ? satSum / satCnt : 0;
    const grad: number = gradCnt > 0 ? gradSum / gradCnt : 0;
    return [sat, grad];
  }

  private static async buildSubjectMask(pm: image.PixelMap,
    srcW: number, srcH: number): Promise<ArrayBuffer | null> {
    let fg: image.PixelMap | null = null;
    const n: number = srcW * srcH;
    try {
      const initOk: boolean = await subjectSegmentation.init();
      if (!initOk) {
        console.warn('SR3D subj seg init failed');
        return null;
      }
      const config: subjectSegmentation.SegmentationConfig = {
        maxCount: 1,
        enableSubjectDetails: false,
        enableSubjectForegroundImage: true,
      };
      const visionInfo: subjectSegmentation.VisionInfo = { pixelMap: pm };
      const result: subjectSegmentation.SegmentationResult =
        await subjectSegmentation.doSegmentation(visionInfo, config);
      const full: subjectSegmentation.SubjectResult = result.fullSubject;
      if (full === null || full === undefined) {
        console.warn('SR3D subj seg empty result');
        return null;
      }
      // 蒙版直接分配为独立 ArrayBuffer（Uint8Array 只是它的视图，无副本），返回值即持有它。
      // ⚠️ 不可写 mask.buffer：ArkTS 中 Uint8Array.buffer 的静态类型不是 ArrayBuffer
      //（结构类型不受支持，编译报 arkts-no-structural-typing，坑 63）。
      const maskBuf: ArrayBuffer = new ArrayBuffer(n);
      const mask: Uint8Array = new Uint8Array(maskBuf);
      let via: string = '';
      const matting: Int32Array = full.mattingList;
      if (matting !== null && matting !== undefined && matting.length === n) {
        via = 'matting';
        for (let i = 0; i < n; i++) {
          const v: number = matting[i];
          mask[i] = v > 255 ? 255 : (v < 0 ? 0 : v);
        }
      } else {
        // 回退：前景图 alpha（与输入同尺寸的 RGBA，alpha 即前景蒙版）
        const fgPm: image.PixelMap = full.foregroundImage;
        if (fgPm === null || fgPm === undefined) {
          console.warn('SR3D subj seg no mask: mattingLen=' +
            (matting === undefined ? -1 : matting.length) + ' expect=' + n);
          return null;
        }
        fg = fgPm;
        const info: image.ImageInfo = await fgPm.getImageInfo();
        const fw: number = info.size.width;
        const fh: number = info.size.height;
        // 行跨距防御（坑 65）：字节数与"紧凑 RGBA"不符时读出的 alpha 会逐行错位，宁可不分层
        if (fw !== srcW || fh !== srcH || fgPm.getPixelBytesNumber() !== fw * fh * 4) {
          console.warn(`SR3D subj seg fg mismatch ${fw}x${fh} bytes=${fgPm.getPixelBytesNumber()}`);
          return null;
        }
        via = 'fgAlpha';
        const px: ArrayBuffer = new ArrayBuffer(fw * fh * 4);
        await fgPm.readPixelsToBuffer(px);
        const u8: Uint8Array = new Uint8Array(px);
        for (let i = 0; i < n; i++) {
          mask[i] = u8[i * 4 + 3] > 128 ? 255 : 0;
        }
      }
      let subj: number = 0;
      for (let i = 0; i < n; i++) {
        if (mask[i] > 127) {
          subj++;
        }
      }
      const ratio: number = subj / n;
      console.info(`SR3D subj mask via=${via} ${srcW}x${srcH} subj=${subj} ratio=${ratio.toFixed(4)}`);
      if (ratio < SpatialReconService.MIN_SUBJECT_RATIO) {
        console.info('SR3D subj mask too small, treat as no subject');
        return null;
      }
      if (ratio > SpatialReconService.MAX_SUBJECT_RATIO) {
        console.info('SR3D subj mask too large (would become a floating slab), treat as no subject');
        return null;
      }
      return maskBuf;
    } catch (e) {
      console.warn('SR3D subj seg error: ' + JSON.stringify(e));
      return null;
    } finally {
      try { if (fg !== null) { fg.release(); } } catch (e) { /* ignore */ }
      try { await subjectSegmentation.release(); } catch (e) { /* ignore */ }
    }
  }

  /**
   * 相册 URI → 端侧重建 → MP4 沙箱路径。
   * scene=3 时先做主体分割：识别到主体 → 分层深度场景（主体层 + 背景补洞层）；
   * 识别不到 → 自动退回单层立体场景（scene=1），并在结果里说明。
   * 失败返回 null（读取/解码失败）或抛错（native 管线失败，message 含错误码）。
   */
  static async generateFromImage(ctx: common.UIAbilityContext, uri: string,
    adapt?: SpatialReconAdaptOptions): Promise<SpatialReconResult> {
    const decoded: SpatialReconResult = new SpatialReconResult();
    SpatialReconService.phaseVal = SpatialReconPhase.PREPARE;   // 阶段：读图/裁剪/预处理
    SpatialReconService.retryNoteVal = '';                      // 清上一次的重试提示
    let file: fileIo.File | null = null;
    let source: image.ImageSource | null = null;
    let pixelMap: image.PixelMap | null = null;
    /** 本次生成用到的会话工作目录（Kit 专用；每次尝试一个全新目录，结束即回收） */
    const runDirs: string[] = [];
    try {
      file = fileIo.openSync(uri, fileIo.OpenMode.READ_ONLY);
      source = image.createImageSource(file.fd);
      const info: image.ImageInfo = await source.getImageInfo();
      const srcW: number = info.size.width;
      const srcH: number = info.size.height;
      const scale: number = Math.min(1,
        SpatialReconService.MAX_DECODE_SIDE / Math.max(srcW, srcH));
      const options: image.DecodingOptions = {
        desiredSize: {
          width: Math.max(1, Math.round(srcW * scale)),
          height: Math.max(1, Math.round(srcH * scale))
        },
        desiredPixelFormat: image.PixelMapFormat.RGBA_8888
      };
      pixelMap = await source.createPixelMap(options);
      // 路线B 深度网格（scene=5）：Kit 只认 3:4 → 非 3:4 照片先居中裁剪（§3.5）。
      // 必须在解码后立刻裁：后续"深度估计"与"贴图"共用这张裁剪后的位图，必然 1:1 对齐。
      if ((adapt?.scene ?? 1) === 5) {
        await SpatialReconService.cropTo3to4(pixelMap);
      }
      const decodedInfo: image.ImageInfo = pixelMap.getImageInfoSync();
      decoded.srcW = decodedInfo.size.width;
      decoded.srcH = decodedInfo.size.height;
      // 源图尺寸进日志：排查"比例/尺寸相关"的失败（正面贴图按宽高比 contain，比例异常会被看出来）
      console.info(`SR3D src decoded=${decoded.srcW}x${decoded.srcH} ` +
        `src=${srcW}x${srcH} scale=${scale.toFixed(3)}`);

      // 需要主体的场景（scene=3 分层深度 / scene=4 盒中空间）：先做主体分割
      //（用与 native 完全同源的解码位图，蒙版尺寸必然一致）。
      // 拿不到蒙版时不阻断：scene=4 的盒内空间几何不依赖蒙版（只是没有主体层）；
      // scene=3 在 native 侧会自动退回 scene=1（单层立体）。
      let mask: ArrayBuffer | null = null;
      const wantedScene: number = adapt?.scene ?? 1;
      const wantsMask: boolean = (wantedScene === 3 || wantedScene === 4);
      // D2 主体对齐：scene=5 也要蒙版（默认开，2026-10-04 起）——识别不到主体时静默跳过
      //（native 侧忽略缺失蒙版，等价于旧行为）
      const wantsSubjectDepth: boolean = (wantedScene === 5 && adapt?.subjectAlign === 1);
      if (wantsMask || wantsSubjectDepth) {
        mask = await SpatialReconService.buildSubjectMask(pixelMap, decoded.srcW, decoded.srcH);
        if (wantsMask) {
          if (mask === null) {
            decoded.layerNote = wantedScene === 4
              ? '未识别到主体：按空间场景生成'
              : '未识别到主体：按整图生成';
          } else {
            decoded.layerNote = '已识别主体：主体/背景分层生成';
          }
        } else if (mask === null) {
          console.warn('SR3D subject align: no mask, D2 skipped');
        }
      }

      const buf: ArrayBuffer = new ArrayBuffer(pixelMap.getPixelBytesNumber());
      await pixelMap.readPixelsToBuffer(buf);

      // 坑 134 输入端预判（2026-10-02 加探针，2026-10-03 三点标定后接入决策）：
      // R/B 交换敏感度 = 逐像素 |R−B|/max(R,G,B) 均值（近黑跳过，1/4 采样）。
      // 标定点：0.207 夜景=交换后重建成功且颜色正确；0.334 动漫=交换后匹配崩塌
      // （快速失败→自动反交换重试兜住，代价 ~4s + 颜色对调）；**0.617 动漫(大幅蓝天)=
      // 交换后"重建成功"但模型垃圾（初始视角全灰/模糊混乱）且阻断了重试**——最恶劣区。
      // 策略（坑 143，默认 swapFirst）：所有档位一律"交换优先"，交换档成功后做**产物自检**
      // （checkProduct），判"垃圾成功"自动退回自然 RGB；policy=skipSwap（关开关）才回到旧行为
      // ——高危档（swapRisk 越阈值）直接跳过交换档，单次自然 RGB（颜色对调但重建干净）。
      // 阈值见 SWAP_RISK_THRESHOLD（仅 skipSwap 档使用，可随标定数据调整）。
      // 阶梯回退开关（默认开 = 两档阶梯 [交换@首选位 → 交换@另一位]；关 = 尝试序列只按用户
      // 当前选择的方式执行一次，不再自动换档兜底——测试用，坑 143 轮 9：回退会掩盖所选档位
      // 的真实结局，单档才能看到真实结局）。日志对账：`SR3D precheck … ladder=on|off`。
      const ladderOn: boolean = adapt?.attemptLadder !== false;
      let swapRisk: number = 0;
      {
        const u8: Uint8Array = new Uint8Array(buf);
        let sum: number = 0;
        let cnt: number = 0;
        for (let i = 0; i + 3 < u8.length; i += 16) {   // 1/4 像素采样（步长 4 像素）
          const r: number = u8[i];
          const g: number = u8[i + 1];
          const b: number = u8[i + 2];
          const mx: number = Math.max(r, g, b);
          if (mx < 8) {
            continue;   // 近黑像素不计（夜景大量暗部，避免稀释）
          }
          sum += Math.abs(r - b) / mx;
          cnt++;
        }
        swapRisk = cnt > 0 ? sum / cnt : 0;
        // 口径（坑 143 起）：默认 policy=swapFirst（高危档也交换优先，成功后用产物自检兜底"垃圾成功"）；
        // policy=skipSwap 才是旧行为（高危档直接跳过交换档）。highRisk 仅表示 swapRisk 越阈值。
        console.info(`SR3D precheck swapRisk=${swapRisk.toFixed(3)} px=${cnt} ` +
          `highRisk=${swapRisk > SpatialReconService.SWAP_RISK_THRESHOLD ? 1 : 0} ` +
          `policy=${adapt?.riskSwapFirst !== false ? 'swapFirst' : 'skipSwap'} ` +
          `ladder=${ladderOn ? 'on' : 'off'}`);
      }

      // 路线B 深度网格（scene=5）：先跑端侧深度估计（复用阶段1 已验证的推理链路），再把 .bin 回读为参数。
      // 深度与贴图同源同尺寸（都是这张裁剪后的位图）→ 网格 UV 与深度网格 1:1 对齐。
      // 「深度预估」开关（默认开）：关 = 本阶段整体跳过（省推理耗时），depthBuf 保持 null，
      // native 侧按既有降级链退回 scene=1（与"推理运行时不可用"同一条路径，不阻断出片）。
      let depthBuf: ArrayBuffer | null = null;
      let depthW: number = 0;
      let depthH: number = 0;
      if (wantedScene === 5 && adapt?.depthEstimate === false) {
        console.info('SR3D depth estimate off (switch), expect scene=1 fallback');
      }
      if (wantedScene === 5 && adapt?.depthEstimate !== false) {
        SpatialReconService.phaseVal = SpatialReconPhase.DEPTH;   // 阶段：端侧深度估计（进度暂无数据）
        const ddir: string = ctx.filesDir + '/depth_selftest';
        if (!fileIo.accessSync(ddir)) {
          fileIo.mkdirSync(ddir, true);
        }
        const prefix: string = `${ddir}/depth_${Date.now()}`;
        const dres: string = await depthEstimateTest({
          rgbaBuffer: buf,
          width: decoded.srcW,
          height: decoded.srcH,
          outPrefix: prefix,
          resMgr: ctx.resourceManager,
        });
        console.info(`SR3D depth for scene5 ${dres}`);
        depthBuf = SpatialReconService.readFileBuffer(prefix + '.depth.f32.bin');
        // 从统计串解析 "out=WxH"（不用正则，ArkTS 类型更稳）
        const k: number = dres.indexOf('out=');
        if (k >= 0) {
          const seg: string = dres.substring(k + 4);
          const xp: number = seg.indexOf('x');
          if (xp > 0) {
            const tail: string = seg.substring(xp + 1);
            const sp: number = tail.indexOf(' ');
            depthW = Number.parseInt(seg.substring(0, xp), 10);
            depthH = Number.parseInt(sp < 0 ? tail : tail.substring(0, sp), 10);
          }
        }
        if (depthBuf === null || depthW <= 0 || depthH <= 0) {
          console.warn('SR3D scene5 depth unavailable, will fall back to scene=1');
        }
      }

      // 智能构图初档（层 1，2026-10-04；**默认开**——开关「构图 → 智能构图」，背景见坑 151）：
      // 照片描述子 → 参数初值。只在用户保留默认值时调整（显式设置优先），每次调整打日志
      // （`SR3D comp ...`）对账；阈值按逆光海边样本（lowTex 0.30 / 双峰 0.39+0.49）首轮标定。
      // ⚠️ 纵深默认已=自适应（1），故「双峰→自适应」支只在用户显式选「固定」时才可能触发
      //（此时 arm 由 `effAdaptiveK === 0` 判定）；常驻效果 = 弱纹理/上远景图帧数 72→96。
      let effAdaptiveK: number = adapt?.adaptiveK ?? 1;
      let effFrameCount: number = adapt?.frameCount ?? 72;
      if (adapt?.smartCompose === true) {
        const comp: PhotoCompProfile = SpatialReconService.analyzePhotoComp(
          buf, decoded.srcW, decoded.srcH, depthBuf, depthW, depthH);
        console.info(`SR3D comp profile bimodal=${comp.bimodal ? 1 : 0}` +
          ` near=${comp.nearFrac.toFixed(2)} farTop=${comp.farTopFrac.toFixed(2)}` +
          ` lowTex=${comp.lowTexFrac.toFixed(2)}`);
        if (comp.bimodal && effAdaptiveK === 0) {
          effAdaptiveK = 1;
          console.info('SR3D comp auto: adaptiveK 0 -> 1 (bimodal depth, D3 拉满起伏增强视差)');
        }
        if ((comp.lowTexFrac > 0.25 || comp.farTopFrac > 0.60) && effFrameCount < 96) {
          effFrameCount = 96;
          console.info('SR3D comp auto: frameCount -> 96 (weak texture / large far-top, 更多关键帧候选)');
        }
      }

      // 工作目录必须已存在（Kit 约束），首次先建
      const workDir: string = SpatialReconService.workDir(ctx);
      if (!fileIo.accessSync(workDir)) {
        fileIo.mkdirSync(workDir, true);
      }
      // ⚠️ 每次生成必须用一个全新的空会话目录：Kit 会把工作目录里已有的数据当作
      // "上次训练的历史现场"而进入 resume 模式，该模式下被选中的关键帧会被直接拒收
      // （Kit 日志：WorkPath exists with historical data, setting mode to resume training /
      //  Resume training mode, can not push frame），零关键帧入库 → STAGE02 no 3D points。
      // 产物 MP4 落父目录，不受会话目录回收影响。
      // 清掉历史遗留（早期失败会话残留 / 崩溃遗留的会话目录），保留产物 MP4
      const removed: number = SpatialReconService.cleanLegacy(ctx);
      if (removed > 0) {
        console.info(`SR3D cleanup removed ${removed} legacy entries`);
      }

      // 尝试序列（2026-10-02 起逐轮收敛 → 2026-10-04 定案改两档：自然档移出阶梯）。
      //   实测矩阵（同图同配置，多轮）：
      //     · 推帧口交换：比例稳（0.985 / 1.009 / 1.009）；高饱和素材匹配崩塌（~4s 快速失败）；
      //     · 贴图前交换：高饱和也能过匹配（颜色正确）；比例可能漂移（1.112~1.266，native 比例
      //       门控 |aspect−1| ≤ 0.25 放行交付，> 0.25 返回 -8 换档/兜底）；
      //     · 自然 RGB：匹配/比例都稳但颜色对调——不再作为自动档位（手动诊断用）。
      //   两档全失败时宁可明确失败（需自然产物时手动关「通道交换」重试）。
      //   native 错误码：-7 = 保存阶段挂死（Kit 编码器停滞，文件不完整，不留）；
      //   -8 = 比例严重漂移（产物完整，留作兜底交付）。
      const userSwap: boolean = adapt?.bgrSwap !== false;
      const riskSwapFirst: boolean = adapt?.riskSwapFirst !== false;   // 默认开（坑 143）
      const highRisk: boolean = swapRisk > SpatialReconService.SWAP_RISK_THRESHOLD;
      const skipSwap: boolean = userSwap && !riskSwapFirst && highRisk;
      const userPos: boolean = adapt?.swapRgbAtInput === true;   // 交换位置（默认推帧口；诊断开关）
      // 阶梯关（测试）：单候选 = 用户选择的那个方式（通道交换开 = 交换@选择的位置，关 = 自然
      // RGB）；高危预判跳档（skipSwap）不参与——显式开关优先。本档自身的产物自检/比例门控
      // 兜底交付不变（交付的仍是所选方式的产物，不属"换档回退"）。
      const candidates: SpatialReconAttempt[] = !ladderOn
        ? [userSwap
          ? { sc: 5, swap: true, atInput: userPos }
          : { sc: 5, swap: false, atInput: false }]
        : skipSwap
          ? [{ sc: 5, swap: false, atInput: false }]      // 旧行为（诊断）：高危跳档，自然单跑
          : userSwap
            ? [
                { sc: 5, swap: true, atInput: userPos },    // 第 1 档：交换@首选位置（默认推帧口）
                { sc: 5, swap: true, atInput: !userPos },   // 第 2 档：交换@另一位置（默认贴图前）
              ]
            : [{ sc: 5, swap: false, atInput: false }];    // 交换关 = 自然单档（显式选择，不再暗中回退交换）
      let lastErrMsg: string = '';
      /** 被判"垃圾成功"的产物（后续档位全部失败时的兜底交付） */
      let junkPath: string = '';
      let junkDetail: string = '';
      /** 被比例门控拦截（-8）的产物：保存完整、只是纵横比严重漂移（坑 145 兜底，优先于垃圾产物） */
      let driftPath: string = '';
      let driftDetail: string = '';
      for (let attempt = 0; attempt < candidates.length; attempt++) {
        const sc: number = candidates[attempt].sc;
        const dir: string = SpatialReconService.freshKitDir(ctx);
        fileIo.mkdirSync(dir);
        runDirs.push(dir);
        const opts: SpatialReconTestOptions = {
          workPath: dir + '/',
          modelPath: workDir + `/model_${Date.now()}.mp4`,
          rgbaBuffer: buf,
          srcWidth: decoded.srcW,
          srcHeight: decoded.srcH,
          invertQuat: adapt?.invertQuat ?? true,
          roll180: adapt?.roll180 === true,
          worldFlip180: adapt?.worldFlip180 !== false,   // 默认开（抵消 Kit 模型 180° 翻转）
          poseFamily: adapt?.poseFamily ?? 0,
          focalPx: adapt?.focalPx ?? 750,   // 默认 750（典型广角先验；"宽度压缩修复值"结论已证伪，见 坑 131）
          orbitDeg: adapt?.orbitDeg ?? 5,   // 轨道半角 ±Θ：scene=5 默认 5（判据口径），对角弧 Φ=Θ；native 侧带自适应钳
          frameCount: effFrameCount,   // 构图初档（智能调整后的有效值；日志 `SR3D comp auto`）
          depthEdge: adapt?.depthEdge ?? 1,   // D1/D4（默认 1=增强；改默认须真机 A/B，见坑 147/154）
          adaptiveK: effAdaptiveK,   // D3（默认 1=自适应；智能初档可在用户选「固定」时拉回 1）
          subjectAlign: adapt?.subjectAlign ?? 1,   // D2（默认 1=开；需蒙版，上方已按需构建）
          scene: sc,
          // 交换**位置**（坑 145 阶梯）：由尝试档决定（首选位 → 另一位），两者互斥，
          // "本档是否补偿"的语义仍由 candidates[attempt].swap 表达。
          bgrSwap: candidates[attempt].swap && !candidates[attempt].atInput,
          swapRgbAtInput: candidates[attempt].swap && candidates[attempt].atInput,
          rgbaInput: adapt?.rgbaInput === true,
          subjectDepth: adapt?.subjectDepth ?? SpatialReconService.DEFAULT_SUBJECT_DEPTH,
          // 坑 146 后续：同会话双产物——每档成功后同时落一份 PLY 侧车（失败不致命）。
          // 用途：模型几何/颜色的唯一可解剖产物（模型侧离线诊断的输入源）。
          // 「导出 PLY」优先用这份侧车，不再依赖 spatialEdit.saveToPLY（真机恒失败）。
          alsoPly: true,
        };
        // mask 非空判定：ArkTS 不做跨变量的类型收窄，故取局部变量再判一次
        const m: ArrayBuffer | null = mask;
        if (m !== null) {
          opts.subjectMask = m;
        }
        // scene=5：把端侧深度（原始相对逆深度）一并传入（bridge 侧构建三层网格）
        if (sc === 5 && depthBuf !== null && depthW > 0 && depthH > 0) {
          opts.depthBuffer = depthBuf;
          opts.depthWidth = depthW;
          opts.depthHeight = depthH;
        }
        SpatialReconService.phaseVal = SpatialReconPhase.RUN;   // 阶段：native 合成 + 重建（进度由 native 给）
        try {
          const produced: string = await spatialReconTestGenerate(opts);
          // 产物自检（坑 143）：交换档"成功"不等于产物可用——真机 0.617 案例是"重建成功但
          // 初始视角全灰/旋转后混乱"，且成功态会阻断后续重试。这里在采用前判一次，异常则退回下一档。
          if (riskSwapFirst && candidates[attempt].swap) {
            const chk: ProductCheck =
              await SpatialReconService.checkProduct(produced, buf, decoded.srcW, decoded.srcH);
            if (!chk.ok) {
              junkPath = produced;
              junkDetail = chk.detail;
              console.warn(`SR3D junk success on swap attempt ${attempt} (${chk.detail}), fall back`);
              // 用户可见提示（2026-10-05）：多候选模式下（阶梯回退/智能构图）出现"检验不合格→
              // 换下一候选"，必须让用户知道发生了什么，否则只会看到一次漫长的等待。
              if (ladderOn || adapt?.smartCompose === true) {
                SpatialReconService.retryNoteVal = '本次重建结果不可用，正在尝试新的效果…';
              }
              continue;
            }
          }
          decoded.modelPath = produced;
          // UI 文案纪律：只说结果与下一步，不写机理。**仅自然档（未交换）交付才提示颜色可能有偏差**
          // ——坑 145 阶梯下"另一位置的交换档"交付时颜色是对的，不能误报（旧逻辑只看 attempt>0）。
          if (!candidates[attempt].swap && (attempt > 0 || skipSwap)) {
            decoded.layerNote = `${decoded.layerNote.length > 0 ? decoded.layerNote + '；' : ''}已按备用方案生成，颜色可能有偏差`;
          }
          console.info(`SR3D ok attempt=${attempt} swap=${candidates[attempt].swap} ` +
            `atInput=${candidates[attempt].atInput}${skipSwap ? ', skipSwap' : ''}`);
          break;
        } catch (e) {
          lastErrMsg = typeof e === 'string' ? e : ((e as Error).message || String(e));
          // 用户终止（native 返回 -6）：不再尝试备用档——否则第二次尝试会在 native 入口把
          // g_srAbort 清零，用户点了「终止」反而继续跑完整轮（坑 137）。直接上抛，页面显示"已终止"。
          if (lastErrMsg.includes('-6')) {
            console.info('SR3D aborted by user, stop retry');
            throw new Error(lastErrMsg);
          }
          // 坑 145：-8 = 比例门控拦截（保存已完成、文件完整，只是纵横比严重漂移）——
          // 留作全档失败时的兜底交付；-7 = 保存挂死（文件不完整），不留。
          if (lastErrMsg.includes('-8')) {
            driftPath = opts.modelPath;
            driftDetail = lastErrMsg;
            // 同上：多候选模式下把"这一档不合格、正在换下一档"如实告诉用户（最后还有候选才提示）
            if ((ladderOn || adapt?.smartCompose === true) && attempt + 1 < candidates.length) {
              SpatialReconService.retryNoteVal = '本次重建结果不可用，正在尝试新的效果…';
            }
          }
          console.warn(`SR3D scene=${sc} swap=${candidates[attempt].swap} ` +
            `atInput=${candidates[attempt].atInput} failed (${lastErrMsg}), next attempt`);
        }
      }
      if (decoded.modelPath.length === 0) {
        // 兜底：交换档产物被门控判为异常、而后续档位也未成功时，仍交付"最不坏"的产物
        // （用户至少拿到一个文件；状态行给出"画面可能有偏差"，不与"颜色可能有偏差"混用）。
        // 优先级（坑 145）：比例漂移产物（完整出片、仅纵横比变形）> 垃圾成功产物（全灰/崩塌）。
        if (driftPath.length > 0 || junkPath.length > 0) {
          const useDrift: boolean = driftPath.length > 0;
          decoded.modelPath = useDrift ? driftPath : junkPath;
          decoded.layerNote = `${decoded.layerNote.length > 0 ? decoded.layerNote + '；' : ''}已按备用方案生成，画面可能有偏差`;
          console.warn(`SR3D all attempts exhausted, deliver degraded product (${useDrift ? `aspect drift: ${driftDetail}` : junkDetail})`);
        } else {
          throw new Error(lastErrMsg.length > 0 ? lastErrMsg : 'spatialReconTestGenerate failed');
        }
      }
      await SpatialReconService.logOutputProbe(decoded.modelPath);
      return decoded;
    } finally {
      SpatialReconService.phaseVal = SpatialReconPhase.IDLE;
      // 会话已在 native 侧销毁：回收本次会话数据（关键帧等，体积可观；每次尝试一个目录）
      for (let i = 0; i < runDirs.length; i++) {
        try {
          fileIo.rmdirSync(runDirs[i]);
        } catch (e) {
          // 回收失败不影响主流程
        }
      }
      if (pixelMap !== null) {
        void pixelMap.release();
      }
      if (source !== null) {
        void source.release();
      }
      if (file !== null) {
        try {
          fileIo.closeSync(file);
        } catch (e) {
          // 关闭失败不影响主流程
        }
      }
    }
  }

  /** 预置数据集 manifest 的帧条目（rawfile/preset/earth/manifest.json，离线抽取器产出） */
  private static parsePresetManifest(raw: Uint8Array): PresetManifest {
    const textDecoder: util.TextDecoder = util.TextDecoder.create('utf-8', { ignoreBOM: true });
    const text: string = textDecoder.decodeWithStream(raw, { stream: false });
    return JSON.parse(text) as PresetManifest;
  }

  /** 小睡（预置灌帧的背压退让：让出事件循环，替代阻塞式等待） */
  private static delay(ms: number): Promise<void> {
    return new Promise<void>((resolve: () => void): void => {
      setTimeout(resolve, ms);
    });
  }

  /** 解码一帧预置 JPEG → RGBA 1080×1440 紧凑缓冲（复用工程已验证的 image kit 解码链路） */
  private static async decodePresetFrame(data: Uint8Array): Promise<ArrayBuffer> {
    const ab: ArrayBuffer = data.buffer.slice(data.byteOffset, data.byteOffset + data.byteLength);
    const src: image.ImageSource = image.createImageSource(ab);
    let pm: image.PixelMap | null = null;
    try {
      pm = await src.createPixelMap({ desiredPixelFormat: image.PixelMapFormat.RGBA_8888 });
      const info: image.ImageInfo = pm.getImageInfoSync();
      if (info.size.width !== 1080 || info.size.height !== 1440) {
        throw new Error(`preset frame ${info.size.width}x${info.size.height} != 1080x1440`);
      }
      const buf: ArrayBuffer = new ArrayBuffer(pm.getPixelBytesNumber());
      await pm.readPixelsToBuffer(buf);
      return buf;
    } finally {
      if (pm !== null) {
        void pm.release();
      }
      void src.release();
    }
  }

  /**
   * 预置建模（rawfile/preset/<id> 预置数据集）：按目标帧数对数据集**等距抽样**后逐帧解码
   * 灌入 native 预置帧队列 → 端侧 3DGS 重建 → MP4 沙箱路径。位姿由 manifest 的
   * thetaDeg/cameraRadius 闭环给出（数据集由 tools/depth_mesh/earth_scene.py 离线生成，
   * 帧与位姿声明**同源同参数**；一致性由 tools/earth_preset_check.js 静态核验），
   * 不经源图合成。
   * ⚠️ 真机实测：120 帧全量喂给 Kit 构建约 2 小时（进度约 3%/分，远重于合成帧路线
   * 45~80s 量级）+ GPU device-lost 复位风暴——故由调用方给目标帧数（编辑页档位，
   * 抽样恒含首帧 θ0），起步小档 24 帧逐步向上一档试。
   * 尝试序列：交换档 → 自然档。⚠️ 交换档在部分素材上会直接匹配失败（`STAGE02 no 3D points`
   * 一类快速失败），退回自然档才出片 ⇒ 素材饱和度（交换后色彩分布大变）会影响学习型匹配器。
   * 修法方向 = 素材侧（海洋带结构 + 低饱和），见 tools/depth_mesh。
   * 产物语义（2026-10-05 起）：比例门控（-8）与产物自检退档在预置路线均已放开——那两道是
   * 我们自己的检测，曾有轮次被 -8 拦下而成片形态其实是有效证据；保存成功即交付，
   * 自检异常只打日志；仅真故障（解码失败 / -6 用户终止 / 两档 native 全败）才报失败。
   */
  static async generatePresetEarth(ctx: common.UIAbilityContext,
    framesWanted?: number, presetId?: string): Promise<SpatialReconResult> {
    // 数据集选择：rawfile/preset/<presetId>/（目录名即数据集 id；每个数据集自带 manifest.json
    // ——帧清单 + 环绕半径 + 位姿参数域等）
    const dataDir: string = 'preset/' + (presetId && presetId.length > 0 ? presetId : 'earth') + '/';
    const decoded: SpatialReconResult = new SpatialReconResult();
    SpatialReconService.phaseVal = SpatialReconPhase.PREPARE;
    SpatialReconService.retryNoteVal = '';
    const runDirs: string[] = [];
    try {
      const rm = ctx.resourceManager;
      const man: PresetManifest =
        SpatialReconService.parsePresetManifest(await rm.getRawFileContent(dataDir + 'manifest.json'));
      console.info(`SR3D preset dataset=${dataDir}`);
      const count: number = man.count;
      if (!(count >= 2 && man.frames.length === count && man.cameraRadius > 1)) {
        throw new Error(`preset manifest invalid: count=${count} frames=${man.frames.length}`);
      }
      // 帧数选择：对数据集等距抽样（含首帧 θ0 与末帧），跨度不变、步距拉大
      let want: number = Math.round(framesWanted ?? count);
      if (!(want >= 2)) {
        want = 2;
      }
      if (want > count) {
        want = count;
      }
      const theta: number[] = [];
      const files: string[] = [];
      for (let i = 0; i < want; i++) {
        const idx: number = Math.round(i * (count - 1) / (want - 1));
        theta.push(man.frames[idx].thetaDeg);
        files.push(man.frames[idx].file);
      }
      const stepDeg: number = theta.length > 1 ? Math.abs(theta[theta.length - 1] - theta[0]) / (theta.length - 1) : 0;
      console.info(`SR3D preset select frames=${want}/${count} step=${stepDeg.toFixed(2)}deg`);
      // 参考帧（产物自检用，与 native 侧首帧同一文件）
      const refBuf: ArrayBuffer =
        await SpatialReconService.decodePresetFrame(await rm.getRawFileContent(dataDir + files[0]));
      const removed: number = SpatialReconService.cleanLegacy(ctx);
      if (removed > 0) {
        console.info(`SR3D cleanup removed ${removed} legacy entries`);
      }
      const workDir: string = SpatialReconService.workDir(ctx);
      if (!fileIo.accessSync(workDir)) {
        fileIo.mkdirSync(workDir, true);
      }
      // 位姿约定 = 照片生成真机定论组合（AR 系 + 世界→相机 + 世界翻转补偿）；交换两档兜底
      const swaps: boolean[] = [true, false];
      let lastErrMsg: string = '';
      // 交换两档：先交换、后自然（两档成片颜色表现不同，见档案 F7）
      for (let attempt = 0; attempt < swaps.length; attempt++) {
        const dir: string = SpatialReconService.freshKitDir(ctx);
        fileIo.mkdirSync(dir);
        runDirs.push(dir);
        const opts: SpatialReconTestOptions = {
          workPath: dir + '/',
          modelPath: workDir + `/model_${dataDir.includes('africa') ? 'earth_africa' : 'earth'}_${Date.now()}.mp4`,
          preset: true,
          presetTheta: theta,
          presetRadius: man.cameraRadius,
          focalPx: man.focalPx > 0 ? man.focalPx : 750,
          focalYratio: (man.focalYRatio != null && man.focalYRatio > 0) ? man.focalYRatio : 1,
          frameCount: theta.length,
          invertQuat: true,
          // 上报位姿的世界翻转：**只影响上报位姿，不合成帧**（图像与"旋转后的世界"仍自洽
          // ⇒ 重建等价），用途 = 抵消 Kit 导出时绕水平轴的 180° 翻转。该翻转**随内容变**
          // （实测同一构建下不同素材的导出朝向不一致），故由数据集声明：
          // manifest.worldFlip180（缺省 true = 历史口径，照片路线不受影响）。
          worldFlip180: man.worldFlip180 !== false,
          poseFamily: 0,
          bgrSwap: swaps[attempt],
          alsoPly: true,
        };
        // 位姿参数域由 manifest 决定：**优先显式 poseDomain**（'diag-arc' = 相机位置同时扫
        // yaw+pitch，破 (纵向尺度, fy) 精确退化，与照片路线 scene=5 同口径）；缺省按
        // camPitchDeg 平环俯仰 / camAxis 轴式环择一。各域互斥，native 侧有防呆。
        // ArkTS 不做跨变量类型收窄，先取局部变量再判
        if (man.poseDomain === 'diag-arc') {
          opts.presetDiagArc = true;
          console.info('SR3D preset poseDomain=diag-arc (photo-route Phi=Theta 口径移植)');
        } else {
          const pitchDeg: number | undefined = man.camPitchDeg;
          if (pitchDeg != null) {
            opts.presetPitchDeg = pitchDeg;
          } else {
            opts.presetAxis = man.camAxis ?? [0, 1, 0];
          }
        }
        SpatialReconService.phaseVal = SpatialReconPhase.RUN;
        // 先发起（native 异步起跑、在推帧循环里阻塞消费），再逐帧解码灌队。
        // 背压经 await 退让（**不阻塞 UI 线程**）：队列满（Push 返回 1）→ 小睡后重试；
        // 生成已结算（成功/失败/终止）→ 停灌；-6（终止/消费端离开/生产失败）→ 停灌。
        let genSettled: boolean = false;
        const genPromise: Promise<string> = spatialReconTestGenerate(opts)
          .then((p: string): string => {
            genSettled = true;
            return p;
          })
          .catch((e: Error): string => {
            genSettled = true;
            throw e;
          });
        let producerErr: string = '';
        // ⚠️ 上界必须是抽样后的帧数 files.length（不是数据集总量 count）——2026-10-04 首测 bug：
        // 用 count 会在第 want 次迭代读 files[want]=undefined → GetRawFileContent failed
        for (let i = 0; i < files.length; i++) {
          let rgba: ArrayBuffer;
          try {
            rgba = await SpatialReconService.decodePresetFrame(
              await rm.getRawFileContent(dataDir + files[i]));
          } catch (e) {
            producerErr = `preset frame ${i} decode failed: ${((e as Error).message || String(e))}`;
            break;
          }
          let rc: number = spatialReconPresetPush(rgba);
          while (rc === 1 && !genSettled) {
            await SpatialReconService.delay(8);
            rc = spatialReconPresetPush(rgba);
          }
          if (rc !== 0) {
            console.info(`SR3D preset push stop at ${i}/${count} rc=${rc} settled=${genSettled ? 1 : 0}`);
            break;
          }
        }
        if (producerErr.length > 0) {
          // 生产端失败（解码错误）：放行 native（其因队列放行以 -6 退出）——等待结算回收
          // Promise 后上抛生产错误本身（与"用户终止"区分；解码失败换档重试也无意义）
          console.warn(`SR3D ${producerErr}`);
          spatialReconPresetFail();
          try {
            await genPromise;
          } catch (e) {
            // 预期中的 -6：此处只回收，不掩盖生产错误
          }
          throw new Error(producerErr);
        }
        try {
          const produced: string = await genPromise;
          // 产物自检（与照片路线 checkProduct 同款语义）：参考帧对比。⚠️ 预置路线**不因自检
          // 失败退档**——自检与比例门控都是我们自己的检测，曾有轮次被其拦下而成片形态其实
          // 是有效证据；chk.detail 照打供判读、本档产物直接交付。
          const chk: ProductCheck =
            await SpatialReconService.checkProduct(produced, refBuf, 1080, 1440);
          if (!chk.ok) {
            console.warn(`SR3D preset self-check flagged attempt ${attempt} (${chk.detail}) — deliver anyway (evidence mode)`);
          }
          decoded.modelPath = produced;
          if (!swaps[attempt]) {
            decoded.layerNote = '已按备用方案生成，颜色可能有偏差';
          }
          console.info(`SR3D preset ok attempt=${attempt} swap=${swaps[attempt]}`);
          // 模型几何自检：模型侧唯一可解剖产物（PLY 侧车）位于应用沙箱，
          // 主机侧 `hdc file recv`/`ls` 对该目录 permission denied（真机实测）⇒ 把关键几何统计量
          // **打成日志**（`SR3D plydiag`，`hdc shell hilog -x | grep plydiag` 取回）：
          // 黑洞是"缺几何"还是"黑点挡前"、以及 R*（模型里球壳半径，与 `glmmeta initialCamera`
          // 的 |pos| 相除即 camR/R*，与数据集声明的 cameraRadius 对照）。只读 + 只打日志，
          // 失败不影响交付。
          try {
            // 侧车命名与 native 侧同规则：`.mp4` → `.ply`，否则直接追加
            const plyPath: string = produced.endsWith('.mp4')
              ? produced.substring(0, produced.length - 4) + '.ply' : produced + '.ply';
            EarthPlyReport.run(plyPath);
          } catch (e) {
            console.warn(`SR3D plydiag skipped ${(e as Error).message || String(e)}`);
          }
          break;
        } catch (e) {
          lastErrMsg = typeof e === 'string' ? e : ((e as Error).message || String(e));
          // 用户终止（-6）：直接上抛（坑 137：终止不是失败，不换档重试）
          if (lastErrMsg.includes('-6')) {
            console.info('SR3D preset aborted by user, stop retry');
            throw new Error(lastErrMsg);
          }
          console.warn(`SR3D preset attempt ${attempt} swap=${swaps[attempt]} failed (${lastErrMsg}), next attempt`);
        }
      }
      if (decoded.modelPath.length === 0) {
        throw new Error(lastErrMsg.length > 0 ? lastErrMsg : 'preset generation failed');
      }
      await SpatialReconService.logOutputProbe(decoded.modelPath);
      return decoded;
    } finally {
      SpatialReconService.phaseVal = SpatialReconPhase.IDLE;
      for (let i = 0; i < runDirs.length; i++) {
        try {
          fileIo.rmdirSync(runDirs[i]);
        } catch (e) {
          // 回收失败不影响主流程
        }
      }
    }
  }

  /**
   * **智能构图初档 · 照片描述子（层 1，2026-10-04）**：RGBA 缓冲 + 深度场（生成管线里本就在
   * 重建前跑完）就地统计，零额外推理。只描述、不改参数——参数决策在 generateFromImage 里
   * （且仅在用户保留默认值时调整）。深度缺失（关「深度预估」/推理不可用）时纹理描述子仍
   * 有效、深度类描述子保持保守值。口径与离线诊断一致（逆光海边样本）。
   */
  private static analyzePhotoComp(rgba: ArrayBuffer, w: number, h: number,
    depth: ArrayBuffer | null, dw: number, dh: number): PhotoCompProfile {
    const prof: PhotoCompProfile = { bimodal: false, lowTexFrac: 0, farTopFrac: 0, nearFrac: 0 };
    // 低纹理：16×12 网格、块内梯度均值（步长 4 采样；阈值 6 与离线诊断脚本同口径）
    {
      const u8: Uint8Array = new Uint8Array(rgba);
      const GX: number = 16;
      const GY: number = 12;
      let low: number = 0;
      let tot: number = 0;
      for (let gy = 0; gy < GY; gy++) {
        const y0: number = Math.floor(h * gy / GY);
        const y1: number = Math.floor(h * (gy + 1) / GY);
        for (let gx = 0; gx < GX; gx++) {
          const x0: number = Math.floor(w * gx / GX);
          const x1: number = Math.floor(w * (gx + 1) / GX);
          let gsum: number = 0;
          let cnt: number = 0;
          for (let y = y0; y < y1; y += 4) {
            for (let x = x0; x < x1; x += 4) {
              if (x + 1 < w && y + 1 < h) {
                const o: number = (y * w + x) * 4;
                gsum += Math.abs(u8[o + 4] - u8[o]) + Math.abs(u8[o + w * 4] - u8[o]);
                cnt++;
              }
            }
          }
          tot++;
          if (cnt > 0 && gsum / cnt < 6) {
            low++;
          }
        }
      }
      prof.lowTexFrac = tot > 0 ? low / tot : 0;
    }
    if (depth !== null && dw > 0 && dh > 0) {
      const f32: Float32Array = new Float32Array(depth);
      const sample: number[] = [];
      for (let i = 0; i < f32.length; i += 2) {
        sample.push(f32[i]);
      }
      sample.sort((a: number, b: number): number => a - b);
      const p1: number = sample[Math.floor(sample.length * 0.01)];
      const p99: number = sample[Math.floor(sample.length * 0.99)];
      const span: number = p99 - p1;
      if (span > 1e-6) {
        let near: number = 0;
        let far: number = 0;
        let farTop: number = 0;
        let topTot: number = 0;
        let tot: number = 0;
        for (let i = 0; i < f32.length; i += 2) {
          const s: number = Math.max(0, Math.min(1, (f32[i] - p1) / span));
          if (s > 0.5) {
            near++;
          }
          if (s < 0.2) {
            far++;
          }
          if (Math.floor(i / dw) < dh * 0.45) {
            topTot++;
            if (s < 0.05) {
              farTop++;
            }
          }
          tot++;
        }
        prof.nearFrac = tot > 0 ? near / tot : 0;
        prof.bimodal = prof.nearFrac > 0.25 && tot > 0 && far / tot > 0.25;
        prof.farTopFrac = topTot > 0 ? farTop / topTot : 0;
      }
    }
    return prof;
  }

  /**
   * 路线B 阶段1 自检：对一张照片跑**端侧深度估计**（MindSpore Lite + Depth Anything V2 Small .ms）。
   * 产物：`<filesDir>/depth_selftest/depth_<ts>.depth.f32.bin`（原始相对逆深度 float32，越大越近）
   *      + 同名 `.meta.txt`（统计）；返回 native 统计串，供与离线判据的结果比对。
   * 失败抛错（运行时不可用 / 模型不可用 / 推理失败），message 带诊断串。
   */
  static async depthSelfTest(ctx: common.UIAbilityContext, uri: string): Promise<string> {
    const probe: string = depthProbe();
    console.info(`SRD probe ${probe}`);
    if (!probe.startsWith('rc=0')) {
      throw new Error(`推理运行时不可用：${probe}`);
    }
    let file: fileIo.File | null = null;
    let source: image.ImageSource | null = null;
    let pixelMap: image.PixelMap | null = null;
    try {
      file = fileIo.openSync(uri, fileIo.OpenMode.READ_ONLY);
      source = image.createImageSource(file.fd);
      const info: image.ImageInfo = await source.getImageInfo();
      const scale: number = Math.min(1,
        SpatialReconService.MAX_DECODE_SIDE / Math.max(info.size.width, info.size.height));
      const options: image.DecodingOptions = {
        desiredSize: {
          width: Math.max(1, Math.round(info.size.width * scale)),
          height: Math.max(1, Math.round(info.size.height * scale))
        },
        desiredPixelFormat: image.PixelMapFormat.RGBA_8888
      };
      pixelMap = await source.createPixelMap(options);
      const di: image.ImageInfo = pixelMap.getImageInfoSync();
      const buf: ArrayBuffer = new ArrayBuffer(pixelMap.getPixelBytesNumber());
      await pixelMap.readPixelsToBuffer(buf);
      const dir: string = ctx.filesDir + '/depth_selftest';
      if (!fileIo.accessSync(dir)) {
        fileIo.mkdirSync(dir, true);
      }
      const outPrefix: string = `${dir}/depth_${Date.now()}`;
      console.info(`SRD selftest start ${di.size.width}x${di.size.height} -> ${outPrefix}`);
      const opts: DepthSelfTestOptions = {
        rgbaBuffer: buf,
        width: di.size.width,
        height: di.size.height,
        outPrefix: outPrefix,
        resMgr: ctx.resourceManager,
      };
      const res: string = await depthEstimateTest(opts);
      console.info(`SRD selftest ok ${res}`);
      return res;
    } finally {
      if (pixelMap !== null) {
        void pixelMap.release();
      }
      if (source !== null) {
        void source.release();
      }
      if (file !== null) {
        try {
          fileIo.closeSync(file);
        } catch (e) {
          // 关闭失败不影响主流程
        }
      }
    }
  }
}
