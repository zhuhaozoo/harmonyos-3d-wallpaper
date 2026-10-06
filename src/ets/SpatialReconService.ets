/**
 * 3D壁纸 · Spatial Recon Kit 管线服务
 *
 * 职责：把相册图片解码为 RGBA（分层深度场景另做一次主体分割取蒙版）后交给 native 管线
 * （合成已知位姿帧序列 → 端侧 3DGS 重建 → MP4），产出沙箱 MP4 交由页面保存到图库。
 * 设备支持性探测与进度查询走同一条 native 桥。
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
import { subjectSegmentation } from '@kit.CoreVisionKit';
import {
  spatialReconProbe, spatialReconTestGenerate, getSpatialReconProgress,
  spatialReconPause, spatialReconResume, spatialReconAbort,
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
   *  （坑 134 的"A/B 定案"已撤销，见档案 F7）——默认值与"失败自动反交换重试"属工程折中；
   *  传 false 可手动直通 RGB */
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
}

/** 生成尝试档（坑 134）：场景 + 通道交换组合（ArkTS 要求对象字面量对应显式声明的接口） */
interface SpatialReconAttempt {
  /** 合成场景（固定 5 = 深度网格） */
  sc: number;
  /** 推帧是否交换 R/B：首轮 = 用户/默认（开）；失败重试轮 = 反交换。
   *  成因未定论（见档案 F7），此处仅为工程上的试错序列 */
  swap: boolean;
}

export class SpatialReconService {
  /** 源图解码长边上限（验证管线用；产品化的基准尺寸另行设计） */
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

  /** 设备支持性：0=支持；-1=能力库绑定失败；801=设备不支持（Kit 状态码） */
  static probe(): number {
    return spatialReconProbe();
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

  /** 请求终止本次生成（保存阶段会被忽略：产物照常保存并返回成功） */
  static abort(): number {
    return spatialReconAbort();
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
   * 保留 workDir 下的产物 model_*.mp4。返回清理掉的条目数（>0 时打日志）。
   */
  static cleanLegacy(ctx: common.UIAbilityContext): number {
    let removed: number = 0;
    const legacy: string = SpatialReconService.workDir(ctx);
    if (fileIo.accessSync(legacy)) {
      for (const name of fileIo.listFileSync(legacy)) {
        if (name.startsWith('model_') && name.endsWith('.mp4')) {
          continue;   // 保留产物
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
   * 长度不可信时回退读前景图 alpha（项目既有做法，见 RippleOpeningService.buildSubjectMask，
   * 并做 getPixelBytesNumber 校验防 DMA 行跨距，坑 65）。
   *
   * @returns null = 未识别到主体（初始化失败/结果缺失/尺寸不符/主体占比过小）——
   *          调用方按单层立体场景生成，不阻断流程。
   */
  /** 居中裁剪到 3:4（原地）。Kit 只认 1080×1440；非 3:4 照片先裁剪，
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
      console.info(`SR3D src decoded=${decoded.srcW}x${decoded.srcH}`);

      // 需要主体的场景（scene=3 分层深度 / scene=4 盒中空间）：先做主体分割
      //（用与 native 完全同源的解码位图，蒙版尺寸必然一致）。
      // 拿不到蒙版时不阻断：scene=4 的盒内空间几何不依赖蒙版（只是没有主体层）；
      // scene=3 在 native 侧会自动退回 scene=1（单层立体）。
      let mask: ArrayBuffer | null = null;
      const wantedScene: number = adapt?.scene ?? 1;
      const wantsMask: boolean = (wantedScene === 3 || wantedScene === 4);
      if (wantsMask) {
        mask = await SpatialReconService.buildSubjectMask(pixelMap, decoded.srcW, decoded.srcH);
        if (mask === null) {
          decoded.layerNote = wantedScene === 4
            ? '未识别到主体：按空间场景生成'
            : '未识别到主体：按整图生成';
        } else {
          decoded.layerNote = '已识别主体：主体/背景分层生成';
        }
      }

      const buf: ArrayBuffer = new ArrayBuffer(pixelMap.getPixelBytesNumber());
      await pixelMap.readPixelsToBuffer(buf);

      // 输入端预判（坑 134 的成因判断已撤销，见档案 F7）：以 |R−B| 均值作为素材"色彩饱和
      // 程度"的粗略度量，仅用于挑选本轮的试错序列——高饱和素材在交换档上曾出现"重建成功但
      // 模型不可用"（该状态会阻断重试），故风险高时直接走自然 RGB 单档。
      // ⚠️ 阈值 0.5 来自少量样本的工程标定，不是结论；可随数据积累调整。
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
        console.info(`SR3D precheck swapRisk=${swapRisk.toFixed(3)} px=${cnt} ` +
          `skipSwap=${swapRisk > 0.5 ? 1 : 0}（>0.5 跳过交换档：防"成功但模型不可用"阻断重试）`);
      }

      // 路线B 深度网格（scene=5）：先跑端侧深度估计（复用阶段1 已验证的推理链路），再把 .bin 回读为参数。
      // 深度与贴图同源同尺寸（都是这张裁剪后的位图）→ 网格 UV 与深度网格 1:1 对齐。
      let depthBuf: ArrayBuffer | null = null;
      let depthW: number = 0;
      let depthH: number = 0;
      if (wantedScene === 5) {
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

      // 尝试序列（工程上的试错序列；成因未定论，见档案 F7）：
      //   常规：scene=5 主推，通道顺序按用户/默认值；失败自动反交换重试一次。
      //   高 |R−B| 素材（预判）：直接跳过交换档——该档曾出现"重建成功但模型不可用"且成功态
      //   会阻断重试（比失败更不利）；改为单次自然 RGB 生成，重建结果更干净（颜色可能对调，
      //   状态行会提示）。
      const userSwap: boolean = adapt?.bgrSwap !== false;
      const skipSwap: boolean = userSwap && swapRisk > 0.5;
      const candidates: SpatialReconAttempt[] = skipSwap
        ? [{ sc: 5, swap: false }]
        : [
            { sc: 5, swap: userSwap },
            { sc: 5, swap: !userSwap },
          ];
      let lastErrMsg: string = '';
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
          scene: sc,
          bgrSwap: candidates[attempt].swap,   // 坑 134：首轮 = 用户/默认（开）；失败重试轮 = 反交换
          rgbaInput: adapt?.rgbaInput === true,
          subjectDepth: adapt?.subjectDepth ?? SpatialReconService.DEFAULT_SUBJECT_DEPTH,
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
        try {
          decoded.modelPath = await spatialReconTestGenerate(opts);
          if (attempt > 0 || skipSwap) {
            // UI 文案纪律：只说结果与下一步，不写机理
            decoded.layerNote = `${decoded.layerNote.length > 0 ? decoded.layerNote + '；' : ''}已按备用方案生成，颜色可能有偏差`;
            console.info(`SR3D fallback ok bgrSwap=${candidates[attempt].swap} (attempt ${attempt}${skipSwap ? ', skipSwap' : ''})`);
          }
          break;
        } catch (e) {
          lastErrMsg = typeof e === 'string' ? e : ((e as Error).message || String(e));
          console.warn(`SR3D scene=${sc} bgrSwap=${candidates[attempt].swap} failed (${lastErrMsg}), next attempt`);
        }
      }
      if (decoded.modelPath.length === 0) {
        throw new Error(lastErrMsg.length > 0 ? lastErrMsg : 'spatialReconTestGenerate failed');
      }
      await SpatialReconService.logOutputProbe(decoded.modelPath);
      return decoded;
    } finally {
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

  /**
   * 路线B 阶段1 自检：对一张照片跑**端侧深度估计**（MindSpore Lite + Depth Anything V2 Small .ms）。
   * 产物：`<filesDir>/depth_selftest/depth_<ts>.depth.f32.bin`（原始相对逆深度 float32，越大越近）
   *      + 同名 `.meta.txt`（统计）；返回 native 统计串，供与离线判据结果比对。
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
