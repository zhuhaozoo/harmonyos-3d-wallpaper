/**
 * 3D壁纸子页面（步骤2：管线验证壳）
 * 流程：选图 → 生成（合成已知位姿帧 → 端侧重建 → MP4）→ 保存到图库。
 * 生成完成后用户可在图库中把产物设置为 3D 影像壁纸。
 *
 * 场景固定「深度网格」（scene=5，路线B 主推：端侧深度 → 三层网格）；
 * 其余旧测试场景在当前 Kit 版本下无法正确生成，已从 UI 移除。
 *
 * 本页只跑**照片路线**（SpatialReconService.generateFromImage）。另一条**预置路线**
 * （SpatialReconService.generatePresetEarth：离线合成数据集 → 预置帧队列 → 重建，
 * 用于"地球"等预置成品）不走本页 UI，驱动方式见服务注释与 tools/earth_preset_check.js。
 * 质量增强档（深度边缘 D1/D4、自适应纵深 D3、主体对齐 D2）由服务默认开启，
 * 本页不单独暴露开关。
 *
 * 「适配选项」为诊断开关（产品化时整块移除），默认值 = 真机验证命中的组合：
 * - 相机系：AR 系（Y上/Z后，命中） / CV 系（Y下/Z前）——两种坐标标签，像素一致；
 * - 四元数：世界→相机（命中） / 相机→世界；
 * - 画面朝向：标准 / 对调（图像+位姿同步滚，几何不变；实测无法修模型级翻转，留作对照）；
 * - 空间朝向：标准 / **翻转（默认开，命中）**——世界绕世界 X 轴 180°，只转上报位姿，
 *   抵消 Kit 模型导出/输出的 180° 翻转；
 * - 焦距（滑杆 200~6000px）：合成相机焦距，默认 750 = 真实主摄 1080 宽预览典型广角焦距
 *   （"a=750/fx 横向变形"结论已证伪，见坑 131；此滑杆现仅用于视场/视差强度实验）。
 * - 通道交换：推帧前 R/B 交换（可选开关，默认开）；成因未定论（见档案 F7），服务层仅以
 *   预判/重试作兼容性兜底。
 * 「输入」RGB/RGBA 档已移除：官方约束输入仅支持 RGB。
 *
 * 热管理与中断（官方建议）：订阅热公共事件，≥HOT(3) 自动暂停并在降温后（≤NORMAL(1)）自动继续；
 * 同时提供 暂停/继续/终止 三个手动选项（终止在保存阶段会被忽略——保存不可中断）。
 */
import { HdsNavDestination } from '@kit.UIDesignKit';
import { common } from '@kit.AbilityKit';
import { BusinessError, commonEventManager } from '@kit.BasicServicesKit';
import { photoAccessHelper } from '@kit.MediaLibraryKit';
import { MediaKindType, GallerySaver } from './GallerySaver';
import {
  SpatialReconService, SpatialReconAdaptOptions,
} from './SpatialReconService';

/** 多选段控行（适配选项用）：@Prop 承载选中下标（坑 115），回调收口在父组件 */
@Component
struct SrSegRow {
  label: string = '';
  options: string[] = [];
  @Prop selectedIdx: number = 0;
  choose: (idx: number) => void = (_i: number): void => {};

  build() {
    Row({ space: 8 }) {
      Text(this.label)
        .fontSize(13)
        .fontColor($r('app.color.text_secondary'))
        .width(64)
      // 档位多（「场景」6 档）时固定 Row 会横向溢出、末档被裁掉 → 改为横向可滚动；
      // 显式给高度上界，避免 Scroll 撑满父容器（坑 6）
      Scroll() {
        Row({ space: 8 }) {
          ForEach(this.options, (opt: string, idx: number) => {
            Text(opt)
              .fontSize(13)
              .fontColor(idx === this.selectedIdx
                ? $r('app.color.text_on_brand') : $r('app.color.text_secondary'))
              .backgroundColor(idx === this.selectedIdx
                ? $r('app.color.accent_primary') : $r('app.color.bg_input'))
              .borderRadius(14)
              .padding({ left: 12, right: 12, top: 5, bottom: 5 })
              .onClick((): void => {
                this.choose(idx);
              })
          }, (opt: string): string => opt)
        }
      }
      .scrollable(ScrollDirection.Horizontal)
      .scrollBar(BarState.Off)
      .layoutWeight(1)
      .align(Alignment.Start)
      .constraintSize({ maxHeight: 40 })
    }
    .width('100%')
  }
}

/** 数值滑杆行（适配选项用）：label + 滑杆 + 当前值（真机手测定位用） */
@Component
struct SrSliderRow {
  label: string = '';
  @Prop value: number = 0;
  min: number = 0;
  max: number = 100;
  step: number = 1;
  unit: string = '';
  slide: (v: number) => void = (_v: number): void => {};

  build() {
    Row({ space: 8 }) {
      Text(this.label)
        .fontSize(13)
        .fontColor($r('app.color.text_secondary'))
        .width(64)
      Slider({ value: this.value, min: this.min, max: this.max, step: this.step, style: SliderStyle.InSet })
        .layoutWeight(1)
        .onChange((v: number): void => {
          this.slide(Math.round(v));
        })
      Text(`${this.value}${this.unit}`)
        .fontSize(13)
        .fontColor($r('app.color.text_secondary'))
        .width(64)
        .textAlign(TextAlign.End)
    }
    .width('100%')
  }
}

@Component
export struct Wallpaper3DPage {
  @Consume('NavPathStack') pageStack: NavPathStack;
  @StorageProp('mu_is_wide') isWide: boolean = false;
  @State private pickedUri: string = '';
  @State private generating: boolean = false;
  @State private progress: number = 0;
  @State private statusText: string = '';
  @State private invertQuat: boolean = true;
  /** 诊断 A/B：画面绕视轴滚 180°（图像+位姿同步滚）——默认关（实测无法修正模型级翻转，留作对照） */
  @State private roll180: boolean = false;
  /** **默认开**（坑 132）：推帧前交换红蓝通道。该开关会改变成片颜色表现，但成因**尚无定论**
   *  （坑 134 的"A/B 定案"已撤销，见档案 F7）——生成失败时服务层自动反交换重试一次；
   *  本开关保留作手动 A/B */
  @State private bgrSwap: boolean = true;
  /** **默认开**：世界绕世界 X 轴转 180°（只转位姿、图像不变）——抵消 Kit 的模型 180° 翻转
   *  （成片"上下颠倒 + 人物在背景后"），2026-10-01 真机二次复测确认为正确朝向 */
  @State private worldFlip180: boolean = true;
  @State private poseFamily: number = 0;
  /** 合成场景：固定「深度网格」（scene=5，路线B 主推：端侧深度 → 三层网格 → 可设 3D 壁纸）。
   *  2026-10-01 用户指令：其余档位（平面/立体/纯纹理/分层深度/盒中空间）系旧测试场景，
   *  在当前 Kit 版本下无法正确生成，已从 UI 移除（native 侧 scene 参数保留不动） */
  @State private sceneIdx: number = 5;
  /** 合成相机焦距 px（诊断滑杆，范围 = 端侧支持上下限 200~6000）。默认 750 = 真实主摄
   *  1080 宽预览典型广角焦距。⚠️ 坑 131：曾认为"输入焦距偏离 750 会导致成片按 a=750/fx
   *  横向变形"——已证伪（比例失真真因 = 轨道退化，修复在 native 对角弧）；此滑杆现仅用于
   *  视场/视差强度实验 */
  @State private focalPx: number = 750;
  /** 合成轨道半角 ±Θ（度，1~45，默认 5 = 判据口径）。scene=5 下为对角弧（yaw+pitch 同步
   *  各扫 ±Θ，坑 131 修复）。native 侧有位移带自适应钳制（带外值被钳回并打
   *  `SRCHK orbit clamp` 日志），真机日志里的实际 Θ 可能与滑杆值不同 */
  @State private orbitDeg: number = 5;
  @State private showAdvanced: boolean = false;
  /** 路线B 阶段1 深度自检进行中（端侧推理，独立于生成流程） */
  @State private depthTesting: boolean = false;
  /** 最近一次生成的分层说明（主体识别/回退），追加到完成后的状态行 */
  private lastLayerNote: string = '';
  /** 是否处于暂停（手动或过热自动） */
  @State private paused: boolean = false;
  /** 暂停是否由过热自动触发（决定降温后是否自动继续） */
  private pausedByThermal: boolean = false;
  private subscriber: commonEventManager.CommonEventSubscriber | undefined = undefined;
  private pollTimer: number = -1;

  aboutToAppear(): void {
    this.subscribeThermal();
  }

  aboutToDisappear(): void {
    this.stopPoll();
    this.unsubscribeThermal();
  }

  private stopPoll(): void {
    if (this.pollTimer >= 0) {
      clearInterval(this.pollTimer);
      this.pollTimer = -1;
    }
  }

  /** 订阅热公共事件（官方建议：温度过高自动暂停并提示用户） */
  private subscribeThermal(): void {
    if (this.subscriber !== undefined) {
      return;
    }
    const info: commonEventManager.CommonEventSubscribeInfo = {
      events: [commonEventManager.Support.COMMON_EVENT_THERMAL_LEVEL_CHANGED]
    };
    commonEventManager.createSubscriber(info,
      (err: BusinessError, sub: commonEventManager.CommonEventSubscriber) => {
        if (err) {
          console.warn(`SR3D thermal createSubscriber failed: code=${err.code} msg=${err.message}`);
          return;
        }
        this.subscriber = sub;
        commonEventManager.subscribe(sub,
          (e: BusinessError, data: commonEventManager.CommonEventData) => {
            if (e) {
              return;
            }
            const level: number = this.thermalLevelOf(data);
            console.info(`SR3D thermal event code=${data.code} level=${level} params=${JSON.stringify(data.parameters)}`);
            this.onThermalLevel(level);
          });
      });
  }

  private unsubscribeThermal(): void {
    if (this.subscriber !== undefined) {
      commonEventManager.unsubscribe(this.subscriber);
      this.subscriber = undefined;
    }
  }

  /** 取热等级：优先 CommonEventData.code（0~7 合法），其次 parameters['thermalLevel']；都认不出为 -1 */
  private thermalLevelOf(data: commonEventManager.CommonEventData): number {
    const code: number | undefined = data.code;
    if (code !== undefined && code >= 0 && code <= 7) {
      return code;
    }
    const params: Record<string, Object> | undefined =
      data.parameters as Record<string, Object> | undefined;
    if (params === undefined) {
      return -1;
    }
    const raw: Object | undefined = params['thermalLevel'];
    if (raw === undefined) {
      return -1;
    }
    const level: number = Number(raw as number);
    return Number.isFinite(level) && level >= 0 && level <= 7 ? level : -1;
  }

  /** ≥HOT(3) 自动暂停；≤NORMAL(1) 且是自动暂停的才自动继续（避免温度抖动反复暂停/继续） */
  private onThermalLevel(level: number): void {
    if (!this.generating) {
      return;
    }
    if (level >= 3 && !this.paused) {
      const st: number = SpatialReconService.pause();
      console.info(`SR3D thermal auto-pause level=${level} status=${st}`);
      if (st === 0) {
        this.paused = true;
        this.pausedByThermal = true;
        this.statusText = '设备温度较高，已自动暂停；降温后会继续';
      }
      return;
    }
    if (level >= 0 && level <= 1 && this.paused && this.pausedByThermal) {
      const st: number = SpatialReconService.resume();
      console.info(`SR3D thermal auto-resume level=${level} status=${st}`);
      if (st === 0) {
        this.paused = false;
        this.pausedByThermal = false;
        this.statusText = '温度已恢复，继续生成…';
      }
    }
  }

  /** 手动暂停/继续（手动操作会清掉"自动暂停"标记，避免随后被热事件自动继续） */
  private togglePause(): void {
    if (!this.generating) {
      return;
    }
    if (this.paused) {
      const st: number = SpatialReconService.resume();
      if (st === 0) {
        this.paused = false;
        this.pausedByThermal = false;
        this.statusText = '已继续生成…';
      }
      return;
    }
    const st: number = SpatialReconService.pause();
    if (st === 0) {
      this.paused = true;
      this.pausedByThermal = false;
      this.statusText = '已暂停';
    }
  }

  /** 终止本次生成（若已进入保存阶段，终止会被忽略，产物照常保存） */
  private abortGenerate(): void {
    if (!this.generating) {
      return;
    }
    SpatialReconService.abort();
    this.statusText = '正在终止…';
  }

  /** 本次生成结束后的状态复位 */
  private resetRunState(): void {
    this.stopPoll();
    this.generating = false;
    this.paused = false;
    this.pausedByThermal = false;
  }

  /** native 错误码 → 用户可读文案（失败码集中在 SpatialReconService 契约里） */
  private static failText(msg: string): string {
    if (msg.includes('-6')) {
      return '已终止生成';
    }
    if (msg.includes('-5')) {
      return '生成失败：画面配置异常';
    }
    if (msg.includes('-100')) {
      return '已有生成任务在进行中';
    }
    if (msg.includes('801')) {
      return '本机不支持空间重建功能';
    }
    if (msg.includes('1023700007')) {
      return '重建失败，请换一张照片重试';
    }
    return `生成失败（${msg}）`;
  }

  /**
   * 选择照片（示例实现：系统 PhotoViewPicker；接入方可替换为自有选择器）。
   * 注意：返回的临时授权 URI 可直接被 fileIo.openSync 读取（见 SpatialReconService 解码流程）。
   * 若需处理专业相机图（横放像素 + EXIF Orientation 6/8），请自行在此处做方向矫正。
   */
  private async pickImage(): Promise<void> {
    if (this.generating) {
      this.statusText = '正在生成，请稍候';
      return;
    }
    try {
      const picker: photoAccessHelper.PhotoViewPicker = new photoAccessHelper.PhotoViewPicker();
      const options: photoAccessHelper.PhotoSelectOptions = new photoAccessHelper.PhotoSelectOptions();
      options.MIMEType = photoAccessHelper.PhotoViewMIMETypes.IMAGE_TYPE;
      options.maxSelectNumber = 1;
      const result: photoAccessHelper.PhotoSelectResult = await picker.select(options);
      if (result.photoUris.length === 0) {
        return;
      }
      this.pickedUri = result.photoUris[0];
      this.statusText = '';
    } catch (e) {
      this.statusText = '选择照片失败';
    }
  }

  /** 单次生成：按当前适配选项跑完整管线并保存到图库 */
  private async runOnce(ctx: common.UIAbilityContext, adapt: SpatialReconAdaptOptions): Promise<boolean> {
    const result = await SpatialReconService.generateFromImage(ctx, this.pickedUri, adapt);
    this.lastLayerNote = result.layerNote;
    this.statusText = '正在保存到图库…';
    return await GallerySaver.saveMedia(ctx, {
      sourcePath: result.modelPath,
      mediaType: MediaKindType.VIDEO,
      ext: 'mp4',
      title: `3D壁纸_${Date.now()}`
    });
  }

  /** 状态行尾注：分层深度的主体识别结果（空串则不加） */
  private noteSuffix(): string {
    return this.lastLayerNote.length > 0 ? `；${this.lastLayerNote}` : '';
  }

  private makeAdapt(family: number, invert: boolean): SpatialReconAdaptOptions {
    return {
      invertQuat: invert,
      roll180: this.roll180,
      bgrSwap: this.bgrSwap,
      worldFlip180: this.worldFlip180,
      poseFamily: family,
      focalPx: this.focalPx,
      orbitDeg: this.orbitDeg,
      scene: this.sceneIdx,
    };
  }

  private async startGenerate(): Promise<void> {
    if (this.generating) {
      return;
    }
    if (this.pickedUri.length === 0) {
      this.statusText = '请先选择一张照片';
      return;
    }
    const probe: number = SpatialReconService.probe();
    if (probe !== 0) {
      this.statusText = '本机不支持空间重建功能';
      return;
    }
    const ctx: common.UIAbilityContext = getContext(this) as common.UIAbilityContext;
    this.generating = true;
    this.progress = 0;
    this.lastLayerNote = '';
    this.statusText = '正在准备…';
    this.stopPoll();
    this.pollTimer = setInterval((): void => {
      this.progress = SpatialReconService.progress();
    }, 500);
    try {
      const saved: boolean = await this.runOnce(ctx, this.makeAdapt(this.poseFamily, this.invertQuat));
      const tag: string = `${this.poseFamily === 1 ? 'CV系' : 'AR系'}·${this.invertQuat ? '世界→相机' : '相机→世界'}` +
        `${this.roll180 ? '·朝向对调' : ''}${this.worldFlip180 ? '·空间翻转' : ''}` +
        `·焦距${this.focalPx}·轨道${this.orbitDeg}°`;
      this.statusText = (saved
        ? `完成（${tag}），已保存到图库，可在图库中设置为 3D 壁纸`
        : '3D 壁纸模型已生成，但未确认保存到图库') + this.noteSuffix();
    } catch (e) {
      const msg: string = typeof e === 'string' ? e : ((e as Error).message || String(e));
      console.info(`SR3D generate failed: ${msg}`);
      this.statusText = Wallpaper3DPage.failText(msg);
    } finally {
      this.resetRunState();
    }
  }

  /** 自动扫描：相机系 × 四元数 四种组合依次尝试，命中即停；已验证命中的组合排最前 */
  private async startAutoScan(): Promise<void> {
    if (this.generating) {
      return;
    }
    if (this.pickedUri.length === 0) {
      this.statusText = '请先选择一张照片';
      return;
    }
    if (SpatialReconService.probe() !== 0) {
      this.statusText = '本机不支持空间重建功能';
      return;
    }
    const ctx: common.UIAbilityContext = getContext(this) as common.UIAbilityContext;
    const combos: number[][] = [[0, 1], [0, 0], [1, 0], [1, 1]];
    const names: string[] = ['AR系·世界→相机', 'AR系·相机→世界', 'CV系·相机→世界', 'CV系·世界→相机'];
    this.generating = true;
    this.lastLayerNote = '';
    try {
      for (let k = 0; k < combos.length; k++) {
        this.statusText = `扫描 ${k + 1}/${combos.length}：${names[k]}…`;
        this.progress = 0;
        this.stopPoll();
        this.pollTimer = setInterval((): void => {
          this.progress = SpatialReconService.progress();
        }, 500);
        try {
          const saved: boolean = await this.runOnce(ctx, this.makeAdapt(combos[k][0], combos[k][1] === 1));
          this.stopPoll();
          this.poseFamily = combos[k][0];
          this.invertQuat = combos[k][1] === 1;
          this.statusText = (saved
            ? `命中 ${names[k]}，已保存到图库，可在图库中设置为 3D 壁纸`
            : `命中 ${names[k]}，但未确认保存到图库`) + this.noteSuffix();
          this.resetRunState();
          return;
        } catch (e) {
          this.stopPoll();
          console.info(`SR3D scan combo ${k} (${names[k]}) failed: ` +
            (typeof e === 'string' ? e : ((e as Error).message || String(e))));
        }
      }
      this.statusText = '四种组合均失败，请查看日志中 GSRECON 阶段信息';
    } finally {
      this.resetRunState();
    }
  }

  @Builder
  private PreviewArea() {
    Column() {
      if (this.pickedUri.length > 0) {
        Image(this.pickedUri)
          .objectFit(ImageFit.Contain)
          .width('100%')
          .height('100%')
      } else {
        Column({ space: 12 }) {
          SymbolGlyph($r('sys.symbol.square_stack_3d'))
            .fontSize(56)
            .fontColor([$r('app.color.text_tertiary')])
          Text('选择一张照片，生成可设为 3D 壁纸的立体空间')
            .fontSize(14)
            .fontColor($r('app.color.text_tertiary'))
            .textAlign(TextAlign.Center)
        }
        .width('100%')
      }
    }
    .layoutWeight(this.isWide ? 1 : 0)
    .width('100%')
    .height(this.isWide ? '100%' : 280)
    .justifyContent(FlexAlign.Center)
    .alignItems(HorizontalAlign.Center)
  }

  @Builder
  private AdaptOptionsArea() {
    Column({ space: 8 }) {
      Button(this.showAdvanced ? '收起测试参数' : '测试参数')
        .width('100%')
        .height(40)
        .onClick((): void => {
          this.showAdvanced = !this.showAdvanced;
        })
      if (this.showAdvanced) {
        Divider()
        SrSegRow({
            label: '相机系',
            options: ['AR系', 'CV系'],
            selectedIdx: this.poseFamily,
            choose: (idx: number): void => { this.poseFamily = idx; }
        })
        SrSegRow({
            label: '四元数',
            options: ['相机→世界', '世界→相机'],
            selectedIdx: this.invertQuat ? 1 : 0,
            choose: (idx: number): void => { this.invertQuat = idx === 1; }
        })
        SrSegRow({
            label: '画面朝向',
            options: ['标准', '对调'],
            selectedIdx: this.roll180 ? 1 : 0,
            choose: (idx: number): void => { this.roll180 = idx === 1; }
        })
        SrSegRow({
            label: '通道交换',
            options: ['关', '开'],
            selectedIdx: this.bgrSwap ? 1 : 0,
            choose: (idx: number): void => { this.bgrSwap = idx === 1; }
        })
        SrSegRow({
            label: '空间朝向',
            options: ['标准', '翻转'],
            selectedIdx: this.worldFlip180 ? 1 : 0,
            choose: (idx: number): void => { this.worldFlip180 = idx === 1; }
        })
        SrSliderRow({
          label: '焦距', value: this.focalPx, min: 200, max: 6000, step: 50, unit: 'px',
          slide: (v: number): void => { this.focalPx = v; }
        })
        SrSliderRow({
          label: '轨道角', value: this.orbitDeg, min: 1, max: 45, step: 1, unit: '°',
          slide: (v: number): void => { this.orbitDeg = v; }
        })
        Column({ space: 8 }) {
          Button('自动扫描并保存')
            .width('100%')
            .height(44)
            .enabled(!this.generating && this.pickedUri.length > 0)
            .onClick((): void => { this.startAutoScan(); })
          Button(this.depthTesting ? '深度自检中…' : '深度自检')
            .width('100%')
            .height(44)
            .enabled(!this.generating && !this.depthTesting && this.pickedUri.length > 0)
            .onClick((): void => { this.runDepthSelfTest(); })
        }
      }
      // 「输入」RGB/RGBA 档已移除：官方约束输入仅支持 RGB（4 字节/像素为未定义行为）
    }
    .width('100%')
    .padding({ left: 24, right: 24 })
  }

  @Builder
  private ControlsArea() {
    Column({ space: 12 }) {
      if (this.progress > 0 && this.progress < 1) {
        Progress({ value: Math.round(this.progress * 100), total: 100, type: ProgressType.Linear })
          .width('100%')
      }
      if (this.statusText.length > 0) {
        Text(this.statusText)
          .fontSize(13)
          .fontColor($r('app.color.text_tertiary'))
          .textAlign(TextAlign.Center)
          .width('100%')
      }
      if (this.generating) {
        Row({ space: 12 }) {
          // 暂停/继续：合成推帧阶段（进度 <0.3）Kit 尚未开始重建，暂停无效，故先禁用
          Button(this.paused ? '继续' : '暂停')
              .layoutWeight(1)
              .height(40)
              .enabled(this.progress >= 0.3)
              .onClick((): void => { this.togglePause(); })
            Button('终止')
              .layoutWeight(1)
              .height(40)
              .onClick((): void => { this.abortGenerate(); })
        }
        .width('100%')
      }
      Row({ space: 12 }) {
        Button('选择照片')
          .layoutWeight(1)
          .height(48)
          .enabled(!this.generating)
          .onClick((): void => { this.pickImage(); })
        Button(this.generating ? (this.paused ? '已暂停' : '生成中…') : '生成并保存')
          .layoutWeight(2)
          .height(48)
          .enabled(!this.generating && this.pickedUri.length > 0)
          .onClick((): void => { this.startGenerate(); })
      }
      .width('100%')
    }
    .width('100%')
    .padding({ left: 24, right: 24, bottom: 24 })
  }

  /**
   * 路线B 阶段1 自检：对当前选中照片跑端侧深度估计。
   * 产物：`<filesDir>/depth_selftest/depth_<ts>.depth.f32.bin`（原始相对逆深度，越大越近）
   *      + `.meta.txt`；状态行显示 native 统计串（含尺寸/layout/min/max/mean/耗时）。
   * 与离线判据比对：把 bin 取回后用 tools/depth_mesh 的判据脚本核对。
   */
  private async runDepthSelfTest(): Promise<void> {
    if (this.pickedUri.length === 0) {
      this.statusText = '请先选择照片';
      return;
    }
    this.depthTesting = true;
    this.statusText = '深度自检中（端侧推理）…';
    try {
      const ctx: common.UIAbilityContext = getContext(this) as common.UIAbilityContext;
      const info: string = await SpatialReconService.depthSelfTest(ctx, this.pickedUri);
      this.statusText = info;
    } catch (e) {
      const msg: string = typeof e === 'string' ? e : ((e as Error).message || String(e));
      this.statusText = `深度自检失败：${msg}`;
      console.warn(`SRD selftest failed ${msg}`);
    } finally {
      this.depthTesting = false;
    }
  }

  build() {
    HdsNavDestination() {
      if (this.isWide) {
        Row({ space: 24 }) {
          Column() {
            this.PreviewArea()
          }
          .layoutWeight(1)
          .height('100%')
          Column() {
            Scroll() {
              this.AdaptOptionsArea()
            }
            .layoutWeight(1)
            .width('100%')
            this.ControlsArea()
          }
          .width(380)
          .height('100%')
        }
        .width('100%')
        .height('100%')
        .padding(24)
      } else {
        Column() {
          this.PreviewArea()
          Scroll() {
            this.AdaptOptionsArea()
          }
          .layoutWeight(1)
          .width('100%')
          .scrollBar(BarState.Auto)
          this.ControlsArea()
        }
        .width('100%')
        .height('100%')
      }
    }
    .titleBar({ content: { title: { mainTitle: '3D壁纸' } } })
    .hideTitleBar(false)
    .onBackPressed((): boolean => {
      this.pageStack.pop();
      return true;
    })
  }
}
