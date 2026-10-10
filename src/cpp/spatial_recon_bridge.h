/**
 * Spatial Recon Kit 桥接（3D壁纸 · 步骤2管线验证）
 *
 * 职责：把"单张图片 + 已知虚拟相机位姿"合成的多视角帧序列喂给 Spatial Recon Kit
 * （HMS_SpatialRecon_PushFrame 自定义数据路径），启动端侧 3DGS 重建并保存 MP4，
 * 验证「自渲染已知位姿帧 → PushFrame → 重建 → MP4」全链路是否成立。
 *
 * 设计要点：
 * - libspatial_recon_ndk.z.so 是设备相关系统能力（旗舰芯片 + 中国境内 + 6.1.0(23)+）。
 *   绝不加入 target_link_libraries——DT_NEEDED 会让不支持设备在加载 libglassrender.so
 *   时直接加载失败（启动即崩）；改为运行期 dlopen + dlsym 绑定，失败按"设备不支持"上报。
 * - 场景模型：源图 contain 贴在 z=-D 平面上，虚拟相机绕平面中心做小角度圆弧轨道
 *   （半径 D，总摆角 orbitDeg），每帧的渲染与位姿（位置+四元数）由同一模型给出，
 *   保证 (图像, 内参, 位姿) 三元组自洽——Kit 拿到的是一组几何一致的多视角输入。
 * - Kit 的相机坐标约定无官方文档。有效相机系只有两种（同一点像素一致，见
 *   tools/sr_frames_check.js 判据 [2]）：poseFamily 0 = AR 系（X右/Y上/Z后，ARKit/AR Engine）；
 *   1 = CV 系（X右/Y下/Z前，OpenCV）。invertQuat 独立切换四元数方向（c2w ⇄ w2c）。
 *   ⚠️ 图像与位姿必须自洽（图像行向下 ↔ 相机 Y 轴向下）；首版垂直镜像导致真机
 *   [GSRECON][STAGE02] no 3D points，且任何位姿约定都无法补偿镜像。
 * - scene 开关：0=平面贴图；1=立体（主盒贴源图 + 副盒 + z=-8 远景，均用不可重复程序化纹理）；
 *   2=同上但主盒正面也用程序纹理（隔离源图纹理质量）；3=分层深度（主盒正面贴"主体已补洞"的
 *   背景层，源图按主体蒙版另贴一张更近的前景平面 → 相机环绕时主体/背景产生真实视差；
 *   参考位姿处两层投影重合，故合成画面 == 原照片）。SfM 需要非共面观测：纯平面/单深度层
 *   是退化配置（首版"立体"场景的面填满画面，与平面等价）。另：**周期纹理（棋盘格）会令特征
 *   匹配歧义**，故合成场景一律用随机晶格纹理（处处唯一）。
 * - 输出帧严格 1080x1440 RGB（Kit 硬性要求，其余尺寸结果未定义）。
 */

#ifndef GLASSVIDEO_SPATIAL_RECON_BRIDGE_H
#define GLASSVIDEO_SPATIAL_RECON_BRIDGE_H

#include <cstdint>
#include <string>
#include <vector>

namespace glassvideo {

/** presetPitchDeg 的"未用"哨兵（有效值域 [-89, 89] 之外即视为未传） */
constexpr float kPresetPitchUnused = -1000.0f;

/** 探测当前设备是否支持空间重建（含 so 绑定）。0=支持；非 0=Kit 状态码或 -1=绑定失败。 */
int32_t SpatialReconProbe();

/** 步骤2验证参数（源图 RGBA_8888 由 ArkTS 解码后传入） */
struct SpatialReconTestParams {
    std::string workPath;      /**< 会话工作目录（必须已存在，且在应用文件目录下） */
    std::string modelPath;     /**< 输出 MP4 绝对路径（同样须在应用文件目录下） */
    std::vector<uint8_t> rgba; /**< 源图 RGBA_8888 像素（srcW*srcH*4 字节） */
    int32_t srcW = 0;          /**< 源图宽（px） */
    int32_t srcH = 0;          /**< 源图高（px） */
    int32_t frameCount = 72;   /**< 合成帧数（Kit 自动选关键帧，超限报 EXCEEDS_MAXIMUM） */
    float orbitDeg = 5.0f;     /**< 虚拟相机轨道半角（度，±orbit） */
    float focalPx = 1500.0f;   /**< 虚拟相机焦距（px；主点恒为画面中心，零畸变） */
    float focalYratio = 1.0f;  /**< **各向异性内参声明（2026-10-09 加，默认 1.0）**：上报
                                *   `focalY = focalYratio × focalX`。仅预置路线生效（照片路线
                                *   恒各向同性）。背景：Kit 的 BA 无锁内参，
                                *   refined 纵横比是**内容相关**漂移（跨素材实测 0.3~3.5）；帧保持各向同性时，
                                *   **只改声明**即可按 `模型纵横比 ≈ B·ρ_d/ρ_r` 把纵横比拉回 1（ρ_d = 1/B，
                                *   见 docs/3d-wallpaper-archive.md）。
                                *   钳制 [0.3, 3.0]，越界按 1.0 处理。 */
    bool invertQuat = false;   /**< A/B：四元数取共轭（cam→world ⇄ world→cam），与相机系正交 */
    bool roll180 = false;      /**< A/B（默认关）：世界绕「视轴」滚 180°——合成帧与上报位姿**同步**滚
                                *   （渲染基取反 X_cam/Y_cam 两列），射线集合恒等 ⇒ 重建几何不变。
                                *   用于排查 Kit 输出视频"上下颠倒"（Kit 输出渲染按 [VideoGenerator]
                                *   gravityCoordinate 定向；翻转不在我们的合成帧，见 docs/3d-wallpaper-archive.md）。 */
    bool worldFlip180 = true;  /**< **默认开**：世界绕世界 X 轴转 180°（up→−up、z→−z）——
                                *   只把**上报位姿**同步旋转（位置 y/z 取反 + 渲染基三列 y/z 分量取反），
                                *   **合成帧不变**：图像与"旋转后的世界"仍自洽（模型整体旋转，重建等价）。
                                *   为什么默认开：真机实测 Kit 的模型导出/输出把模型绕水平轴翻了 180°
                                *   （成片上下颠倒 + 人物跑到背景后面 = 深度镜像），本旋转在输入侧抵消它
                                *   ——2026-10-01 真机二次复测确认"翻转后正确"。关掉 = 回到 Kit 原始朝向。 */
    int32_t poseFamily = 0;    /**< 上报相机系：0=AR 系（X右/Y上/Z后，默认）；1=CV 系（X右/Y下/Z前） */
    int32_t scene = 1;         /**< 合成场景：0=平面贴图；1=立体（主盒正面贴源图 + 副盒/侧面/远景用不可重复程序纹理，默认）；2=同 1 但主盒正面也换程序纹理（隔离源图纹理质量）；3=分层深度（主体/背景两层，需 mask 有效，退化时自动退回 1）；
                                *   4=盒中空间（放大主盒：正面铺满画幅、照片面积×3，需 mask）；5=**深度网格（路线B）**：
                                *   由端侧深度估计（DA-V2 Small）逐像素展开成三层（照片网格 + 背景层补洞 + 远景板兜底），
                                 *   逐帧用 bridge 单源投影光栅化；轨道口径由判据界定（±kDepthThetaDeg，stepNear≈66px）；
                                 *   深度缺失/构建失败自动退回 1。 */
    bool rgbaInput = false;    /**< A/B：按 4 字节/像素（RGBA，Alpha=255）上报，默认 RGB 3 字节 */
    bool bgrSwap = true;       /**< **默认开**（坑 132）：推帧前交换 R/B。该开关会改变成片的颜色
                                *   表现，但成因**尚无定论**（坑 134 的"A/B 定案"已撤销，见档案 F7）；
                                *   默认值与 Service 层的"失败自动反交换重试"属工程折中，
                                *   不代表结论；bgrSwap=false 可手动回到直通 RGB。 */
    bool swapRgbAtInput = false; /**< **诊断 A/B（默认关，坑 143 轮 8）**：交换**位置**——true 时在
                                *   "深度之后、贴图/网格构建之前"对源缓冲（本地副本）整体交换一次 R/B，
                                *   推帧口**不再**交换（二者互斥）；false = 现行"推帧口逐帧交换"。
                                *   语义上二者等价（唯一差别：远景板压暗系数 0.72/0.72/0.75 会落在
                                *   对调后的通道上，远景区每通道 ~4% 差异，可忽略）；`SRCHK src` 缩略图
                                *   与 ArkTS 侧深度/判据仍用自然缓冲。 */
    std::vector<uint8_t> mask; /**< 主体蒙版（scene=3 用；srcW*srcH，0=背景 255=主体，中间为概率；缺失/尺寸不符自动退回 scene=1） */
    float subjectDepth = 0.90f; /**< 主体平面深度 / 主盒正面深度（<1 = 主体更近；钳 0.60~0.95）。
                                 *   取值受"可匹配位移量级"约束：背景层一段 ±8° 轨道位移 ≈145px（已验证可匹配），
                                 *   主体层位移随 (1−k) 增大（0.84→326px / 0.90→251px / 0.94→206px）。
                                 *   原默认 0.80（≈382px）在真机上使稠密初始化失败（points 太少），故收到 0.90。 */
    std::vector<float> depth;  /**< 路线B 深度网格场景（scene=5）用：端侧深度估计的**原始相对逆深度**
                                *   （float32，越大越近；尺寸 depthW×depthH，通常 546×728）。
                                *   由服务层先跑 depthEstimateTest 再随参数传入；缺失/尺寸不符自动退回 scene=1。 */
    int32_t depthW = 0;        /**< 深度网格宽（px） */
    int32_t depthH = 0;        /**< 深度网格高（px） */
    bool alsoPly = false;      /**< **同会话双产物（坑 146 后续，默认关、Service 侧开启）**：MP4 自动保存
                                *   完成并过比例门控后，再用官方 HMS_SpatialRecon_SaveResultToFile 追加
                                *   保存一份 **PLY**（modelPath 把 .mp4 后缀换成 .ply）。
                                *   动机：spatialEdit.saveToPLY 对"MP4 容器加载的节点"真机恒失败
                                *   （引擎 `setPromise failed same model`，两轮修复无效）；而 PLY 是
                                *   ①模型几何/颜色的唯一可解剖产物 ②模型侧离线诊断（几何解剖、
                                *   颜色分布统计）的输入源。失败不致命：只打日志，不影响 MP4 交付。 */
    int32_t depthEdge = 1;     /**< **D1/D4 深度边缘（默认 1=增强；2026-10-04 起产品默认档
                                 *   整体上调为质量增强档——此前"默认 0=旧行为"的约定作废）**：
                                 *   1 = D1 JBU（RGB 引导上采样到画布分辨率再建网格，深度边缘贴颜色边缘）；
                                 *   2 = D4 保边预平滑 + D1（再消细碎台阶）；0 = 关（旧行为）。
                                 *   离线验证与口径见 tools/depth_mesh/depth_edge_check.js
                                 *   （边缘对齐分 0.206→0.445、MAE↓）。
                                 *   ⚠️ 本档改喂给 Kit 的帧统计（坑 147 家族）——坑 154 真机记录过
                                 *   「增强→人物侧竖直拉伸带」「全开→逆光海边图左半背景纵向拉丝」，
                                 *   默认变更须随真机同图 A/B 复核。 */
    int32_t adaptiveK = 1;     /**< **D3 自适应纵深（默认 1=自适应；2026-10-04 同上）**：
                                 *   按深度直方图 Otsu 分离度取 k∈[2.2,3.8]，**d0 恒 2 公式**
                                 *   （zNear=4/(1+k)、zFar=4k/(1+k)），轨道枢轴/相机数学/位移守卫零改动；
                                 *   0 = 固定 k=3（旧行为）。日志 `SRCHK adaptive k=`。 */
    int32_t subjectAlign = 1;  /**< **D2 主体对齐（默认 1=开；2026-10-04 同上）**：用 subjectMask
                                 *   （srcW×srcH 源分辨率）对深度场做蒙版门控保边平滑（主体内部抑噪、
                                 *   蒙版外逐字节不变）；0 = 关（旧行为）。scene=5 需服务层在
                                 *   subjectAlign>0 时构建并传入蒙版（识别不到主体则静默跳过）。 */
    bool composeTrial = false; /**< **构图搜索试验档（2026-10-04，默认关）**：StartSession 的 writeInfo
                                 *   传空（官方明文"writeInfo 为空时可在重建结束后手动 SaveResultToFile"），
                                 *   重建完成（FINISHED、未保存）即读 refined 比例——严重漂移直接销毁换档
                                 *   （省 STAGE04 编码保存 44~76s/次，不留文件）；合格才手动 SaveResultToFile。
                                 *   ⚠️ 前置实验点：FINISHED 窗口 refined 是否可读官方未规定——读不到时
                                 *   自动回退"保存后门控"（本档退化为常规成本，语义不变），日志
                                 *   `SRCHK trial refined pre-save readable|unreadable`。 */
    bool preset = false;       /**< **预置建模（地球等预置数据集，2026-10-04）**：true 时走预设帧管线——
                                 *   帧不再由源图合成，而是消费 ArkTS 侧逐帧解码后经
                                 *   SpatialReconPresetQueuePush 灌入的 RGBA 1080×1440（队列容量 3 流式
                                 *   过界，内存峰值单帧）；位姿由 presetTheta/presetRadius 闭环给出
                                 *   （整圈环绕、目标=世界原点、上行=+Y）。rgba/mask/depth/scene 等
                                 *   合成参数在本模式下全部旁路（Service 侧不传）。 */
    std::vector<float> presetTheta; /**< 预置帧逐帧环绕角 θ（度）。帧数 = size()（覆盖 frameCount，
                                 *   上限 240 同 Kit 约束）；θ=0 = 参考帧（数据集 manifest 的 0° 帧）；
                                 *   方向由数据集标定（地球：视频时间正方向 = θ 递减）。 */
    float presetRadius = 0.0f;  /**< 环绕半径（世界单位）。球体半径归一为 1，半径必须 >1（相机在球外），
                                 *   且与帧内视半径/焦距自洽（地球 v3：长焦 D=8，F_out≈3326）。
                                 *   ⚠️ 该值由离线几何测量得到（纹理自洽扫描定 D，帧重投影误差判据
                                 *   ~4/255），不是先验估值——帧与位姿必须同源同参数。 */
    std::vector<float> presetAxis; /**< 自转轴方向（世界系单位向量，3 个 float；空 = 默认 (0,1,0)）。
                                 *   声称位姿 basis_i = R_A(θ_i)（行序 [right; up; −fwd]，即 R_Aᵀ 的行）、
                                 *   pos_i = R_A(θ_i)·(0,0,radius)。轴倾角打破"相机 up 恒定 ⇒ (纵向尺度, fy)
                                 *   精确退化"（坑 156/131）；地球 v3 实测轴 (0, 0.944, 0.330)（朝观察者倾
                                 *   19.3°），退化判据 162px（可观测）。**必须与离线数据集同式同值**。 */
    float presetPitchDeg = kPresetPitchUnused; /**< v5 参数域：平环（绕 +Y）+ 相机自身绕 right 轴俯仰
                                 *   （度，SrPresetFlatPitchBasis）。有效值域 [-89, 89]；哨兵
                                 *   kPresetPitchUnused = 未用（走 presetAxis 轴式域）。两域互斥，
                                 *   同传报错——位姿公式由数据集 manifest 的参数域决定（v3=camAxis、
                                 *   v5=camPitchDeg），与离线重投影同式同值。 */
    bool presetDiagArc = false; /**< **对角弧域（2026-10-09，默认关）**：true 时走
                                 *   SrPresetDiagArcBasis——相机**位置**同时扫 yaw 与 pitch（pos.y ≠ 0，
                                 *   φ ≡ θ），即照片路线 scene=5 `SrFrameBasis`（Φ=Θ）的同口径移植。
                                 *   为什么必须有：平环（含仅姿态俯仰）下 (纵向尺度, fy) 是**精确
                                 *   不可观测量**（Y 只出现在分子），Kit 无锁内参 API ⇒ 其 BA 沿零梯度
                                 *   山谷随机漂（真机实测 refined fy/fx = 0.155/2.47/2.69；地球预置用
                                 *   flat-pitch 域实测 0.119/0.474/0.254）。让相机位置离开 y=0 平面后
                                 *   退化破除（照片路线修复后 0.937~1.009，成片比例正常）。
                                 *   数据集侧配对字段 = manifest.poseDomain == "diag-arc"。
                                 *   与 presetAxis / presetPitchDeg **互斥**（同传报错）。 */
};

/**
 * 阻塞执行验证管线：合成帧序列 → CreateSession → PushFrame×N → StartSession（重建+自动保存）
 * → 等待完成回调 → 校验产物。供 napi async work 的 Execute 调用。
 * @param params 验证参数
 * @param outModelPath 成功时回填输出文件路径
 * @return 0=成功；-100=已有任务在跑；-1=so 绑定失败；其余=Kit 状态码/内部错误码
 */
int32_t SpatialReconTestRun(const SpatialReconTestParams &params, std::string &outModelPath);

/**
 * 预置帧队列（preset=true 时 SpatialReconTestRun 的帧来源）。
 * ArkTS 侧逐帧解码 JPEG（RGBA 1080×1440）后调用 Push 灌入；运行线程在推帧循环里
 * 阻塞消费。容量 3，**Push 非阻塞**——灌帧循环跑在 JS/UI 线程上，队列满立即返回 1
 * 由调用方 await 退让重试（绝不冻结 UI 线程）；用户终止（SpatialReconAbort）/生产失败
 * （QueueFail）/消费端离开都会让 Push 以 -6 退出。每次 SpatialReconTestRun 起跑前的
 * napi 入口（JS 线程）清队（SpatialReconPresetQueueReset，防"worker 清队吞帧"竞态）。
 * @param rgba 帧字节（RGBA，4 字节/像素，须为 1080×1440）
 * @param bytes 字节数
 * @return 0=已入队；1=队列满（await 退让后重试）；-1=参数空；-6=已终止或消费端已离开（停止灌帧）
 */
int32_t SpatialReconPresetQueuePush(const uint8_t *rgba, size_t bytes);

/** 预置帧生产失败（解码错误等）：放行消费端（Pop 以 -6 退出）并唤醒可能阻塞的生产者。@return 0 */
int32_t SpatialReconPresetQueueFail();

/** 清空预置帧队列并复位标志。napi 入口（JS 线程）在每次生成调用时同步执行——
 *  与 Service 随后的灌帧循环同线程有序，消除"worker 线程 reset 吞帧"竞态。 */
void SpatialReconPresetQueueReset();

/** 最近一次/进行中验证任务的进度（0.0~1.0），供 ArkTS 轮询 */
float SpatialReconTestProgress();

/**
 * 最近一次/进行中重建的 Kit 阶段（HMS_SpatialReconStage：0=INIT / 1=BUILDING / 2=PAUSED /
 * 3=FINISHED / 4=SAVING / 5=UNKNOWN；-1=本次任务尚未开始），供 ArkTS 轮询显示真实阶段
 * （UI 文案据此区分"重建中/保存中/已暂停"）。
 */
int32_t SpatialReconTestStage();

/**
 * 暂停进行中的重建（Kit 官方 HMS_SpatialRecon_PauseSession）。
 * 仅对"已 StartSession 且未结束"的会话有效；其余情况 Kit 返回错误码。
 * @return 0=成功；-1=能力库未绑定/无活动会话；其余=Kit 状态码
 */
int32_t SpatialReconPause();

/** 继续已暂停的重建（Kit 官方 HMS_SpatialRecon_ResumeSession）。@return 同上 */
int32_t SpatialReconResume();

/**
 * 终止进行中的重建：置终止标志并立即 Pause（停止重计算）。
 * 官方（Spatial Recon Kit C API）**没有提供"取消/终止"接口**——API 全集里只有
 * Pause/Resume（暂停/继续）与 DestroySession（销毁会话，文档原文："终止空间重建会话并释放
 * 资源……所有未保存的重建数据将会丢失"）。本接口按官方语义实现为"请求终止"：置标志 →
 * 运行线程在阶段检查点（推帧/训练等待/试验档保存等待）以 -6 退出 → 统一销毁会话。
 * ⚠️ 两点设计取舍（保守，均非文档强制）：
 *   ① 不在"任务在途"时销毁（我们选择先让运行线程退出再销毁）；
 *   ② 保存收尾阶段（g_srInSave=1）不响应终止——Kit 对保存任务没有暂停接口，且该阶段是
 *      编码落盘在途，中途销毁的风险我们选择不承担。UI 侧必须在此时禁用终止并说明原因
 *      （`SpatialReconInSave()` 供判定）。
 * @return 0=已请求终止（Pause 结果见 hilog）；-1=能力库未绑定
 */
int32_t SpatialReconAbort();

/**
 * 是否处于"保存收尾"阶段（1=是；0=否）——UI 的暂停/终止按钮可用性判定依据。
 * ⚠️ 不能用 Kit 的 stage 值替代：保存期 Kit 可能报 FINISHED(3) 而非 SAVING(4)，
 * 会导致按钮看着可点、点下去无效（用户感知"点了没反应"）。
 */
int32_t SpatialReconInSave();

/**
 * 前后台切换 → 更新运行模式（官方规则：每次 StartSession 之后必须调用 SetRunningMode，
 * 应用切前后台时也要更新）。无会话/未绑定能力库时为 no-op（官方："非活动会话期间调用无效果"）；
 * 期望值会被记录并在下一次 StartSession 后立即套用。@return 0=no-op 或 Kit 状态码
 */
int32_t SpatialReconSetForeground(int32_t foreground);

} // namespace glassvideo

#endif // GLASSVIDEO_SPATIAL_RECON_BRIDGE_H
