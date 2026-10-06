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
    bool invertQuat = false;   /**< A/B：四元数取共轭（cam→world ⇄ world→cam），与相机系正交 */
    bool roll180 = false;      /**< A/B（默认关）：世界绕「视轴」滚 180°——合成帧与上报位姿**同步**滚
                                *   （渲染基取反 X_cam/Y_cam 两列），射线集合恒等 ⇒ 重建几何不变。
                                *   用于排查 Kit 输出视频"上下颠倒"（Kit 输出渲染按 [VideoGenerator]
                                *   gravityCoordinate 定向；翻转不在合成帧侧，见 docs/3d-wallpaper-archive.md）。 */
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
                                *   深度缺失/构建失败自动退回 1；设计见 docs/3d-wallpaper-route-b-implementation.md。 */
    bool rgbaInput = false;    /**< A/B：按 4 字节/像素（RGBA，Alpha=255）上报，默认 RGB 3 字节 */
    bool bgrSwap = true;       /**< **默认开**（坑 132）：推帧前交换 R/B。该开关会改变成片的颜色
                                *   表现，但成因**尚无定论**（坑 134 的"A/B 定案"已撤销，见档案 F7）；
                                *   默认值与 Service 层的"失败自动反交换重试"属工程折中，
                                *   不代表结论；bgrSwap=false 可手动回到直通 RGB。 */
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
};

/**
 * 阻塞执行验证管线：合成帧序列 → CreateSession → PushFrame×N → StartSession（重建+自动保存）
 * → 等待完成回调 → 校验产物。供 napi async work 的 Execute 调用。
 * @param params 验证参数
 * @param outModelPath 成功时回填输出文件路径
 * @return 0=成功；-100=已有任务在跑；-1=so 绑定失败；其余=Kit 状态码/内部错误码
 */
int32_t SpatialReconTestRun(const SpatialReconTestParams &params, std::string &outModelPath);

/** 最近一次/进行中验证任务的进度（0.0~1.0），供 ArkTS 轮询 */
float SpatialReconTestProgress();

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
 * ⚠️ Kit 官方未提供"中途销毁"能力：正在执行任务时 DestroySession 属未定义行为，
 * 故本接口只"请求终止"，由运行线程在阶段落到 PAUSED 后再销毁会话；
 * 保存阶段（SAVING）不可中断——该阶段调用会被忽略，产物照常保存并返回成功。
 * @return 0=已请求终止（Pause 结果见 hilog）；-1=能力库未绑定
 */
int32_t SpatialReconAbort();

} // namespace glassvideo

#endif // GLASSVIDEO_SPATIAL_RECON_BRIDGE_H
