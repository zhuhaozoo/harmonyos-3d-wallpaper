/**
 * Spatial Recon Kit 桥接实现（3D壁纸 · 步骤2管线验证）
 * 头文件与几何/位姿模型说明见 spatial_recon_bridge.h。
 *
 * 相机系与位姿约定（poseFamily / invertQuat，均只改上报位姿、不改渲染）：
 * - 渲染基（固定）：X右 / Y上 / Z后（看 -Z，右手系，即 ARKit/AR Engine 相机系），
 *   列存储 [right, up, -fwd]，世界系 = 第 0 帧相机系，场景位于 z<0；
 * - family 0（AR 系）：直接上报 basis；
 * - family 1（CV 系）：R = basis · diag(1,-1,-1)（X右/Y下/Z前，OpenCV 相机系）；
 * - invertQuat：四元数取共轭（w2c 存储）。
 * ⚠️ 图像与位姿必须自洽：图像行向下 ↔ 相机 Y 轴向下。首版写成行向下 ↔ Y 轴向上，
 * 等于把画面垂直镜像后再配对（镜像无法由任何位姿约定补偿），真机报 STAGE02 no 3D points。
 * 另：镜像世界 + 共轭、单列取反（反射矩阵）均为废弃方案，勿再用。
 */
#include "spatial_recon_bridge.h"

#include <spatial/spatial_recon_interface.h>

#include "spatial_recon_layers.h"
#include "spatial_recon_depth_mesh.h"   // 路线B 深度网格：深度→四边形/光栅化（纯 2D 单元）

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <dlfcn.h>
#include <mutex>
#include <thread>
#include <unistd.h>

#include "vg_common.h"

namespace glassvideo {

namespace {

// ---------------- Kit API 运行期绑定（dlopen + dlsym，不进链接清单） ----------------

typedef HMS_SpatialReconStatus (*FnIsSupport)(HMS_SpatialReconModelType);
typedef HMS_SpatialReconStatus (*FnCreateSession)(HMS_SpatialReconModelType, const char *,
                                                  HMS_SpatialRecon_Session **);
typedef HMS_SpatialReconStatus (*FnDestroySession)(HMS_SpatialRecon_Session *);
typedef HMS_SpatialReconStatus (*FnPushFrame)(HMS_SpatialRecon_Session *, HMS_SpatialRecon_DataFrame *);
typedef HMS_SpatialReconStatus (*FnStartSession)(HMS_SpatialRecon_Session *,
                                                 HMS_SpatialRecon_ModelWriteInfo *,
                                                 HMS_SpatialReconCallbackFunc);
typedef HMS_SpatialReconStatus (*FnSetRunningMode)(HMS_SpatialRecon_Session *, HMS_SpatialReconRunningMode);
typedef HMS_SpatialReconStatus (*FnGetProgress)(HMS_SpatialRecon_Session *, float *, HMS_SpatialReconStage *);
typedef HMS_SpatialReconStatus (*FnPauseSession)(HMS_SpatialRecon_Session *);
typedef HMS_SpatialReconStatus (*FnResumeSession)(HMS_SpatialRecon_Session *);
// 官方《spatial_recon_interface.h》：取重建优化后的帧内外参（不返回像素，imageData=null）
typedef HMS_SpatialReconStatus (*FnGetRefinedFrame)(HMS_SpatialRecon_Session *, int,
                                                    HMS_SpatialRecon_DataFrame *);
typedef HMS_SpatialReconStatus (*FnSaveResultToFile)(HMS_SpatialRecon_Session *,
                                                      HMS_SpatialRecon_ModelWriteInfo *,
                                                      HMS_SpatialReconCallbackFunc);
struct SrApi {
    FnIsSupport isSupport = nullptr;
    FnCreateSession createSession = nullptr;
    FnDestroySession destroySession = nullptr;
    FnPushFrame pushFrame = nullptr;
    FnStartSession startSession = nullptr;
    FnSetRunningMode setRunningMode = nullptr;
    FnGetProgress getProgress = nullptr;
    FnPauseSession pauseSession = nullptr;
    FnResumeSession resumeSession = nullptr;
    FnGetRefinedFrame getRefinedFrame = nullptr;
    FnSaveResultToFile saveResultToFile = nullptr;
    bool ok = false;
};

SrApi g_srApi;
std::once_flag g_srApiOnce;

void SrBindApi()
{
    std::call_once(g_srApiOnce, []() {
        void *h = dlopen("libspatial_recon_ndk.z.so", RTLD_NOW | RTLD_LOCAL);
        if (h == nullptr) {
            VG_LOGE("SRCHK dlopen libspatial_recon_ndk.z.so failed: %{public}s", dlerror());
            return;
        }
#define SR_DLSYM(fnType, field, symName)                                              \
    do {                                                                              \
        g_srApi.field = reinterpret_cast<fnType>(dlsym(h, "HMS_SpatialRecon_" #symName)); \
        if (g_srApi.field == nullptr) {                                               \
            VG_LOGE("SRCHK dlsym HMS_SpatialRecon_" #symName " failed");              \
            return;                                                                   \
        }                                                                             \
    } while (0)
        SR_DLSYM(FnIsSupport, isSupport, IsSupport);
        SR_DLSYM(FnCreateSession, createSession, CreateSession);
        SR_DLSYM(FnDestroySession, destroySession, DestroySession);
        SR_DLSYM(FnPushFrame, pushFrame, PushFrame);
        SR_DLSYM(FnStartSession, startSession, StartSession);
        SR_DLSYM(FnSetRunningMode, setRunningMode, SetRunningMode);
        SR_DLSYM(FnGetProgress, getProgress, GetProgress);
#undef SR_DLSYM
        // 可选符号：老 ROM 若缺 Pause/Resume，只降级"暂停/继续/终止"，不影响重建主链路
#define SR_DLSYM_OPT(fnType, field, symName)                                              \
    do {                                                                                  \
        g_srApi.field = reinterpret_cast<fnType>(dlsym(h, "HMS_SpatialRecon_" #symName)); \
        if (g_srApi.field == nullptr) {                                                   \
            VG_LOGW("SRCHK dlsym HMS_SpatialRecon_" #symName " missing (optional)");      \
        }                                                                                 \
    } while (0)
        SR_DLSYM_OPT(FnPauseSession, pauseSession, PauseSession);
        SR_DLSYM_OPT(FnResumeSession, resumeSession, ResumeSession);
        // 诊断用（可选）：6.1.0(23)+ 即有；缺符号只少一条对账日志，不影响主链路
        SR_DLSYM_OPT(FnGetRefinedFrame, getRefinedFrame, GetRefinedFrame);
        // 双产物（坑 146 后续，可选）：重建完成后手动 SaveResultToFile 追加 PLY；
        // 缺符号只少一份诊断副产物，不影响 MP4 交付
        SR_DLSYM_OPT(FnSaveResultToFile, saveResultToFile, SaveResultToFile);
#undef SR_DLSYM_OPT
        g_srApi.ok = true;
        VG_LOGI("SRCHK api bound ok");
    });
}

// ---------------- 运行状态（供 napi 层轮询） ----------------

std::atomic<float> g_srProgress{0.0f};
// 最近一次 Kit 自身报出的进度（0~1，**按阶段重置**：重建域 / 保存域是两个 0~1）。
// 存在的意义：镜像时必须与"上一次的 Kit 值"比较，不能与合成后的总值（0.30~0.90）比较——
// 老写法 `kitProgress > g_srProgress` 会把 0~0.30 段的 Kit 进度全部吞掉（进度卡在 30%，坑 140）。
std::atomic<float> g_srKitProgress{0.0f};
// Kit 阶段（HMS_SpatialReconStage：0=INIT / 1=BUILDING / 2=PAUSED / 3=FINISHED / 4=SAVING /
// 5=UNKNOWN；-1=本次任务尚未开始）：与进度值一起轮询给 UI，用于如实显示"重建中/保存中/已暂停"
// （保存阶段不可中断，UI 据此禁用「终止」并说明原因）。
std::atomic<int32_t> g_srStage{-1};
std::atomic<bool> g_srBusy{false};
// StartSession 完成回调在 Kit 线程触发：只写原子量，不做任何 JS/重活
std::atomic<int32_t> g_srCbStatus{-1};
// 当前活动会话句柄（暂停/继续/终止接口据此操作；运行线程创建后写入、销毁前清空）
std::atomic<HMS_SpatialRecon_Session *> g_srSession{nullptr};
// 用户/热管理暂停标记（仅记录，不影响 Kit 状态）
std::atomic<bool> g_srPaused{false};
// 终止请求（运行线程在推帧/训练等待/保存等待循环里检查 → 统一以 -6 退出并销毁会话）
std::atomic<int32_t> g_srAbort{0};
// 保存域标志（2026-10-05）：1 = 已进入"保存收尾等待"（STAGE04 编码落盘阶段）。
// 用途：UI 的暂停/终止按钮可用性判定。**不能用 Kit 的 stage 值判**——保存期 Kit 可能报
// FINISHED(3) 而非 SAVING(4)（真机日志 `SRCHK stage -> 3 at kit=1.000` 即此），此时按钮看着
// 可点、点下去却无效（Kit 对保存任务没有暂停/取消接口）→ 用户感知"点了没反应"。
std::atomic<int32_t> g_srInSave{0};
// 期望运行模式（2026-10-04 官方规则补全）：前后台切换时由 SpatialReconSetForeground 更新，
// StartSession 之后立即套用——官方："每次 StartSession 之后必须调用 SetRunningMode；
// 应用切前后台时也要更新"（前台分配更多资源更快；后台系统优先前台应用、重建较慢）。
std::atomic<int32_t> g_srWantForeground{1};

// ---------------- 预置帧队列（preset=true 的帧来源）----------------
// ArkTS 侧逐帧解码（image kit，已验证的端侧解码链路）后经 SpatialReconPresetQueuePush 灌入，
// 运行线程在推帧循环里 SrPresetQueuePop 阻塞消费——避免把全部帧驻留内存（120×6.2MB=746MB 不可行）。
// 容量 3；**Push 非阻塞**（灌帧循环跑在 JS/UI 线程上，队列满立即返回 1 由调用方 await 退让重试，
// 绝不阻塞 UI 线程）；Pop 阻塞等待。终止（g_srAbort）/生产失败（g_srPqAbort）/消费端离开
// （g_srPqConsumerGone）三方互为唤醒条件，无死锁角。每次生成在 napi 入口（JS 线程）清队。
static std::mutex g_srPqMx;
static std::condition_variable g_srPqCv;
static std::deque<std::vector<uint8_t>> g_srPq;
static bool g_srPqAbort = false;
static bool g_srPqConsumerGone = false;
static constexpr size_t kSrPqCap = 3;

static int32_t SrPresetQueuePushImpl(const uint8_t *rgba, size_t bytes)
{
    if (rgba == nullptr || bytes == 0) {
        return -1;
    }
    {
        std::lock_guard<std::mutex> lk(g_srPqMx);
        if (g_srPqAbort || g_srPqConsumerGone || g_srAbort.load() != 0) {
            return -6;
        }
        if (g_srPq.size() >= kSrPqCap) {
            return 1;   // 队列满：非阻塞返回，JS 侧 await 退让后重试（不冻结 UI 线程）
        }
        g_srPq.emplace_back(rgba, rgba + bytes);
    }
    g_srPqCv.notify_all();
    return 0;
}

static int32_t SrPresetQueueFailImpl()
{
    {
        std::lock_guard<std::mutex> lk(g_srPqMx);
        g_srPqAbort = true;
    }
    g_srPqCv.notify_all();
    return 0;
}

/** 运行线程消费一帧（阻塞直至有帧/终止/生产失败）。@return 0=取到帧；-6=终止或生产失败 */
static int32_t SrPresetQueuePop(std::vector<uint8_t> &out)
{
    std::unique_lock<std::mutex> lk(g_srPqMx);
    g_srPqCv.wait(lk, [] {
        return !g_srPq.empty() || g_srPqAbort || g_srAbort.load() != 0;
    });
    if (g_srPq.empty()) {
        return -6;
    }
    out = std::move(g_srPq.front());
    g_srPq.pop_front();
    g_srPqCv.notify_all();
    return 0;
}

/** 每次生成入口清队（新任务新状态；含标志复位）。
 *  ⚠️ 必须在 napi 入口（JS 线程）同步调用、不在运行线程里调用：Service 发起生成后立即
 *  开始灌帧，若 reset 发生在 worker 线程，存在"先灌的帧被 reset 清掉/错位"竞态。 */
static void SrPresetQueueReset()
{
    std::lock_guard<std::mutex> lk(g_srPqMx);
    g_srPq.clear();
    g_srPqAbort = false;
    g_srPqConsumerGone = false;
}

/** 消费端离开（推帧循环任何出口后调用）：唤醒可能阻塞的生产者并清掉余帧 */
static void SrPresetQueueLeave()
{
    {
        std::lock_guard<std::mutex> lk(g_srPqMx);
        g_srPqConsumerGone = true;
        g_srPq.clear();
    }
    g_srPqCv.notify_all();
}

void SrOnFinished(HMS_SpatialReconStatus status)
{
    g_srCbStatus.store(static_cast<int32_t>(status));
    VG_LOGI("SRCHK onFinished status=%{public}d", static_cast<int32_t>(status));
}

// ---------------- 几何工具 ----------------

struct SrVec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

SrVec3 SrCross(const SrVec3 &a, const SrVec3 &b)
{
    return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

float SrDot(const SrVec3 &a, const SrVec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

SrVec3 SrNormalize(const SrVec3 &a)
{
    const float len = std::sqrt(SrDot(a, a));
    if (len <= 1e-8f) {
        return { 0.0f, 0.0f, 0.0f };
    }
    return { a.x / len, a.y / len, a.z / len };
}

/** 旋转矩阵（列存储：m[0..2]=第一列）→ 四元数 [x,y,z,w]（标准 trace 分支法，R=R(q) 主动式） */
void SrMatToQuat(const float m[9], float q[4])
{
    const float trace = m[0] + m[4] + m[8];
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        w = 0.25f * s;
        x = (m[5] - m[7]) / s;
        y = (m[6] - m[2]) / s;
        z = (m[1] - m[3]) / s;
    } else if (m[0] > m[4] && m[0] > m[8]) {
        const float s = std::sqrt(1.0f + m[0] - m[4] - m[8]) * 2.0f;
        w = (m[5] - m[7]) / s;
        x = 0.25f * s;
        y = (m[3] + m[1]) / s;
        z = (m[6] + m[2]) / s;
    } else if (m[4] > m[8]) {
        const float s = std::sqrt(1.0f + m[4] - m[0] - m[8]) * 2.0f;
        w = (m[6] - m[2]) / s;
        x = (m[3] + m[1]) / s;
        y = 0.25f * s;
        z = (m[7] + m[5]) / s;
    } else {
        const float s = std::sqrt(1.0f + m[8] - m[0] - m[4]) * 2.0f;
        w = (m[1] - m[3]) / s;
        x = (m[6] + m[2]) / s;
        y = (m[7] + m[5]) / s;
        z = 0.25f * s;
    }
    const float len = std::sqrt(x * x + y * y + z * z + w * w);
    if (len > 1e-8f) {
        x /= len;
        y /= len;
        z /= len;
        w /= len;
    }
    q[0] = x;
    q[1] = y;
    q[2] = z;
    q[3] = w;
}

// ---------------- 场景规格 ----------------

// 输出帧规格：Kit 硬性要求（文档明示其余尺寸结果未定义）
constexpr int32_t kOutW = 1080;
constexpr int32_t kOutH = 1440;
// 场景中心距离（各场景共用；任意单位，位姿与渲染同源即可）
constexpr float kSceneDist = 2.0f;
// 立体场景（scene=1）：主盒（正面贴源图 + 10% 程序纹理补边）+ 副盒（更远）+ 远景背景板。
// 尺寸经 tools/sr_frames_check.js 校验：正面占画面约 59%×59%，四周露出远景（z=-8），
// 形成两个以上深度层。历史教训：首版"立体"场景的盒子大到正面填满整个画面 → 全部像素共面
// → 与平面场景等价，SfM 退化，真机报 [GSRECON][STAGE02] no 3D points。
// 2026-10-01 二次修正：正面照片曾"整幅拉伸铺满"→ 非 3:4 照片变形（产品缺陷）、且近景只有照片
// 一处可匹配特征（实测 Kit 长期报 Too few points，自相似照片直接 0 点失败）。
// 现改为"照片按自身宽高比 contain + 内缩 10% 补程序纹理"：照片不变形，且与照片同深度处
// 恒有一圈独有特征，重建不再由照片内容独自承担。
constexpr float kBoxHX = 0.36f;
constexpr float kBoxHY = 0.48f;
constexpr float kBoxHZ = 0.30f;
// 主盒正面深度（参考位姿下相机到正面的距离）：分层深度场景（scene=3）以它为"背景层深度"，
// 主体层放在 kSubj 倍的更近处（见 SrLayerCtx）。
constexpr float kFaceZ = kSceneDist - kBoxHZ;
constexpr float kBox2CX = 0.62f;
constexpr float kBox2CY = -0.30f;
constexpr float kBox2CZ = -2.60f;
constexpr float kBox2H = 0.22f;
constexpr float kBackdropZ = -8.0f;
// 放大主盒（scene=4 =「盒中空间」）：**几何与 scene=3 完全同构，只把主盒放大到"正面铺满画幅"**
// （正面半尺寸 = 参考画幅在 z=-kFaceZ 处的视锥：kFaceZ·(W/2)/fx × kFaceZ·(H/2)/fy）。
// 为什么这样做：scene=1/3 的照片先按 contain 缩进盒面、盒面又只占画面 ≈59%，照片只剩 19.7%
// （9:16）/26.4%（3:4）面积，其余是程序纹理——真机反馈"重建的空间里看不出原图痕迹"（坑 127）。
// 放大盒保持**全部已验证结构**：照片 contain + 10% 内缩补边（近景特征锚，坑 122 的原配方，
// 边框与照片同深度）、盒侧面/副盒/远景板（相机一动就从画幅边缘露出来 → 空间感 + 深度层），
// 而照片面积提到 ≈3 倍（判据 [10] 出数：9:16 60.7% / 3:4 81% / 16:9 34.2%）。
// （2026-10-01 曾试过"隧道式盒中空间"：远端墙 + 四壁程序纹理。真机上对 9:16 夜景人像
//  STAGE02 `no 3D points` 失败——把近景可匹配特征全押在"画幅边缘的壁面带"上不如"照片同深度的
//  补边 + 盒侧面/远景"稳；已回退到本方案。教训：近景特征锚要贴着照片，不要只在画幅边缘。）

/**
 * 路线B 深度网格场景（scene=5）的常量——**与离线判据 tools/depth_mesh/depth_mesh_check.js 同口径**。
 * 判据 [7] 会解析下面三个常量并与判据脚本比对：**改名或改值必须同步改判据脚本**。
 * 口径出处（2026-10-01 复核，历史文档数字 ≈ 精确投影值 2 倍，已纠正）：
 *   Θ=±5° → 最近层相邻关键帧位移 ≈66px（可匹配界 80px；既有成功轮 ≈70px / 失败轮 ≈100px）。
 */
constexpr float kDepthZNear = 1.0f;      // 最近内容世界深度（zNear；参考相机光轴距离）
constexpr float kDepthRatio = 3.0f;      // 远/近比 k（zFar = k·zNear）→ 轨道枢轴 d0 = (zNear+zFar)/2 = kSceneDist
constexpr float kDepthThetaDeg = 5.0f;   // 轨道半角 ±Θ（度）——判据扫描表：4/5/6° 均不超界，5° 为默认
constexpr float kKfStepMinPx = 30.0f;    // 相邻关键帧总位移下界（2026-10-02 真机标定，坑 131 配套：
                                         //   fx=700/Θ=2° ≈12px → STAGE02 no 3D points；Θ=5° ≈31px 可建）
constexpr float kKfStepMaxPx = 80.0f;    // 相邻关键帧总位移上界（既有位移守卫口径，判据 GUARD_STEP_PX）
constexpr int32_t kDepthStride = 4;      // 网格抽稀步长（**深度场像素**；深度场 546×728 → 画布 1080×1440）
constexpr int32_t kDepthSkirt = 80;      // 照片网格向画幅外的延展（**画布像素**，与离线判据同单位；
                                         //   承接轨道端点的边缘空隙；单元内部换算到深度场像素）
constexpr float kDepthBgZFactor = 2.5f;  // 远景板深度 = kDepthBgZFactor·zFar
constexpr float kDepthPlaneHX = 7.4f;    // 远景板半宽（世界单位；2026-10-02 随 fx=750 广角+对角弧放大：
                                         //   画幅四角×轨道两端点的对角射线精确需求 7.24，旧 4.4 是
                                         //   fx=1500 窄视场口径，端点对角象限会露黑角；与判据 BG_HALF_W 同值）
constexpr float kDepthPlaneHY = 9.6f;    // 远景板半高（同上；角落对角射线精确需求 9.43，与判据 BG_HALF_H 同值）
// 比例门控阈值（坑 145，2026-10-03 五轮真机数据定档）：refined fy/fx 相对 1 的偏离量。
// 实测分布：正常轮 0.937~1.009（|Δ|≤0.066，用户观感"比例正常"）；"贴图前交换"轮漂到
// 1.121 / 1.266（用户观感"横向轻微拉伸"）。取 0.25 = 只拦**严重**漂移（成片明显变形，
// 不可交付）；轻度漂移（0.09~0.25）仍交付——高饱和素材"交换档通过 + 轻微拉伸"仍优于
// "自然档颜色对调"。拦截时返回 -8，上层尝试序列换档重试；产物文件已完整落盘，可兜底交付。
constexpr float kAspectDriftLimit = 0.25f;

/**
 * 第 i 帧轨道位姿：绕场景中心 c=(0,0,-kSceneDist) 半径 kSceneDist 的**对角弧**，
 * yaw=theta 与 pitch=phi 同步从 −半角 线性扫到 +半角（t = 2i/(n−1) − 1，t=0 恰在原点）。
 * 返回相机位置 pos 与"渲染基"basis（列存储 [right, up, -fwd]：X右/Y上/Z后，右手系，
 * 即 ARKit/AR Engine 相机系；开关只作用于上报位姿，渲染基不变）。
 *
 * ⚠️ 为什么必须是"对角弧"而不是纯水平弧（坑 131，2026-10-02 定案）：
 * 纯水平弧（相机全在 y=0 平面、姿态只含 yaw）下，每个世界点的投影像点坐标满足
 *   v_i = cy − fy · Y / depth_i(X, Z, θ_i)   —— Y 只出现在分子
 * ⇒ 替换 (Y → αY, fy → fy/α) 后**所有帧的所有像素不变**：纵向尺度与 fy 是一对精确
 * 不可观测量，Kit 的 BA 沿该退化方向随机漂移（真机实测 refined fy/fx = 0.155 / 2.47 / 2.69，
 * 对应成片"宽度被压缩 / 宽度拉伸 / 不可名状"，与上报的 fx=fy 无确定函数关系）。
 * 加 pitch（相机中心离开 y=0 平面 + 姿态含俯仰）后 Y 同时进入分子与 depth、且相机 Y 基线
 * 非零 ⇒ fy/纵向尺度变为可观测，各向异性漂移被消除。pitchDeg=0 时本公式与旧"竖直圆弧"
 * 逐位一致（cos0=1/sin0=0），scene=0..4 的已验证行为不受影响。
 */
void SrFrameBasis(int32_t i, int32_t n, float orbitDeg, float pitchDeg, SrVec3 &pos, float basis[9])
{
    const float t = (n > 1) ? (2.0f * i / (n - 1) - 1.0f) : 0.0f;
    const float theta = orbitDeg * static_cast<float>(M_PI) / 180.0f * t;
    const float phi = pitchDeg * static_cast<float>(M_PI) / 180.0f * t;
    const float st = std::sin(theta), ct = std::cos(theta);
    const float sp = std::sin(phi), cp = std::cos(phi);
    // 球坐标绕中心：offset = kSceneDist·(sinθ·cosφ, sinφ, cosθ·cosφ)（φ>0 = 相机在中心上方）
    pos.x = kSceneDist * st * cp;
    pos.y = kSceneDist * sp;
    pos.z = -kSceneDist + kSceneDist * ct * cp;
    const SrVec3 center = { 0.0f, 0.0f, -kSceneDist };
    const SrVec3 fwd = SrNormalize({ center.x - pos.x, center.y - pos.y, center.z - pos.z });
    const SrVec3 worldUp = { 0.0f, 1.0f, 0.0f };
    const SrVec3 right = SrNormalize(SrCross(fwd, worldUp));
    const SrVec3 up = SrCross(right, fwd);
    basis[0] = right.x;
    basis[1] = right.y;
    basis[2] = right.z;
    basis[3] = up.x;
    basis[4] = up.y;
    basis[5] = up.z;
    basis[6] = -fwd.x;
    basis[7] = -fwd.y;
    basis[8] = -fwd.z;
}

/**
 * 预置建模的**对角弧**位姿（地球等预置数据集，2026-10-09）——**照片路线坑 131 正解的同口径移植**。
 *
 * 与 SrFrameBasis（照片路线 scene=5，Φ=Θ）逐句同式：球心 = 世界原点、球半径归一 1，
 * 相机**位置**同时扫 yaw 与 pitch，始终看向球心：
 *   pos(θ) = radius·(sinθ·cosθ, sinθ, cosθ·cosθ)      // φ ≡ θ（Φ=Θ）
 *   fwd = −normalize(pos); right = normalize(fwd × upWorld); up = right × fwd; back = −fwd
 * θ=0 时 pos=(0,0,radius) 且位姿恒等（正对球心，与 flat-pitch 域的 θ=0 差一个"姿态俯仰"）。
 *
 * ⚠️ **为什么必须有这个域**（照片路线真机定案，见 SrFrameBasis 注释与坑 131）：
 * 平环环绕（相机全部共面 y=0 + 姿态只含 yaw/仅姿态俯仰）下，像点
 *   v_i = cy − fy·Y / depth_i(X,Z,θ_i)     —— Y 只出现在分子
 * ⇒ 替换 (Y→αY, fy→fy/α) 后**所有帧的所有像素不变**：纵向尺度与 fy 是**一对精确不可观测量**，
 * 而 Kit 没有锁内参 API，其 BA 会沿该零梯度山谷随机漂（真机实测 refined fy/fx 漂到
 * 0.155 / 2.47 / 2.69 → 成片"宽度被压缩 / 被拉伸 / 不可名状"）。照片路线的解法就是让
 * **相机位置离开 y=0 平面**：Y 同时进入分子与 depth、且相机 Y 基线非零 ⇒ 退化破除、
 * 各向异性漂移被消除（照片路线修复后 refined fy/fx = 0.937~1.009）。
 * 地球预置此前长期用 flat-pitch 域（pos.y ≡ 0，仅姿态俯仰），真机读数 0.119 / 0.474 / 0.254
 * ——正是这个未破除的退化；本域即该正解的移植。
 */
static void SrPresetDiagArcBasis(float thetaDeg, float radius, SrVec3 &pos, float basis[9])
{
    const float th = thetaDeg * static_cast<float>(M_PI) / 180.0f;
    const float st = std::sin(th), ct = std::cos(th);
    pos.x = radius * st * ct;
    pos.y = radius * st;
    pos.z = radius * ct * ct;
    const SrVec3 fwd = SrNormalize({ -pos.x, -pos.y, -pos.z });
    const SrVec3 worldUp = { 0.0f, 1.0f, 0.0f };
    const SrVec3 right = SrNormalize(SrCross(fwd, worldUp));
    const SrVec3 up = SrCross(right, fwd);
    const SrVec3 back = { -fwd.x, -fwd.y, -fwd.z };
    basis[0] = right.x;
    basis[1] = right.y;
    basis[2] = right.z;
    basis[3] = up.x;
    basis[4] = up.y;
    basis[5] = up.z;
    basis[6] = back.x;
    basis[7] = back.y;
    basis[8] = back.z;
}

/**
 * 预置建模的整圈环绕基（地球等预置数据集，2026-10-04 v3）：球心=世界原点、球半径归一 1。
 * 位姿 = 绕数据集标定轴 A（presetAxis，世界系单位向量）整圈环绕：
 *   basis_i = R_A(θ_i)（行序 [right; up; −fwd]，即 R_A 的转置的行）、pos_i = R_A(θ_i)·(0,0,radius)。
 * θ=0 时位姿恒等（相机在 +Z 看向 −Z），对应数据集 0° 参考帧；A=(0,1,0)
 * 时退化为经典水平环绕（旧赤道行为，与既有公式逐位一致）。
 *
 * 为什么用数据集标定的轴（坑 156）：纯水平 look-at 环绕（全部相机 up≡世界 +Y、机位共面）
 * 对 (场景纵向尺度, fy) 是**精确退化**（投影级判据 0.000px）——Kit BA 无锁内参 API，会在零
 * 梯度山谷随机漂（照片路线实锤见坑 131；地球 v3 前两轮真机 fy/fx=3.984 / 0.452 同样是它）。
 * 轴倾角使 up 随环绕变化 → 退化破除（v3 判据 162px）。
 * ⚠️ 轴/半径/焦距三者由离线几何测量得出（tools/depth_mesh：patch-flow 轴拟合 + 纹理自洽
 * 扫描定 D=8 + 留出帧重投影误差 4/255 验证），**两侧必须同式同值**；假设值（如早期 D=2.05、
 * 竖直轴）与真实画面不自洽，会让 Kit 用荒唐比例硬凑（两轮真机 aspect 3.984/0.452 的根因）。
 */
static void SrPresetRingBasis(float thetaDeg, float radius, const float axis[3], SrVec3 &pos, float basis[9])
{
    // Rodrigues：R = I + sinθ·K + (1−cosθ)·K²，K = [a]×
    SrVec3 a = { axis[0], axis[1], axis[2] };
    const float an = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (an < 1e-6f) {
        a = { 0.0f, 1.0f, 0.0f };
    } else {
        a.x /= an;
        a.y /= an;
        a.z /= an;
    }
    const float th = thetaDeg * static_cast<float>(M_PI) / 180.0f;
    const float c = std::cos(th);
    const float s = std::sin(th);
    const float t = 1.0f - c;
    const float R[3][3] = {
        { c + a.x * a.x * t,          a.x * a.y * t - a.z * s, a.x * a.z * t + a.y * s },
        { a.y * a.x * t + a.z * s,    c + a.y * a.y * t,       a.y * a.z * t - a.x * s },
        { a.z * a.x * t - a.y * s,    a.z * a.y * t + a.x * s, c + a.z * a.z * t },
    };
    pos.x = R[0][2] * radius;
    pos.y = R[1][2] * radius;
    pos.z = R[2][2] * radius;
    // basis 行 = R 的转置的行（R·e_j 的转置）：
    basis[0] = R[0][0];
    basis[1] = R[1][0];
    basis[2] = R[2][0];
    basis[3] = R[0][1];
    basis[4] = R[1][1];
    basis[5] = R[2][1];
    basis[6] = R[0][2];
    basis[7] = R[1][2];
    basis[8] = R[2][2];
}

/**
 * v5 参数域的"平环 + 相机俯仰"环绕基（地球等预置数据集，2026-10-05 随 v5 停点启用）。
 * 公式与数据集 manifest.poseConvention 逐句对应（其帧 =
 * 按实测几何把原始视频逐帧精确重投影到本声明几何，"两侧同式同值"铁律）：
 *   pos = R·(sinθ, 0, cosθ)（平环：绕世界 +Y 整圈环绕球心）；
 *   fwd0 = −normalize(pos)；right = normalize(fwd0×upWorld)；up0 = right×fwd0；
 *   相机绕 right 轴俯仰 β = camPitchDeg：up = cosβ·up0 + sinβ·(−fwd0)、
 *   back = −sinβ·up0 + cosβ·(−fwd0)；basis 行序 [right; up; back]。
 * β≠0 使相机 up 随俯仰偏离世界 +Y，打破纯平环对 (纵向尺度, fy) 的精确退化
 * （v5 knownRisks：纯平环 0.0px → 12° 俯仰 79.7px 可观测）。θ=0 位姿恒等（参考帧约定）。
 * ⚠️ 与 presetAxis 轴式域互斥：参数域由数据集 manifest 决定（v3=camAxis / v5=camPitchDeg），
 * 上层按 manifest 字段择一透传，运行期同传防呆见 preset 校验处。
 */
static void SrPresetFlatPitchBasis(float thetaDeg, float radius, float pitchDeg, SrVec3 &pos, float basis[9])
{
    const float th = thetaDeg * static_cast<float>(M_PI) / 180.0f;
    pos.x = radius * std::sin(th);
    pos.y = 0.0f;
    pos.z = radius * std::cos(th);
    const SrVec3 fwd0 = SrNormalize({ -pos.x, -pos.y, -pos.z });
    const SrVec3 worldUp = { 0.0f, 1.0f, 0.0f };
    const SrVec3 right = SrNormalize(SrCross(fwd0, worldUp));
    const SrVec3 up0 = SrCross(right, fwd0);
    const float beta = pitchDeg * static_cast<float>(M_PI) / 180.0f;
    const float cb = std::cos(beta);
    const float sb = std::sin(beta);
    // 绕 right 轴俯仰 β：up0 与 (−fwd0) 张成的平面内旋转
    const SrVec3 up = {
        cb * up0.x - sb * fwd0.x, cb * up0.y - sb * fwd0.y, cb * up0.z - sb * fwd0.z
    };
    const SrVec3 back = {
        -sb * up0.x - cb * fwd0.x, -sb * up0.y - cb * fwd0.y, -sb * up0.z - cb * fwd0.z
    };
    basis[0] = right.x;
    basis[1] = right.y;
    basis[2] = right.z;
    basis[3] = up.x;
    basis[4] = up.y;
    basis[5] = up.z;
    basis[6] = back.x;
    basis[7] = back.y;
    basis[8] = back.z;
}

/**
 * 由渲染基构造"上报给 Kit 的位姿"。
 *
 * 有效相机系只有两种（都是右手系，只差坐标轴标签，对同一像素给出的 (u,v) 完全一致——
 * 见 tools/sr_frames_check.js 判据 [2]，故可安全 A/B）：
 * - family 0（AR 系）：X右 / Y上 / Z后（看 -Z）→ R = basis（ARKit/AR Engine 相机系）；
 * - family 1（CV 系）：X右 / Y下 / Z前（看 +Z）→ R = basis · diag(1,-1,-1)（OpenCV 相机系）。
 * 注意：单列取反式翻转会产生行列式 -1 的反射矩阵、四元数无从提取；"镜像世界 + 共轭"式
 * 换算会破坏针孔模型（图像与位姿不再自洽）——两者都是废弃方案，勿再用。
 * invertQuat 与相机系正交（w2c 存储时对四元数取共轭，不改动世界/位置）。
 */
void SrReportPose(int32_t family, bool invertQuat, const float basis[9], const SrVec3 &pos,
                  float q[4], SrVec3 &posOut)
{
    float m[9];
    std::memcpy(m, basis, sizeof(m));
    posOut = pos;   // 相机系换标签不移动相机中心
    if (family == 1) {
        // 右乘 diag(1,-1,-1)：取反第 1、2 列（Y 轴、Z 轴）
        m[3] = -m[3];
        m[4] = -m[4];
        m[5] = -m[5];
        m[6] = -m[6];
        m[7] = -m[7];
        m[8] = -m[8];
    }
    SrMatToQuat(m, q);
    if (invertQuat) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
    }
}

/**
 * 双线性采样（出界返回暗底）。ch = 每像素字节数：源图为 4（RGBA），
 * 分层深度场景的背景层为 3（RGB，见 spatial_recon_layers.h）。
 */
void SrSampleImg(const uint8_t *img, int32_t imgW, int32_t imgH, int32_t ch,
                 float sx, float sy, uint8_t out[3])
{
    if (sx < 0.0f || sy < 0.0f || sx > imgW - 1 || sy > imgH - 1) {
        out[0] = 16;
        out[1] = 16;
        out[2] = 20;
        return;
    }
    const int32_t x0 = static_cast<int32_t>(sx);
    const int32_t y0 = static_cast<int32_t>(sy);
    const int32_t x1 = std::min(x0 + 1, imgW - 1);
    const int32_t y1 = std::min(y0 + 1, imgH - 1);
    const float ax = sx - x0;
    const float ay = sy - y0;
    for (int32_t c = 0; c < 3; c++) {
        const float p00 = img[(static_cast<size_t>(y0) * imgW + x0) * ch + c];
        const float p10 = img[(static_cast<size_t>(y0) * imgW + x1) * ch + c];
        const float p01 = img[(static_cast<size_t>(y1) * imgW + x0) * ch + c];
        const float p11 = img[(static_cast<size_t>(y1) * imgW + x1) * ch + c];
        const float top = p00 + (p10 - p00) * ax;
        const float bot = p01 + (p11 - p01) * ax;
        float val = top + (bot - top) * ay;
        if (val < 0.0f) val = 0.0f;
        if (val > 255.0f) val = 255.0f;
        out[c] = static_cast<uint8_t>(val + 0.5f);
    }
}

/** 通道序探针（2026-10-04 坑 132 复核）：mean(R−B)——对"R/B 是否被交换"最敏感、对亮度加权
 *  不变（ASCII 缩略图正是亮度加权，对交换全盲，坑 132 的教训）。1/16 像素采样（与 ArkTS 侧
 *  swapRisk 预判同密度）。用途：把"自定义帧生成→推帧侧无通道交换"从代码审计升级为字节级
 *  判据——srcRB（自然源缓冲，swapRgbAtInput 之前取值）vs frameRB（最终推给 Kit 的字节，
 *  交换补丁之后取值）：bgrSwap=0 时两者**同号** ⇒ 我方管线在推帧前未改动通道（反相现象
 *  发生在推帧之后，成因未定论，见档案 F7）；**异号** ⇒ 我方管线自身在交换，须回头修正。
 *  bgrSwap=1 时异号是预期（补丁生效的自证）。测试素材选红/蓝倾向强的照片（swapRisk 高者，
 *  判据余度大；R≈B 的灰绿图符号不可判）。 */
static float SrMeanRmB(const uint8_t *px, size_t bytes, int32_t ch) {
    double sum = 0.0;
    int64_t cnt = 0;
    for (size_t p = 0; p + 2 < bytes; p += static_cast<size_t>(ch) * 16) {
        sum += static_cast<int32_t>(px[p]) - static_cast<int32_t>(px[p + 2]);
        cnt++;
    }
    return cnt > 0 ? static_cast<float>(sum / cnt) : 0.0f;
}

/**
 * refined 比例探针（坑 131/145）：选帧读回"重建优化后"的内外参，取首个有效读数的 fy/fx。
 * tag 标注调用窗口（试验档 "pre" = 保存前 FINISHED 窗口 / 常规档 "post" = 保存后），
 * 日志前缀保持 `SRCHK refined`（grep 口径不变）。比值语义：>1 横向拉伸、<1 横向压缩；
 * 绝对值无意义（BA 整体尺度规约自由）。不返回像素（imageData=null）；不抛错只打日志。
 */
static bool SrProbeRefinedAspect(HMS_SpatialRecon_Session *session, int32_t frameCount,
                                 const char *tag, float &outAspect) {
    outAspect = 0.0f;
    bool have = false;
    if (g_srApi.getRefinedFrame == nullptr) {
        return false;
    }
    const int32_t probeIdx[3] = { 0, frameCount / 2, frameCount - 1 };
    for (int32_t k = 0; k < 3; k++) {
        HMS_SpatialRecon_DataFrame rf{};
        const HMS_SpatialReconStatus rs = g_srApi.getRefinedFrame(session, probeIdx[k], &rf);
        if (rs != SPATIAL_RECON_STATUS_SUCCESS) {
            VG_LOGW("SRCHK refined[%{public}s] i=%{public}d failed status=%{public}d",
                    tag, probeIdx[k], static_cast<int32_t>(rs));
            continue;
        }
        const float aspect = (std::fabs(rf.focalX) > 1e-9f) ? rf.focalY / rf.focalX : 0.0f;
        VG_LOGI("SRCHK refined[%{public}s] i=%{public}d fx=%{public}f fy=%{public}f aspect=%{public}.3f cx=%{public}f cy=%{public}f "
                "pos=(%{public}f,%{public}f,%{public}f) quat=(%{public}f,%{public}f,%{public}f,%{public}f)",
                tag, probeIdx[k], rf.focalX, rf.focalY, aspect, rf.principalX, rf.principalY,
                rf.position[0], rf.position[1], rf.position[2],
                rf.rotation[0], rf.rotation[1], rf.rotation[2], rf.rotation[3]);
        if (aspect > 0.0f && !have) {
            outAspect = aspect;
            have = true;
        }
        if (aspect > 0.0f && std::fabs(aspect - 1.0f) > 0.25f) {
            VG_LOGW("SRCHK refined[%{public}s] anisotropy fy/fx=%{public}.3f (pushed 1.0) —— 纵横尺度比漂移 "
                    "超 25%%，退化漂移嫌疑（坑 131；对角弧修复后应 ≈1，若仍出现需复查轨道）", tag, aspect);
        }
    }
    return have;
}

/** 主体蒙版双线性采样 → 主体概率 [0,1]（软边 matte；出界=0） */
float SrSampleMask(const uint8_t *mask, int32_t srcW, int32_t srcH, float sx, float sy)
{
    if (sx < 0.0f || sy < 0.0f || sx > srcW - 1 || sy > srcH - 1) {
        return 0.0f;
    }
    const int32_t x0 = static_cast<int32_t>(sx);
    const int32_t y0 = static_cast<int32_t>(sy);
    const int32_t x1 = std::min(x0 + 1, srcW - 1);
    const int32_t y1 = std::min(y0 + 1, srcH - 1);
    const float ax = sx - x0;
    const float ay = sy - y0;
    const float p00 = SrMaskAlpha(mask[static_cast<size_t>(y0) * srcW + x0]);
    const float p10 = SrMaskAlpha(mask[static_cast<size_t>(y0) * srcW + x1]);
    const float p01 = SrMaskAlpha(mask[static_cast<size_t>(y1) * srcW + x0]);
    const float p11 = SrMaskAlpha(mask[static_cast<size_t>(y1) * srcW + x1]);
    const float top = p00 + (p10 - p00) * ax;
    const float bot = p01 + (p11 - p01) * ax;
    return top + (bot - top) * ay;
}

/**
 * 分层深度场景素材（scene=3；主体蒙版有效时启用）：
 * - 背景层 = layers->bgRgb（主体区域已补洞），仍贴在主盒正面 z = -kFaceZ；
 * - 主体层 = 源图本身，贴在"主盒正面内容按 kSubj 等比缩放"的近平面 z = -kFaceZ·kSubj。
 *
 * 两层在参考位姿（相机在原点且无旋转）下**投影严格重合**：同一像素取到的源图坐标相同，
 * 故参考视图 == 原照片；相机离开参考位姿后才出现视差，视差量随 kSubj→1 而趋零。
 * kSubj 越小主体越"前凸"：视差更大、但主体相对背景的滑移也更大（露出的补洞区更宽）。
 */
struct SrLayerCtx {
    const uint8_t *mask = nullptr;   /**< srcW*srcH，0~255（语义见 SrMaskAlpha） */
    const uint8_t *bgRgb = nullptr;  /**< srcW*srcH*3，主体区域已补洞的背景层 */
    float kSubj = 0.90f;             /**< 主体平面深度比（<1 = 比背景层更近；取值受可匹配位移量级约束，见头文件） */
};

// 立体场景命中信息
struct SrBoxHitInfo {
    float t = -1.0f;      /**< 命中距离（>0 有效） */
    int32_t axis = -1;    /**< 命中面法线轴 0/1/2 */
    int32_t sign = 0;     /**< 命中面朝向 -1/+1 */
    SrVec3 point;         /**< 命中点（世界系） */
};

/** 射线-AABB（slab 法）求交：盒子中心 (cx,cy,cz)，半长 (hx,hy,hz) */
SrBoxHitInfo SrBoxIntersect(const SrVec3 &o, const SrVec3 &d,
                            float cx, float cy, float cz, float hx, float hy, float hz)
{
    SrBoxHitInfo hit;
    const float minV[3] = { cx - hx, cy - hy, cz - hz };
    const float maxV[3] = { cx + hx, cy + hy, cz + hz };
    const float oo[3] = { o.x, o.y, o.z };
    const float dd[3] = { d.x, d.y, d.z };
    float tmin = -1e30f;
    float tmax = 1e30f;
    int32_t axisMin = -1;
    int32_t signMin = 0;
    for (int32_t a = 0; a < 3; a++) {
        if (std::fabs(dd[a]) < 1e-9f) {
            if (oo[a] < minV[a] || oo[a] > maxV[a]) {
                return SrBoxHitInfo();
            }
            continue;
        }
        const float inv = 1.0f / dd[a];
        float t1 = (minV[a] - oo[a]) * inv;
        float t2 = (maxV[a] - oo[a]) * inv;
        int32_t sign = -1;
        if (t1 > t2) {
            const float tmp = t1;
            t1 = t2;
            t2 = tmp;
            sign = 1;
        }
        if (t1 > tmin) {
            tmin = t1;
            axisMin = a;
            signMin = sign;
        }
        if (t2 < tmax) {
            tmax = t2;
        }
        if (tmin > tmax) {
            return SrBoxHitInfo();
        }
    }
    if (axisMin < 0 || tmin < 1e-4f) {
        return SrBoxHitInfo(); // 相机在盒内/过近：按未命中处理
    }
    hit.t = tmin;
    hit.axis = axisMin;
    hit.sign = signMin;
    hit.point = { o.x + d.x * tmin, o.y + d.y * tmin, o.z + d.z * tmin };
    return hit;
}

/**
 * 伪随机哈希：整数晶格 (gx,gy) + 种子 → [0,1)。
 * 用于生成"处处不同"的纹理——棋盘格是周期图案，特征匹配会大量歧义（同一角落重复出现），
 * 对 SfM 是坏输入；随机晶格纹理的每个角落都唯一。
 */
float SrHash01(float gx, float gy, float seed)
{
    const int32_t ix = static_cast<int32_t>(std::floor(gx));
    const int32_t iy = static_cast<int32_t>(std::floor(gy));
    uint32_t h = static_cast<uint32_t>(ix) * 73856093u
               ^ static_cast<uint32_t>(iy) * 19349663u
               ^ static_cast<uint32_t>(seed) * 83492791u;
    h ^= h >> 13;
    h *= 1274126177u;
    h ^= h >> 16;
    return static_cast<float>(h & 0xFFFFu) * (1.0f / 65536.0f);
}

/** 非重复纹理：两级随机晶格叠加（粗 0.16 单位 + 细 0.05 单位），特征唯一且有明显台阶边 */
uint8_t SrRandomTexture(float u, float v, float seed)
{
    const float c1 = SrHash01(u / 0.16f, v / 0.16f, seed);
    const float c2 = SrHash01(u / 0.05f, v / 0.05f, seed + 31.0f);
    float val = 26.0f + 128.0f * c1 + 78.0f * c2;
    if (val < 0.0f) val = 0.0f;
    if (val > 255.0f) val = 255.0f;
    return static_cast<uint8_t>(val);
}

/**
 * 诊断：把一帧降采样成 ASCII 缩略图打进 hilog（人工核验合成帧是否正常：
 * 内容是否正确、是否上下颠倒、是否全黑/全白）。channels = 3(RGB) 或 4(RGBA)。
 */
void SrLogAscii(const char *tag, const uint8_t *data, int32_t w, int32_t h, int32_t channels)
{
    constexpr int32_t kCols = 48;
    constexpr int32_t kRows = 32;
    const char *ramp = " .:-=+*#%@";
    int64_t sum = 0;
    int32_t lumMin = 255;
    int32_t lumMax = 0;
    for (int32_t r = 0; r < kRows; r++) {
        char line[kCols + 1];
        for (int32_t c = 0; c < kCols; c++) {
            const int32_t x = (c * w) / kCols;
            const int32_t y = (r * h) / kRows;
            const size_t o = (static_cast<size_t>(y) * w + x) * channels;
            const int32_t lum = (data[o] * 30 + data[o + 1] * 59 + data[o + 2] * 11) / 100;
            sum += lum;
            if (lum < lumMin) lumMin = lum;
            if (lum > lumMax) lumMax = lum;
            line[c] = ramp[(lum * 9) / 255];
        }
        line[kCols] = '\0';
        VG_LOGI("SRCHK %{public}s r%{public}02d |%{public}s|", tag, r, line);
    }
    VG_LOGI("SRCHK %{public}s lum avg=%{public}d min=%{public}d max=%{public}d",
            tag, static_cast<int32_t>(sum / (kCols * kRows)), lumMin, lumMax);
}

/**
 * 渲染一帧：相机位于 pos、渲染基 basis，逐像素发射线。
 * scene=0：平面贴图（源图 contain 在 z=-kSceneDist 平面，出界暗底）；
 * scene=1：立体场景（主盒正面贴源图 + 主盒侧面/副盒非重复纹理 + z=-8 远景非重复纹理背景板）；
 * scene=2：同 scene=1 但主盒正面也换成非重复纹理（不放源图）——用于隔离"源图纹理质量"
 *          对 SfM 的影响（纯程序化输入是特征匹配的最优情形）；
 * scene=3：分层深度（主体/背景两层，layers 非空时生效）——主盒正面贴"主体区域已补洞"的
 *          背景层，源图（按蒙版 alpha）另贴在一张更近的前景平面上，相机环绕时主体与背景
 *          产生真实视差；参考位姿处两层投影重合（合成画面 == 原照片）。layers 为空时
 *          本参数等价于 scene=1（蒙版退化/设备无分割能力时的自动回退）；
 * scene=4：盒中空间（同一套分层思路的**放大主盒**几何）——与 scene=3 完全同构，只把主盒正面
 *          放大到"铺满参考画幅"：照片仍按同一套 contain + 10% 内缩贴着该面（照片占画面
 *          60.7%/81%/34.2% 对应 9:16/3:4/16:9，约是 box-face 方案的 3 倍），照片之外那一圈
 *          仍是与照片同深度的程序纹理补边（近景特征锚，坑 122），盒侧面/副盒/远景板在相机
 *          一动时从画幅边缘露出来（空间感 + 深度层）；主体层同 scene=3（layers 非空时）。
 *
 * 射线约定（关键，勿改）：像素 (u,v) 的光线 = a·right + y·up + fwd，其中
 *   a = (u+0.5-cx)/fx，y = -(v+0.5-cy)/fy。
 * 即图像行向下 ↔ 相机 Y 轴向下。历史教训：首版写成 y = +(v+0.5-cy)/fy，等于把画面
 * 垂直镜像后再与位姿配对（镜像不能由任何位姿约定补偿），真机表现为 STAGE02 no 3D points。
 * 见 tools/sr_frames_check.js 判据 [1]。
 *
 * ⚠️ basis 的第 2 列（basis[6..8]）存的是 **-fwd**（camera 系 Z 后），所以代码里这一列必须
 * **取负**才是 +fwd。次级教训：改成"按列取用"时漏了这一步负号，射线整根朝相机背后 →
 * 每帧只剩出界底色（真机日志 f000 缩略图 lum 恒 16、重建仍报 STAGE02 no 3D points）。
 * 返回值为"无几何命中"像素数：>50% 即整帧空，属配置错误（见 tools/sr_frames_check.js 判据 [6]）。
 */
int64_t SrRenderFrame(const uint8_t *src, int32_t srcW, int32_t srcH,
                      const SrVec3 &pos, const float basis[9],
                      float fx, float fy, float cx, float cy,
                      int32_t scene, const SrLayerCtx *layers,
                      std::vector<uint8_t> &rgb)
{
    rgb.resize(static_cast<size_t>(kOutW) * kOutH * 3);
    int64_t missPx = 0;
    // contain：源图等比铺进 1080x1440 画布（平面场景的贴图定位）
    const float scale = std::min(static_cast<float>(kOutW) / srcW, static_cast<float>(kOutH) / srcH);
    const float dispW = srcW * scale;
    const float dispH = srcH * scale;
    const float ox = (kOutW - dispW) * 0.5f;
    const float oy = (kOutH - dispH) * 0.5f;
    const bool textureFace = (scene == 2);   // 主盒正面也用程序化纹理
    // "照片平面"的深度与半尺寸（逐场景）：
    // - scene 0~3：主盒正面（z=-kFaceZ，半尺寸 kBoxHX/kBoxHY）；
    // - scene 4：放大主盒正面（z 仍是 -kFaceZ，半尺寸 = 参考画幅在 z=-kFaceZ 处的视锥）。
    float faceZ = kFaceZ;
    float faceHX = kBoxHX;
    float faceHY = kBoxHY;
    // 放大主盒（scene=4）：正面 = 参考画幅在 z=-kFaceZ 处的视锥（铺满画幅）→ 照片按同一套
    // contain+内缩映射贴着放大后的面，照片面积随之变成原来的 ≈3 倍。
    const float boxHX = (scene == 4) ? (kFaceZ * static_cast<float>(kOutW) * 0.5f / fx) : kBoxHX;
    const float boxHY = (scene == 4) ? (kFaceZ * static_cast<float>(kOutH) * 0.5f / fy) : kBoxHY;
    if (scene == 4) {
        faceHX = boxHX;
        faceHY = boxHY;
        faceZ = kFaceZ;
    }
    // 照片矩形（面内归一化，与判据工具 facePhotoRect 同构）：照片按自身宽高比
    // contain 到"照片平面"内，再统一内缩 kPhotoInset（近景特征锚，坑 122）。两处消费
    // （平面采样与分层深度的主体层命中）共用这一份，逐帧首次计算后复用。scene=4 的平面半尺寸
    // 已按源图宽高比 contain 过，故这里的 contain 退化为恒等，只留内缩那一圈。
    const float imgAspect = static_cast<float>(srcW) / static_cast<float>(srcH);
    const float faceAspect = faceHX / faceHY;
    constexpr float kPhotoInset = 0.90f;
    float photoW = 1.0f;
    float photoH = faceAspect / imgAspect;
    if (imgAspect < faceAspect) {
        photoH = 1.0f;
        photoW = imgAspect / faceAspect;
    }
    photoW *= kPhotoInset;
    photoH *= kPhotoInset;
    const float photoU0 = (1.0f - photoW) * 0.5f;
    const float photoV0 = (1.0f - photoH) * 0.5f;

    for (int32_t v = 0; v < kOutH; v++) {
        for (int32_t u = 0; u < kOutW; u++) {
            // 像素中心 → 世界空间光线（Y 向下 ↔ 图像行向下，见函数头注释）：
            // dir = a·X_cam + b·Y_cam + fwd，而 basis 第 2 列是 -fwd，故该项取负号（勿写成 +）。
            const float a = (u + 0.5f - cx) / fx;
            const float b = -(v + 0.5f - cy) / fy;
            const SrVec3 dir = {
                basis[0] * a + basis[3] * b - basis[6],
                basis[1] * a + basis[4] * b - basis[7],
                basis[2] * a + basis[5] * b - basis[8],
            };
            uint8_t col[3] = { 16, 16, 20 };
            bool covered = false;   // 本像素是否有几何命中（false 即底色，用于空帧自检）
            if (scene >= 1 && scene <= 3) {
                // 立体场景：主盒 / 副盒 / 远景背景板，取最近命中
                const SrBoxHitInfo bh = SrBoxIntersect(pos, dir, 0.0f, 0.0f, -kSceneDist,
                                                       boxHX, boxHY, kBoxHZ);
                const SrBoxHitInfo b2 = SrBoxIntersect(pos, dir, kBox2CX, kBox2CY, kBox2CZ,
                                                       kBox2H, kBox2H, kBox2H);
                float tBd = -1.0f;
                if (dir.z < -1e-6f) {
                    tBd = (kBackdropZ - pos.z) / dir.z;
                }
                const bool useBox = bh.t > 0.0f && (b2.t <= 0.0f || bh.t < b2.t) && (tBd <= 0.0f || bh.t < tBd);
                const bool useBox2 = !useBox && b2.t > 0.0f && (tBd <= 0.0f || b2.t < tBd);
                if (useBox) {
                    covered = true;
                    if (bh.axis == 2 && bh.sign > 0 && !textureFace) {
                        // 主盒正面（朝向第 0 帧相机）：源图**按自身宽高比 contain 贴合**（不拉伸、不裁切），
                        // 面内余量用程序纹理补（与侧面同种子，视觉连贯）。
                        // 历史教训：首版把源图整幅铺满 0.6x0.8 的面 → 非 3:4 照片被各向异性拉伸
                        // （16:9 照片横向压缩 1.78 倍），成片里照片是变形的；且极端比例的图会挤成窄条。
                        // 补边的第二个作用：让"近景、与照片同深度"处也有可匹配的独有特征——
                        // 实测 Kit 三角化长期贴在阈值下限（成功轮也报 `Too few points`），
                        // 不能再让照片内容独自承担重建（自相似/周期纹理的照片会把它压到 0 个点）。
                        const float u01 = bh.point.x / faceHX * 0.5f + 0.5f;   // 面上归一化坐标
                        const float v01 = 0.5f - bh.point.y / faceHY * 0.5f;
                        const float fu = (u01 - photoU0) / photoW;
                        const float fv = (v01 - photoV0) / photoH;
                        if (fu >= 0.0f && fu <= 1.0f && fv >= 0.0f && fv <= 1.0f) {
                            // 分层深度场景：正面贴的是"主体区域已补洞"的背景层（其余像素与原图逐像素相同）
                            if (layers != nullptr && layers->bgRgb != nullptr) {
                                SrSampleImg(layers->bgRgb, srcW, srcH, 3,
                                            fu * static_cast<float>(srcW - 1),
                                            fv * static_cast<float>(srcH - 1), col);
                            } else {
                                SrSampleImg(src, srcW, srcH, 4,
                                            fu * static_cast<float>(srcW - 1),
                                            fv * static_cast<float>(srcH - 1), col);
                            }
                        } else {
                            const uint8_t tone = SrRandomTexture(bh.point.x, bh.point.y + bh.point.z, 11.0f);
                            col[0] = tone;
                            col[1] = tone;
                            col[2] = tone;
                        }
                    } else {
                        const uint8_t tone = SrRandomTexture(bh.point.x, bh.point.y + bh.point.z, 11.0f);
                        col[0] = tone;
                        col[1] = tone;
                        col[2] = tone;
                    }
                } else if (useBox2) {
                    covered = true;
                    const uint8_t tone = SrRandomTexture(b2.point.x + b2.point.z, b2.point.y, 23.0f);
                    col[0] = tone;
                    col[1] = tone;
                    col[2] = tone;
                } else if (tBd > 0.0f) {
                    covered = true;
                    const float bx = pos.x + dir.x * tBd;
                    const float by = pos.y + dir.y * tBd;
                    const uint8_t tone = SrRandomTexture(bx, by, 37.0f);
                    col[0] = tone;
                    col[1] = tone;
                    col[2] = tone;
                }
            } else {
                // 平面场景：与 z=-kSceneDist 求交 → 投影回第 0 帧相机（单位位姿）→ contain 采样
                if (std::fabs(dir.z) > 1e-9f) {
                    const float t = (-kSceneDist - pos.z) / dir.z;
                    if (t > 0.0f) {
                        const SrVec3 ph = { pos.x + dir.x * t, pos.y + dir.y * t, pos.z + dir.z * t };
                        if (-ph.z > 1e-6f) {
                            const float u0 = cx + fx * ph.x / -ph.z;
                            const float v0 = cy - fy * ph.y / -ph.z;   // 与光线约定配套，勿改为 +
                            SrSampleImg(src, srcW, srcH, 4, ox + u0, oy + v0, col);
                            covered = true;
                        }
                    }
                }
            }
            if (!covered) {
                missPx++;
            }
            // 分层深度（scene=3 / scene=4）：主体层按蒙版 alpha 叠加在场景色之上。
            // 主体层 = "照片平面内容按 kSubj 等比缩放"的近平面（scene=3 的主盒正面 / scene=4 的开口面）。
            // 近平面与照片平面共用上文同一套射线公式，只是"命中点折算回照片平面坐标"后再走同一份
            // contain 映射取源图坐标（射线约定只有一处，勿在此重写方向式——坑 119/120）。
            // 参考位姿（相机在原点、无旋转）处两层取到同一源图坐标 → 合成画面恒等于原照片。
            if (layers != nullptr && layers->mask != nullptr && dir.z < -1e-6f) {
                const float tSubj = (-faceZ * layers->kSubj - pos.z) / dir.z;
                if (tSubj > 0.0f) {
                    const float invK = 1.0f / layers->kSubj;
                    const float xs = (pos.x + dir.x * tSubj) * invK;
                    const float ys = (pos.y + dir.y * tSubj) * invK;
                    const float u01 = xs / faceHX * 0.5f + 0.5f;
                    const float v01 = 0.5f - ys / faceHY * 0.5f;
                    const float fu = (u01 - photoU0) / photoW;
                    const float fv = (v01 - photoV0) / photoH;
                    if (fu >= 0.0f && fu <= 1.0f && fv >= 0.0f && fv <= 1.0f) {
                        const float sx = fu * static_cast<float>(srcW - 1);
                        const float sy = fv * static_cast<float>(srcH - 1);
                        const float m = SrSampleMask(layers->mask, srcW, srcH, sx, sy);
                        if (m > 0.0f) {
                            uint8_t subj[3];
                            SrSampleImg(src, srcW, srcH, 4, sx, sy, subj);
                            for (int32_t c = 0; c < 3; c++) {
                                col[c] = static_cast<uint8_t>(
                                    m * subj[c] + (1.0f - m) * col[c] + 0.5f);
                            }
                        }
                    }
                }
            }
            const size_t o = (static_cast<size_t>(v) * kOutW + u) * 3;
            rgb[o] = col[0];
            rgb[o + 1] = col[1];
            rgb[o + 2] = col[2];
        }
    }
    return missPx;
}

} // namespace

// ---------------- 对外接口 ----------------

int32_t SpatialReconProbe()
{
    SrBindApi();
    if (!g_srApi.ok) {
        return -1;
    }
    const HMS_SpatialReconStatus st = g_srApi.isSupport(SPATIAL_RECON_MODEL_TYPE_GS);
    VG_LOGI("SRCHK probe status=%{public}d", static_cast<int32_t>(st));
    return static_cast<int32_t>(st);
}

float SpatialReconTestProgress()
{
    return g_srProgress.load();
}

int32_t SpatialReconTestStage()
{
    return g_srStage.load();
}

int32_t SpatialReconPause()
{
    SrBindApi();
    HMS_SpatialRecon_Session *s = g_srSession.load();
    if (!g_srApi.ok || s == nullptr || g_srApi.pauseSession == nullptr) {
        return -1;
    }
    const HMS_SpatialReconStatus st = g_srApi.pauseSession(s);
    VG_LOGI("SRCHK pause status=%{public}d", static_cast<int32_t>(st));
    if (st == SPATIAL_RECON_STATUS_SUCCESS) {
        g_srPaused.store(true);
    }
    return static_cast<int32_t>(st);
}

int32_t SpatialReconResume()
{
    SrBindApi();
    HMS_SpatialRecon_Session *s = g_srSession.load();
    if (!g_srApi.ok || s == nullptr || g_srApi.resumeSession == nullptr) {
        return -1;
    }
    const HMS_SpatialReconStatus st = g_srApi.resumeSession(s);
    VG_LOGI("SRCHK resume status=%{public}d", static_cast<int32_t>(st));
    if (st == SPATIAL_RECON_STATUS_SUCCESS) {
        g_srPaused.store(false);
    }
    return static_cast<int32_t>(st);
}

int32_t SpatialReconAbort()
{
    SrBindApi();
    VG_LOGI("SRCHK abort requested");
    g_srAbort.store(1);
    // 立即暂停以停掉重计算；若当前在保存阶段 Pause 会失败（保存不可中断），
    // 此时终止请求被忽略，产物照常保存并返回成功。
    return SpatialReconPause();
}

/**
 * 当前是否处于"保存收尾"阶段（1=是；0=否）。供 UI 精确判定暂停/终止按钮的可用性。
 * ⚠️ 必须以本标志为准、**不能用 Kit 的 stage 值**：保存期 Kit 可能报 FINISHED(3) 而非
 * SAVING(4)（真机日志 `SRCHK stage -> 3 at kit=1.000`），此时按钮看着可点、点下去却无效
 * ——用户感知"点了没反应"（2026-10-05 用户实测反馈的根因）。
 */
int32_t SpatialReconInSave()
{
    return g_srInSave.load();
}

/**
 * 前后台切换 → 更新运行模式（2026-10-04，官方规则补全）。
 * ⚠️ 不主动 SrBindApi：未用过 3D 功能的进程调用应零成本返回（官方："非活动会话期间调用无效果"）；
 * 期望值先记录，StartSession 时套用（覆盖"推帧阶段就切后台"的窗口）。
 */
int32_t SpatialReconSetForeground(int32_t fg)
{
    g_srWantForeground.store(fg != 0 ? 1 : 0);
    if (!g_srApi.ok) {
        return 0;
    }
    HMS_SpatialRecon_Session *s = g_srSession.load();
    if (s == nullptr || g_srApi.setRunningMode == nullptr) {
        return 0;
    }
    const HMS_SpatialReconStatus st = g_srApi.setRunningMode(s, fg != 0
        ? SPATIAL_RECON_RUNNING_FOREGROUND_MODE : SPATIAL_RECON_RUNNING_BACKGROUND_MODE);
    VG_LOGI("SRCHK running mode fg=%{public}d status=%{public}d", fg != 0 ? 1 : 0,
            static_cast<int32_t>(st));
    return static_cast<int32_t>(st);
}

/** 路线B 深度网格场景（scene=5）素材：纯 2D 单元构建的三层几何 + 远景板纹理 */
struct SrDepthMeshCtx {
    glassvideo::SrDepthMesh mesh;      // 前景层（照片网格）+ 背景层（补洞）
    std::vector<uint8_t> planeRgb;     // 远景板兜底纹理（源帧强模糊 + 压暗）
    int planeW = 0;
    int planeH = 0;
    bool valid = false;
};

/**
 * scene=5：**把深度网格的（画布像素 + 光轴深度）一次性固化成世界点**（世界系 = 参考位姿相机系，
 * 即 §3.1 的"参考帧就是照片本身、参考位姿 = identity"，反投影 P = z·dir(u,v)）。
 *
 * ⚠️ 为什么必须"一次性"（坑 129，真机实测失败根因之一）：若把这一步放在**逐帧**渲染里、
 * 用**当前帧**的 basis 重建世界点，则每个顶点的投影恒等于它的像素坐标（u ≡ q.x，与 θ 无关）
 * → 72 帧画面**逐帧完全相同（零视差）**，而上报位姿照常变化 → Kit 特征匹配"有图像无基线"，
 * 三角化 0 点（`[GSRECON][STAGE02] no 3D points` → `STAGE03 Adapting failed`）。
 * 世界点只此一份，逐帧只做投影（SrRasterizeDepthMesh 内不再出现射线/反投影）。
 */
void SrBakeDepthMeshWorld(glassvideo::SrDepthMesh &mesh, float fx, float fy, float cx, float cy)
{
    const auto bake = [&](std::vector<glassvideo::SrDepthQuad> &quads) {
        for (auto &q : quads) {
            for (int k = 0; k < 4; k++) {
                const float a = (q.x[k] - cx) / fx;    // 参考位姿下的射线方向（前进分量 1）
                const float b = -(q.y[k] - cy) / fy;
                const float z = q.z[k];
                q.x[k] = a * z;
                q.y[k] = b * z;
                q.z[k] = -z;
            }
        }
    };
    bake(mesh.front);
    bake(mesh.bg);
}

/**
 * scene=5：深度网格多视角帧合成（离线判据 tools/depth_mesh/depth_mesh_check.js 的 C++ 实装，
 * 判据合成 18/0、真实照片 16/0 已证几何；三层占比实测 前景≈90.6% / 背景层≈9.4% / 远景板 0.0%）。
 * 相机数学**只此一份**（与 SrRenderFrame 同源；坑 119 的 b 取负、坑 120 的 basis[6..8] 取负）：
 *   世界点 → 像素：p = basisᵀ(P−pos)，u = cx + fx·p.x/(−p.z)，v = cy − fy·p.y/(−p.z)
 *   顶点世界点：由 SrBakeDepthMeshWorld 在构建时固化（**逐帧不得重建**，见该函数注释 / 坑 129）
 * 纯 2D 部分（深度→四边形、光栅化）在 spatial_recon_depth_mesh.cpp（本文件不重复其算法）。
 * 绘制顺序：远景板 → 前景层 → 背景层（共享 z-buffer；等深度处先画者胜）。
 * 返回值 = "无几何命中"像素数（与 SrRenderFrame 同契约；>50% 视为配置错误）。
 */
int64_t SrRasterizeDepthMesh(const SrDepthMeshCtx &dm, const SrVec3 &pos, const float basis[9],
                             float fx, float fy, float cx, float cy, std::vector<uint8_t> &rgb,
                             bool logLayerCover = false)
{
    rgb.assign(static_cast<size_t>(kOutW) * kOutH * 3, 0);
    const size_t total = static_cast<size_t>(kOutW) * kOutH;
    std::vector<float> zbuf(total, 0.0f);
    // 诊断（坑 143 轮 9）：逐像素**最终可见图层**统计（0=无命中 1=远景板 2=前景层 3=背景层）。
    // 用途：回答"远景板到底可见吗"——两模式唯一已知的管线内不对称（远景板压暗 0.72/0.72/0.75
    // 落在对调通道上，差 ~4%）只作用于**远景板可见像素**；覆盖率≈0 即两模式推帧字节应完全相同。
    std::vector<uint8_t> layerMap(logLayerCover ? total : 0, 0);

    // 世界点 → 屏幕顶点（本函数的唯一投影实现，口径同上）
    const auto worldToVert = [&](const SrVec3 &P, float tu, float tv,
                                 glassvideo::SrScreenVert &o) -> bool {
        const SrVec3 d = { P.x - pos.x, P.y - pos.y, P.z - pos.z };
        const float px = basis[0] * d.x + basis[1] * d.y + basis[2] * d.z;   // X_cam
        const float py = basis[3] * d.x + basis[4] * d.y + basis[5] * d.z;   // Y_cam
        const float pz = basis[6] * d.x + basis[7] * d.y + basis[8] * d.z;   // Z_cam（basis 第 2 列 = −fwd）
        const float depth = -pz;
        if (depth <= 1e-6f) return false;
        o.u = cx + fx * px / depth;
        o.v = cy - fy * py / depth;
        o.zInv = 1.0f / depth;
        o.tu = tu;
        o.tv = tv;
        return true;
    };
    // 网格四角：世界点已在构建时固化（SrBakeDepthMeshWorld）→ 这里**只做投影**，不做反投影
    const auto quadToScreen = [&](const glassvideo::SrDepthQuad &q,
                                  glassvideo::SrScreenVert out[4]) -> bool {
        for (int k = 0; k < 4; k++) {
            const SrVec3 P = { q.x[k], q.y[k], q.z[k] };
            if (!worldToVert(P, q.u[k], q.v[k], out[k])) return false;
        }
        return true;
    };

    size_t written = 0;
    std::vector<glassvideo::SrScreenVert> verts;
    const auto rasterLayer = [&](const std::vector<glassvideo::SrDepthQuad> &quads, const uint8_t *tex,
                                 int tw, int th, uint8_t layerId) {
        verts.resize(quads.size() * 4);
        size_t kept = 0;
        for (const auto &q : quads) {
            glassvideo::SrScreenVert v4[4];
            if (!quadToScreen(q, v4)) continue;   // 相机背后（本场景轨道极小，正常不触发）
            std::memcpy(&verts[kept * 4], v4, sizeof(v4));
            kept++;
        }
        if (kept > 0) {
            written += glassvideo::SrRasterizeQuads(verts.data(), kept, kOutW, kOutH, tex, tw, th,
                                                    rgb.data(), zbuf.data(),
                                                    layerMap.empty() ? nullptr : layerMap.data(),
                                                    layerId);
        }
    };

    // ① 远景板（世界常量深度平面；纹理=模糊源帧；UV 口径与离线判据一致：画布像素坐标 / 模糊因子）
    {
        const float bgZ = kDepthBgZFactor * dm.mesh.zFar;
        const SrVec3 corners[4] = {
            { -kDepthPlaneHX, kDepthPlaneHY, -bgZ }, { kDepthPlaneHX, kDepthPlaneHY, -bgZ },
            { kDepthPlaneHX, -kDepthPlaneHY, -bgZ }, { -kDepthPlaneHX, -kDepthPlaneHY, -bgZ },
        };
        // 远景板纹理是"画布 1/16 盒式模糊"，UV（画布像素坐标）要除以模糊因子换算到贴图像素空间
        const float su = static_cast<float>(dm.planeW) / static_cast<float>(kOutW);
        const float sv = static_cast<float>(dm.planeH) / static_cast<float>(kOutH);
        glassvideo::SrScreenVert v4[4];
        bool ok = true;
        for (int k = 0; k < 4 && ok; k++) {
            const float tu = (cx + corners[k].x * (fx / bgZ)) * su;
            const float tv = (cy - corners[k].y * (fy / bgZ)) * sv;
            ok = worldToVert(corners[k], tu, tv, v4[k]);
        }
        if (ok) {
            written += glassvideo::SrRasterizeQuads(v4, 1, kOutW, kOutH, dm.planeRgb.data(),
                                                    dm.planeW, dm.planeH, rgb.data(), zbuf.data(),
                                                    layerMap.empty() ? nullptr : layerMap.data(), 1);
        }
    }
    // ② 前景层（照片网格）→ ③ 背景层（补洞，承接消隐带）；图层号 2=前景 3=背景（仅诊断用）
    rasterLayer(dm.mesh.front, dm.mesh.frontRgb.data(), dm.mesh.w, dm.mesh.h, 2);
    rasterLayer(dm.mesh.bg, dm.mesh.bgRgb.data(), dm.mesh.w, dm.mesh.h, 3);

    if (logLayerCover && !layerMap.empty()) {
        size_t cPlane = 0, cFront = 0, cBg = 0, cNone = 0;
        for (size_t i = 0; i < total; i++) {
            const uint8_t l = layerMap[i];
            if (l == 1) cPlane++;
            else if (l == 2) cFront++;
            else if (l == 3) cBg++;
            else cNone++;
        }
        VG_LOGI("SRCHK layer cover plane=%{public}.2f%% front=%{public}.2f%% bg=%{public}.2f%% none=%{public}.2f%%",
                100.0 * static_cast<double>(cPlane) / static_cast<double>(total),
                100.0 * static_cast<double>(cFront) / static_cast<double>(total),
                100.0 * static_cast<double>(cBg) / static_cast<double>(total),
                100.0 * static_cast<double>(cNone) / static_cast<double>(total));
    }

    return static_cast<int64_t>(total - (written < total ? written : total));
}

// —— 预置帧队列公开入口（头文件声明；状态与实现在上方匿名命名空间内）——
int32_t SpatialReconPresetQueuePush(const uint8_t *rgba, size_t bytes)
{
    return SrPresetQueuePushImpl(rgba, bytes);
}

int32_t SpatialReconPresetQueueFail()
{
    return SrPresetQueueFailImpl();
}

void SpatialReconPresetQueueReset()
{
    SrPresetQueueReset();
}

int32_t SpatialReconTestRun(const SpatialReconTestParams &params, std::string &outModelPath)
{
    if (g_srBusy.exchange(true)) {
        // 已有任务在跑：本调用立即返回（异步 Promise 即刻结算）——预置生产者的重试循环经
        // "生成已结算"感知自然退出，无需（也不得）在此处置消费端离开标志，否则会毒化在途任务
        return -100;
    }
    g_srProgress.store(0.0f);
    g_srKitProgress.store(0.0f);
    g_srStage.store(-1);
    g_srCbStatus.store(-1);
    g_srAbort.store(0);
    g_srPaused.store(false);
    g_srInSave.store(0);
    // 预置队列的清队在 napi 入口（JS 线程）已完成（SpatialReconPresetQueueReset，
    // 见其注释里的竞态说明）——运行线程不再重复清队，避免吞掉 JS 先行灌入的帧
    int32_t result = -1;
    HMS_SpatialRecon_Session *session = nullptr;

    do {
        SrBindApi();
        if (!g_srApi.ok) {
            result = -1;
            break;
        }
        // 预置建模（地球等预置数据集）：帧来自 ArkTS 灌入的预置帧队列、位姿由 presetTheta 闭环
        // 给出；源图/场景/深度/蒙版全部旁路（Service 侧不传）。帧数以 theta 数组为准（≤240 同 Kit）。
        const bool presetMode = params.preset && params.presetTheta.size() >= 2;
        // v5 参数域有效性（值域 [-89, 89]；哨兵 kPresetPitchUnused 在值域外 = 走轴式域）
        const bool presetPitchValid = params.presetPitchDeg >= -89.0f && params.presetPitchDeg <= 89.0f;
        if (presetMode && (params.presetTheta.size() > 240
            || !(params.presetRadius > 1.0f))) {
            VG_LOGE("SRCHK invalid preset theta=%{public}zu radius=%{public}f (need 240>=n>=2, radius>1)",
                    params.presetTheta.size(), params.presetRadius);
            result = -2;
            break;
        }
        // 参数域互斥防呆：位姿公式域由数据集 manifest 二选一（v3=camAxis 轴式 / v5=camPitchDeg
        // 平环+俯仰），同传 = 代码/数据配错，直接拒（防"错配组合"静默跑出不可解释产物）
        if (presetMode && presetPitchValid && !params.presetAxis.empty()) {
            VG_LOGE("SRCHK invalid preset: both axis(%{public}zu floats) and pitch(%{public}f deg) domains set",
                    params.presetAxis.size(), params.presetPitchDeg);
            result = -2;
            break;
        }
        // 对角弧域与另两域互斥（2026-10-09 新增域；同传 = 数据/代码配错，直接拒）
        if (presetMode && params.presetDiagArc && (presetPitchValid || !params.presetAxis.empty())) {
            VG_LOGE("SRCHK invalid preset: diagArc set together with pitch(%{public}f deg)/axis(%{public}zu floats)",
                    params.presetPitchDeg, params.presetAxis.size());
            result = -2;
            break;
        }
        if (!presetMode && (params.srcW <= 0 || params.srcH <= 0
            || params.rgba.size() != static_cast<size_t>(params.srcW) * params.srcH * 4)) {
            VG_LOGE("SRCHK invalid src %{public}dx%{public}d buf=%{public}zu",
                    params.srcW, params.srcH, params.rgba.size());
            result = -2;
            break;
        }
        int32_t frameCount = presetMode ? static_cast<int32_t>(params.presetTheta.size())
                                        : params.frameCount;
        if (frameCount < 2) frameCount = 2;
        if (frameCount > 240) frameCount = 240;
        float orbitDeg = params.orbitDeg;
        if (orbitDeg < 1.0f) orbitDeg = 1.0f;
        if (orbitDeg > 45.0f) orbitDeg = 45.0f;
        float fx = params.focalPx;
        if (fx < 200.0f) fx = 200.0f;
        if (fx > 6000.0f) fx = 6000.0f;
        // 各向异性内参（2026-10-09，**仅预置路线**）：Kit 的 BA 无锁内参，refined 纵横比是内容相关
        // 漂移（模型纵横比 ≈ B·ρ_d/ρ_r，见 docs/3d-wallpaper-archive.md）。帧保持各向同性（ρ_r=1）时，把声明按
        // ρ_d = 1/B 反向偏置即可把模型纵横比拉回 1。照片路线一律各向同性（ratio 恒 1）。
        float yRatio = 1.0f;
        if (presetMode) {
            yRatio = params.focalYratio;
            if (!(yRatio >= 0.3f && yRatio <= 3.0f)) yRatio = 1.0f;   // 越界/NaN 按 1.0（防呆）
        }
        const float fy = fx * yRatio;
        const float cx = kOutW * 0.5f;
        const float cy = kOutH * 0.5f;
        int32_t scene = (params.scene >= 0 && params.scene <= 5) ? params.scene : 1;
        const int32_t poseFamily = (params.poseFamily == 1) ? 1 : 0;
        // 诊断：源图缩略图（验证 ArkTS→native 的 RGBA 交接是否正确）
        // ⚠️ 该缩略图必须用**自然缓冲**——交换（若开）
        // 发生在本行之后，缩略图语义不受 swapRgbAtInput 影响。
        // 预置模式：无源图，srcRmB 推迟到首帧灌入后回填（见推帧循环）。
        float srcRmB = 0.0f;
        if (!presetMode) {
            SrLogAscii("src", params.rgba.data(), params.srcW, params.srcH, 4);
            // 通道序探针（坑 132 复核）：自然源缓冲的 mean(R−B)——必须在 swapRgbAtInput 之前取值；
            // 与推帧后的 frameRB（交换补丁之后）同号比对，判读规则见 SrMeanRmB 注释。
            srcRmB = SrMeanRmB(params.rgba.data(), params.rgba.size(), 4);
            VG_LOGI("SRCHK chprobe srcRB=%{public}f", srcRmB);
        }

        // 诊断 A/B（坑 143 轮 8，默认关）：交换**位置**="深度之后、贴图/网格之前"。
        // 深度估计在 ArkTS 侧先行（吃自然色），这里把源缓冲整体换一次，其后所有消费者
        // （分层资产/深度网格/远景板模糊与压暗/逐帧光栅化）都用交换后的数据；推帧口不再交换。
        // 互斥说明：真值来自 Service（同一尝试档只会置一个）；native 侧再兜一道防呆（见推帧循环）。
        std::vector<uint8_t> rgbaInputSwap;
        const uint8_t *srcRgba = params.rgba.data();
        if (!presetMode && params.swapRgbAtInput) {
            rgbaInputSwap = params.rgba;   // 本地副本，不改动调用方的缓冲
            for (size_t p = 0; p + 2 < rgbaInputSwap.size(); p += 4) {
                const uint8_t t = rgbaInputSwap[p];
                rgbaInputSwap[p] = rgbaInputSwap[p + 2];
                rgbaInputSwap[p + 2] = t;
            }
            srcRgba = rgbaInputSwap.data();
            VG_LOGI("SRCHK input swap applied (RGBA 4ch; after src thumb, before mesh/texture build)");
        }

        // 分层场景（scene=3 主盒面 / scene=4 盒中空间）：构建"主体区域已补洞"的背景层 + 主体层上下文。
        // 蒙版缺失/尺寸不符/退化（无主体或几乎全为主体）时**自动退回 scene=1**（已真机验证的
        // 单层立体场景），不阻断生成——ArkTS 侧同款判断另有一道（状态行文案用）。
        SrLayerAssets layerAssets;
        SrLayerCtx layerCtx;
        const SrLayerCtx *layers = nullptr;
        if (scene == 3 || scene == 4) {
            // scene=3/4 的照片平面都在 z=-kFaceZ（scene=4 只是把主盒正面放大到铺满画幅），
            // 所以两者的"主体层深度比"语义相同，都由参数给。
            float kSubj = params.subjectDepth;
            if (!(kSubj >= 0.60f && kSubj <= 0.95f)) {   // 含 NaN 防御：非法值回默认
                kSubj = 0.90f;
            }
            const size_t need = static_cast<size_t>(params.srcW) * params.srcH;
            if (params.mask.size() != need) {
                VG_LOGW("SRCHK layered mask size %{public}zu != %{public}zu, fallback scene=1",
                        params.mask.size(), need);
            } else if (SrBuildLayerAssets(srcRgba, params.srcW, params.srcH,
                                          params.mask.data(), layerAssets)) {
                layerCtx.mask = params.mask.data();
                layerCtx.bgRgb = layerAssets.background.data();
                layerCtx.kSubj = kSubj;
                layers = &layerCtx;
                VG_LOGI("SRCHK layered subj=%{public}d ratio=%{public}f kSubj=%{public}f",
                        layerAssets.subjectPx,
                        static_cast<float>(layerAssets.subjectPx) / static_cast<float>(need), kSubj);
            } else {
                VG_LOGW("SRCHK layered assets failed, fallback scene=1");
            }
            if (layers == nullptr) {
                scene = 1;
            }
        }

        // 路线B 深度网格场景（scene=5）：服务层已先跑端侧深度估计（原始相对逆深度随参数传入）。
        // 深度缺失/尺寸不符/构建失败 → 自动退回 scene=1（与既有降级链一致，不阻断出片）；
        // 轨道改用 kDepthThetaDeg（判据界定的口径：±5° → stepNear≈66px ≤ 守卫 80px）。
        SrDepthMeshCtx depthCtx;
        if (scene == 5) {
            if (params.depthW <= 0 || params.depthH <= 0 ||
                params.depth.size() != static_cast<size_t>(params.depthW) * params.depthH) {
                VG_LOGW("SRCHK depth mesh: depth missing (%{public}d x %{public}d, %{public}zu), fallback scene=1",
                        params.depthW, params.depthH, params.depth.size());
                scene = 1;
            } else {
                // D3 自适应纵深（**默认开 = 自适应**，2026-10-04 起；0 = 固定 k=3 = 旧行为）：**d0 恒 2 公式**——zNear=4/(1+k)、
                // zFar=4k/(1+k)，(zNear+zFar)/2 ≡ 2 = kSceneDist：轨道枢轴/相机数学/位移守卫零改动，
                // 只改深度起伏比 zFar/zNear = k。k∈[2.2,3.8]（远景板 7.4×9.6 在 z_bg=2.5·zFar 的
                // 覆盖域内：k=3.8 → z_bg≈7.92，需要半宽 ≈7.92×540/750+轨道余量 ≈6.2 < 7.4 ✓）。
                float zNearEff = kDepthZNear;
                float zFar = kDepthZNear * kDepthRatio;
                if (params.adaptiveK > 0) {
                    const float kAd = glassvideo::SrAdaptiveDepthK(params.depth.data(), params.depthW,
                                                                   params.depthH, 2.2f, 3.8f);
                    zNearEff = 4.0f / (1.0f + kAd);
                    zFar = 4.0f * kAd / (1.0f + kAd);
                    VG_LOGI("SRCHK adaptive k=%{public}.2f (d0=2 -> zNear=%{public}.3f zFar=%{public}.3f)",
                            kAd, zNearEff, zFar);
                }
                // D1/D4（edgeMode）与 D2（subjectAlign+mask）经 opts 传入；默认全关 = 旧行为
                //（坑 147 纪律：这些开关改变喂给 Kit 的帧字节，真机同图 A/B 后才可改默认）。
                glassvideo::SrDepthMeshOpts meshOpts;
                meshOpts.edgeMode = params.depthEdge;
                if (params.subjectAlign > 0 &&
                    params.mask.size() == static_cast<size_t>(params.srcW) * params.srcH) {
                    meshOpts.subjectMask = params.mask.data();
                }
                const int32_t rc = glassvideo::SrBuildDepthMesh(params.depth.data(), params.depthW,
                    params.depthH, srcRgba, params.srcW, params.srcH,
                    zNearEff, zFar, kDepthStride, kDepthSkirt,
                    kOutW, kOutH, depthCtx.mesh, meshOpts);
                if (rc != 0) {
                    VG_LOGW("SRCHK depth mesh build failed rc=%{public}d, fallback scene=1", rc);
                    scene = 1;
                } else {
                    // 世界点一次性固化（参考位姿反投影）——逐帧只做投影；**不得**移进逐帧循环（坑 129）
                    SrBakeDepthMeshWorld(depthCtx.mesh, fx, fy, cx, cy);
                    glassvideo::SrMakeBlurRgb(depthCtx.mesh.frontRgb.data(), depthCtx.mesh.w,
                                              depthCtx.mesh.h, 16, depthCtx.planeRgb,
                                              depthCtx.planeW, depthCtx.planeH);
                    // 远景板压暗（与离线判据一致：×0.72/×0.72/×0.75）。
                    // ⚠️ 2026-10-03 深夜曾试过对称化（统一 0.72，试图消除"交换位置"的字节差异以
                    // 争取"匹配过 + 比例不漂"的盆地）——真机出现新问题，按用户指示**已回退**；
                    // 高饱和素材接受"交换档 + 轻微拉伸"（比例门控拦严重漂移）。
                    // 此系数被两交换位置以不同通道
                    // 语义消费（坑 145 机理），无新证据不得再动。
                    for (size_t p = 0; p + 2 < depthCtx.planeRgb.size(); p += 3) {
                        depthCtx.planeRgb[p] = static_cast<uint8_t>(depthCtx.planeRgb[p] * 0.72f);
                        depthCtx.planeRgb[p + 1] = static_cast<uint8_t>(depthCtx.planeRgb[p + 1] * 0.72f);
                        depthCtx.planeRgb[p + 2] = static_cast<uint8_t>(depthCtx.planeRgb[p + 2] * 0.75f);
                    }
                    depthCtx.valid = true;
                    // 轨道半角 ±Θ：默认 kDepthThetaDeg（判据口径）；诊断参数可覆盖
                    // （真机手测：增大输入视角覆盖，Kit 运镜大角度段减少外推）。
                    orbitDeg = (params.orbitDeg >= 1.0f && params.orbitDeg <= 45.0f)
                        ? params.orbitDeg : kDepthThetaDeg;
                    // Θ 自适应钳制（坑 131 修复配套 + 坑 133 守护深度，2026-10-02）：对角弧
                    //   （yaw+pitch 各 ±Θ）下相邻关键帧总位移 ≈ √2 · coef · fx · Θ_rad，
                    //   coef(z) = 0.5·|z−d0|/z（z=zNear 时 = 0.5，与判据脚本精确投影同口径：
                    //   fx=1500/Θ=5° → 65.4px 复核过）。真机标定带：总位移 ∈ [30, 80]px
                    //   （fx700/Θ2°≈12px 硬失败、Θ3°≈18px 模型碎、Θ5°≈31px 可建）。
                    //   守护深度取 **s 的 p90**（面积自适应，坑 133）：近层面积过小的图（纵深贴
                    //   枢轴平面）在 zNear 处位移达标不代表实际内容达标，按 p90 评估更保守；
                    //   近层充足的图 sP90≈0.9+ → coef≈0.43~0.5 ≈ 旧 zNear 口径，钳制行为不变
                    //   （该守卫为稳健性改进，必要性尚未被真机证伪/证实，见坑文档坑 133"轮 4 修正注记"）。
                    //   带界：守护深度总位移 ∈ [30, 80]px；守护深度端点单轴位移 2·coef·fx·Θ ≤ 裙边
                    //   80px；zNear 处总位移另设 160px 硬顶（近层稀缺时允许其过曝失配，但不失控）。
                    {
                        const float sGuard = depthCtx.mesh.sP90;
                        const float zGuard = 1.0f / (sGuard / zNearEff + (1.0f - sGuard) / zFar);  // 与 zOfS 同式
                        float coef = 0.5f * std::fabs(zGuard - kSceneDist) / std::max(zGuard, 1e-3f);
                        if (coef < 0.05f) {
                            // 极平场景（内容几乎全贴枢轴平面）：防除零；Θ 会被推到上界，仍建不出属预期
                            coef = 0.05f;
                        }
                        const float coefNear = 0.5f * (kSceneDist - zNearEff) / zNearEff;   // k=3 时 = 0.5
                        const float rad2deg = 180.0f / static_cast<float>(M_PI);
                        const float diag = std::sqrt(2.0f) * coef * fx;              // 守护深度位移/弧度
                        const float thLo = kKfStepMinPx / diag * rad2deg;            // 位移下界对应 Θ
                        const float thHiStep = kKfStepMaxPx / diag * rad2deg;        // 位移上界对应 Θ
                        const float thHiSkirt = kDepthSkirt / (2.0f * coef * fx) * rad2deg;   // 裙边界对应 Θ
                        const float thHiNear = 160.0f / (std::sqrt(2.0f) * coefNear * fx) * rad2deg;  // zNear 硬顶
                        const float thHi = std::min(std::min(thHiStep, thHiSkirt), std::min(thHiNear, 45.0f));
                        const float thClamped = std::min(std::max(orbitDeg, thLo), thHi);
                        if (std::fabs(thClamped - orbitDeg) > 1e-3f) {
                            VG_LOGW("SRCHK orbit clamp %{public}.3f -> %{public}.3f deg (fx=%{public}f, "
                                    "guard sP90=%{public}.3f z=%{public}.3f, kf-step band %{public}.0f..%{public}.0fpx, "
                                    "skirt/near caps %{public}.2f/%{public}.2f deg)",
                                    orbitDeg, thClamped, fx, sGuard, zGuard, kKfStepMinPx, kKfStepMaxPx,
                                    thHiSkirt, thHiNear);
                            orbitDeg = thClamped;
                        }
                    }
                    VG_LOGI("SRCHK depth mesh ok depth=%{public}dx%{public}d canvas=%{public}dx%{public}d "
                            "orbit=%{public}f k=%{public}f zNear=%{public}f stride=%{public}d skirt=%{public}d "
                            "front=%{public}zu bg=%{public}zu plane=%{public}dx%{public}d",
                            params.depthW, params.depthH, kOutW, kOutH, orbitDeg, zFar / zNearEff,
                            zNearEff, kDepthStride, kDepthSkirt, depthCtx.mesh.front.size(),
                            depthCtx.mesh.bg.size(), depthCtx.planeW, depthCtx.planeH);
                }
            }
        }

        const HMS_SpatialReconStatus sup = g_srApi.isSupport(SPATIAL_RECON_MODEL_TYPE_GS);
        if (sup != SPATIAL_RECON_STATUS_SUCCESS) {
            result = static_cast<int32_t>(sup); break;
        }
        HMS_SpatialReconStatus st = g_srApi.createSession(SPATIAL_RECON_MODEL_TYPE_GS,
            params.workPath.c_str(), &session);
        if (st != SPATIAL_RECON_STATUS_SUCCESS || session == nullptr) {
            VG_LOGE("SRCHK createSession failed status=%{public}d path=%{public}s",
                    static_cast<int32_t>(st), params.workPath.c_str());
            result = static_cast<int32_t>(st); break;
        }
        VG_LOGI("SRCHK session ok workPath=%{public}s", params.workPath.c_str());
        g_srSession.store(session);

        // 2. 逐帧合成 + 推帧（0.0~0.3）；时间戳 30fps 等间隔 ns
        // 推帧耗时计时（诊断，2026-10-04 预置首测日志缺推帧速率取样）：完成时一行 `SRCHK push done`
        const auto tPush0 = std::chrono::steady_clock::now();
        for (int32_t i = 0; i < frameCount; i++) {
            if (g_srAbort.load()) {
                VG_LOGI("SRCHK abort during push at %{public}d", i);
                result = -6;
                break;
            }
            SrVec3 pos;
            float basis[9];
            if (presetMode && params.presetDiagArc) {
                // 对角弧域（2026-10-09，照片路线坑 131 正解移植；与数据集 poseDomain=diag-arc 配对）
                SrPresetDiagArcBasis(params.presetTheta[i], params.presetRadius, pos, basis);
            } else if (presetMode && presetPitchValid) {
                // v5 域：平环 + 相机俯仰（与 v5 manifest.poseConvention 同式同值，见函数注释）
                SrPresetFlatPitchBasis(params.presetTheta[i], params.presetRadius,
                                       params.presetPitchDeg, pos, basis);
            } else if (presetMode) {
                // 预置建模（v3 域）：绕数据集标定轴 A 整圈环绕（几何/退化分析见 SrPresetRingBasis 注释）
                const float ax[3] = {
                    params.presetAxis.size() >= 3 ? params.presetAxis[0] : 0.0f,
                    params.presetAxis.size() >= 3 ? params.presetAxis[1] : 1.0f,
                    params.presetAxis.size() >= 3 ? params.presetAxis[2] : 0.0f,
                };
                SrPresetRingBasis(params.presetTheta[i], params.presetRadius, ax, pos, basis);
            } else {
                // 对角弧（坑 131）：scene=5 的俯仰半角与偏航同值（Φ=Θ，判据同口径）；
                // scene=0..4 保持纯水平弧（pitch=0，与既有真机验证行为逐位一致）
                SrFrameBasis(i, frameCount, orbitDeg, (scene == 5) ? orbitDeg : 0.0f, pos, basis);
            }
            // 诊断 A/B（默认关，roll180=true 时生效）：世界绕「视轴」滚 180°——
            // 取反渲染基的 X_cam/Y_cam 两列（行列式不变，仍是旋转）：**合成帧与上报位姿同步滚**，
            // 相机位置/朝向不变、射线集合恒等 ⇒ Kit 看到的几何信息完全不变（重建结果等价）。
            // 用途：Kit 的输出视频由它自己的 [VideoGenerator] 按 gravityCoordinate 定向，
            // 我们的合成帧经设备日志逐格比对已证"正立且与源图一致"（fmid ≈ src 98%）——
            // 若输出仍上下颠倒，用本开关做 A/B 定位是"Kit 的定向约定"还是"渲染基准差"。
            if (params.roll180) {
                for (int k = 0; k < 6; k++) {
                    basis[k] = -basis[k];
                }
            }
            // 上报位姿（与渲染基解耦）：世界朝向补偿 A/B（默认关）——
            // 世界绕世界 X 轴转 180°（up→−up、z→−z）：**只转上报的位姿，合成帧不变**。
            // 自洽性：把世界整体旋转 R 后，图像 I_i 与新位姿 (R·C_i, R·B_i) 仍然自洽
            // （重建出的场景 = R·S，与"把 S 也转过去"等价）⇒ 重建必然同样成功、几何等价，
            // 只是 Kit 看到的世界朝向转了 180°。用途：真机实测成片"上下颠倒 + 人物跑到背景
            // 后面"（= 深度镜像）= Kit 的模型导出/输出把模型绕水平轴翻了 180° → 本开关抵消它。
            float rbasis[9];
            std::memcpy(rbasis, basis, sizeof(rbasis));
            SrVec3 rpos = pos;
            if (params.worldFlip180) {
                rbasis[1] = -rbasis[1];
                rbasis[2] = -rbasis[2];
                rbasis[4] = -rbasis[4];
                rbasis[5] = -rbasis[5];
                rbasis[7] = -rbasis[7];
                rbasis[8] = -rbasis[8];
                rpos.y = -rpos.y;
                rpos.z = -rpos.z;
            }
            float q[4];
            SrVec3 posReport;
            SrReportPose(poseFamily, params.invertQuat, rbasis, rpos, q, posReport);

            std::vector<uint8_t> rgb;
            // 坑 145：layer cover 探针在**首帧与末帧**都打——首帧是轨道端点（t=-1）、
            // fmid 才近参考位姿；"远景板压暗不对称（0.72/0.72/0.75 落在对调通道）"若要在
            // 端点露出的裙边/边角起作用，末帧读数才是判据（坑 143 轮 9 只打了 i=0）。
            int64_t missPx = 0;
            if (presetMode) {
                // 预置帧：阻塞取一帧 RGBA（ArkTS 解码灌入），转 RGB 三字节后走与合成帧
                // 完全相同的下游（bgrSwap/签名/推送/门控）。尺寸不符即参数错误，直接判失败。
                std::vector<uint8_t> frameRgba;
                if (SrPresetQueuePop(frameRgba) != 0) {
                    VG_LOGE("SRCHK preset queue pop failed at %{public}d (producer fail/abort)",
                            i);
                    result = -6;
                    break;
                }
                const size_t need = static_cast<size_t>(kOutW) * kOutH * 4;
                if (frameRgba.size() != need) {
                    VG_LOGE("SRCHK preset frame size %{public}zu != %{public}zu at %{public}d",
                            frameRgba.size(), need, i);
                    result = -2;
                    break;
                }
                if (i == 0) {
                    // 预置模式的通道序探针基线：自然帧（交换前）的 mean(R−B)，
                    // 与下方 frameRB 同号比对（判读规则同 SrMeanRmB 注释）
                    srcRmB = SrMeanRmB(frameRgba.data(), frameRgba.size(), 4);
                    VG_LOGI("SRCHK chprobe srcRB=%{public}f (preset f0)", srcRmB);
                }
                rgb.resize(need / 4 * 3);
                for (size_t p = 0, q = 0; p < need; p += 4, q += 3) {
                    rgb[q] = frameRgba[p];
                    rgb[q + 1] = frameRgba[p + 1];
                    rgb[q + 2] = frameRgba[p + 2];
                }
            } else {
                missPx = (scene == 5)
                    ? SrRasterizeDepthMesh(depthCtx, pos, basis, fx, fy, cx, cy, rgb,
                                           i == 0 || i == frameCount - 1)
                    : SrRenderFrame(srcRgba, params.srcW, params.srcH,
                                    pos, basis, fx, fy, cx, cy, scene, layers, rgb);
            }
            // 坑 132（2026-10-02 观察）：不交换则成片红蓝对调（橙色路灯 ↔ 青蓝、蓝色招牌 ↔ 橙红，
            // 用户报告"蓝黄对掉"）；在推帧口统一交换 R/B 使输出颜色正确。成因未定论（档案 F7）
            // ——这里只提供开关与工程兜底，不据此推断归属。ASCII 缩略图用亮度加权不受影响，
            // 内部管线（贴图/深度/补洞）不换。
            // ⚠️ 坑 134（2026-10-02 观察）：交换后高度饱和素材（动漫图）重建匹配会失败（muted
            // 素材不受影响）——Service 层失败时自动反交换重试一次；bgrSwap=false 可手动回到直通 RGB。
            // ⚠️ 与 swapRgbAtInput 互斥（坑 143 轮 8）：输入侧已整体换过时这里必须置 false，
            // 否则两次交换互相抵消。Service 已保证互斥，这里再兜一道防呆并留日志。
            if (params.bgrSwap && params.swapRgbAtInput) {
                VG_LOGW("SRCHK bgrSwap and swapRgbAtInput both set; push-point swap skipped (mutually exclusive)");
            } else if (params.bgrSwap) {
                for (size_t p = 0; p + 2 < rgb.size(); p += 3) {
                    const uint8_t tmpR = rgb[p];
                    rgb[p] = rgb[p + 2];
                    rgb[p + 2] = tmpR;
                }
            }
            // 可选：补 Alpha 通道成 RGBA（4 字节/像素）——备选输入布局假设，默认关
            std::vector<uint8_t> rgba4;
            if (params.rgbaInput) {
                rgba4.resize(static_cast<size_t>(kOutW) * kOutH * 4);
                for (size_t p = 0, q4 = 0; q4 < rgba4.size(); p += 3, q4 += 4) {
                    rgba4[q4] = rgb[p];
                    rgba4[q4 + 1] = rgb[p + 1];
                    rgba4[q4 + 2] = rgb[p + 2];
                    rgba4[q4 + 3] = 255;
                }
            }
            const uint8_t *frameData = params.rgbaInput ? rgba4.data() : rgb.data();

            // 帧指纹（诊断，坑 143 轮 9）：对**最终推给 Kit 的字节**做 6×8 分块 FNV-1a（每块取低 16 位）。
            // 为什么需要它：稀疏 ASCII 缩略图只能证明"看起来一样"，无法排除"某些区域字节不同"——
            // 而"交换位置是否等价""同一配置是否稳定复现"这类问题必须做**字节级**对账。分块粒度还能
            // 直接指出差异集中在哪几块（例如"远景板压暗系数 0.72/0.72/0.75 落在对调通道上"只应影响
            // 边缘/裙边那几块）。对比方法：同图同参数两次运行，逐块比 4 位十六进制串。
            if (i == 0 || i == frameCount / 2) {
                constexpr int kTX = 6;
                constexpr int kTY = 8;
                std::string sig;
                for (int ty = 0; ty < kTY; ty++) {
                    for (int tx = 0; tx < kTX; tx++) {
                        const int x0 = kOutW * tx / kTX;
                        const int x1 = kOutW * (tx + 1) / kTX;
                        const int y0 = kOutH * ty / kTY;
                        const int y1 = kOutH * (ty + 1) / kTY;
                        uint32_t h = 2166136261u;
                        for (int y = y0; y < y1; y += 3) {   // 行方向 1/3 抽样：够指纹、省时间
                            for (int x = x0; x < x1; x++) {
                                const size_t o = (static_cast<size_t>(y) * kOutW + x) * 3;
                                h = (h ^ rgb[o]) * 16777619u;
                                h = (h ^ rgb[o + 1]) * 16777619u;
                                h = (h ^ rgb[o + 2]) * 16777619u;
                            }
                        }
                        char cell[5];
                        static const char *kHex = "0123456789abcdef";
                        const uint32_t v = (h >> 16) & 0xffffu;
                        cell[0] = kHex[(v >> 12) & 0xfu];
                        cell[1] = kHex[(v >> 8) & 0xfu];
                        cell[2] = kHex[(v >> 4) & 0xfu];
                        cell[3] = kHex[v & 0xfu];
                        cell[4] = '\0';
                        sig += cell;
                    }
                }
                VG_LOGI("SRCHK frame sig i=%{public}d bgrSwap=%{public}d inpSwap=%{public}d %{public}s",
                        i, params.bgrSwap ? 1 : 0, params.swapRgbAtInput ? 1 : 0, sig.c_str());
                // 通道序探针（坑 132 复核）：最终推给 Kit 的字节的 mean(R−B)（交换补丁之后、
                // 按实际推送布局 frameData 取值）。与 srcRB 同号比对的判读规则见 SrMeanRmB 注释。
                const float frameRmB = SrMeanRmB(frameData,
                    params.rgbaInput ? rgba4.size() : rgb.size(), params.rgbaInput ? 4 : 3);
                VG_LOGI("SRCHK chprobe i=%{public}d frameRB=%{public}f srcRB=%{public}f bgrSwap=%{public}d inpSwap=%{public}d",
                        i, frameRmB, srcRmB, params.bgrSwap ? 1 : 0, params.swapRgbAtInput ? 1 : 0);
            }

            HMS_SpatialRecon_DataFrame frame{};
            frame.focalX = fx;
            frame.focalY = fy;
            frame.principalX = cx;
            frame.principalY = cy;
            // distortionCoef[8] 已零初始化：合成相机无畸变
            frame.imageWidth = kOutW;
            frame.imageHeight = kOutH;
            frame.position[0] = posReport.x;
            frame.position[1] = posReport.y;
            frame.position[2] = posReport.z;
            frame.rotation[0] = q[0];
            frame.rotation[1] = q[1];
            frame.rotation[2] = q[2];
            frame.rotation[3] = q[3];
            frame.timestamp = static_cast<int64_t>(i) * 33333333LL;
            frame.imageData = const_cast<uint8_t *>(frameData);
            frame.format = SPATIAL_RECON_IMAGEDATA_FORMAT_RGB;

            const HMS_SpatialReconStatus ps = g_srApi.pushFrame(session, &frame);
            if (ps != SPATIAL_RECON_STATUS_SUCCESS) {
                VG_LOGE("SRCHK pushFrame %{public}d failed status=%{public}d", i, static_cast<int32_t>(ps));
                result = static_cast<int32_t>(ps); break;
            }
            if (i % 12 == 0) {
                // bgrSwap 进推帧行：本轮会话的通道交换在**这里**（推帧口，逐帧、Kit 阶段之前）完成，
                // 日志对账时不必再翻到后面的 `SRCHK building` 行（坑 143）
                VG_LOGI("SRCHK push %{public}d/%{public}d bgrSwap=%{public}d pos=(%{public}f,%{public}f,%{public}f) quat=(%{public}f,%{public}f,%{public}f,%{public}f) miss=%{public}d",
                        i, frameCount, params.bgrSwap ? 1 : 0, posReport.x, posReport.y, posReport.z,
                        q[0], q[1], q[2], q[3], static_cast<int32_t>(missPx));
            }
            // 诊断：首帧与中帧的缩略图（人工核验合成帧内容/朝向/亮度）
            if (i == 0) {
                SrLogAscii("f000", rgb.data(), kOutW, kOutH, 3);
            } else if (i == frameCount / 2) {
                SrLogAscii("fmid", rgb.data(), kOutW, kOutH, 3);
            }
            // 空帧自检：本管线场景（主盒+副盒+远景板）设计上应近乎满覆盖；
            // 首帧过半无命中 = 射线/场景配置错（历史坑：射线朝后 → 整帧底色 → 重建必报 no 3D points）
            if (i == 0 && missPx * 2 > static_cast<int64_t>(kOutW) * kOutH) {
                VG_LOGE("SRCHK frame0 blank miss=%{public}d/%{public}d (ray/scene misconfigured)",
                        static_cast<int32_t>(missPx), kOutW * kOutH);
                result = -5;
                break;
            }
            g_srProgress.store(0.3f * (i + 1) / frameCount);
        }
        if (result == -1) {
            const int64_t pushMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - tPush0).count();
            VG_LOGI("SRCHK push done frames=%{public}d elapsed=%{public}dms preset=%{public}d fps=%{public}.2f",
                    frameCount, static_cast<int32_t>(pushMs), presetMode ? 1 : 0,
                    pushMs > 0 ? 1000.0 * frameCount / static_cast<double>(pushMs) : 0.0);
        }
        if (result != -1) {
            // 推帧阶段已带错误码退出（result 离开初始值 -1 且非 0）
            break;
        }

        // 3. 启动重建，随后按前台模式分配资源。常规档：writeInfo 非空（重建完成自动保存 MP4）；
        // 试验档（composeTrial，构图搜索用，2026-10-04）：writeInfo 传空——官方明文"writeInfo
        // 为空时可在重建结束后手动保存结果"，让比例门控提前到**保存之前**（省 STAGE04 编码
        // 44~76s/次），不合格的构图候选直接销毁换档、不留文件。
        HMS_SpatialRecon_ModelWriteInfo info{};
        info.modelFormat = SPATIAL_RECON_OUTPUT_FORMAT_MP4;
        info.modelFile = params.modelPath.c_str();
        info.audioFile = nullptr;
        info.longitude = 0.0f;
        info.latitude = 0.0f;
        g_srCbStatus.store(-1);
        st = g_srApi.startSession(session, params.composeTrial ? nullptr : &info, &SrOnFinished);
        if (st != SPATIAL_RECON_STATUS_SUCCESS) {
            VG_LOGE("SRCHK startSession failed status=%{public}d", static_cast<int32_t>(st));
            result = static_cast<int32_t>(st);
            break;
        }
        g_srApi.setRunningMode(session, g_srWantForeground.load() != 0
            ? SPATIAL_RECON_RUNNING_FOREGROUND_MODE : SPATIAL_RECON_RUNNING_BACKGROUND_MODE);
        // ⚠️ 2026-10-09 修复：本行的 `pitch=`/`focal=` 曾**缺参数**（56aea29 加了 poseDomain 却删掉
        // 这两个浮点实参），导致 printf 读未初始化寄存器——真机日志会印成
        // `pitch=2.35 focal=-5 radius=5`（错位），对账时极易误判。现补齐（pitch = 照片路线
        // scene=5 的 Φ，预置路线恒 0）并在末尾补 focalYratio 供各向异性声明对账。
        VG_LOGI("SRCHK building src=%{public}dx%{public}d frames=%{public}d orbit=%{public}f pitch=%{public}f focal=%{public}f scene=%{public}d family=%{public}d invQuat=%{public}d bgrSwap=%{public}d inpSwap=%{public}d trial=%{public}d preset=%{public}d radius=%{public}f theta0=%{public}f thetaN=%{public}f yRatio=%{public}f poseDomain=%{public}s",
                params.srcW, params.srcH, frameCount, orbitDeg,
                (scene == 5) ? orbitDeg : 0.0f, fx, scene,
                poseFamily, params.invertQuat ? 1 : 0, params.bgrSwap ? 1 : 0,
                params.swapRgbAtInput ? 1 : 0, params.composeTrial ? 1 : 0,
                presetMode ? 1 : 0, params.presetRadius,
                presetMode ? params.presetTheta.front() : 0.0f,
                presetMode ? params.presetTheta.back() : 0.0f,
                yRatio,
                // 位姿域判读用（真机对账第一眼就该看到是哪一域）
                (!presetMode ? "photo-diagArc" : (params.presetDiagArc ? "preset-diagArc"
                    : (presetPitchValid ? "preset-flatPitch" : "preset-ringAxis"))));

        // 4. 等待完成回调（上限 30 分钟），期间镜像 Kit 进度。
        // 进度映射（全程单调）：合成推帧 0.00~0.30 → 重建 0.30~0.90 → 保存 0.90~0.99 → 校验通过 1.00
        constexpr int32_t kMaxWaitMs = 30 * 60 * 1000;
        // 保存阶段挂死判据（坑 145，2026-10-03 真机实锤）：Kit 的保存编码器可能中途停滞——
        // 进度恒 0.93、onFinished 回调永不到达、用户只能杀后台脱困（当轮无 session destroyed 日志）。
        // 正常保存全程远短于 3 分钟（真机 onFinished 前 ENCODER 300 帧 Packing 仅数秒~十几秒），
        // 故 SAVING 域内连续 kSaveStallMs 无任何进展即判挂死：置 -7 走统一销毁退出，
        // 上层尝试序列自动换档重试。⚠️ 挂死中的保存无法正常收尾，销毁仍属"在途销毁"
        // （官方未定义行为）——但比起把 UI 永久钉在 93%，这是更可辩护的退出方式，且留全量日志。
        constexpr int32_t kSaveStallMs = 180 * 1000;
        int32_t waitedMs = 0;
        float lastKitK = 0.0f;          // 本域内上一次 Kit 进度（换域时归零）
        bool savingDomain = false;      // 是否已切到"保存进度"域
        int32_t noAdvanceMs = 0;        // Kit 连续无进展时长（兜底推进用）
        int32_t saveStallMs = 0;        // SAVING 域内连续无进展时长（挂死检测，坑 145）
        float estBase = g_srProgress.load();
        int32_t estBaseMs = 0;
        bool estLogged = false;
        // 阶段变化日志（诊断，2026-10-04 预置首测日志无法分辨慢在重建还是保存）：每次
        // HMS 阶段值变化打一行，长任务日志据它可判瓶颈阶段（-2 = 尚未读到过阶段）
        int32_t lastLoggedStage = -2;
        while (g_srCbStatus.load() < 0 && waitedMs < kMaxWaitMs) {
            if (g_srAbort.load()) {
                VG_LOGI("SRCHK abort during training at %{public}dms", waitedMs);
                result = -6;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            waitedMs += 200;
            float kitProgress = 0.0f;
            HMS_SpatialReconStage stage = SPATIAL_RECON_STAGE_INIT;
            bool advanced = false;
            if (g_srApi.getProgress(session, &kitProgress, &stage) == SPATIAL_RECON_STATUS_SUCCESS) {
                g_srStage.store(static_cast<int32_t>(stage));
                if (static_cast<int32_t>(stage) != lastLoggedStage) {
                    lastLoggedStage = static_cast<int32_t>(stage);
                    VG_LOGI("SRCHK stage -> %{public}d at kit=%{public}.3f t=%{public}dms preset=%{public}d",
                            lastLoggedStage, kitProgress, waitedMs, presetMode ? 1 : 0);
                }
                if (stage == SPATIAL_RECON_STAGE_SAVING) {
                    // 保存域：官方"重建完成后进度只反映保存"，是另一个 0~1 域 → 映射到 0.90~0.99
                    if (!savingDomain) {
                        savingDomain = true;
                        lastKitK = 0.0f;
                    }
                    if (kitProgress > lastKitK) {
                        lastKitK = kitProgress;
                        const float v = 0.90f + 0.09f * kitProgress;
                        if (v > g_srProgress.load()) { g_srProgress.store(v); }
                        advanced = true;
                    }
                } else if (kitProgress > lastKitK) {
                    // ⚠️ 与"上一次的 Kit 值"比（不是与合成后的总值比，见坑 140）
                    lastKitK = kitProgress;
                    const float v = 0.30f + 0.60f * kitProgress;
                    if (v > g_srProgress.load()) { g_srProgress.store(v); }
                    advanced = true;
                }
            }
            if (advanced) {
                noAdvanceMs = 0;
                saveStallMs = 0;
                estBase = g_srProgress.load();
                estBaseMs = waitedMs;
                continue;
            }
            noAdvanceMs += 200;
            // 保存挂死检测（坑 145）：SAVING 域内无进展持续累积；越界即判编码器挂死
            if (stage == SPATIAL_RECON_STAGE_SAVING) {
                saveStallMs += 200;
                if (saveStallMs >= kSaveStallMs) {
                    VG_LOGE("SRCHK save stalled %{public}dms at kit=%{public}.3f (encoder hang), abort attempt",
                            saveStallMs, lastKitK);
                    result = -7;
                    break;
                }
            }
            // 兜底推进（有界时间估计）：部分机型/版本在重建期 getProgress 成功但进度恒 0（或调用失败），
            // 一条 45~80s 静止的进度条无法让用户判断死活。仅在"重建阶段且连续 4s 无任何进展"时启用，
            // 渐近逼近 0.88 **永不超过真实的 0.90 档**（真进度一出现立即压过它），且只打一次日志便于对账。
            if (stage == SPATIAL_RECON_STAGE_BUILDING && noAdvanceMs >= 4000) {
                const float t = static_cast<float>(waitedMs - estBaseMs) / 45000.0f;   // 时间常数 ≈45s
                const float est = estBase + (0.88f - estBase) * (1.0f - std::exp(-t));
                if (est > g_srProgress.load()) {
                    g_srProgress.store(est);
                }
                if (!estLogged) {
                    estLogged = true;
                    VG_LOGI("SRCHK progress est fallback (kit reports no progress) base=%{public}.3f",
                            estBase);
                }
            }
        }
        if (result == -6) {
            // 用户终止（Pause 已请求）：走下面的落定 + 销毁，不再判回调
            break;
        }
        const int32_t cb = g_srCbStatus.load();
        if (cb < 0) {
            VG_LOGE("SRCHK timeout wait %{public}dms", waitedMs);
            result = -3;
            break;
        }
        if (cb != SPATIAL_RECON_STATUS_SUCCESS) {
            VG_LOGE("SRCHK rebuild failed cb=%{public}d", cb);
            result = cb;
            break;
        }

        // 5. 保存收尾与比例门控（2026-10-04 构图搜索起两档路径分岔）：
        //    常规档（composeTrial=false）：StartSession 已带 writeInfo，重建完成自动保存——等保存
        //    阶段收尾即可（回调若先于保存完成触发，销毁会话会撞上在途保存=未定义行为）。
        float gateAspect = 0.0f;   // 门控采用的 refined fy/fx（pre 或 post 窗口，只采一次）
        bool gateHave = false;
        g_srInSave.store(1);   // 保存域开始（UI 据此禁用暂停、并把终止的后果讲清楚）
        if (params.composeTrial) {
            // 试验档：writeInfo 为空启动，此刻未保存——先在 FINISHED 窗口读 refined 做早期门控
            //（前置实验点：该窗口可读性官方未规定；读不到自动回退保存后门控，语义不变），
            // 合格才手动 SaveResultToFile（官方明文通路）并等收尾。价值：比例不合格的构图候选
            // 不进 44~76s 的 STAGE04 编码，直接 -8 换档、不留文件。
            gateHave = SrProbeRefinedAspect(session, frameCount, "pre", gateAspect);
            VG_LOGI("SRCHK trial refined pre-save %{public}s",
                    gateHave ? "readable" : "unreadable (fallback to post-save gate)");
            if (gateHave && std::fabs(gateAspect - 1.0f) > kAspectDriftLimit) {
                VG_LOGW("SRCHK aspect gate rejected (trial pre-save): fy/fx=%{public}.3f —— "
                        "不保存直接换档（省编码耗时，不留文件）", gateAspect);
                result = -8;
                break;
            }
            if (g_srApi.saveResultToFile == nullptr) {
                VG_LOGE("SRCHK trial: SaveResultToFile symbol unavailable (old ROM)");
                result = -9;
                break;
            }
            const HMS_SpatialReconStatus mst = g_srApi.saveResultToFile(session, &info, nullptr);
            if (mst != SPATIAL_RECON_STATUS_SUCCESS) {
                VG_LOGE("SRCHK trial save start failed status=%{public}d", static_cast<int32_t>(mst));
                result = static_cast<int32_t>(mst);
                break;
            }
            // 手动保存收尾等待：观察到 SAVING 后离开 = 完成；刚启动时短暂停在 FINISHED 属正常，
            // 停留时钟与 SAVING 内无进展共用挂死判据（kSaveStallMs，坑 145 同款 → -7）。
            // 进度映射与自动保存同口径（保存域 0~1 → 0.90~0.99，单调不回退）。
            int32_t tWaitMs = 0;
            int32_t tStallMs = 0;
            float tLastK = 0.0f;
            bool tEntered = false;
            while (tWaitMs < kMaxWaitMs) {
                if (g_srAbort.load()) {
                    VG_LOGI("SRCHK abort during trial save at %{public}dms", tWaitMs);
                    result = -6;
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                tWaitMs += 200;
                tStallMs += 200;
                float kp = 0.0f;
                HMS_SpatialReconStage tst = SPATIAL_RECON_STAGE_UNKNOWN;
                if (g_srApi.getProgress(session, &kp, &tst) != SPATIAL_RECON_STATUS_SUCCESS) {
                    break;   // 查询失败视作已收尾（与 alsoPly 等待同款出口）
                }
                g_srStage.store(static_cast<int32_t>(tst));
                if (tst == SPATIAL_RECON_STAGE_SAVING) {
                    if (!tEntered) {
                        tEntered = true;
                        tStallMs = 0;
                    }
                    if (kp > tLastK) {
                        tLastK = kp;
                        tStallMs = 0;
                        const float v = 0.90f + 0.09f * kp;
                        if (v > g_srProgress.load()) {
                            g_srProgress.store(v);
                        }
                    }
                    continue;
                }
                if (tEntered) {
                    break;   // 进过 SAVING 后离开 = 保存完成
                }
                if (tStallMs >= kSaveStallMs) {
                    VG_LOGE("SRCHK trial save stalled %{public}dms (never entered SAVING / encoder hang)",
                            tStallMs);
                    result = -7;
                    break;
                }
            }
            // ⚠️ 只对本循环实际设置的失败码 break（坑 151：result 初始值是 -1，"非零即败"
            // 会把未设置过 result 的正常出口——进过 SAVING 后离开/查询失败/超时——当成
            // 失败上抛；正常出口落到下面的公共文件校验（-4 兜底）与成功路径）。
            if (result == -6 || result == -7) {
                break;
            }
        } else {
            // 常规档：等自动保存收尾
            int32_t saveWaitMs = 0;
            float lastSaveK = 0.0f;
            int32_t saveStallMs2 = 0;   // 保存挂死检测（坑 145，同上 kSaveStallMs）
            HMS_SpatialReconStage stage = SPATIAL_RECON_STAGE_UNKNOWN;
            while (saveWaitMs < kMaxWaitMs) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                saveWaitMs += 200;
                stage = SPATIAL_RECON_STAGE_UNKNOWN;
                float kitProgress = 0.0f;
                if (g_srApi.getProgress(session, &kitProgress, &stage) != SPATIAL_RECON_STATUS_SUCCESS) {
                    break;
                }
                g_srStage.store(static_cast<int32_t>(stage));
                // 保存域（此处的比例是"保存任务"自己的 0~1）→ 映射到 0.90~0.99，单调不回退
                if (kitProgress > lastSaveK) {
                    lastSaveK = kitProgress;
                    saveStallMs2 = 0;
                    const float v = 0.90f + 0.09f * kitProgress;
                    if (v > g_srProgress.load()) {
                        g_srProgress.store(v);
                    }
                } else if (stage == SPATIAL_RECON_STAGE_SAVING) {
                    saveStallMs2 += 200;
                    if (saveStallMs2 >= kSaveStallMs) {
                        VG_LOGE("SRCHK save stalled %{public}dms at kit=%{public}.3f after callback (encoder hang), abort attempt",
                                saveStallMs2, lastSaveK);
                        result = -7;
                        break;
                    }
                }
                if (stage != SPATIAL_RECON_STAGE_SAVING) {
                    break;
                }
            }
        }
        g_srInSave.store(0);   // 保存域结束
        // 终止请求在保存期生效 = 放弃本次结果（-6）：跳过产物校验直接走销毁收尾
        //（否则下面的"文件不存在 → -4"会把 -6 覆盖，用户看到的是"失败"而不是"已终止"）
        if (result == -6) {
            break;
        }
        // 产物校验（会话销毁前确认文件已落盘；试验档在手动保存收尾后同样适用）
        if (::access(params.modelPath.c_str(), F_OK) != 0) {
            VG_LOGE("SRCHK model file missing %{public}s", params.modelPath.c_str());
            result = -4;
            break;
        }
        // refined 比例门控（坑 131 定案 → 坑 145 交付门控；探针实现见 SrProbeRefinedAspect）：
        // 我们上报 fx=fy（方像素合成相机），Kit 的 BA 若把 fy/fx 精修到远离 1，即"纵向尺度/fy
        // 退化方向"漂移——refined fy/fx 直接等于重建模型的横向/纵向尺度比（>1 拉伸、<1 压缩），
        // 绝对值无意义（BA 整体尺度规约自由）。试验档已在保存前门控过（pre 窗口）的不再重复；
        // 常规档 / 试验档 pre 窗口不可读时在保存后取读数。|aspect−1| > kAspectDriftLimit（0.25，
        // 严重漂移）→ 本档判失败（-8）上层换档重试；轻度漂移（≤0.25）仍交付（高饱和素材
        // "交换档通过 + 轻微拉伸"优于"自然档颜色对调"）。探不到 refined（老 ROM/非关键帧
        // 索引失败）不拦截——门控只在"有读数且读数严重超标"时动作。
        // ⚠️ preset 模式例外（2026-10-05 用户决策）：比例门控是我们自己的检测，曾有轮次
        // STAGE01~04 全过、保存完成，却被它拦下；预置路线需要保留**成片形态**这一手证据
        // ——refined 读数照打、产物照常交付，不再 -8。
        if (!gateHave) {
            gateHave = SrProbeRefinedAspect(session, frameCount, "post", gateAspect);
        }
        if (gateHave && std::fabs(gateAspect - 1.0f) > kAspectDriftLimit) {
            if (presetMode) {
                VG_LOGW("SRCHK aspect drift tolerated (preset mode): refined fy/fx=%{public}.3f "
                        "beyond +/-%{public}.3f —— 照常交付（预置路线不拦截，读数见上）",
                        gateAspect, kAspectDriftLimit);
            } else {
                VG_LOGW("SRCHK aspect gate rejected: refined fy/fx=%{public}.3f beyond +/-%{public}.3f —— "
                        "模型纵横比严重漂移，本档判失败换档重试（产物已落盘，可兜底交付）",
                        gateAspect, kAspectDriftLimit);
                result = -8;
                break;
            }
        }
        // 同会话双产物（坑 146 后续）：重建完成且过门控后，用官方 SaveResultToFile 追加保存
        // 一份 PLY（modelPath 的 .mp4 换 .ply）。动机：spatialEdit.saveToPLY 对"MP4 容器加载
        // 的节点"真机恒失败（`setPromise failed same model`，两轮修复无效）；而 PLY 是模型几何/
        // 颜色的唯一可解剖产物（模型侧离线诊断输入）。⚠️ 失败不致命：只打日志，MP4 照常交付。
        // 语义依据（官方《重建三维场景》）：writeInfo 为空时可在重建结束后手动 SaveResultToFile；
        // 本流程 writeInfo 非空（已自动保存 MP4），在保存完成后追加第二次保存（PLY）——同会话
        // 顺序保存，不违反"同一时刻仅一个保存任务"约束。完成判定：阶段离开 SAVING 且文件存在
        // （不依赖回调——旧式回调与 StartSession 的回调同槽，覆盖有风险）。
        if (params.alsoPly && g_srApi.saveResultToFile != nullptr) {
            std::string plyPath = params.modelPath;
            const std::string ext = ".mp4";
            if (plyPath.size() > ext.size() &&
                plyPath.compare(plyPath.size() - ext.size(), ext.size(), ext) == 0) {
                plyPath.replace(plyPath.size() - ext.size(), ext.size(), ".ply");
            } else {
                plyPath += ".ply";
            }
            HMS_SpatialRecon_ModelWriteInfo plyInfo{};
            plyInfo.modelFormat = SPATIAL_RECON_OUTPUT_FORMAT_PLY;
            plyInfo.modelFile = plyPath.c_str();
            plyInfo.audioFile = nullptr;
            plyInfo.longitude = 0.0f;
            plyInfo.latitude = 0.0f;
            const HMS_SpatialReconStatus ps =
                g_srApi.saveResultToFile(session, &plyInfo, nullptr);
            if (ps != SPATIAL_RECON_STATUS_SUCCESS) {
                VG_LOGW("SRCHK ply save start failed status=%{public}d (non-fatal)", static_cast<int32_t>(ps));
            } else {
                int32_t plyWaitMs = 0;
                int32_t plyStallMs = 0;
                bool plySaving = false;
                while (plyWaitMs < kMaxWaitMs) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    plyWaitMs += 200;
                    float kp = 0.0f;
                    HMS_SpatialReconStage pst = SPATIAL_RECON_STAGE_UNKNOWN;
                    if (g_srApi.getProgress(session, &kp, &pst) != SPATIAL_RECON_STATUS_SUCCESS) {
                        break;
                    }
                    g_srStage.store(static_cast<int32_t>(pst));
                    if (pst == SPATIAL_RECON_STAGE_SAVING) {
                        plySaving = true;
                        plyStallMs += 200;   // PLY 无进度上报，用停留时长近似（保存应远短于 3 分钟）
                        if (plyStallMs >= kSaveStallMs) {
                            VG_LOGW("SRCHK ply save stalled %{public}dms (non-fatal)", plyStallMs);
                            break;
                        }
                        continue;
                    }
                    if (plySaving || pst == SPATIAL_RECON_STAGE_FINISHED) {
                        break;   // 曾进入 SAVING 后离开，或直接回到 FINISHED
                    }
                }
                if (::access(plyPath.c_str(), F_OK) == 0) {
                    VG_LOGI("SRCHK ply saved %{public}s (%{public}dms)", plyPath.c_str(), plyWaitMs);
                } else {
                    VG_LOGW("SRCHK ply save not on disk after %{public}dms (non-fatal)", plyWaitMs);
                }
            }
        }
        g_srProgress.store(1.0f);
        outModelPath = params.modelPath;
        result = 0;
        VG_LOGI("SRCHK done %{public}s", params.modelPath.c_str());
    } while (false);

    // 预置队列消费端离开（所有退出路径都经此）：唤醒可能还阻塞在灌帧的 JS 线程并清掉余帧
    SrPresetQueueLeave();

    // 会话销毁前置（坑 136）：只要完成回调未到（g_srCbStatus < 0），会话就可能仍在重建中
    // ——先请求 Pause、等阶段落到 PAUSED/INIT，再销毁。"任务在途时销毁会话"官方明示为未定义行为。
    // 未 Start 的会话 Pause 返回错误码，无害；Pause 符号缺失（老 ROM）时跳过等待直接销毁。
    if (session != nullptr && g_srApi.ok && g_srCbStatus.load() < 0) {
        if (g_srApi.pauseSession != nullptr) {
            const HMS_SpatialReconStatus pst = g_srApi.pauseSession(session);
            VG_LOGI("SRCHK teardown pause status=%{public}d result=%{public}d",
                    static_cast<int32_t>(pst), result);
        }
        for (int32_t w = 0; w < 25; w++) {   // 最多等 5s 让阶段落定
            float kitProgress = 0.0f;
            HMS_SpatialReconStage stage = SPATIAL_RECON_STAGE_UNKNOWN;
            if (g_srApi.getProgress(session, &kitProgress, &stage) != SPATIAL_RECON_STATUS_SUCCESS) {
                break;
            }
            if (stage == SPATIAL_RECON_STAGE_PAUSED || stage == SPATIAL_RECON_STAGE_INIT) {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        VG_LOGI("SRCHK teardown settled (paused/init)");
    }

    // 所有退出路径都必须销毁会话（坑 136）：原先"仅当回调已到（g_srCbStatus >= 0）或用户终止
    // （-6）"才销毁，于是 push 失败 / 首帧空（-5）/ StartSession 失败 / 训练超时（-3）四条路径
    // 都把**仍活在 Kit 里**的会话漏掉——native 侧引用已置空、再也触达不到，而 Kit 仍持有它，
    // 下一次 CreateSession 即违反官方"同一时刻只允许一个重建会话"，属未定义行为（且内存不回收）。
    if (session != nullptr && g_srApi.ok && g_srApi.destroySession != nullptr) {
        const HMS_SpatialReconStatus dst = g_srApi.destroySession(session);
        VG_LOGI("SRCHK session destroyed result=%{public}d status=%{public}d",
                result, static_cast<int32_t>(dst));
    }
    g_srSession.store(nullptr);
    g_srPaused.store(false);
    // 终止不属于"任务失败"清进度：保留停在原地的进度值，UI 显示更直观
    if (result != 0 && result != -6) {
        g_srProgress.store(0.0f);
    }
    g_srBusy.store(false);
    return result;
}

} // namespace glassvideo
