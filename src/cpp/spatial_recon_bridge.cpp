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
#include <cstring>
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
#undef SR_DLSYM_OPT
        g_srApi.ok = true;
        VG_LOGI("SRCHK api bound ok");
    });
}

// ---------------- 运行状态（供 napi 层轮询） ----------------

std::atomic<float> g_srProgress{0.0f};
std::atomic<bool> g_srBusy{false};
// StartSession 完成回调在 Kit 线程触发：只写原子量，不做任何 JS/重活
std::atomic<int32_t> g_srCbStatus{-1};
// 当前活动会话句柄（暂停/继续/终止接口据此操作；运行线程创建后写入、销毁前清空）
std::atomic<HMS_SpatialRecon_Session *> g_srSession{nullptr};
// 用户/热管理暂停标记（仅记录，不影响 Kit 状态）
std::atomic<bool> g_srPaused{false};
// 终止请求（运行线程在推帧/训练等待循环里检查；保存阶段不可中断，故意不检查）
std::atomic<int32_t> g_srAbort{0};

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
 * 判据 [7] 会解析下面三个常量并与判据脚本比对：**改名或改值必须同步改判据脚本与实现文档 §附录B**。
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
                             float fx, float fy, float cx, float cy, std::vector<uint8_t> &rgb)
{
    rgb.assign(static_cast<size_t>(kOutW) * kOutH * 3, 0);
    const size_t total = static_cast<size_t>(kOutW) * kOutH;
    std::vector<float> zbuf(total, 0.0f);

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
                                                    rgb.data(), zbuf.data(), nullptr, layerId);
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
                                                    nullptr, 0);
        }
    }
    // ② 前景层（照片网格）→ ③ 背景层（补洞，承接消隐带）
    rasterLayer(dm.mesh.front, dm.mesh.frontRgb.data(), dm.mesh.w, dm.mesh.h, 1);
    rasterLayer(dm.mesh.bg, dm.mesh.bgRgb.data(), dm.mesh.w, dm.mesh.h, 2);

    return static_cast<int64_t>(total - (written < total ? written : total));
}

int32_t SpatialReconTestRun(const SpatialReconTestParams &params, std::string &outModelPath)
{
    if (g_srBusy.exchange(true)) {
        return -100;
    }
    g_srProgress.store(0.0f);
    g_srCbStatus.store(-1);
    g_srAbort.store(0);
    g_srPaused.store(false);
    int32_t result = -1;
    HMS_SpatialRecon_Session *session = nullptr;

    do {
        SrBindApi();
        if (!g_srApi.ok) {
            result = -1;
            break;
        }
        if (params.srcW <= 0 || params.srcH <= 0
            || params.rgba.size() != static_cast<size_t>(params.srcW) * params.srcH * 4) {
            VG_LOGE("SRCHK invalid src %{public}dx%{public}d buf=%{public}zu",
                    params.srcW, params.srcH, params.rgba.size());
            result = -2;
            break;
        }
        int32_t frameCount = params.frameCount;
        if (frameCount < 2) frameCount = 2;
        if (frameCount > 240) frameCount = 240;
        float orbitDeg = params.orbitDeg;
        if (orbitDeg < 1.0f) orbitDeg = 1.0f;
        if (orbitDeg > 45.0f) orbitDeg = 45.0f;
        float fx = params.focalPx;
        if (fx < 200.0f) fx = 200.0f;
        if (fx > 6000.0f) fx = 6000.0f;
        const float fy = fx;
        const float cx = kOutW * 0.5f;
        const float cy = kOutH * 0.5f;
        int32_t scene = (params.scene >= 0 && params.scene <= 5) ? params.scene : 1;
        const int32_t poseFamily = (params.poseFamily == 1) ? 1 : 0;
        // 诊断：源图缩略图（验证 ArkTS→native 的 RGBA 交接是否正确）
        SrLogAscii("src", params.rgba.data(), params.srcW, params.srcH, 4);

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
            } else if (SrBuildLayerAssets(params.rgba.data(), params.srcW, params.srcH,
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
                const float zFar = kDepthZNear * kDepthRatio;
                const int32_t rc = glassvideo::SrBuildDepthMesh(params.depth.data(), params.depthW,
                    params.depthH, params.rgba.data(), params.srcW, params.srcH,
                    kDepthZNear, zFar, kDepthStride, kDepthSkirt,
                    kOutW, kOutH, depthCtx.mesh);
                if (rc != 0) {
                    VG_LOGW("SRCHK depth mesh build failed rc=%{public}d, fallback scene=1", rc);
                    scene = 1;
                } else {
                    // 世界点一次性固化（参考位姿反投影）——逐帧只做投影；**不得**移进逐帧循环（坑 129）
                    SrBakeDepthMeshWorld(depthCtx.mesh, fx, fy, cx, cy);
                    glassvideo::SrMakeBlurRgb(depthCtx.mesh.frontRgb.data(), depthCtx.mesh.w,
                                              depthCtx.mesh.h, 16, depthCtx.planeRgb,
                                              depthCtx.planeW, depthCtx.planeH);
                    // 远景板压暗（与离线判据一致：×0.72/×0.72/×0.75）
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
                        const float zGuard = 1.0f / (sGuard / kDepthZNear + (1.0f - sGuard) / zFar);  // 与 zOfS 同式
                        float coef = 0.5f * std::fabs(zGuard - kSceneDist) / std::max(zGuard, 1e-3f);
                        if (coef < 0.05f) {
                            // 极平场景（内容几乎全贴枢轴）：防除零；Θ 会被推到上界，仍建不出属预期
                            coef = 0.05f;
                        }
                        const float coefNear = 0.5f * (kSceneDist - kDepthZNear) / kDepthZNear;   // = 0.5
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
                            params.depthW, params.depthH, kOutW, kOutH, orbitDeg, kDepthRatio,
                            kDepthZNear, kDepthStride, kDepthSkirt, depthCtx.mesh.front.size(),
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
        for (int32_t i = 0; i < frameCount; i++) {
            if (g_srAbort.load()) {
                VG_LOGI("SRCHK abort during push at %{public}d", i);
                result = -6;
                break;
            }
            SrVec3 pos;
            float basis[9];
            // 对角弧（坑 131）：scene=5 的俯仰半角与偏航同值（Φ=Θ，判据同口径）；
            // scene=0..4 保持纯水平弧（pitch=0，与既有真机验证行为逐位一致）
            SrFrameBasis(i, frameCount, orbitDeg, (scene == 5) ? orbitDeg : 0.0f, pos, basis);
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
            const int64_t missPx = (scene == 5)
                ? SrRasterizeDepthMesh(depthCtx, pos, basis, fx, fy, cx, cy, rgb)
                : SrRenderFrame(params.rgba.data(), params.srcW, params.srcH,
                                pos, basis, fx, fy, cx, cy, scene, layers, rgb);
            // 坑 132：在推帧口可选交换 R/B（默认开）——开发期观察到该开关会改变成片的颜色表现，
            // 但成因**尚无定论**（坑 134 的"A/B 定案"已撤销，见档案 F7）。此处只提供开关；
            // Service 层失败时会自动反交换重试一次，bgrSwap=false 可手动回到直通 RGB。
            // 只换推帧缓冲：ASCII 缩略图用亮度加权不受影响，内部管线（贴图/深度/补洞）不换。
            if (params.bgrSwap) {
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
                VG_LOGI("SRCHK push %{public}d/%{public}d pos=(%{public}f,%{public}f,%{public}f) quat=(%{public}f,%{public}f,%{public}f,%{public}f) miss=%{public}d",
                        i, frameCount, posReport.x, posReport.y, posReport.z, q[0], q[1], q[2], q[3],
                        static_cast<int32_t>(missPx));
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
        if (result != -1) {
            // 推帧阶段已带错误码退出（result 离开初始值 -1 且非 0）
            break;
        }

        // 3. 启动重建（writeInfo 非空：重建完成自动保存 MP4），随后按前台模式分配资源
        HMS_SpatialRecon_ModelWriteInfo info{};
        info.modelFormat = SPATIAL_RECON_OUTPUT_FORMAT_MP4;
        info.modelFile = params.modelPath.c_str();
        info.audioFile = nullptr;
        info.longitude = 0.0f;
        info.latitude = 0.0f;
        g_srCbStatus.store(-1);
        st = g_srApi.startSession(session, &info, &SrOnFinished);
        if (st != SPATIAL_RECON_STATUS_SUCCESS) {
            VG_LOGE("SRCHK startSession failed status=%{public}d", static_cast<int32_t>(st));
            result = static_cast<int32_t>(st);
            break;
        }
        g_srApi.setRunningMode(session, SPATIAL_RECON_RUNNING_FOREGROUND_MODE);
        VG_LOGI("SRCHK building src=%{public}dx%{public}d frames=%{public}d orbit=%{public}f pitch=%{public}f focal=%{public}f scene=%{public}d family=%{public}d invQuat=%{public}d bgrSwap=%{public}d",
                params.srcW, params.srcH, frameCount, orbitDeg, (scene == 5) ? orbitDeg : 0.0f, fx, scene,
                poseFamily, params.invertQuat ? 1 : 0, params.bgrSwap ? 1 : 0);

        // 4. 等待完成回调（上限 30 分钟），期间镜像 Kit 进度（0.3~1.0）
        constexpr int32_t kMaxWaitMs = 30 * 60 * 1000;
        int32_t waitedMs = 0;
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
            if (g_srApi.getProgress(session, &kitProgress, &stage) == SPATIAL_RECON_STATUS_SUCCESS) {
                if (kitProgress > g_srProgress.load()) {
                    g_srProgress.store(0.3f + 0.7f * kitProgress);
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

        // 5. 等保存阶段收尾（回调若先于保存完成触发，销毁会话会撞上在途保存=未定义行为）
        int32_t saveWaitMs = 0;
        HMS_SpatialReconStage stage = SPATIAL_RECON_STAGE_UNKNOWN;
        while (saveWaitMs < kMaxWaitMs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            saveWaitMs += 200;
            stage = SPATIAL_RECON_STAGE_UNKNOWN;
            float kitProgress = 0.0f;
            if (g_srApi.getProgress(session, &kitProgress, &stage) != SPATIAL_RECON_STATUS_SUCCESS) {
                break;
            }
            if (kitProgress > 0.0f) {
                g_srProgress.store(0.3f + 0.7f * kitProgress);
            }
            if (stage != SPATIAL_RECON_STAGE_SAVING) {
                break;
            }
        }
        // 产物校验（会话销毁前确认文件已落盘）
        if (::access(params.modelPath.c_str(), F_OK) != 0) {
            VG_LOGE("SRCHK model file missing %{public}s", params.modelPath.c_str());
            result = -4;
            break;
        }
        // 诊断对账（官方 GetRefinedFrame，选帧读回"重建优化后"的内外参）：
        // 用途——比例失真排查（坑 131 定案）：我们上报 fx=fy（方像素合成相机），Kit 的 BA 若把
        // fy/fx 精修到远离 1，即"纵向尺度/fy 退化方向"发生漂移——refined fy/fx 比值
        // 直接等于重建模型的"横向/纵向尺度比"（>1 = 横向拉伸、<1 = 横向压缩），是失真的
        // 定量指标。绝对数值无意义（BA 整体尺度规约自由，实测同配置两轮 26541 vs 28.5）。
        // 修复（对角弧）落地后该值应 ≈1；不返回像素（imageData=null）；失败只告警，不影响出片。
        if (g_srApi.getRefinedFrame != nullptr) {
            const int32_t probeIdx[3] = { 0, frameCount / 2, frameCount - 1 };
            for (int32_t k = 0; k < 3; k++) {
                HMS_SpatialRecon_DataFrame rf{};
                const HMS_SpatialReconStatus rs = g_srApi.getRefinedFrame(session, probeIdx[k], &rf);
                if (rs == SPATIAL_RECON_STATUS_SUCCESS) {
                    const float aspect = (std::fabs(rf.focalX) > 1e-9f) ? rf.focalY / rf.focalX : 0.0f;
                    VG_LOGI("SRCHK refined i=%{public}d fx=%{public}f fy=%{public}f aspect=%{public}.3f cx=%{public}f cy=%{public}f "
                            "pos=(%{public}f,%{public}f,%{public}f) quat=(%{public}f,%{public}f,%{public}f,%{public}f)",
                            probeIdx[k], rf.focalX, rf.focalY, aspect, rf.principalX, rf.principalY,
                            rf.position[0], rf.position[1], rf.position[2],
                            rf.rotation[0], rf.rotation[1], rf.rotation[2], rf.rotation[3]);
                    if (aspect > 0.0f && std::fabs(aspect - 1.0f) > 0.25f) {
                        VG_LOGW("SRCHK refined anisotropy fy/fx=%{public}.3f (pushed 1.0) —— 纵横尺度比漂移 "
                                "超 25%%，退化漂移嫌疑（坑 131；对角弧修复后应 ≈1，若仍出现需复查轨道）", aspect);
                    }
                } else {
                    VG_LOGW("SRCHK refined i=%{public}d failed status=%{public}d",
                            probeIdx[k], static_cast<int32_t>(rs));
                }
            }
        }
        g_srProgress.store(1.0f);
        outModelPath = params.modelPath;
        result = 0;
        VG_LOGI("SRCHK done %{public}s", params.modelPath.c_str());
    } while (false);

    // 用户终止：等 Kit 把阶段落到暂停态（PauseSession 生效）再销毁，
    // 避免"任务在途时销毁会话"（官方明示为未定义行为）
    if (result == -6 && session != nullptr && g_srApi.ok) {
        for (int32_t w = 0; w < 15; w++) {
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
        VG_LOGI("SRCHK abort settled (paused), destroying session");
    }

    // 回调已收到（或任务根本没启动、或用户终止并已落定）后销毁会话；
    // 在途回调中销毁 = 未定义行为（终止路径已等 PAUSED）
    if (session != nullptr && (g_srCbStatus.load() >= 0 || result == -6)) {
        g_srApi.destroySession(session);
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
