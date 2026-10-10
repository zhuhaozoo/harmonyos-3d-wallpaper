/**
 * 3D壁纸 · 路线B 阶段2：深度网格场景的**纯 2D 单元**（与相机无关）
 *
 * 常量与口径：与离线判据 `tools/depth_mesh/depth_mesh_check.js` 一致（判据含源码级守卫）。
 * 划分依据：
 *   - 共享相机/射线/位姿/内参 → **不许拆**：投影与射线公式留在 spatial_recon_bridge.cpp（唯一来源）；
 *   - 与相机无关的纯 2D 算法 → **必须拆**：本单元只做"深度场 → 图像空间四边形 + 贴图"与
 *     "已投影好的屏幕顶点 → 光栅化"，**不出现任何射线/位姿/内参推导**（判据 [7] 同步守卫）。
 * 口径与离线判据 `tools/depth_mesh/depth_mesh_check.js` 一致（合成 18/0、真实 16/0 已证）：
 *   s=1 最近（相对逆深度）→ z = 1/(s/zNear + (1-s)/zFar)；四角 z 比 > 1.15 的格子细分到 1px
 *   **正面平行小片（四点同深，绝不跨台阶连接）**；照片网格向画幅外延展 skirt 像素（采样钳制）；
 *   背景层 = 近区 push-pull 补洞 + 仅近区膨胀内生成 + 深度 ×(1-2e-3) 的 tie 破断。
 * ⚠️ 坐标空间（坑 129）：深度场分辨率（如 546×728）与输出画布（1080×1440）**不是同一个像素空间**。
 *   本单元把网格角点**映射到画布像素空间**（x·outW/dw），贴图也按画布分辨率给出（UV = 画布像素坐标），
 *   与离线判据"参考帧逐像素反投影"同口径。若直接用深度场像素坐标当画布坐标 → 照片缩在画幅左上角、
 *   参考位姿不恒等（`SRCHK fmid` 可判）。此处只做 2D 画布映射，不含任何射线/位姿/内参。
 */
#ifndef GLASSRENDER_SPATIAL_RECON_DEPTH_MESH_H
#define GLASSRENDER_SPATIAL_RECON_DEPTH_MESH_H

#include <cstdint>
#include <cstddef>
#include <vector>

namespace glassvideo {

/** 图像空间四边形：4 角的**画布空间**连续像素坐标 (x,y) + 参考相机光轴深度 z + 贴图坐标 (u,v)
 *  （贴图与 x/y 同空间：本单元输出的贴图均为画布分辨率；光栅化时按透视正确插值采样） */
struct SrDepthQuad {
    float x[4];
    float y[4];
    float z[4];
    float u[4];
    float v[4];
};

/** 三层场景的两个网格层（远景板由 bridge 自建，单四边形即可） */
struct SrDepthMesh {
    std::vector<SrDepthQuad> front;   // 前景层（照片网格，含裙边）
    std::vector<SrDepthQuad> bg;      // 背景层（补洞后，仅近区膨胀内生成）
    std::vector<uint8_t> frontRgb;    // w*h*3（源帧像素，**画布分辨率**：由源 RGBA 双线性重采样）
    std::vector<uint8_t> bgRgb;       // w*h*3（push-pull 补洞后的背景色，已升采样到画布分辨率）
    int w = 0;                        // 画布宽（= outW；贴图分辨率）
    int h = 0;                        // 画布高（= outH）
    float zNear = 1.0f;
    float zFar = 3.0f;
    float nearSplitS = 0.5f;          // 近区判定（s > 该值 = 近区/主体）
    int coarseQuads = 0;              // 诊断：粗格数
    int refinedQuads = 0;             // 诊断：台阶细分小片数
    float maxZRatio = 1.0f;           // 结构守卫：粗格四角最大 z 比（应 ≤ 1.2；>1.15 的格子已细分）
    float p1 = 0.0f;                  // 诊断：深度稳健归一化的分位
    float p99 = 0.0f;
    float sP90 = 0.9f;                // 坑 133：s 的 p90（面积自适应"守护深度"分位）——
                                      //   近层面积过小的图（纵深贴枢轴平面）用 zNear 评估位移带会虚高，
                                      //   bridge 的 Θ 钳制据此深度取 lever（近层充足时 ≈ zNear，行为不变）
    float nearRatio = 0.0f;           // 诊断：近区（s > nearSplitS）像素占比
};

/** 深度场质量选项（质量优化第二档 D1/D2/D4，2026-10-04）。
 *  ⚠️ 坑 147 纪律：这些开关改变喂给 Kit 的帧字节——**产品默认档已于 2026-10-04 上调为质量增强档**
 *  （bridge 参数 depthEdge/adaptiveK/subjectAlign 默认 1；坑 154 记录过真机观感风险，
 *  变更须随真机同图 A/B 复核）。**本结构自身的缺省仍是 0/旧行为**——只在调用方不传 opts 时生效
 *  （bridge 恒传 opts，缺省路径仅供旧签名/诊断用）。
 *  口径与离线判据 tools/depth_mesh/depth_edge_check.js 严格同源（改一处必须同步两处）。
 *  真实数据验证（金标 = 画布分辨率参考深度）：JBU 边缘对齐分 0.206→0.445、
 *  渗色带宽 5.97→4.89px、MAE 0.0034→0.0029；D4 在理想化参考数据上收益弱，端侧 A/B 定档。 */
struct SrDepthMeshOpts {
    int edgeMode = 0;           /**< 0=旧行为（低分辨场双线性采样建网格）；1=D1 JBU 边缘对齐
                                 *   （RGB 引导上采样到画布分辨率再建网格，网格角点密度不变）；
                                 *   2=D4+D1（先保边预平滑再 JBU，消细碎台阶） */
    int preSmoothR = 2;         /**< D4：保边预平滑半径（深度场像素） */
    float preSmoothSigmaR = 0.10f;  /**< D4：值域 σ（s∈[0,1]；Δ≥0.3 的台阶跨边权重 ~e^-11 不糊） */
    int jbuRadius = 2;          /**< D1：JBU 窗口半径（低分辨率采样点数） */
    float jbuSigmaS = 1.6f;     /**< D1：空间域 σ（低分辨率像素） */
    float jbuSigmaR = 0.10f;    /**< D1：值域 σ（RMS 通道差 0~1） */
    const uint8_t *subjectMask = nullptr;  /**< D2：主体蒙版（**rw×rh 源分辨率**，0=背景/255=主体，
                                 *   内部重采样到深度场分辨率；null=不启用） */
    float subjectSigmaR = 0.08f;    /**< D2：值域 σ */
};

/**
 * D3 自适应纵深 k（纯 2D 分析；质量实验，bridge 在算 zFar 之前调用）。
 * 分离度 = s 场 64 桶直方图 Otsu 两分的类均值距 × 类平衡权重（两类各须 ≥5% 面积）；
 * k = kMin + (kMax−kMin)·clamp((sep−0.10)/(0.30−0.10), 0, 1)。
 * ⚠️ **d0 恒 2 公式**：调用方应取 zNear=4/(1+k)、zFar=4k/(1+k)——(zNear+zFar)/2 ≡ 2，
 * 轨道枢轴/相机数学/位移守卫零改动，只改深度起伏比。与 depth_edge_check.js OPTS 同源。
 */
float SrAdaptiveDepthK(const float *rawDepth, int dw, int dh, float kMin, float kMax);

/**
 * 由「原始相对逆深度（越大越近）+ 源帧 RGBA」构建三层场景所需的几何与贴图（纯 2D）。
 * @param rawDepth dw*dh float（模型原始输出；内部做 p1/p99 稳健归一化；s=1 最近）
 * @param rgba     源帧 rw*rh*4（行跨距须为 rw*4 紧凑缓冲；与深度网格**分辨率可不同**，
 *                 构建时按比例双线性重采样到 outW×outH —— 端侧深度是 546×728、源图可能是 809×1440）
 * @param zNear/zFar 深度映射参数（由 bridge 传入，常量在 bridge）
 * @param stride   网格抽稀步长（**深度场像素**，默认 4）——深度场比画布小，步长也随之按场分辨率取
 * @param skirt    照片网格向画幅外的延展（**画布像素**，默认 80；承接轨道端点的边缘空隙）
 * @param outW/outH 输出画布尺寸（1080×1440；网格角点与贴图都映射到该空间——纯 2D 约定）
 * @return 0=成功；-1=入参非法；-2=深度无效（p99-p1 过小）
 */
int32_t SrBuildDepthMesh(const float *rawDepth, int dw, int dh,
                         const uint8_t *rgba, int rw, int rh,
                         float zNear, float zFar, int stride, int skirt,
                         int outW, int outH, SrDepthMesh &out,
                         const SrDepthMeshOpts &opts = SrDepthMeshOpts());

/**
 * 相机无关的光栅化顶点：屏幕坐标 + 深度倒数 + 贴图坐标（由 bridge 用其单源投影算好后传入）。
 * 约定：屏幕像素 (px,py) 的采样点为 (px+0.5, py+0.5)；zInv 大者更近；等深度处**先画者胜**。
 */
struct SrScreenVert {
    float u;      // 屏幕 x（连续像素坐标）
    float v;      // 屏幕 y
    float zInv;   // 1 / 前方深度
    float tu;     // 贴图 x（连续像素坐标，采样钳制）
    float tv;     // 贴图 y
};

/**
 * 按 quad 列表光栅化（每个 quad 拆两个三角形，透视正确 UV 插值，z-buffer 共享）。
 * 绘制顺序 = 调用顺序（bridge 负责：远景板 → 前景层 → 背景层）；等深度处先画者胜。
 * @param verts 每 quad 4 个顶点，共 quadCount*4
 * @param tex/texW/texH 贴图（RGB 紧凑，越界钳制）
 * @param dstRgb 目标帧 RGB（outW*outH*3，紧凑；行跨距 == outW*3）
 * @param zbuf   深度缓冲（outW*outH float，存 1/z；调用方清零）
 * @param label  归属标签（可空）：写入 layerId
 * @param layerId 本层的归属 id（0=远景板 1=前景层 2=背景层）
 * @return 本层实际写入的像素数
 */
size_t SrRasterizeQuads(const SrScreenVert *verts, size_t quadCount, int outW, int outH,
                        const uint8_t *tex, int texW, int texH,
                        uint8_t *dstRgb, float *zbuf, uint8_t *label, uint8_t layerId);

/** 远景板兜底纹理：源帧 1/factor 盒式模糊（RGB 紧凑输出，尺寸 = ceil(w/factor)×ceil(h/factor)） */
void SrMakeBlurRgb(const uint8_t *rgb, int w, int h, int factor, std::vector<uint8_t> &out,
                   int &bw, int &bh);

/** 深度着色工具（调试/产物自检用）：s∈[0,1] → 10 档 ASCII 字符（0=' ' 远 … 9='@' 近） */
char SrDepthAsciiLevel(float s);

}  // namespace glassvideo

#endif  // GLASSRENDER_SPATIAL_RECON_DEPTH_MESH_H
