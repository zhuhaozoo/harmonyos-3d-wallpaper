/**
 * 3D壁纸 · 路线B 阶段2：深度网格场景的纯 2D 实现（口径与 tools/depth_mesh/depth_mesh_check.js 一致）
 * 见 include/spatial_recon_depth_mesh.h 的契约说明。**本文件不得出现相机/射线/位姿/内参推导。**
 */
#include "spatial_recon_depth_mesh.h"

#include <hilog/log.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <string>

#undef LOG_DOMAIN
#undef LOG_TAG
#define LOG_DOMAIN 0x3200
#define LOG_TAG "SRDM"
#define SRDM_LOGI(...) ((void)OH_LOG_Print(LOG_APP, LOG_INFO, LOG_DOMAIN, LOG_TAG, __VA_ARGS__))

namespace glassvideo {
namespace {

constexpr float kRefineRatio = 1.15f;   // 四角 z 比 > 该值 → 细分到 1px 正面平行小片
constexpr int kDilateRadius = 24;       // 背景层生成范围的膨胀半径（px）

inline float Clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/** 稳健归一化：直方图求 p1/p99（避免排序 40 万浮点） */
void Percentiles(const float *a, size_t n, float &p1, float &p99) {
    float mn = a[0], mx = a[0];
    for (size_t i = 1; i < n; i++) {
        if (a[i] < mn) mn = a[i];
        if (a[i] > mx) mx = a[i];
    }
    constexpr int kBins = 4096;
    std::vector<uint32_t> hist(kBins, 0);
    const float span = (mx - mn) > 1e-9f ? (mx - mn) : 1.0f;
    for (size_t i = 0; i < n; i++) {
        int b = static_cast<int>((a[i] - mn) / span * (kBins - 1));
        hist[Clampf(static_cast<float>(b), 0.0f, static_cast<float>(kBins - 1))]++;
    }
    const size_t lo = static_cast<size_t>(0.01 * static_cast<double>(n));
    const size_t hi = static_cast<size_t>(0.99 * static_cast<double>(n));
    size_t acc = 0;
    p1 = mn;
    p99 = mx;
    for (int b = 0; b < kBins; b++) {
        acc += hist[b];
        const float val = mn + span * (static_cast<float>(b) + 0.5f) / kBins;
        if (acc >= lo && p1 == mn) p1 = val;
        if (acc >= hi) { p99 = val; break; }
    }
}

/** s 场双线性采样（连续像素坐标，越界钳制；与 JS sampleS 同语义） */
float SampleS(const std::vector<float> &s, int w, int h, float x, float y) {
    const float fx = Clampf(x - 0.5f, 0.0f, static_cast<float>(w) - 1.001f);
    const float fy = Clampf(y - 0.5f, 0.0f, static_cast<float>(h) - 1.001f);
    const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
    const float tx = fx - x0, ty = fy - y0;
    const int x1 = std::min(w - 1, x0 + 1), y1 = std::min(h - 1, y0 + 1);
    const float v00 = s[static_cast<size_t>(y0) * w + x0], v10 = s[static_cast<size_t>(y0) * w + x1];
    const float v01 = s[static_cast<size_t>(y1) * w + x0], v11 = s[static_cast<size_t>(y1) * w + x1];
    return (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
}

/** 贴图采样（RGB 紧凑，连续像素坐标，越界钳制；与 JS sampleRgb 同语义） */
inline void SampleRgb(const uint8_t *tex, int w, int h, float x, float y, uint8_t out[3]) {
    const float fx = Clampf(x - 0.5f, 0.0f, static_cast<float>(w) - 1.001f);
    const float fy = Clampf(y - 0.5f, 0.0f, static_cast<float>(h) - 1.001f);
    const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
    const float tx = fx - x0, ty = fy - y0;
    const int x1 = std::min(w - 1, x0 + 1), y1 = std::min(h - 1, y0 + 1);
    for (int c = 0; c < 3; c++) {
        const float v00 = tex[(static_cast<size_t>(y0) * w + x0) * 3 + c];
        const float v10 = tex[(static_cast<size_t>(y0) * w + x1) * 3 + c];
        const float v01 = tex[(static_cast<size_t>(y1) * w + x0) * 3 + c];
        const float v11 = tex[(static_cast<size_t>(y1) * w + x1) * 3 + c];
        out[c] = static_cast<uint8_t>(Clampf((v00 * (1 - tx) + v10 * tx) * (1 - ty) +
                                             (v01 * (1 - tx) + v11 * tx) * ty, 0.0f, 255.0f));
    }
}

/** 源帧采样（RGBA，4 字节/像素；连续像素坐标，越界钳制）——用于按画布分辨率重采样源图 */
inline void SampleRgba(const uint8_t *tex, int w, int h, float x, float y, uint8_t out[3]) {
    const float fx = Clampf(x - 0.5f, 0.0f, static_cast<float>(w) - 1.001f);
    const float fy = Clampf(y - 0.5f, 0.0f, static_cast<float>(h) - 1.001f);
    const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
    const float tx = fx - x0, ty = fy - y0;
    const int x1 = std::min(w - 1, x0 + 1), y1 = std::min(h - 1, y0 + 1);
    for (int c = 0; c < 3; c++) {
        const float v00 = tex[(static_cast<size_t>(y0) * w + x0) * 4 + c];
        const float v10 = tex[(static_cast<size_t>(y0) * w + x1) * 4 + c];
        const float v01 = tex[(static_cast<size_t>(y1) * w + x0) * 4 + c];
        const float v11 = tex[(static_cast<size_t>(y1) * w + x1) * 4 + c];
        out[c] = static_cast<uint8_t>(Clampf((v00 * (1 - tx) + v10 * tx) * (1 - ty) +
                                             (v01 * (1 - tx) + v11 * tx) * ty, 0.0f, 255.0f));
    }
}

/** push-pull 补洞（蒙版加权金字塔；顶层兜底：最近有效邻居播种 → 全局加权均值；push 用双线性）
 *  ch=1 填深度场 s；ch=3 填 RGB。mask=1 的像素为待填。 */
void PushPullFill(std::vector<float> &field, int ch, const std::vector<uint8_t> &mask, int w, int h) {
    struct Level { std::vector<float> v; std::vector<float> wt; int w, h; };
    std::vector<Level> levs;
    {
        Level l0;
        l0.v = field;
        l0.wt.assign(static_cast<size_t>(w) * h, 0.0f);
        for (size_t i = 0; i < l0.wt.size(); i++) l0.wt[i] = mask[i] ? 0.0f : 1.0f;
        l0.w = w;
        l0.h = h;
        levs.push_back(std::move(l0));
    }
    int cw = w, chh = h;
    while (cw > 4 && chh > 4) {
        const Level &cur = levs.back();
        const int nw = (cw + 1) / 2, nh = (chh + 1) / 2;
        Level nx;
        nx.w = nw;
        nx.h = nh;
        nx.v.assign(static_cast<size_t>(nw) * nh * ch, 0.0f);
        nx.wt.assign(static_cast<size_t>(nw) * nh, 0.0f);
        for (int y = 0; y < nh; y++) {
            for (int x = 0; x < nw; x++) {
                float wt = 0.0f;
                float acc[3] = {0.0f, 0.0f, 0.0f};
                for (int dy = 0; dy < 2; dy++) {
                    for (int dx = 0; dx < 2; dx++) {
                        const int sx = std::min(cw - 1, x * 2 + dx), sy = std::min(chh - 1, y * 2 + dy);
                        const size_t k = static_cast<size_t>(sy) * cw + sx;
                        const float wk = cur.wt[k];
                        if (wk <= 0.0f) continue;
                        for (int c = 0; c < ch; c++) acc[c] += cur.v[k * ch + c] * wk;
                        wt += wk;
                    }
                }
                const size_t k2 = static_cast<size_t>(y) * nw + x;
                if (wt > 0.0f) {
                    for (int c = 0; c < ch; c++) nx.v[k2 * ch + c] = acc[c] / wt;
                    nx.wt[k2] = wt;
                }
            }
        }
        levs.push_back(std::move(nx));
        cw = nw;
        chh = nh;
    }
    // 顶层兜底（与 JS 版同款；不做这一步会让主体内部保留源色/近深度，消隐带露鬼影）
    Level &top = levs.back();
    float gmean[3] = {0.0f, 0.0f, 0.0f};
    {
        const Level &l0 = levs[0];
        double gwt = 0.0;
        for (size_t i = 0; i < l0.wt.size(); i++) {
            const float wk = l0.wt[i];
            if (wk <= 0.0f) continue;
            for (int c = 0; c < ch; c++) gmean[c] += l0.v[i * ch + c] * wk;
            gwt += wk;
        }
        if (gwt > 0.0) for (int c = 0; c < ch; c++) gmean[c] = static_cast<float>(gmean[c] / gwt);
    }
    for (int pass = 0; pass < 16; pass++) {
        int changed = 0;
        const std::vector<float> pv = top.v, pw = top.wt;
        for (int y = 0; y < top.h; y++) {
            for (int x = 0; x < top.w; x++) {
                const size_t k = static_cast<size_t>(y) * top.w + x;
                if (pw[k] > 0.0f) continue;
                float wt = 0.0f;
                float acc[3] = {0.0f, 0.0f, 0.0f};
                const int nbr[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
                for (auto &d : nbr) {
                    const int xx = x + d[0], yy = y + d[1];
                    if (xx < 0 || yy < 0 || xx >= top.w || yy >= top.h) continue;
                    const size_t kk = static_cast<size_t>(yy) * top.w + xx;
                    if (pw[kk] <= 0.0f) continue;
                    for (int c = 0; c < ch; c++) acc[c] += pv[kk * ch + c];
                    wt += 1.0f;
                }
                if (wt > 0.0f) {
                    for (int c = 0; c < ch; c++) top.v[k * ch + c] = acc[c] / wt;
                    top.wt[k] = 1e-6f;
                    changed++;
                }
            }
        }
        if (changed == 0) break;
    }
    for (size_t i = 0; i < top.wt.size(); i++) {
        if (top.wt[i] > 0.0f) continue;
        for (int c = 0; c < ch; c++) top.v[i * ch + c] = gmean[c];
        top.wt[i] = 1e-6f;
    }
    // push（自顶向下；双线性采样父层）
    for (size_t L = levs.size() - 1; L > 0; L--) {
        const Level &up = levs[L];
        Level &dn = levs[L - 1];
        for (int y = 0; y < up.h; y++) {
            for (int x = 0; x < up.w; x++) {
                const size_t ku = static_cast<size_t>(y) * up.w + x;
                if (up.wt[ku] <= 0.0f) continue;
                for (int dy = 0; dy < 2; dy++) {
                    for (int dx = 0; dx < 2; dx++) {
                        const int sx = std::min(dn.w - 1, x * 2 + dx), sy = std::min(dn.h - 1, y * 2 + dy);
                        const size_t kd = static_cast<size_t>(sy) * dn.w + sx;
                        if (dn.wt[kd] > 0.0f) continue;
                        const float px = Clampf((sx + 0.5f) / 2.0f - 0.5f, 0.0f, static_cast<float>(up.w) - 1.001f);
                        const float py = Clampf((sy + 0.5f) / 2.0f - 0.5f, 0.0f, static_cast<float>(up.h) - 1.001f);
                        const int x0 = static_cast<int>(px), y0 = static_cast<int>(py);
                        const float tx = px - x0, ty = py - y0;
                        const int x1 = std::min(up.w - 1, x0 + 1), y1 = std::min(up.h - 1, y0 + 1);
                        for (int c = 0; c < ch; c++) {
                            const float v00 = up.v[(static_cast<size_t>(y0) * up.w + x0) * ch + c];
                            const float v10 = up.v[(static_cast<size_t>(y0) * up.w + x1) * ch + c];
                            const float v01 = up.v[(static_cast<size_t>(y1) * up.w + x0) * ch + c];
                            const float v11 = up.v[(static_cast<size_t>(y1) * up.w + x1) * ch + c];
                            dn.v[kd * ch + c] = (v00 * (1 - tx) + v10 * tx) * (1 - ty) +
                                                (v01 * (1 - tx) + v11 * tx) * ty;
                        }
                        dn.wt[kd] = 1e-6f;
                    }
                }
            }
        }
    }
    field = levs[0].v;
}

/** 光栅化一个三角形（透视正确 UV；zInv 大者近；等深度先画者胜） */
void RasterTri(const SrScreenVert &p0, const SrScreenVert &p1, const SrScreenVert &p2,
               int outW, int outH, const uint8_t *tex, int texW, int texH,
               uint8_t *dstRgb, float *zbuf, uint8_t *label, uint8_t layerId, size_t &written) {
    const float dz0 = p0.zInv, dz1 = p1.zInv, dz2 = p2.zInv;
    const float area = (p1.u - p0.u) * (p2.v - p0.v) - (p2.u - p0.u) * (p1.v - p0.v);
    if (std::fabs(area) < 1e-9f) return;
    const float inv = 1.0f / area;
    const int minX = std::max(0, static_cast<int>(std::floor(std::min({p0.u, p1.u, p2.u}))));
    const int maxX = std::min(outW - 1, static_cast<int>(std::ceil(std::max({p0.u, p1.u, p2.u}))));
    const int minY = std::max(0, static_cast<int>(std::floor(std::min({p0.v, p1.v, p2.v}))));
    const int maxY = std::min(outH - 1, static_cast<int>(std::ceil(std::max({p0.v, p1.v, p2.v}))));
    for (int y = minY; y <= maxY; y++) {
        const float sy = y + 0.5f;
        for (int x = minX; x <= maxX; x++) {
            const float sx = x + 0.5f;
            const float l0 = ((p1.u - sx) * (p2.v - sy) - (p2.u - sx) * (p1.v - sy)) * inv;
            if (l0 < -1e-6f) continue;
            const float l1 = ((p2.u - sx) * (p0.v - sy) - (p0.u - sx) * (p2.v - sy)) * inv;
            if (l1 < -1e-6f) continue;
            const float l2 = 1.0f - l0 - l1;
            if (l2 < -1e-6f) continue;
            const float zInv = l0 * dz0 + l1 * dz1 + l2 * dz2;
            const size_t idx = static_cast<size_t>(y) * outW + x;
            if (zInv <= zbuf[idx]) continue;   // 等深度先画者胜
            zbuf[idx] = zInv;
            const float tu = (l0 * p0.tu * dz0 + l1 * p1.tu * dz1 + l2 * p2.tu * dz2) / zInv;
            const float tv = (l0 * p0.tv * dz0 + l1 * p1.tv * dz1 + l2 * p2.tv * dz2) / zInv;
            uint8_t col[3];
            SampleRgb(tex, texW, texH, tu, tv, col);
            dstRgb[idx * 3] = col[0];
            dstRgb[idx * 3 + 1] = col[1];
            dstRgb[idx * 3 + 2] = col[2];
            if (label != nullptr) label[idx] = layerId;
            written++;
        }
    }
}

/** 掩膜膨胀（两趟最大值窗口，O(n)；替代逐次 4 邻域膨胀） */
void DilateMask(const std::vector<uint8_t> &in, int w, int h, int radius, std::vector<uint8_t> &out) {
    std::vector<uint8_t> tmp(static_cast<size_t>(w) * h, 0);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            uint8_t v = 0;
            const int x0 = std::max(0, x - radius), x1 = std::min(w - 1, x + radius);
            for (int xx = x0; xx <= x1 && !v; xx++) v = in[static_cast<size_t>(y) * w + xx];
            tmp[static_cast<size_t>(y) * w + x] = v;
        }
    }
    out.assign(static_cast<size_t>(w) * h, 0);
    for (int y = 0; y < h; y++) {
        const int y0 = std::max(0, y - radius), y1 = std::min(h - 1, y + radius);
        for (int x = 0; x < w; x++) {
            uint8_t v = 0;
            for (int yy = y0; yy <= y1 && !v; yy++) v = tmp[static_cast<size_t>(yy) * w + x];
            out[static_cast<size_t>(y) * w + x] = v;
        }
    }
}

// ===================== 深度场质量算法（D1/D2/D4；与 tools/depth_mesh/depth_edge_check.js 同源） =====================

/** D4：s 场保边预平滑（单遍 bilateral；JS bilateralSmooth 同式） */
std::vector<float> BilateralSmoothS(const std::vector<float> &s, int w, int h, int r, float sigmaR) {
    std::vector<float> out(s.size());
    const float sigmaS = static_cast<float>(r) / 1.5f;
    const float inv2sr = 1.0f / (2.0f * sigmaR * sigmaR);
    const float inv2ss = 1.0f / (2.0f * sigmaS * sigmaS);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            const float v0 = s[static_cast<size_t>(y) * w + x];
            float acc = 0.0f, wsum = 0.0f;
            for (int dy = -r; dy <= r; dy++) {
                const int yy = y + dy;
                if (yy < 0 || yy >= h) continue;
                for (int dx = -r; dx <= r; dx++) {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= w) continue;
                    const float v = s[static_cast<size_t>(yy) * w + xx];
                    const float dv = v - v0;
                    const float wgt = std::exp(-static_cast<float>(dx * dx + dy * dy) * inv2ss - dv * dv * inv2sr);
                    acc += v * wgt;
                    wsum += wgt;
                }
            }
            out[static_cast<size_t>(y) * w + x] = wsum > 1e-9f ? acc / wsum : v0;
        }
    }
    return out;
}

/** D2：主体蒙版门控保边平滑（两遍；蒙版外逐字节不变）——mask 为深度场分辨率（0/255 或概率） */
void SubjectAlignS(std::vector<float> &s, int w, int h, const std::vector<uint8_t> &mask, float sigmaR) {
    const int r = 2;
    const float sigmaS = static_cast<float>(r) / 1.5f;
    const float inv2sr = 1.0f / (2.0f * sigmaR * sigmaR);
    const float inv2ss = 1.0f / (2.0f * sigmaS * sigmaS);
    for (int pass = 0; pass < 2; pass++) {
        const std::vector<float> cur = s;
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                const size_t k = static_cast<size_t>(y) * w + x;
                if (mask[k] < 128) continue;   // 蒙版外不动
                const float v0 = cur[k];
                float acc = 0.0f, wsum = 0.0f;
                for (int dy = -r; dy <= r; dy++) {
                    const int yy = y + dy;
                    if (yy < 0 || yy >= h) continue;
                    for (int dx = -r; dx <= r; dx++) {
                        const int xx = x + dx;
                        if (xx < 0 || xx >= w) continue;
                        const size_t kk = static_cast<size_t>(yy) * w + xx;
                        if (mask[kk] < 128) continue;   // 只聚合主体内邻居（蒙版门控）
                        const float v = cur[kk];
                        const float dv = v - v0;
                        const float wgt = std::exp(-static_cast<float>(dx * dx + dy * dy) * inv2ss - dv * dv * inv2sr);
                        acc += v * wgt;
                        wsum += wgt;
                    }
                }
                if (wsum > 1e-9f) s[k] = acc / wsum;
            }
        }
    }
}

/** exp(−t) 查找表（t∈[0,30]、4096 档、线性插值）：JBU 热路径 ~4000 万次权重，避免逐次 std::exp。
 *  与解析 exp 偏差 < 1e-4，对归一化权重无影响。 */
struct ExpNegLut {
    std::vector<float> v;
    ExpNegLut() : v(4096) {
        for (int i = 0; i < 4096; i++) v[i] = std::exp(-static_cast<float>(i) * (30.0f / 4096.0f));
    }
};
const ExpNegLut kExpNegLut;
inline float ExpNeg(float t) {
    if (t <= 0.0f) return 1.0f;
    const float fi = t * (4096.0f / 30.0f);
    const int i = static_cast<int>(fi);
    if (i >= 4095) return 0.0f;
    const float fr = fi - static_cast<float>(i);
    return kExpNegLut.v[i] * (1.0f - fr) + kExpNegLut.v[i + 1] * fr;
}

/** D1：JBU（joint bilateral upsample）——s 低分辨场 → 画布分辨率，引导图 = 画布分辨率 RGB
 *  （guide 为 outW×outH×3 紧凑 RGB，通常传 frontRgb）。JS jbuUpsample 同式：
 *  tap 处引导色 G[q] 预采样；权重 = exp(−|lp−q|²/2σs²)·exp(−d²/2σr²)，d = RMS 通道差(0~1)；
 *  Σw < 1e-6 回退双线性。 */
std::vector<float> JbuUpsampleS(const std::vector<float> &s, int dw, int dh,
                                const uint8_t *guide, int W, int H, const SrDepthMeshOpts &o) {
    const int r = std::max(1, o.jbuRadius);
    const float inv2ss = 1.0f / (2.0f * o.jbuSigmaS * o.jbuSigmaS);
    const float inv2sr = 1.0f / (2.0f * o.jbuSigmaR * o.jbuSigmaR);
    // 引导图在低分辨 tap 处的预采样 G[q]
    std::vector<float> G(static_cast<size_t>(dw) * dh * 3);
    for (int qy = 0; qy < dh; qy++) {
        for (int qx = 0; qx < dw; qx++) {
            uint8_t col[3];
            SampleRgb(guide, W, H, (static_cast<float>(qx) + 0.5f) * W / dw,
                      (static_cast<float>(qy) + 0.5f) * H / dh, col);
            const size_t q = static_cast<size_t>(qy) * dw + qx;
            G[q * 3] = static_cast<float>(col[0]);
            G[q * 3 + 1] = static_cast<float>(col[1]);
            G[q * 3 + 2] = static_cast<float>(col[2]);
        }
    }
    std::vector<float> out(static_cast<size_t>(W) * H);
    for (int y = 0; y < H; y++) {
        const float lpy = (static_cast<float>(y) + 0.5f) * dh / H;
        const int qy0 = std::max(0, std::min(dh - 1, static_cast<int>(std::lround(lpy))));
        for (int x = 0; x < W; x++) {
            const float lpx = (static_cast<float>(x) + 0.5f) * dw / W;
            const int qx0 = std::max(0, std::min(dw - 1, static_cast<int>(std::lround(lpx))));
            const size_t p = static_cast<size_t>(y) * W + x;
            const float pr = static_cast<float>(guide[p * 3]) / 255.0f;
            const float pg = static_cast<float>(guide[p * 3 + 1]) / 255.0f;
            const float pb = static_cast<float>(guide[p * 3 + 2]) / 255.0f;
            float acc = 0.0f, wsum = 0.0f;
            for (int dy = -r; dy <= r; dy++) {
                const int qy = qy0 + dy;
                if (qy < 0 || qy >= dh) continue;
                for (int dx = -r; dx <= r; dx++) {
                    const int qx = qx0 + dx;
                    if (qx < 0 || qx >= dw) continue;
                    const size_t q = static_cast<size_t>(qy) * dw + qx;
                    const float ex = lpx - static_cast<float>(qx);
                    const float ey = lpy - static_cast<float>(qy);
                    const float dr = pr - G[q * 3] / 255.0f;
                    const float dg = pg - G[q * 3 + 1] / 255.0f;
                    const float db = pb - G[q * 3 + 2] / 255.0f;
                    const float wgt = ExpNeg((ex * ex + ey * ey) * inv2ss +
                                             (dr * dr + dg * dg + db * db) / 3.0f * inv2sr);
                    acc += s[q] * wgt;
                    wsum += wgt;
                }
            }
            out[p] = wsum > 1e-6f ? acc / wsum : SampleS(s, dw, dh, lpx + 0.5f, lpy + 0.5f);
        }
    }
    return out;
}

/** D3：Otsu 分离度 → k（JS adaptiveK 同式；映射端点 sepLo=0.10/sepHi=0.30 与 depth_edge_check.js 同源） */
constexpr float kAdaptiveSepLo = 0.10f;
constexpr float kAdaptiveSepHi = 0.30f;

}  // namespace

int32_t SrBuildDepthMesh(const float *rawDepth, int dw, int dh,
                         const uint8_t *rgba, int rw, int rh,
                         float zNear, float zFar, int stride, int skirt,
                         int outW, int outH, SrDepthMesh &out,
                         const SrDepthMeshOpts &opts) {
    if (rawDepth == nullptr || rgba == nullptr || dw <= 0 || dh <= 0 || rw <= 0 || rh <= 0 ||
        stride <= 0 || zFar <= zNear || outW <= 0 || outH <= 0) {
        return -1;
    }
    const int w = dw;
    const int h = dh;
    out.w = outW;
    out.h = outH;
    out.zNear = zNear;
    out.zFar = zFar;

    // ① 稳健归一化 s ∈ [0,1]（1 = 最近）
    const size_t n = static_cast<size_t>(w) * h;
    Percentiles(rawDepth, n, out.p1, out.p99);
    const float span = out.p99 - out.p1;
    if (!(span > 1e-6f)) {
        SRDM_LOGI("depth invalid: p99-p1 = %{public}.6f", static_cast<double>(span));
        return -2;
    }
    std::vector<float> s(n);
    for (size_t i = 0; i < n; i++) s[i] = Clampf((rawDepth[i] - out.p1) / span, 0.0f, 1.0f);
    // 质量选项（坑 147 纪律：默认全关 = 旧行为；开=改深度场，喂给 Kit 的帧随之变化）
    // D4（edgeMode>=2）：保边预平滑——消细碎台阶（refinedQuads 下降），Δ 大的台阶不糊。
    const auto tEdge0 = std::chrono::steady_clock::now();
    if (opts.edgeMode >= 2) {
        s = BilateralSmoothS(s, w, h, opts.preSmoothR, opts.preSmoothSigmaR);
    }
    // D2：主体蒙版对齐——蒙版重采样到深度场分辨率后做两遍门控保边平滑（蒙版外逐字节不变）。
    bool subjectOn = false;
    if (opts.subjectMask != nullptr) {
        // 蒙版是标量场（1B/px，rw×rh 源分辨率）→ 双线性重采样到深度场分辨率（与 SampleS 同语义）
        const auto sampleMask = [&](float fx0, float fy0) -> float {
            const float fx = Clampf(fx0 - 0.5f, 0.0f, static_cast<float>(rw) - 1.001f);
            const float fy = Clampf(fy0 - 0.5f, 0.0f, static_cast<float>(rh) - 1.001f);
            const int x0 = static_cast<int>(fx), y0 = static_cast<int>(fy);
            const float tx = fx - x0, ty = fy - y0;
            const int x1 = std::min(rw - 1, x0 + 1), y1 = std::min(rh - 1, y0 + 1);
            const float v00 = static_cast<float>(opts.subjectMask[static_cast<size_t>(y0) * rw + x0]);
            const float v10 = static_cast<float>(opts.subjectMask[static_cast<size_t>(y0) * rw + x1]);
            const float v01 = static_cast<float>(opts.subjectMask[static_cast<size_t>(y1) * rw + x0]);
            const float v11 = static_cast<float>(opts.subjectMask[static_cast<size_t>(y1) * rw + x1]);
            return (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
        };
        std::vector<uint8_t> maskF(n, 0);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                maskF[static_cast<size_t>(y) * w + x] = static_cast<uint8_t>(Clampf(
                    sampleMask((static_cast<float>(x) + 0.5f) * rw / w,
                               (static_cast<float>(y) + 0.5f) * rh / h) + 0.5f, 0.0f, 255.0f));
            }
        }
        SubjectAlignS(s, w, h, maskF, opts.subjectSigmaR);
        subjectOn = true;
    }
    // 坑 133（2026-10-02）：s 的 p90 = "守护深度"分位（面积自适应）。近层充足的图 p90≈0.9+
    // （行为同旧 zNear 口径）；纵深贴枢轴的图（如整面墙）p90 落在实际内容带上，bridge 据此
    // 校正 Θ 钳制，作为位移带评估的保守口径（该守卫为稳健性改进，非已证必要）。
    // （2026-10-04 起：在 D4/D2 之后的最终低分辨场上取分位——关=旧行为时分位不变。）
    {
        float p10Dummy = 0.0f, p90 = 0.0f;
        Percentiles(s.data(), n, p10Dummy, p90);
        out.sP90 = p90;
    }
    const auto zOfS = [&](float sv) { return 1.0f / (sv / zNear + (1.0f - sv) / zFar); };

    // ② 前景贴图：源帧 RGBA 直接双线性重采样到**画布分辨率**（单步采样自全分辨率源图，
    //    与 scene=1..4 的逐像素 SrSampleImg 同款；不再经深度场分辨率中转，避免二次模糊）
    out.frontRgb.assign(static_cast<size_t>(outW) * outH * 3, 0);
    for (int y = 0; y < outH; y++) {
        for (int x = 0; x < outW; x++) {
            uint8_t col[3];
            SampleRgba(rgba, rw, rh,
                       (static_cast<float>(x) + 0.5f) * rw / outW,
                       (static_cast<float>(y) + 0.5f) * rh / outH, col);
            const size_t od = (static_cast<size_t>(y) * outW + x) * 3;
            out.frontRgb[od] = col[0];
            out.frontRgb[od + 1] = col[1];
            out.frontRgb[od + 2] = col[2];
        }
    }

    // ③ 前景层网格：粗格 + 台阶细分（1px 正面平行小片，四点同深）+ 裙边（采样钳制自动延展）
    //    角点与贴图坐标都在**画布空间**（贴图分辨率 = 画布分辨率 → UV 可直接用画布像素坐标）
    // D1（edgeMode>=1）：JBU 把 s 升到画布分辨率（引导图 = 画布分辨率源图）再建网格——
    //    网格角点密度不变（stride 随分辨率换算），深度边缘贴着颜色边缘走。
    std::vector<float> sCanvas;
    const std::vector<float> *sf = &s;
    int fw = w, fh = h, fstride = stride;
    if (opts.edgeMode >= 1) {
        sCanvas = JbuUpsampleS(s, w, h, out.frontRgb.data(), outW, outH, opts);
        sf = &sCanvas;
        fw = outW;
        fh = outH;
        fstride = std::max(1, static_cast<int>(std::lround(
            static_cast<float>(stride) * outW / static_cast<float>(w))));
    }
    // 场像素 → 画布像素（纯 2D 画布映射；关=旧行为时 fw==w，与旧 sx/sy 逐位一致）
    const float fsx = static_cast<float>(outW) / static_cast<float>(fw);
    const float fsy = static_cast<float>(outH) / static_cast<float>(fh);
    const int skirtF = std::max(1, static_cast<int>(std::lround(
        skirt * fw / static_cast<float>(outW))));
    if (opts.edgeMode >= 1 || subjectOn) {
        const auto ms = std::chrono::duration<float, std::milli>(
            std::chrono::steady_clock::now() - tEdge0).count();
        SRDM_LOGI("edge mode=%{public}d field=%{public}dx%{public}d stride=%{public}d subject=%{public}d ms=%{public}.0f",
                  opts.edgeMode, fw, fh, fstride, subjectOn ? 1 : 0, static_cast<double>(ms));
    }
    out.front.clear();
    const auto pushVertex = [&](SrDepthQuad &q, int k, float x, float y, float z) {
        const float cxv = x * fsx;
        const float cyv = y * fsy;
        q.x[k] = cxv;
        q.y[k] = cyv;
        q.z[k] = z;
        q.u[k] = cxv;   // 贴图坐标 = 角点画布像素坐标（越界由采样钳制 → 边缘延展）
        q.v[k] = cyv;
    };
    const auto emitFlatQuad = [&](float x0, float y0, float x1, float y1, float z) {
        SrDepthQuad q;
        pushVertex(q, 0, x0, y0, z);
        pushVertex(q, 1, x1, y0, z);
        pushVertex(q, 2, x1, y1, z);
        pushVertex(q, 3, x0, y1, z);
        out.front.push_back(q);
    };
    out.coarseQuads = 0;
    out.refinedQuads = 0;
    out.maxZRatio = 1.0f;
    for (int j = -skirtF; j + fstride <= fh + skirtF; j += fstride) {
        for (int i = -skirtF; i + fstride <= fw + skirtF; i += fstride) {
            const float zs[4] = {
                zOfS(SampleS(*sf, fw, fh, static_cast<float>(i), static_cast<float>(j))),
                zOfS(SampleS(*sf, fw, fh, static_cast<float>(i + fstride), static_cast<float>(j))),
                zOfS(SampleS(*sf, fw, fh, static_cast<float>(i + fstride), static_cast<float>(j + fstride))),
                zOfS(SampleS(*sf, fw, fh, static_cast<float>(i), static_cast<float>(j + fstride))),
            };
            const float zmin = std::min(std::min(zs[0], zs[1]), std::min(zs[2], zs[3]));
            const float zmax = std::max(std::max(zs[0], zs[1]), std::max(zs[2], zs[3]));
            const float ratio = zmax / std::max(1e-9f, zmin);
            if (ratio > kRefineRatio) {
                for (int yy = j; yy < j + fstride; yy++) {
                    for (int xx = i; xx < i + fstride; xx++) {
                        const float zc = zOfS(SampleS(*sf, fw, fh, xx + 0.5f, yy + 0.5f));
                        emitFlatQuad(static_cast<float>(xx), static_cast<float>(yy),
                                     static_cast<float>(xx + 1), static_cast<float>(yy + 1), zc);
                        out.refinedQuads++;
                    }
                }
            } else {
                SrDepthQuad q;
                pushVertex(q, 0, static_cast<float>(i), static_cast<float>(j), zs[0]);
                pushVertex(q, 1, static_cast<float>(i + fstride), static_cast<float>(j), zs[1]);
                pushVertex(q, 2, static_cast<float>(i + fstride), static_cast<float>(j + fstride), zs[2]);
                pushVertex(q, 3, static_cast<float>(i), static_cast<float>(j + fstride), zs[3]);
                out.front.push_back(q);
                out.coarseQuads++;
                out.maxZRatio = std::max(out.maxZRatio, ratio);
            }
        }
    }

    // ④ 背景层：近区（s > nearSplitS）push-pull 补洞（深度 + 颜色）→ 偏置 → 仅在膨胀范围内生成
    std::vector<uint8_t> nearMask(n, 0);
    size_t nearCnt = 0;
    for (size_t i = 0; i < n; i++) {
        if (s[i] > out.nearSplitS) {
            nearMask[i] = 1;
            nearCnt++;
        }
    }
    out.nearRatio = static_cast<float>(nearCnt) / static_cast<float>(n);
    std::vector<float> sFill = s;
    PushPullFill(sFill, 1, nearMask, w, h);
    std::vector<float> rgbF(n * 3);
    for (size_t i = 0; i < n; i++) {
        rgbF[i * 3] = static_cast<float>(out.frontRgb[i * 3]);
        rgbF[i * 3 + 1] = static_cast<float>(out.frontRgb[i * 3 + 1]);
        rgbF[i * 3 + 2] = static_cast<float>(out.frontRgb[i * 3 + 2]);
    }
    PushPullFill(rgbF, 3, nearMask, w, h);
    // 背景层贴图：补洞结果在深度场分辨率上计算 → 升采样到画布分辨率（内容为平滑填充场，
    // 双线性升采样与既有采样器同款；保证 UV 口径与前景层一致 = 画布像素坐标）
    std::vector<uint8_t> bgMeshRgb(n * 3, 0);
    for (size_t i = 0; i < n; i++) {
        bgMeshRgb[i * 3] = static_cast<uint8_t>(Clampf(rgbF[i * 3], 0.0f, 255.0f));
        bgMeshRgb[i * 3 + 1] = static_cast<uint8_t>(Clampf(rgbF[i * 3 + 1], 0.0f, 255.0f));
        bgMeshRgb[i * 3 + 2] = static_cast<uint8_t>(Clampf(rgbF[i * 3 + 2], 0.0f, 255.0f));
    }
    out.bgRgb.assign(static_cast<size_t>(outW) * outH * 3, 0);
    for (int y = 0; y < outH; y++) {
        for (int x = 0; x < outW; x++) {
            uint8_t col[3];
            SampleRgb(bgMeshRgb.data(), w, h,
                      (static_cast<float>(x) + 0.5f) * w / outW,
                      (static_cast<float>(y) + 0.5f) * h / outH, col);
            const size_t od = (static_cast<size_t>(y) * outW + x) * 3;
            out.bgRgb[od] = col[0];
            out.bgRgb[od + 1] = col[1];
            out.bgRgb[od + 2] = col[2];
        }
    }
    for (size_t i = 0; i < n; i++) sFill[i] *= (1.0f - 2e-3f);   // tie 破断：与前层重叠处让前层稳定胜出
    std::vector<uint8_t> emitMask;
    DilateMask(nearMask, w, h, kDilateRadius, emitMask);

    out.bg.clear();
    for (int j = 0; j + stride <= h; j += stride) {
        for (int i = 0; i + stride <= w; i += stride) {
            bool any = false;
            for (int yy = j; yy <= j + stride && !any; yy += stride) {
                for (int xx = i; xx <= i + stride && !any; xx += stride) {
                    if (emitMask[static_cast<size_t>(std::min(h - 1, yy)) * w + std::min(w - 1, xx)]) any = true;
                }
            }
            if (!any) continue;
            SrDepthQuad q;
            pushVertex(q, 0, static_cast<float>(i), static_cast<float>(j), zOfS(SampleS(sFill, w, h, static_cast<float>(i), static_cast<float>(j))));
            pushVertex(q, 1, static_cast<float>(i + stride), static_cast<float>(j), zOfS(SampleS(sFill, w, h, static_cast<float>(i + stride), static_cast<float>(j))));
            pushVertex(q, 2, static_cast<float>(i + stride), static_cast<float>(j + stride), zOfS(SampleS(sFill, w, h, static_cast<float>(i + stride), static_cast<float>(j + stride))));
            pushVertex(q, 3, static_cast<float>(i), static_cast<float>(j + stride), zOfS(SampleS(sFill, w, h, static_cast<float>(i), static_cast<float>(j + stride))));
            out.bg.push_back(q);
        }
    }

    SRDM_LOGI("mesh built depth=%{public}dx%{public}d canvas=%{public}dx%{public}d "
              "front=%{public}zu (coarse=%{public}d refined=%{public}d) "
              "bg=%{public}zu near=%{public}zu/%{public}zu sP90=%.3f nearRatio=%.3f p1=%.3f p99=%.3f maxZRatio=%.3f",
              w, h, outW, outH, out.front.size(), out.coarseQuads, out.refinedQuads, out.bg.size(),
              nearCnt, n, static_cast<double>(out.sP90), static_cast<double>(out.nearRatio),
              static_cast<double>(out.p1), static_cast<double>(out.p99),
              static_cast<double>(out.maxZRatio));
    return 0;
}

float SrAdaptiveDepthK(const float *rawDepth, int dw, int dh, float kMin, float kMax) {
    if (rawDepth == nullptr || dw <= 0 || dh <= 0 || !(kMax > kMin)) {
        return (kMin + kMax) * 0.5f;
    }
    const size_t n = static_cast<size_t>(dw) * dh;
    float p1 = 0.0f, p99 = 0.0f;
    Percentiles(rawDepth, n, p1, p99);
    const float span = p99 - p1;
    if (!(span > 1e-6f)) {
        return (kMin + kMax) * 0.5f;
    }
    // 64 桶直方图（归一化 s）→ Otsu 两分（两类各须 ≥5% 面积）→ 类平衡加权分离度 → k 映射
    constexpr int kB = 64;
    float hist[kB] = {0.0f};
    for (size_t i = 0; i < n; i++) {
        const float sv = Clampf((rawDepth[i] - p1) / span, 0.0f, 1.0f);
        hist[std::min(kB - 1, static_cast<int>(sv * kB))]++;
    }
    float sumAll = 0.0f;
    for (int b = 0; b < kB; b++) sumAll += hist[b] * (static_cast<float>(b) + 0.5f) / kB;
    const float n05 = 0.05f * static_cast<float>(n);
    float w0 = 0.0f, sum0 = 0.0f, bestV = -1.0f, bestT = 0.5f;
    for (int b = 0; b < kB - 1; b++) {
        w0 += hist[b];
        sum0 += hist[b] * (static_cast<float>(b) + 0.5f) / kB;
        const float w1 = static_cast<float>(n) - w0;
        if (w0 < n05 || w1 < n05) continue;
        const float m0 = sum0 / w0;
        const float m1 = (sumAll - sum0) / w1;
        const float v = w0 * w1 * (m0 - m1) * (m0 - m1);
        if (v > bestV) {
            bestV = v;
            bestT = static_cast<float>(b + 1) / kB;
        }
    }
    if (bestV < 0.0f) {
        return (kMin + kMax) * 0.5f;   // 无有效两分（近乎单值场）→ 中档
    }
    float a0 = 0.0f, c0 = 0.0f, a1 = 0.0f, c1 = 0.0f;
    for (int b = 0; b < kB; b++) {
        const float v = (static_cast<float>(b) + 0.5f) / kB;
        if (v < bestT) { a0 += hist[b] * v; c0 += hist[b]; } else { a1 += hist[b] * v; c1 += hist[b]; }
    }
    const float sep = std::fabs(a1 / std::max(1.0f, c1) - a0 / std::max(1.0f, c0)) *
                      (2.0f * c0 * c1) / (static_cast<float>(n) * static_cast<float>(n));
    const float t = Clampf((sep - kAdaptiveSepLo) / (kAdaptiveSepHi - kAdaptiveSepLo), 0.0f, 1.0f);
    return kMin + (kMax - kMin) * t;
}

size_t SrRasterizeQuads(const SrScreenVert *verts, size_t quadCount, int outW, int outH,
                        const uint8_t *tex, int texW, int texH,
                        uint8_t *dstRgb, float *zbuf, uint8_t *label, uint8_t layerId) {
    if (verts == nullptr || dstRgb == nullptr || zbuf == nullptr || quadCount == 0) return 0;
    size_t written = 0;
    for (size_t q = 0; q < quadCount; q++) {
        const SrScreenVert *v = verts + q * 4;
        RasterTri(v[0], v[1], v[2], outW, outH, tex, texW, texH, dstRgb, zbuf, label, layerId, written);
        RasterTri(v[0], v[2], v[3], outW, outH, tex, texW, texH, dstRgb, zbuf, label, layerId, written);
    }
    return written;
}

void SrMakeBlurRgb(const uint8_t *rgb, int w, int h, int factor, std::vector<uint8_t> &out,
                   int &bw, int &bh) {
    if (rgb == nullptr || w <= 0 || h <= 0 || factor <= 0) return;
    bw = std::max(1, (w + factor - 1) / factor);
    bh = std::max(1, (h + factor - 1) / factor);
    out.assign(static_cast<size_t>(bw) * bh * 3, 0);
    for (int by = 0; by < bh; by++) {
        for (int bx = 0; bx < bw; bx++) {
            uint32_t acc[3] = {0, 0, 0};
            uint32_t cnt = 0;
            for (int y = by * factor; y < std::min(h, (by + 1) * factor); y++) {
                for (int x = bx * factor; x < std::min(w, (bx + 1) * factor); x++) {
                    const size_t o = (static_cast<size_t>(y) * w + x) * 3;
                    acc[0] += rgb[o];
                    acc[1] += rgb[o + 1];
                    acc[2] += rgb[o + 2];
                    cnt++;
                }
            }
            const size_t od = (static_cast<size_t>(by) * bw + bx) * 3;
            if (cnt > 0) {
                out[od] = static_cast<uint8_t>(acc[0] / cnt);
                out[od + 1] = static_cast<uint8_t>(acc[1] / cnt);
                out[od + 2] = static_cast<uint8_t>(acc[2] / cnt);
            }
        }
    }
}

char SrDepthAsciiLevel(float sv) {
    static const char *kLevels = " .:-=+*#%@";
    int lv = static_cast<int>(Clampf(sv, 0.0f, 1.0f) * 9.0f + 0.5f);
    lv = lv < 0 ? 0 : (lv > 9 ? 9 : lv);
    return kLevels[lv];
}

}  // namespace glassvideo
