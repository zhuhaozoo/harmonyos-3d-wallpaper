/**
 * 3D壁纸 · 分层深度场景的素材构建（实现）。设计说明见 spatial_recon_layers.h。
 */
#include "spatial_recon_layers.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "vg_common.h"

namespace glassvideo {

namespace {

/** 补洞粗层分辨率分母：补洞是低频场，1/4 足够（省内存与耗时），出图前双线性升采样回全分辨率 */
constexpr int32_t kFillDown = 4;

/** 主体占比的可用区间（超出即判蒙版退化，退回单层场景） */
constexpr float kMinSubjectRatio = 0.005f;   /**< <0.5%：没识别到主体 */
// >35%：蒙版把"大半个画面"都当主体。分层时那等于在照片前面 0.17~0.68 世界单位处悬一块
// 占三分之一以上画面的浮空板，成片里会看到"内容被拉伸/错位"（2026-10-01 真机：笔记本照片
// 主体占比 39.1%，反馈"键盘在上屏幕在下、比例被拉伸得非常严重"），且这种"主体"分层也无意义。
constexpr float kMaxSubjectRatio = 0.35f;

/** 加权金字塔的一层：color 为 w*h*3 的 float，weight 为 w*h 的可信度 [0,1]（1=全部由真实背景统计而来） */
struct SrPyramidLevel {
    int32_t w = 0;
    int32_t h = 0;
    std::vector<float> color;
    std::vector<float> weight;
};

/** 双线性采样 float RGB 场（坐标以像素中心为准，出界钳制到边缘） */
void SrSampleColorField(const std::vector<float> &color, int32_t w, int32_t h,
                        float x, float y, float out[3])
{
    if (x < 0.0f) {
        x = 0.0f;
    }
    if (y < 0.0f) {
        y = 0.0f;
    }
    if (x > static_cast<float>(w - 1)) {
        x = static_cast<float>(w - 1);
    }
    if (y > static_cast<float>(h - 1)) {
        y = static_cast<float>(h - 1);
    }
    const int32_t x0 = static_cast<int32_t>(x);
    const int32_t y0 = static_cast<int32_t>(y);
    const int32_t x1 = std::min(x0 + 1, w - 1);
    const int32_t y1 = std::min(y0 + 1, h - 1);
    const float ax = x - static_cast<float>(x0);
    const float ay = y - static_cast<float>(y0);
    for (int32_t c = 0; c < 3; c++) {
        const float p00 = color[(static_cast<size_t>(y0) * w + x0) * 3 + c];
        const float p10 = color[(static_cast<size_t>(y0) * w + x1) * 3 + c];
        const float p01 = color[(static_cast<size_t>(y1) * w + x0) * 3 + c];
        const float p11 = color[(static_cast<size_t>(y1) * w + x1) * 3 + c];
        const float top = p00 + (p10 - p00) * ax;
        const float bot = p01 + (p11 - p01) * ax;
        out[c] = top + (bot - top) * ay;
    }
}

} // namespace

bool SrBuildLayerAssets(const uint8_t *srcRgba, int32_t srcW, int32_t srcH,
                        const uint8_t *mask, SrLayerAssets &out)
{
    out.background.clear();
    out.subjectPx = 0;
    out.srcW = srcW;
    out.srcH = srcH;
    if (srcRgba == nullptr || mask == nullptr || srcW <= 0 || srcH <= 0) {
        return false;
    }
    const int64_t total = static_cast<int64_t>(srcW) * srcH;
    int64_t subj = 0;
    for (int64_t i = 0; i < total; i++) {
        if (SrMaskAlpha(mask[i]) >= 0.5f) {
            subj++;
        }
    }
    const float ratio = static_cast<float>(subj) / static_cast<float>(total);
    if (ratio < kMinSubjectRatio || ratio > kMaxSubjectRatio) {
        VG_LOGW("SRLAYER mask degenerate subj=%{public}lld/%{public}lld ratio=%{public}f",
                static_cast<long long>(subj), static_cast<long long>(total), ratio);
        return false;
    }
    out.subjectPx = static_cast<int32_t>(subj);

    // ---- 第 0 层（1/kFillDown 分辨率）：每个块做蒙版加权的背景统计，主体像素权重 0、不参与 ----
    const int32_t mw = (srcW + kFillDown - 1) / kFillDown;
    const int32_t mh = (srcH + kFillDown - 1) / kFillDown;
    const float blockCap = static_cast<float>(kFillDown * kFillDown);
    SrPyramidLevel base;
    base.w = mw;
    base.h = mh;
    base.color.assign(static_cast<size_t>(mw) * mh * 3, 0.0f);
    base.weight.assign(static_cast<size_t>(mw) * mh, 0.0f);
    double globalR = 0.0;
    double globalG = 0.0;
    double globalB = 0.0;
    double globalW = 0.0;
    for (int32_t by = 0; by < mh; by++) {
        const int32_t yEnd = std::min(srcH, (by + 1) * kFillDown);
        for (int32_t bx = 0; bx < mw; bx++) {
            const int32_t xEnd = std::min(srcW, (bx + 1) * kFillDown);
            double accR = 0.0;
            double accG = 0.0;
            double accB = 0.0;
            double accW = 0.0;
            for (int32_t y = by * kFillDown; y < yEnd; y++) {
                const size_t row = static_cast<size_t>(y) * srcW;
                for (int32_t x = bx * kFillDown; x < xEnd; x++) {
                    const size_t o = row + x;
                    const double w = 1.0 - static_cast<double>(SrMaskAlpha(mask[o]));
                    if (w <= 0.0) {
                        continue;
                    }
                    accR += w * srcRgba[o * 4];
                    accG += w * srcRgba[o * 4 + 1];
                    accB += w * srcRgba[o * 4 + 2];
                    accW += w;
                }
            }
            const size_t t = static_cast<size_t>(by) * mw + bx;
            base.weight[t] = static_cast<float>(std::min(1.0, accW / blockCap));
            if (accW > 1e-6) {
                base.color[t * 3] = static_cast<float>(accR / accW);
                base.color[t * 3 + 1] = static_cast<float>(accG / accW);
                base.color[t * 3 + 2] = static_cast<float>(accB / accW);
            }
            globalR += accR;
            globalG += accG;
            globalB += accB;
            globalW += accW;
        }
    }
    if (globalW < 1e-6) {
        // 占比检查已挡住"全主体"，此处仅防御异常输入
        return false;
    }
    const float meanR = static_cast<float>(globalR / globalW);
    const float meanG = static_cast<float>(globalG / globalW);
    const float meanB = static_cast<float>(globalB / globalW);

    // ---- pull：逐级 2x2 合并（颜色按权重加权；权重是"该层有多少来自真实背景"的归一化可信度） ----
    std::vector<SrPyramidLevel> levels;
    levels.push_back(std::move(base));
    while (levels.back().w > 1 || levels.back().h > 1) {
        const SrPyramidLevel &fine = levels.back();
        SrPyramidLevel coarse;
        coarse.w = (fine.w + 1) / 2;
        coarse.h = (fine.h + 1) / 2;
        coarse.color.assign(static_cast<size_t>(coarse.w) * coarse.h * 3, 0.0f);
        coarse.weight.assign(static_cast<size_t>(coarse.w) * coarse.h, 0.0f);
        for (int32_t y = 0; y < coarse.h; y++) {
            for (int32_t x = 0; x < coarse.w; x++) {
                double accR = 0.0;
                double accG = 0.0;
                double accB = 0.0;
                double accW = 0.0;
                for (int32_t dy = 0; dy < 2; dy++) {
                    const int32_t sy = y * 2 + dy;
                    if (sy >= fine.h) {
                        continue;
                    }
                    for (int32_t dx = 0; dx < 2; dx++) {
                        const int32_t sx = x * 2 + dx;
                        if (sx >= fine.w) {
                            continue;
                        }
                        const size_t s = static_cast<size_t>(sy) * fine.w + sx;
                        const double w = fine.weight[s];
                        if (w <= 0.0) {
                            continue;
                        }
                        accR += w * fine.color[s * 3];
                        accG += w * fine.color[s * 3 + 1];
                        accB += w * fine.color[s * 3 + 2];
                        accW += w;
                    }
                }
                const size_t t = static_cast<size_t>(y) * coarse.w + x;
                coarse.weight[t] = static_cast<float>(std::min(1.0, accW / 4.0));
                if (accW > 1e-6) {
                    coarse.color[t * 3] = static_cast<float>(accR / accW);
                    coarse.color[t * 3 + 1] = static_cast<float>(accG / accW);
                    coarse.color[t * 3 + 2] = static_cast<float>(accB / accW);
                }
            }
        }
        levels.push_back(std::move(coarse));
    }

    // ---- push：自顶向下回填。顶层若仍有洞用全局均值兜底；其余层缺失的可信度由父层颜色补 ----
    {
        SrPyramidLevel &top = levels.back();
        for (size_t t = 0; t < top.weight.size(); t++) {
            const float w = top.weight[t];
            if (w < 1.0f) {
                top.color[t * 3] = static_cast<float>(w) * top.color[t * 3] + (1.0f - w) * meanR;
                top.color[t * 3 + 1] = static_cast<float>(w) * top.color[t * 3 + 1] + (1.0f - w) * meanG;
                top.color[t * 3 + 2] = static_cast<float>(w) * top.color[t * 3 + 2] + (1.0f - w) * meanB;
                top.weight[t] = 1.0f;
            }
        }
    }
    for (int32_t k = static_cast<int32_t>(levels.size()) - 2; k >= 0; k--) {
        SrPyramidLevel &fine = levels[static_cast<size_t>(k)];
        const SrPyramidLevel &coarse = levels[static_cast<size_t>(k) + 1];
        for (int32_t y = 0; y < fine.h; y++) {
            for (int32_t x = 0; x < fine.w; x++) {
                const size_t t = static_cast<size_t>(y) * fine.w + x;
                const float w = fine.weight[t];
                if (w >= 1.0f) {
                    continue;
                }
                // 子块中心在父层的坐标（父层每个纹素对应子层 2x2）
                const float px = (static_cast<float>(x) + 0.5f) * 0.5f - 0.5f;
                const float py = (static_cast<float>(y) + 0.5f) * 0.5f - 0.5f;
                float up[3] = { 0.0f, 0.0f, 0.0f };
                SrSampleColorField(coarse.color, coarse.w, coarse.h, px, py, up);
                fine.color[t * 3] = w * fine.color[t * 3] + (1.0f - w) * up[0];
                fine.color[t * 3 + 1] = w * fine.color[t * 3 + 1] + (1.0f - w) * up[1];
                fine.color[t * 3 + 2] = w * fine.color[t * 3 + 2] + (1.0f - w) * up[2];
                fine.weight[t] = 1.0f;
            }
        }
    }

    // ---- 出图：仅"强主体像素"用补洞色替换，其余逐像素等于原图 ----
    // 阈值取高（≈0.96）而不是 0.5，是为了一条硬约束：**参考位姿处分层合成画面 == 原照片**
    // （主体层与背景层在参考位姿投影重合，合成 = m·源图 + (1-m)·背景层）。若把"弱概率像素"
    // 也换成补洞色，轮廓处会出现一圈静态光晕（(1-m)·|源图-补洞| 在每一帧都可见）；
    // 反之只替换强主体像素时，软边（m ∈ [0.5,0.96)）在主体移开后仅留下 1~2px 的极细残影，
    // 肉眼不可见。代价与收益不对称，故取高阈值。
    constexpr int32_t kFillThreshold = 245;   // = SrMaskAlpha 的饱和点附近（240）再留一点余量
    const SrPyramidLevel &filled = levels.front();
    out.background.resize(static_cast<size_t>(total) * 3);
    const float invDown = 1.0f / static_cast<float>(kFillDown);
    uint8_t *dst = out.background.data();
    for (int32_t y = 0; y < srcH; y++) {
        const size_t row = static_cast<size_t>(y) * srcW;
        for (int32_t x = 0; x < srcW; x++) {
            const size_t o = row + x;
            uint8_t *d = dst + o * 3;
            if (mask[o] < kFillThreshold) {
                d[0] = srcRgba[o * 4];
                d[1] = srcRgba[o * 4 + 1];
                d[2] = srcRgba[o * 4 + 2];
                continue;
            }
            float c[3] = { 0.0f, 0.0f, 0.0f };
            SrSampleColorField(filled.color, filled.w, filled.h,
                               (static_cast<float>(x) + 0.5f) * invDown - 0.5f,
                               (static_cast<float>(y) + 0.5f) * invDown - 0.5f, c);
            for (int32_t ch = 0; ch < 3; ch++) {
                float v = c[ch];
                if (v < 0.0f) {
                    v = 0.0f;
                }
                if (v > 255.0f) {
                    v = 255.0f;
                }
                d[ch] = static_cast<uint8_t>(v + 0.5f);
            }
        }
    }
    VG_LOGI("SRLAYER bg built %{public}dx%{public}d subj=%{public}d ratio=%{public}f levels=%{public}zu",
            srcW, srcH, out.subjectPx, ratio, levels.size());
    return true;
}

} // namespace glassvideo
