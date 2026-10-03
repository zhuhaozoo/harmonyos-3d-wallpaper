/**
 * 3D壁纸 · 分层深度场景的素材构建（scene=3 专用）
 *
 * 目标：把「源图 + 主体蒙版」拆成两层贴图，供 spatial_recon_bridge.cpp 合成多视角帧：
 * - 主体层：源图本身 + 蒙版 alpha（按深度比 k 前移到更近的平面 → 与背景层产生真实视差）；
 * - 背景层：源图，但**主体区域用周围背景平滑外推补洞**（主体层移开后露出的不能是主体本身）；
 *   替换范围只限"强主体像素"（概率趋饱和处），其余像素与原图逐像素相同 —— 这是为了保住
 *   "参考位姿处分层合成画面 == 原照片"这条硬约束（推导见 .cpp 出图段的注释）。
 *
 * 本单元只做 2D 图像数学（蒙版加权统计 + 加权金字塔 push-pull），**不涉及相机与射线**——
 * 射线/位姿公式的唯一来源仍是 spatial_recon_bridge.cpp（见该文件头注释与坑 119/120）。
 *
 * 补洞算法（为什么是 push-pull 而不是模糊/镜像）：
 * - 直接模糊会把主体颜色糊进洞里（深色主体在洞边留下一圈暗晕）；
 * - push-pull（Gortler 式）让主体像素**权重为 0、不参与统计**，洞的颜色只由周围背景逐级外推，
 *   且天然无残留空洞。补洞发生在主体之下（只在主体移开后被看见），是全图唯一被合成的区域。
 */

#ifndef GLASSVIDEO_SPATIAL_RECON_LAYERS_H
#define GLASSVIDEO_SPATIAL_RECON_LAYERS_H

#include <cstdint>
#include <vector>

namespace glassvideo {

/**
 * 蒙版原始值（0~255，官方 mattingList 语义：0=背景 255=主体）→ 主体概率 [0,1]。
 * 低概率噪点清零、高概率饱和：AI 蒙版在背景处常有 1~16 的低概率散点，
 * 若原样使用，这些像素会被抬到主体平面，在轨道端点处表现为一层几乎不可见、
 * 但会随相机移动的"鬼影"。
 */
inline float SrMaskAlpha(uint8_t m)
{
    constexpr int32_t kFloor = 16;    /**< ≤16 判为背景（噪声） */
    constexpr int32_t kCeil = 240;    /**< ≥240 判为主体（饱和） */
    if (m <= kFloor) {
        return 0.0f;
    }
    if (m >= kCeil) {
        return 1.0f;
    }
    return static_cast<float>(m - kFloor) / static_cast<float>(kCeil - kFloor);
}

/** 分层素材（scene=3）：背景层 + 主体像素统计 */
struct SrLayerAssets {
    std::vector<uint8_t> background;   /**< srcW*srcH*3：主体区域已补洞的背景（RGB） */
    int32_t subjectPx = 0;             /**< 主体像素数（蒙版概率 ≥0.5 的计数） */
    int32_t srcW = 0;                  /**< 与源图同尺寸（px） */
    int32_t srcH = 0;
};

/**
 * 构建分层素材。
 * @param srcRgba 源图 RGBA_8888（srcW*srcH*4）
 * @param mask    主体蒙版（srcW*srcH，0~255；见 SrMaskAlpha）
 * @param out     成功时回填背景层与统计
 * @return true=可用；false=蒙版退化（无主体 / 几乎全为主体 / 尺寸不符），调用方应退回单层场景
 */
bool SrBuildLayerAssets(const uint8_t *srcRgba, int32_t srcW, int32_t srcH,
                        const uint8_t *mask, SrLayerAssets &out);

} // namespace glassvideo

#endif // GLASSVIDEO_SPATIAL_RECON_LAYERS_H
