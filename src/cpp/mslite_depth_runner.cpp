/**
 * 3D壁纸 · 路线B 阶段1：端侧深度推理实现（MindSpore Lite Native API）
 * 见 include/mslite_depth_runner.h 的契约说明。
 */
#include "mslite_depth_runner.h"

#include <hilog/log.h>
#include <mindspore/context.h>
#include <mindspore/format.h>
#include <mindspore/model.h>
#include <mindspore/status.h>
#include <mindspore/tensor.h>
#include <mindspore/types.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

#undef LOG_DOMAIN
#undef LOG_TAG
#define LOG_DOMAIN 0x3200
#define LOG_TAG "SRD"
#define SRD_LOGI(...) ((void)OH_LOG_Print(LOG_APP, LOG_INFO, LOG_DOMAIN, LOG_TAG, __VA_ARGS__))
#define SRD_LOGW(...) ((void)OH_LOG_Print(LOG_APP, LOG_WARN, LOG_DOMAIN, LOG_TAG, __VA_ARGS__))
#define SRD_LOGE(...) ((void)OH_LOG_Print(LOG_APP, LOG_ERROR, LOG_DOMAIN, LOG_TAG, __VA_ARGS__))

namespace glassdepth {
namespace {

constexpr int32_t kDefaultInW = 546;   // 14 的倍数、恰 3:4（与离线判据一致；模型输入以张量实测为准）
constexpr int32_t kDefaultInH = 728;
const float kMean[3] = {0.485f, 0.456f, 0.406f};   // 与离线判据同源，勿改
const float kStd[3] = {0.229f, 0.224f, 0.225f};

/** 缓存的模型（自检/后续管线复用；模型不变则只加载一次） */
struct ModelCache {
    std::mutex mtx;
    OH_AI_ModelHandle model = nullptr;
    OH_AI_ContextHandle ctx = nullptr;
    std::string key;          // 模型标识（路径或 rawfile 名 + 大小）
    bool built = false;
};
ModelCache g_cache;

double NowMs() {
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

/** RGBA(w×h) → 双线性缩放到 (dw×dh) 并归一化为 [0,1]/ImageNet 标准化的浮点 RGB（交错存 tmpRgb） */
void ResizeNormalize(const uint8_t *rgba, int32_t w, int32_t h, int32_t dw, int32_t dh,
                     std::vector<float> &tmpRgb) {
    tmpRgb.resize(static_cast<size_t>(dw) * dh * 3);
    const float sx = static_cast<float>(w) / dw;
    const float sy = static_cast<float>(h) / dh;
    for (int32_t y = 0; y < dh; y++) {
        float fy = (y + 0.5f) * sy - 0.5f;
        fy = std::max(0.0f, std::min(static_cast<float>(h - 1), fy));
        const int32_t y0 = static_cast<int32_t>(fy);
        const int32_t y1 = std::min(h - 1, y0 + 1);
        const float ty = fy - y0;
        for (int32_t x = 0; x < dw; x++) {
            float fx = (x + 0.5f) * sx - 0.5f;
            fx = std::max(0.0f, std::min(static_cast<float>(w - 1), fx));
            const int32_t x0 = static_cast<int32_t>(fx);
            const int32_t x1 = std::min(w - 1, x0 + 1);
            const float tx = fx - x0;
            const size_t o00 = (static_cast<size_t>(y0) * w + x0) * 4;
            const size_t o10 = (static_cast<size_t>(y0) * w + x1) * 4;
            const size_t o01 = (static_cast<size_t>(y1) * w + x0) * 4;
            const size_t o11 = (static_cast<size_t>(y1) * w + x1) * 4;
            const size_t od = (static_cast<size_t>(y) * dw + x) * 3;
            for (int c = 0; c < 3; c++) {
                const float v00 = rgba[o00 + c], v10 = rgba[o10 + c];
                const float v01 = rgba[o01 + c], v11 = rgba[o11 + c];
                const float v = (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
                tmpRgb[od + c] = (v / 255.0f - kMean[c]) / kStd[c];
            }
        }
    }
}

/** 把交错 RGB(HWC) 按布局写进输入张量缓冲 */
void FillTensor(const std::vector<float> &rgbHwc, int32_t dw, int32_t dh, int32_t cw, int32_t ch,
                bool nhwc, float *dst) {
    const size_t plane = static_cast<size_t>(cw) * ch;
    for (int32_t y = 0; y < ch; y++) {
        for (int32_t x = 0; x < cw; x++) {
            size_t src;
            if (cw == dw && ch == dh) {
                src = (static_cast<size_t>(y) * dw + x) * 3;
            } else {   // 张量实际尺寸与缩放目标不一致（例如模型要求别处尺寸）→ 最近邻兜底
                const int32_t sx = std::min(dw - 1, x * dw / cw);
                const int32_t sy = std::min(dh - 1, y * dh / ch);
                src = (static_cast<size_t>(sy) * dw + sx) * 3;
            }
            if (nhwc) {
                const size_t d = (static_cast<size_t>(y) * cw + x) * 3;
                dst[d] = rgbHwc[src];
                dst[d + 1] = rgbHwc[src + 1];
                dst[d + 2] = rgbHwc[src + 2];
            } else {
                const size_t d = static_cast<size_t>(y) * cw + x;
                dst[d] = rgbHwc[src];
                dst[plane + d] = rgbHwc[src + 1];
                dst[2 * plane + d] = rgbHwc[src + 2];
            }
        }
    }
}

/** 读张量形状到固定数组（返回维数） */
int32_t TensorDims(OH_AI_TensorHandle t, int64_t *dims, size_t cap) {
    size_t n = 0;
    const int64_t *s = OH_AI_TensorGetShape(t, &n);
    if (s == nullptr) return 0;
    const int32_t cnt = static_cast<int32_t>(std::min(n, cap));
    for (int32_t i = 0; i < cnt; i++) dims[i] = s[i];
    return cnt;
}

std::string DimsToStr(const int64_t *dims, int32_t n) {
    std::string s = "[";
    for (int32_t i = 0; i < n; i++) { s += std::to_string(dims[i]); if (i + 1 < n) s += ","; }
    s += "]";
    return s;
}

/** 构建/复用模型；err 带失败原因 */
OH_AI_ModelHandle EnsureModel(const void *data, size_t size, const std::string &path,
                             const std::string &key, std::string &err, double &buildMs) {
    std::lock_guard<std::mutex> lk(g_cache.mtx);
    if (g_cache.built && g_cache.model != nullptr && g_cache.key == key) {
        buildMs = 0.0;
        return g_cache.model;
    }
    if (g_cache.model != nullptr) {
        OH_AI_ModelDestroy(&g_cache.model);
        g_cache.model = nullptr;
    }
    if (g_cache.ctx != nullptr) {
        OH_AI_ContextDestroy(&g_cache.ctx);
        g_cache.ctx = nullptr;
    }
    const double t0 = NowMs();
    OH_AI_ContextHandle ctx = OH_AI_ContextCreate();
    if (ctx == nullptr) { err = "ContextCreate failed"; return nullptr; }
    OH_AI_ContextSetThreadNum(ctx, 4);
    OH_AI_DeviceInfoHandle dev = OH_AI_DeviceInfoCreate(OH_AI_DEVICETYPE_CPU);
    if (dev == nullptr) { OH_AI_ContextDestroy(&ctx); err = "DeviceInfoCreate failed"; return nullptr; }
    OH_AI_ContextAddDeviceInfo(ctx, dev);
    OH_AI_ModelHandle model = OH_AI_ModelCreate();
    if (model == nullptr) { OH_AI_ContextDestroy(&ctx); err = "ModelCreate failed"; return nullptr; }
    OH_AI_Status st;
    if (data != nullptr && size > 0) {
        st = OH_AI_ModelBuild(model, data, size, OH_AI_MODELTYPE_MINDIR, ctx);
    } else {
        st = OH_AI_ModelBuildFromFile(model, path.c_str(), OH_AI_MODELTYPE_MINDIR, ctx);
    }
    if (st != OH_AI_STATUS_SUCCESS) {
        OH_AI_ModelDestroy(&model);
        OH_AI_ContextDestroy(&ctx);
        err = "ModelBuild failed, status=" + std::to_string(static_cast<int>(st));
        return nullptr;
    }
    g_cache.ctx = ctx;
    g_cache.model = model;
    g_cache.key = key;
    g_cache.built = true;
    buildMs = NowMs() - t0;
    SRD_LOGI("model built in %.0f ms (key=%{public}s)", buildMs, key.c_str());
    return model;
}

}  // namespace

int32_t DepthLibraryProbe(std::string &info) {
    OH_AI_ContextHandle ctx = OH_AI_ContextCreate();
    if (ctx == nullptr) { info = "ContextCreate failed"; return -1; }
    OH_AI_ContextSetThreadNum(ctx, 2);
    OH_AI_DeviceInfoHandle dev = OH_AI_DeviceInfoCreate(OH_AI_DEVICETYPE_CPU);
    if (dev == nullptr) { OH_AI_ContextDestroy(&ctx); info = "DeviceInfoCreate failed"; return -1; }
    OH_AI_ContextAddDeviceInfo(ctx, dev);
    OH_AI_ContextDestroy(&ctx);
    info = "mindspore-lite ok, device=CPU";
    return 0;
}

int32_t DepthSelfTest(const uint8_t *rgba, int32_t w, int32_t h,
                      const void *modelData, size_t modelSize,
                      const std::string &modelFilePath,
                      const std::string &outPrefix,
                      DepthSelfTestResult &out) {
    if (rgba == nullptr || w <= 0 || h <= 0) {
        out.code = -4;
        out.note = "invalid rgba input";
        return out.code;
    }
    const std::string key = (modelData != nullptr)
        ? (std::string("buffer:") + std::to_string(modelSize))
        : (std::string("file:") + modelFilePath);
    std::string err;
    OH_AI_ModelHandle model = EnsureModel(modelData, modelSize, modelFilePath, key, err, out.buildMs);
    if (model == nullptr) {
        out.code = -1;
        out.note = err;
        SRD_LOGE("depth model build failed: %{public}s", err.c_str());
        return out.code;
    }

    // ---- 输入张量：先看实测布局/形状，再决定是否 resize ----
    OH_AI_TensorHandleArray inputs = OH_AI_ModelGetInputs(model);
    if (inputs.handle_num < 1) { out.code = -4; out.note = "no input tensor"; return out.code; }
    OH_AI_TensorHandle in0 = inputs.handle_list[0];
    const OH_AI_Format fmt = OH_AI_TensorGetFormat(in0);
    int64_t dims[8] = {0};
    int32_t nd = TensorDims(in0, dims, 8);
    out.layout = (fmt == OH_AI_FORMAT_NHWC) ? "NHWC" : (fmt == OH_AI_FORMAT_NCHW ? "NCHW" : "UNKNOWN");
    SRD_LOGI("input tensor name=%{public}s shape=%{public}s fmt=%{public}d dataSize=%{public}zu",
             OH_AI_TensorGetName(in0), DimsToStr(dims, nd).c_str(), static_cast<int>(fmt),
             OH_AI_TensorGetDataSize(in0));

    // 目标尺寸：优先沿用张量里已有的具体值（若为动态则用默认 546×728）
    int32_t inW = kDefaultInW, inH = kDefaultInH;
    const bool nhwc = (fmt == OH_AI_FORMAT_NHWC);
    if (nd == 4) {
        const int32_t wi = nhwc ? 2 : 3;
        const int32_t hi = nhwc ? 1 : 2;
        if (dims[wi] > 0) inW = static_cast<int32_t>(dims[wi]);
        if (dims[hi] > 0) inH = static_cast<int32_t>(dims[hi]);
    }
    if (inW % 14 != 0 || inH % 14 != 0 || inW < 14 || inH < 14) {   // 动态 dim 时会是 -1/0
        inW = kDefaultInW;
        inH = kDefaultInH;
    }
    out.inW = inW;
    out.inH = inH;

    const bool needResize = !(nd == 4 && dims[nhwc ? 1 : 2] > 0 && dims[nhwc ? 2 : 3] > 0);
    if (needResize) {
        OH_AI_ShapeInfo si;
        memset(&si, 0, sizeof(si));
        si.shape_num = 4;
        if (nhwc) {
            si.shape[0] = 1; si.shape[1] = inH; si.shape[2] = inW; si.shape[3] = 3;
        } else {
            si.shape[0] = 1; si.shape[1] = 3; si.shape[2] = inH; si.shape[3] = inW;
        }
        const OH_AI_Status rst = OH_AI_ModelResize(model, inputs, &si, 1);
        if (rst != OH_AI_STATUS_SUCCESS) {
            out.code = -2;
            out.note = "ModelResize failed, status=" + std::to_string(static_cast<int>(rst));
            SRD_LOGE("resize failed: %{public}s", out.note.c_str());
            return out.code;
        }
        inputs = OH_AI_ModelGetInputs(model);   // resize 后重新取
        in0 = inputs.handle_list[0];
        int64_t d2[8] = {0};
        const int32_t nd2 = TensorDims(in0, d2, 8);
        SRD_LOGI("after resize: shape=%{public}s dataSize=%{public}zu",
                 DimsToStr(d2, nd2).c_str(), OH_AI_TensorGetDataSize(in0));
    }

    // ---- 预处理 + 填充 ----
    std::vector<float> tmpRgb;
    ResizeNormalize(rgba, w, h, inW, inH, tmpRgb);
    float *inBuf = static_cast<float *>(OH_AI_TensorGetMutableData(in0));
    if (inBuf == nullptr) { out.code = -4; out.note = "input tensor data null"; return out.code; }
    const size_t need = static_cast<size_t>(inW) * inH * 3;
    if (OH_AI_TensorGetDataSize(in0) < need * sizeof(float)) {
        out.code = -4;
        out.note = "input tensor too small: " + std::to_string(OH_AI_TensorGetDataSize(in0));
        return out.code;
    }
    FillTensor(tmpRgb, inW, inH, inW, inH, nhwc, inBuf);

    // ---- 推理 ----
    const double t0 = NowMs();
    OH_AI_TensorHandleArray outputs = OH_AI_ModelGetOutputs(model);
    const OH_AI_Status pst = OH_AI_ModelPredict(model, inputs, &outputs, nullptr, nullptr);
    out.inferMs = NowMs() - t0;
    if (pst != OH_AI_STATUS_SUCCESS) {
        out.code = -3;
        out.note = "ModelPredict failed, status=" + std::to_string(static_cast<int>(pst));
        SRD_LOGE("predict failed: %{public}s", out.note.c_str());
        return out.code;
    }

    // ---- 读输出（相对逆深度，越大越近）----
    if (outputs.handle_num < 1) { out.code = -4; out.note = "no output tensor"; return out.code; }
    OH_AI_TensorHandle o0 = outputs.handle_list[0];
    int64_t od[8] = {0};
    const int32_t ond = TensorDims(o0, od, 8);
    if (ond < 2) { out.code = -4; out.note = "output rank < 2"; return out.code; }
    out.outH = static_cast<int32_t>(od[ond - 2]);
    out.outW = static_cast<int32_t>(od[ond - 1]);
    const float *odata = static_cast<const float *>(OH_AI_TensorGetData(o0));
    const int64_t oelems = OH_AI_TensorGetElementNum(o0);
    if (odata == nullptr || oelems <= 0 || out.outW <= 0 || out.outH <= 0) {
        out.code = -4;
        out.note = "output tensor invalid: shape=" + DimsToStr(od, ond);
        return out.code;
    }
    float mn = odata[0], mx = odata[0];
    double sum = 0.0;
    for (int64_t i = 0; i < oelems; i++) {
        const float v = odata[i];
        if (v < mn) mn = v;
        if (v > mx) mx = v;
        sum += v;
    }
    out.minV = mn;
    out.maxV = mx;
    out.mean = static_cast<float>(sum / static_cast<double>(oelems));

    // ---- 落盘（bin = 原始相对逆深度 float32，行优先；meta = 统计与张量描述）----
    const std::string bin = outPrefix + ".depth.f32.bin";
    FILE *fp = fopen(bin.c_str(), "wb");
    if (fp == nullptr) { out.code = -5; out.note = "open bin failed: " + bin; return out.code; }
    const size_t wrote = fwrite(odata, sizeof(float), static_cast<size_t>(oelems), fp);
    fclose(fp);
    if (wrote != static_cast<size_t>(oelems)) { out.code = -5; out.note = "write bin failed"; return out.code; }
    out.binPath = bin;

    FILE *mp = fopen((outPrefix + ".meta.txt").c_str(), "w");
    if (mp != nullptr) {
        fprintf(mp, "src=%dx%d in=%dx%d out=%dx%d layout=%s elems=%lld min=%.6f max=%.6f mean=%.6f buildMs=%.1f inferMs=%.1f\n",
                w, h, inW, inH, out.outW, out.outH, out.layout.c_str(),
                static_cast<long long>(oelems), out.minV, out.maxV, out.mean, out.buildMs, out.inferMs);
        fclose(mp);
    }
    out.code = 0;
    // 深度缩略图进日志（48×32、10 档，越亮=越近）——设备 shell 取不到应用沙箱
    //（/data/app/.../files Permission denied），日志缩略图是核验"设备上真实深度内容"的唯一通道
    //（手法与既有 SrLogAscii 一致：SRCHK f000/fmid 缩略图）。
    {
        constexpr int32_t TW = 48, TH = 32;
        static const char *kLevels = " .:-=+*#%@";
        char row[TW + 1];
        for (int32_t ty = 0; ty < TH; ty++) {
            const int32_t sy = static_cast<int32_t>((static_cast<int64_t>(ty) * out.outH) / TH);
            for (int32_t tx = 0; tx < TW; tx++) {
                const int32_t sx = static_cast<int32_t>((static_cast<int64_t>(tx) * out.outW) / TW);
                const float v = odata[static_cast<int64_t>(sy) * out.outW + sx];
                float t = (mx > mn) ? (v - mn) / (mx - mn) : 0.0f;
                int32_t lv = static_cast<int32_t>(t * 9.0f + 0.5f);
                lv = lv < 0 ? 0 : (lv > 9 ? 9 : lv);
                row[tx] = kLevels[lv];
            }
            row[TW] = '\0';
            SRD_LOGI("depth d%{public}02d %{public}s", ty, row);
        }
    }
    SRD_LOGI("selftest ok src=%{public}dx%{public}d in=%{public}dx%{public}d out=%{public}dx%{public}d "
             "layout=%{public}s min=%.4f max=%.4f mean=%.4f inferMs=%.0f -> %{public}s",
             w, h, inW, inH, out.outW, out.outH, out.layout.c_str(),
             out.minV, out.maxV, out.mean, out.inferMs, bin.c_str());
    return 0;
}

void DepthRelease() {
    std::lock_guard<std::mutex> lk(g_cache.mtx);
    if (g_cache.model != nullptr) { OH_AI_ModelDestroy(&g_cache.model); g_cache.model = nullptr; }
    if (g_cache.ctx != nullptr) { OH_AI_ContextDestroy(&g_cache.ctx); g_cache.ctx = nullptr; }
    g_cache.built = false;
    g_cache.key.clear();
}

}  // namespace glassdepth
