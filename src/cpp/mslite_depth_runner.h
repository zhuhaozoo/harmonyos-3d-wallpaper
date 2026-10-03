/**
 * 3D壁纸 · 路线B 阶段1：端侧单图深度估计（MindSpore Lite + Depth Anything V2 Small）
 *
 * 设计文档：docs/3d-wallpaper-route-b-implementation.md §2（端侧深度推理）
 * 本单元只做"RGBA → 模型输入 → 深度输出 → 统计与落盘"，**不含任何相机/几何数学**
 * （相机数学单源在 spatial_recon_bridge.cpp，见文档 §4 的单源纪律）。
 *
 * 关键实现约束（来自阶段0 实测，勿改）：
 *  - 模型是**动态 shape** 的 .ms（converter 无法静态化，见文档 §2.3 纪律 0）
 *    → 推理前必须 OH_AI_ModelResize 到模型输入尺寸（默认 1×3×728×546，14 的倍数、3:4）。
 *  - 预处理必须与 PC 端 ref_depth.py 同源：RGB、/255、(x-mean)/std（ImageNet），
 *    mean=[0.485,0.456,0.406]、std=[0.229,0.224,0.225]，**输出语义 = 相对逆深度（越大越近）**。
 *  - 张量布局自适应：读输入张量 format（NCHW=0 / NHWC=1）决定填充顺序，并把实际布局打进日志/note
 *    （不写死——真的布局以设备实测为准，这是本项目"代码不写死"的既有纪律）。
 */
#ifndef GLASSRENDER_MSLITE_DEPTH_RUNNER_H
#define GLASSRENDER_MSLITE_DEPTH_RUNNER_H

#include <cstddef>
#include <cstdint>
#include <string>

namespace glassdepth {

/** 自检结果（全部字段都进日志与返回值，供与 PC 端 ref_depth.py 比对） */
struct DepthSelfTestResult {
    int32_t code = 0;      // 0=成功；-1=模型构建失败；-2=resize 失败；-3=推理失败；-4=输出异常；-5=落盘失败
    int32_t inW = 0;       // 模型输入宽（期望 546）
    int32_t inH = 0;       // 模型输入高（期望 728）
    int32_t outW = 0;      // 输出宽
    int32_t outH = 0;      // 输出高
    float minV = 0.0f;     // 输出最小（相对逆深度）
    float maxV = 0.0f;
    float mean = 0.0f;
    double buildMs = 0.0;  // 模型构建耗时（首次含加载）
    double inferMs = 0.0;  // 纯推理耗时
    std::string layout;    // 实测输入张量布局（NCHW / NHWC / 未知）
    std::string binPath;   // 落盘路径 <prefix>.depth.f32.bin
    std::string note;      // 诊断信息（错误原因 / 张量描述）
};

/**
 * 运行时探针：验证 MindSpore Lite 运行时在设备上可用（建上下文+设备信息再销毁），不碰模型。
 * 返回 0=可用，-1=不可用；info 带后端描述。
 */
int32_t DepthLibraryProbe(std::string &info);

/**
 * 端侧深度自检：RGBA(w×h，任意尺寸) → 双线性缩放+归一化 → resize+推理 → 统计+落盘。
 * @param modelData/modelSize .ms 模型字节（调用方从 rawfile 或沙箱读入；空则由 modelFilePath 加载）
 * @param modelFilePath 沙箱模型路径（modelData 为空时使用）
 * @param outPrefix 产物前缀：写 <outPrefix>.depth.f32.bin 与 <outPrefix>.meta.txt
 */
int32_t DepthSelfTest(const uint8_t *rgba, int32_t w, int32_t h,
                      const void *modelData, size_t modelSize,
                      const std::string &modelFilePath,
                      const std::string &outPrefix,
                      DepthSelfTestResult &out);

/** 释放缓存的模型/上下文（页面退出或模型切换时可选调用） */
void DepthRelease();

}  // namespace glassdepth

#endif  // GLASSRENDER_MSLITE_DEPTH_RUNNER_H
