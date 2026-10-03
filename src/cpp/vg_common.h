/**
 * vg_common.h —— 日志宏最小子集（为本仓库的独立编译提供）
 *
 * 说明：spatial_recon_bridge.cpp / spatial_recon_layers.cpp 仅从原工程的通用头
 * vg_common.h 中使用了以下三个日志宏；本文件是它们的最小等价实现，
 * 使上述源文件无需改动即可独立编译。如接入方已有日志工具，可按需替换。
 */
#ifndef GLASSVIDEO_VG_COMMON_H
#define GLASSVIDEO_VG_COMMON_H

#include <hilog/log.h>

#ifndef VG_LOG_DOMAIN
#define VG_LOG_DOMAIN 0x3301
#endif
#ifndef VG_LOG_TAG
#define VG_LOG_TAG "SpatialRecon"
#endif

#define VG_LOGI(fmt, ...) ((void)OH_LOG_Print(LOG_APP, LOG_INFO, VG_LOG_DOMAIN, VG_LOG_TAG, fmt, ##__VA_ARGS__))
#define VG_LOGW(fmt, ...) ((void)OH_LOG_Print(LOG_APP, LOG_WARN, VG_LOG_DOMAIN, VG_LOG_TAG, fmt, ##__VA_ARGS__))
#define VG_LOGE(fmt, ...) ((void)OH_LOG_Print(LOG_APP, LOG_ERROR, VG_LOG_DOMAIN, VG_LOG_TAG, fmt, ##__VA_ARGS__))

#endif  // GLASSVIDEO_VG_COMMON_H
