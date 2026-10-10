# -*- coding: utf-8 -*-
"""earth_scene.py —— 合成场景数据集生成器（预置 3D 壁纸"地球"用）

自建 3D 空间（精致地球 + 有视差的星空环境）+ 明牌位姿环绕"拍摄"，
替代"用户原始视频 + 黑背景"这族数据（坑 156：黑背景 = 零纹理区 + 球缘深度不连续，
毒化 Kit 深度分析；无环境视差 → fx 无锚点 → aspect 0.218 横向压扁）。

设计（与坑 131/156 教训逐条对应）：
  - 轨道两域可选（--orbit，由 manifest.poseDomain 声明、端侧 bridge 同式同值）：
    "flat-pitch" 域"平环 + 相机绕 right 轴俯仰"（manifest 字段：focalPx=750 /
    cameraRadius / camPitchDeg，位姿公式与 bridge.cpp SrPresetFlatPitchBasis 逐句同式）；
    俯仰打破纯平环 (纵向尺度, fy) 精确退化（离线投影级判据：0px → 79.7px）。
    "diag-arc" 域（默认）= 相机**位置**同时扫 yaw+pitch（pos.y≠0），即照片路线
    scene=5 的同口径移植，属于推荐域。
  - 星空 = 半径 star_shell(12) 的等距柱状星壳纹理（真 3D：按相机位姿采样，随环绕
    平移视差显著——相机绕半径 D 环绕，球面深 1.35~3.35、星点深 ~9.6~14.4，
    深度比 4~10 倍），**不是贴死背景**（早期教训：背景与位姿模型不一致 = 假观测）。
  - 焦距 fx=750（Kit 实测固定预期域）；D=2.35 → 球像高 ≈52% 画面（角直径 ~51°，
    竖直 FOV 87.8° 内余量充足，俯仰摆动不出画）。
  - 120 帧；轨道角由 --arc-deg 决定：缺省 = 整圈 θ_i = -360·i/count（负向惯例，末帧 -357° ∈
    整圈判据域）；--arc-deg 10 = 对称小弧 ±5°（与照片路线"±5° 对角弧"同角度域；
    早前实验曾怀疑整圈环绕超出 Kit 设计域）；--arc-deg 5 = ±2.5°。
  - SSAA 2x（2160×2880 渲染 → LANCZOS 缩 1080×1440）；JPEG q95、4:4:4（判据逐帧 SOF 校验）。

纹理：--earth-tex 任意等距柱状地球图（许可注意：NASA Blue Marble 原始影像为公有领域，
社区镜像需自行核实许可）。--lon-offset 调"θ=0 帧看到哪条经线"（视觉锚定）。

用法：
  python earth_scene.py --earth-tex out/earth_texture_4k.jpg --out out/earth_africa
产物：<out>/f0000..f0119.jpg + manifest.json（可直接拷入应用包 rawfile/preset/<id>/）
"""
import argparse
import hashlib
import json
import math
import os
import time

import numpy as np
from PIL import Image

FRAME_W, FRAME_H = 1080, 1440
STAR_TEX_W, STAR_TEX_H = 4096, 2048


# ---------------- 位姿 ----------------
# 两域互斥，由 manifest.poseDomain 择一，端侧 bridge 同式同值（两侧必须逐位一致）。
def pose_v5(theta_deg, radius, pitch_deg):
    """【flat-pitch】平环 + 相机绕 right 轴俯仰（pos.y ≡ 0，仅姿态俯仰）。返回 (pos,right,up,back)。"""
    th = math.radians(theta_deg)
    pos = np.array([radius * math.sin(th), 0.0, radius * math.cos(th)])
    fwd0 = -pos / np.linalg.norm(pos)
    world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd0, world_up)
    right /= np.linalg.norm(right)
    up0 = np.cross(right, fwd0)
    b = math.radians(pitch_deg)
    cb, sb = math.cos(b), math.sin(b)
    up = cb * up0 + sb * (-fwd0)
    back = -sb * up0 + cb * (-fwd0)
    return pos, right, up, back


def pose_diag_arc(theta_deg, radius, pitch_deg=0.0):
    """【diag-arc】对角弧（照片路线 scene=5 的 `SrFrameBasis` 口径，Φ=Θ ⇒ φ=θ）：
    相机**位置**同时扫 yaw 与 pitch（pos.y ≠ 0），始终看向球心（原点）。
    `pitch_deg` 在本域不使用（俯仰由 θ 本身承担，与照片路线 Φ=Θ 同口径）——保留形参只为与
    `pose_v5` 同签名，便于统一调用。

    为什么这是"比例压缩"的正解（照片路线真机定案，bridge.cpp:378-386）：
    纯水平环绕（相机共面 y=0 + 姿态只含 yaw）下，像点 `v = cy − fy·Y/depth`——
    Y 只出现在分子 ⇒ `(Y→αY, fy→fy/α)` 下**所有帧所有像素不变**，纵向尺度与 fy 成为
    **一对精确不可观测量**，Kit 的 BA（无锁内参 API）沿该零梯度方向随机漂——
    实测成片"宽度被压缩/拉伸"，refined fy/fx 漂到 0.155 / 2.47 / 2.69。
    让相机**位置**离开 y=0 平面后，Y 同时进入分子与 depth、且相机 Y 基线非零 ⇒ 退化破除。
    ⚠️ 只加"姿态俯仰"（pos.y ≡ 0）不够——那是 flat-pitch 域，实测 fy/fx 仍漂到 0.119~0.474。"""
    th = math.radians(theta_deg)
    pos = np.array([radius * math.sin(th) * math.cos(th),
                    radius * math.sin(th),
                    radius * math.cos(th) * math.cos(th)])
    fwd = -pos / np.linalg.norm(pos)
    world_up = np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return pos, right, up, -fwd


POSE_FNS = {'flat-pitch': pose_v5, 'diag-arc': pose_diag_arc}


def theta_of(i, arc_deg, count):
    """第 i 帧的轨道角（度）。arc_deg > 0 = 对称小弧 [−arc/2, +arc/2]（两端闭合）；
    arc_deg <= 0 = 历史整圈 −360·i/count（负向惯例）。"""
    if arc_deg > 0.0:
        return -arc_deg / 2.0 + arc_deg * i / (count - 1)
    return -360.0 * i / count


# ---------------- 等距柱状纹理采样（双线性，向量化） ----------------
def sample_equirect(tex_u8, dirs, lon_offset_rad, globe_tilt_rad=0.0):
    """tex_u8: (th,tw,3) uint8；dirs: (N,3) 单位向量 → (N,3) uint8。

    `globe_tilt_rad`（**默认 0 = 与历史产物逐位一致**）：把**球面纹理**绕相机
    X 轴倾斜——让「θ=0 帧正对的球面点」从赤道挪到某个纬度（经线仍由 `--lon-offset` 决定）。
    为什么需要它：`--lon-offset 105` 只把 105°E 子午线转到画面中心，**纬度仍是 0°（赤道）**——
    可见半球会偏离目标大陆（如大片海洋 + 目标大陆被挤到圆盘上缘，掠射、重度透视压缩）；
    该构图真机实测崩坏（点数 58022→32068、refined fx 468→170、成片相机落进球面内）。
    例如先给经度再倾斜 35°：画面中心 = (35°N, 105°E)，可见半球以目标大陆为主——
    离线实测：盘内陆地占比 35.9% → **53.8%**，盘心区 13.7% → **74.1%**。

    约定 p' = R_x(−tilt)·p（R_x(α): y'=y·cosα−z·sinα, z'=y·sinα+z·cosα），于是
    p=(0,0,1)（θ=0 帧画面中心方向）→ 纬度 = +tilt（北纬），视觉上仍「北在上」。
    背景（远景板）**不参与倾斜**——它采样时单独传 0。
    """
    if globe_tilt_rad != 0.0:
        ca, sa = math.cos(-globe_tilt_rad), math.sin(-globe_tilt_rad)
        dirs = np.stack([dirs[:, 0],
                         dirs[:, 1] * ca - dirs[:, 2] * sa,
                         dirs[:, 1] * sa + dirs[:, 2] * ca], axis=1)
    th_, tw_ = tex_u8.shape[0], tex_u8.shape[1]
    lon = np.arctan2(dirs[:, 0], dirs[:, 2]) + lon_offset_rad
    u = (lon / (2 * np.pi) + 0.5) % 1.0
    lat = np.arcsin(np.clip(dirs[:, 1], -1.0, 1.0))
    v = (0.5 - lat / np.pi) % 1.0
    x = u * (tw_ - 1)
    y = v * (th_ - 1)
    x0 = np.floor(x).astype(np.int64)
    y0 = np.floor(y).astype(np.int64)
    x1 = np.minimum(x0 + 1, tw_ - 1)
    y1 = np.minimum(y0 + 1, th_ - 1)
    fx_ = (x - x0)[:, None]
    fy_ = (y - y0)[:, None]
    t = tex_u8.astype(np.float32)
    c00 = t[y0, x0]
    c01 = t[y0, x1]
    c10 = t[y1, x0]
    c11 = t[y1, x1]
    out = (c00 * (1 - fx_) * (1 - fy_) + c01 * fx_ * (1 - fy_)
           + c10 * (1 - fx_) * fy_ + c11 * fx_ * fy_)
    return np.clip(out, 0, 255).astype(np.uint8)


# ---------------- 星壳纹理（随机星点 + 银河带，处处唯一无周期） ----------------
# 背景档位（2026-10-09 参数化）：地球是**旋转对称体**，球面纹理的投影在
# (Y→αY, fy→fy/α) 下近似不变 ⇒ **只有背景的非对称结构能约束 fy**（本文件头设计说明）。
# 真机实测：背景内容越强，比例读数越接近 1（纯黑 0.218 → 稀疏星 0.216 → 加密星 0.474@72帧），
# 且 24 帧下背景被重建到前景（近场云遮住地球）——两者同因：**背景可匹配特征太少、太弱**。
# 因此把背景拆成四个正交旋钮（都写进 manifest.source 便于追溯与判据）：
#   star_count/star_size —— 点状特征的数量与尺寸（σ 像素）；
#   nebula_count         —— 低频大尺度结构（几十~几百 px 的软斑，跨帧可整块匹配 ⇒ 深度稳健）；
#   noise_floor          —— 极暗的结构化底噪（消灭"纯黑零纹理区"，抑制 Kit 把空域填成近景）。
BG_PROFILES = {
    # 历史档（逐位复现 2026-10-05 起的存量数据集：11000 星、σ 0.5~1.4、银河带加权）
    'classic': dict(star_count=11000, star_size=1.0, nebula_count=0, nebula_scale=1.0, noise_floor=0),
    # 加密档（2026-10-08 晚真机第三轮用的那版：30000 星、σ×2.0）
    'rich': dict(star_count=30000, star_size=2.0, nebula_count=0, nebula_scale=1.0, noise_floor=0),
    # 深空档（推荐）：点多 + 大尺度软斑 + 底噪——目标是把"背景只有离散小点"升级为
    # 多尺度、处处唯一、可整块匹配的场，既给 fy 强锚点，又让背景被正确三角化到远处。
    # 软斑刻意多于"够用"：**唯一的大尺度结构**才是无歧义匹配的主力（星点太小时彼此难分）。
    'deep_field': dict(star_count=20000, star_size=2.0, nebula_count=120, nebula_scale=1.4,
                       noise_floor=8),
}


# ---------------- 海洋加地形起伏（2026-10-10） ----------------
def add_ocean_relief(tex, amp=20.0, seed=20261010, octaves=(512, 128, 32, 8)):
    """给**海洋**合成低频起伏（海底地形感）——让占地球 65% 的海洋区变得**可匹配**。

    动机（2026-10-10 对 4K 素材实测）：NASA Blue Marble 的海洋是近乎均匀的深蓝——
    梯度均值 **11.2**，而陆地 **49.8**（差 4.5 倍）；ORB 关键点落在海洋的比例只有 **51%**，
    而海洋占球面积 **65%**。合成帧里"海洋朝向相机的那一半"因此几乎没有可匹配特征 ⇒
    真机成片表现为**半边精细半边粗糙**（用户读数"右部分精细、左部分粗糙"，因为当时 θ=0 面朝
    非洲、相机左侧是大西洋）。这与用户"替换素材"的思路一致，只是**不必换图**：
    直接在海洋上合成类海底地形结构即可（NASA 的 "topography and bathymetry" 版本来就带这个）。

    返回 (新纹理, 海洋掩膜)。`amp` 是起伏幅度（8bit 级）；脊状成分 `1−|n|` 产生线状结构，
    比纯平滑噪声更容易被特征提取器抓到。
    """
    h, w = tex.shape[:2]
    rng = np.random.default_rng(seed)
    field = np.zeros((h, w), np.float32)
    wsum = 0.0
    for i, g in enumerate(octaves):
        if g < 2:
            continue
        gh = max(2, int(round(g * h / float(w))))
        small = rng.normal(0.0, 1.0, (gh, g)).astype(np.float32)
        nrm = (small - small.min()) / max(float(np.ptp(small)), 1e-6)
        up = np.asarray(Image.fromarray((nrm * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC),
                        dtype=np.float32)
        a = 1.0 / (2 ** i)
        field += (up / 255.0 * 2.0 - 1.0) * a
        wsum += a
    field /= max(wsum, 1e-6)
    ridge = 1.0 - np.abs(field)                 # 海岭/海沟般的线状结构
    f = 0.65 * field + 0.35 * (ridge - 0.5)
    r = tex[:, :, 0].astype(np.float32)
    g = tex[:, :, 1].astype(np.float32)
    b = tex[:, :, 2].astype(np.float32)
    ocean = (b > r + 15.0) & (b > g + 5.0) & (b > 60.0)
    tint = np.array([0.55, 0.75, 1.0], np.float32)      # 偏蓝的明暗起伏，观感仍是海
    delta = (f * float(amp))[:, :, None] * tint[None, None, :]
    out = np.stack([r, g, b], axis=-1) + delta * ocean[:, :, None]
    return np.clip(out, 0, 255).astype(np.uint8), ocean


def make_star_texture(profile='classic', seed=20261005):
    """背景壳等距柱状纹理。profile 见 BG_PROFILES（旋钮：星点数量/尺寸、软斑数、底噪）。"""
    cfg = BG_PROFILES[profile]
    n_stars = int(cfg['star_count'])
    size_mul = float(cfg['star_size'])
    rng = np.random.default_rng(seed)
    tex = np.zeros((STAR_TEX_H, STAR_TEX_W, 3), dtype=np.uint8)
    # 银河平面法向（斜着走，让环繞时银河形态变化更丰富）
    g_norm = np.array([0.35, 0.75, -0.55])
    g_norm /= np.linalg.norm(g_norm)

    # ---- 底噪：极暗但成结构的云雾（先铺，后绘星点）----
    if cfg['noise_floor'] > 0:
        base = rng.normal(0.0, 1.0, (STAR_TEX_H // 8, STAR_TEX_W // 8)).astype(np.float32)
        base = np.asarray(Image.fromarray(((base - base.min()) / max(float(np.ptp(base)), 1e-6) * 255)
                                          .astype(np.uint8)).resize((STAR_TEX_W, STAR_TEX_H),
                                                                    Image.BICUBIC), dtype=np.float32)
        floor = np.clip(base / 255.0 * float(cfg['noise_floor']), 0, 255).astype(np.uint8)
        tex[:] = floor[:, :, None]

    # ---- 大尺度软斑：跨帧可整块匹配的"星云"（低频、处处唯一）----
    for _ in range(int(cfg['nebula_count'])):
        cx0 = rng.uniform(0, STAR_TEX_W)
        # 纬度偏向银河带内（带内结构越丰富，越能约束纵向尺度）
        cy0 = rng.uniform(0, STAR_TEX_H)
        rad = rng.uniform(60, 220) * float(cfg['nebula_scale'])
        amp = rng.uniform(18, 52)
        elong = rng.uniform(0.5, 1.0)          # 各向异性拉长（更强的非对称特征）
        ang = rng.uniform(0, np.pi)
        ca, sa = np.cos(ang), np.sin(ang)
        x0, x1 = int(cx0 - rad * 1.5), int(cx0 + rad * 1.5 + 1)
        y0, y1 = int(cy0 - rad * 1.5), int(cy0 + rad * 1.5 + 1)
        yy, xx = np.mgrid[y0:y1, x0:x1]
        dx = xx - cx0
        dy = (yy - cy0) / elong
        u = (ca * dx + sa * dy) / rad
        v = (-sa * dx + ca * dy) / rad
        w = np.exp(-(u * u + v * v) * 1.6)     # 高斯软斑
        xxw = xx % STAR_TEX_W
        yyw = np.clip(yy, 0, STAR_TEX_H - 1)
        tint = rng.uniform(0, 1, 3) * 0.35 + 0.65   # 弱冷/暖色倾向
        for c in range(3):
            region = tex[yyw, xxw, c].astype(np.float32)
            tex[yyw, xxw, c] = np.clip(region + w * amp * tint[c], 0, 255).astype(np.uint8)

    # 均匀球面方向
    z = rng.uniform(-1, 1, n_stars)
    phi = rng.uniform(0, 2 * np.pi, n_stars)
    r = np.sqrt(1 - z * z)
    dirs = np.stack([r * np.cos(phi), z, r * np.sin(phi)], axis=1)
    # 银河带加权：靠近银河平面 → 提亮 + 加密（用亮度提升替代密度重采样，实现简单）
    dist = np.abs(dirs @ g_norm)
    milky = np.exp(-(dist / 0.16) ** 2)          # 1=带内，0=带外
    # 亮度幂律：多数暗星、少数亮星
    rank = rng.power(2.6, n_stars)               # (0,1]，偏 0
    bright = 40 + rank * 175 + milky * rng.uniform(0, 45, n_stars)
    bright = np.clip(bright, 30, 255)
    # 色温：白为主，少量偏蓝/偏橙
    tint = rng.integers(0, 10, n_stars)
    # 星点大小（亮星大）：1.1~2.6 px 标准差 × size_mul
    sigma = (0.5 + rank * 0.9) * size_mul

    # 光斑直径（像素）
    rad = np.ceil(3 * sigma).astype(int)
    # 转等距柱状像素坐标
    lon = np.arctan2(dirs[:, 0], dirs[:, 2])
    lat = np.arcsin(np.clip(dirs[:, 1], -1, 1))
    px = ((lon / (2 * np.pi) + 0.5) % 1.0) * (STAR_TEX_W - 1)
    py = (0.5 - lat / np.pi) * (STAR_TEX_H - 1)
    # 经度接缝：星点跨 0/W-1 边界时双写（画两次，一次 +W 一次 -W）
    for i in range(n_stars):
        x, y = px[i], py[i]
        b = bright[i]
        s = sigma[i]
        col = np.array([b, b, b], dtype=np.float32)
        if tint[i] < 2:
            col[2] = min(255, b * 1.12)          # 偏蓝（R 低 B 高）
        elif tint[i] >= 8:
            col[0] = min(255, b * 1.10)          # 偏橙
        rr = rad[i]
        x0, x1 = int(x - rr), int(x + rr + 1)
        y0, y1 = int(y - rr), int(y + rr + 1)
        yy, xx = np.mgrid[y0:y1, x0:x1]
        d2 = (xx - x) ** 2 + (yy - y) ** 2
        wgt = np.exp(-d2 / (2 * s * s))
        # wrap x（经度环形）
        xxw = xx % STAR_TEX_W
        yyw = np.clip(yy, 0, STAR_TEX_H - 1)
        for c in range(3):
            region = tex[yyw, xxw, c].astype(np.float32)
            tex[yyw, xxw, c] = np.clip(region + wgt * col[c], 0, 255).astype(np.uint8)
        # 接缝双写（x 附近跨越 W 边界）
        if x0 < 0 or x1 > STAR_TEX_W:
            xx2 = (xx + (STAR_TEX_W if x0 < 0 else -STAR_TEX_W)) % STAR_TEX_W
            for c in range(3):
                region = tex[yyw, xx2, c].astype(np.float32)
                tex[yyw, xx2, c] = np.clip(region + wgt * col[c], 0, 255).astype(np.uint8)
    return tex


# ---------------- 单帧渲染（射线求交：地球单位球 + 星壳） ----------------
def render_frame(theta_deg, radius, pitch_deg, fx, earth_tex, star_tex,
                 star_shell, lon_offset_rad, cam_dirs, ssaa, pose_fn=pose_v5,
                 bg_mode='plate', bg_z=3.0, bg_half=None, globe_tilt_rad=0.0):
    """单帧渲染 = 单位球（地球）+ 背景。

    `bg_mode`（2026-10-09 新增，默认 plate）：
      - 'plate'（默认）：背景 = 球**之后**的一张远景板（平面 z = −bg_z，足迹按画幅精算）。
        与照片路线同构——那条路线的背景就是"远景板"（`kDepthPlaneHX/HY`），而不是包住相机的天空球。
      - 'sky'（历史域）：背景 = 半径 star_shell 的等距柱状天球。**已废弃**，仅用于复现旧数据集。

    ⚠️ **天空球为什么必须废弃**（2026-10-09 真机产物实测定案）：
    star_shell = 12（地球半径归一 1）而相机只在 2.35 处 → **相机被整个罩在壳内**。
    ±5° 弧只观测到壳上 ±5° 的一小块 ⇒ 重建出来的是**一片残缺壳片**：
      ① 那片壳在 Kit 里距原点 ~0.60（12 地球半径），而运镜相机在 0.374（7.5 地球半径）
         ⇒ 相机"看到那片壳"时得到**巨大近场辉光**（成片里那团蓝白大光斑）；
      ② 其余方向没有任何几何 ⇒ 渲染器清屏成中灰 = **空白帧**（实测前 190/299 帧全空白）。
    远景板则永远在球**之后**、不包住相机 ⇒ 任何朝向都能看到地球 + 其后的背景。
    """
    pos, right, up, back = pose_fn(theta_deg, radius, pitch_deg)
    # 世界系射线方向：dir = u·right + v·up − back（相机系 (u,v,-1)：Z_cam=back，看向 −Z）
    # ⚠️ 基向量按【行】堆叠（cam @ B，B 行=基向量）；堆成列（axis=1）= 乘 Bᵀ = 逆旋转
    #（实测症状：俯仰方向翻转、球心投影错位 ~2×tan(β)·fx）
    m = np.stack([right, up, back], axis=0)
    dirs = cam_dirs @ m
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)

    oc = pos
    b = dirs @ oc                                   # d·oc
    c_ball = float(oc @ oc) - 1.0
    disc = b * b - c_ball
    hit_ball = disc > 0.0
    t_ball = np.where(hit_ball, -b - np.sqrt(np.maximum(disc, 0.0)), np.inf)

    # 命中点方向
    p_ball = pos + t_ball[:, None] * dirs           # 单位球上点即法向
    use_ball = hit_ball & (t_ball > 0)

    if bg_mode == 'plate':
        # 射线 × 平面 z = −bg_z（dirs[:,2] < 0 恒成立：相机看向 −Z，FOV 半角 41° < 90°）
        t_plane = (-bg_z - pos[2]) / dirs[:, 2]
        p_plane = pos + t_plane[:, None] * dirs
        # 板外（超出足迹）→ 留黑（足迹按判据覆盖全部帧，正常不会发生）
        bg_hit = (t_plane > 0)
        if bg_half is not None:
            bg_hit &= (np.abs(p_plane[:, 0]) <= bg_half[0]) & (np.abs(p_plane[:, 1]) <= bg_half[1])
        bg_dirs = p_plane / np.maximum(np.linalg.norm(p_plane, axis=1, keepdims=True), 1e-9)
    else:
        c_shell = float(oc @ oc) - star_shell * star_shell
        disc_s = b * b - c_shell
        t_shell = -b + np.sqrt(np.maximum(disc_s, 0.0))   # 远根（相机在壳内）
        p_shell = pos + t_shell[:, None] * dirs
        bg_dirs = p_shell / star_shell               # 归一化（星壳半径归一）
        bg_hit = np.ones(dirs.shape[0], dtype=bool)

    rgb = np.zeros((dirs.shape[0], 3), dtype=np.uint8)
    earth_rgb = sample_equirect(earth_tex, p_ball[use_ball], lon_offset_rad, globe_tilt_rad)
    rgb[use_ball] = earth_rgb
    bg_sel = (~use_ball) & bg_hit
    rgb[bg_sel] = sample_equirect(star_tex, bg_dirs[bg_sel], 0.0)

    hh, ww = FRAME_H * ssaa, FRAME_W * ssaa
    img = rgb.reshape(hh, ww, 3)
    return img


def plate_footprint(arc_deg, count, radius, pitch_deg, fx, pose_fn, bg_z, ssaa=1, margin=1.02):
    """远景板需要的半宽/半高：对**全部帧的四个画幅角**做射线×平面求交，取 |x|、|y| 的最大值。

    与照片路线的 `kDepthPlaneHX/HY` 同思路（那条路线是手算常量；这里改成按实际轨道精确算，
    免得改跨度/焦距后露黑角——照片路线当年就踩过"fx=1500 窄视场口径在端点露黑角"的坑）。"""
    hh, ww = FRAME_H, FRAME_W
    corners = np.array([[0.5, 0.5], [ww - 0.5, 0.5], [0.5, hh - 0.5], [ww - 0.5, hh - 0.5]])
    xs = (corners[:, 0] - ww / 2.0) / fx
    ys = (hh / 2.0 - corners[:, 1]) / fx              # 图像 y 向下 ↔ 相机 Y 向上
    cam_dirs = np.stack([xs, ys, -np.ones(len(corners))], axis=1)
    hw = hh_ = 0.0
    thetas = [theta_of(i, arc_deg, count) for i in range(count)]
    for th in thetas:
        pos, right, up, back = pose_fn(float(th), radius, pitch_deg)
        m = np.stack([right, up, back], axis=0)
        d = cam_dirs @ m
        d /= np.linalg.norm(d, axis=1, keepdims=True)
        t = (-bg_z - pos[2]) / d[:, 2]
        p = pos + t[:, None] * d
        hw = max(hw, float(np.abs(p[:, 0]).max()))
        hh_ = max(hh_, float(np.abs(p[:, 1]).max()))
    return hw * margin, hh_ * margin


def main():
    ap = argparse.ArgumentParser(description='合成场景数据集生成器（预置 3D 壁纸"地球"）')
    ap.add_argument('--earth-tex', required=True, help='等距柱状地球纹理（jpg/png）')
    ap.add_argument('--out', required=True, help='输出目录（帧 + manifest.json）')
    ap.add_argument('--frames', type=int, default=120)
    ap.add_argument('--radius', type=float, default=2.35, help='环绕半径（球半径归一 1；2.35 → 球像高约 52%%）')
    ap.add_argument('--pitch', type=float, default=12.0,
                    help='【仅 flat-pitch 域】相机俯仰角（度）；diag-arc 域不用（俯仰由 θ 承担）')
    ap.add_argument('--orbit', default='diag-arc', choices=sorted(POSE_FNS.keys()),
                    help='轨道位姿域：diag-arc（默认，= 照片路线 scene=5 的 SrFrameBasis 口径，'
                         'Φ=Θ ⇒ 相机位置同时扫 yaw+pitch，pos.y≠0——坑 131"比例压缩"的正解）；'
                         'flat-pitch（历史域：平环 pos.y≡0 + 仅姿态俯仰 12°，实测 fy/fx 漂 0.119~0.474）')
    ap.add_argument('--focal', type=float, default=750.0, help='上报焦距 px（Kit 预期域）')
    ap.add_argument('--focal-y-ratio', type=float, default=1.0,
                    help='**各向异性内参声明 —— 禁用**（默认 1.0 = 诚实各向同性，请保持）。'
                         '历史文档曾推荐用它抵消纵横比漂移（"帧不动、只偏置声明"），'
                         '但真机实测：声明 0.89 ⇒ **重建直接崩、什么都看不到**。'
                         '该做法已废止；本参数只为字段兼容保留。')
    ap.add_argument('--star-shell', type=float, default=12.0, help='星壳半径（世界单位；深度比球面 4~10 倍）')
    ap.add_argument('--lon-offset', type=float, default=0.0,
                    help='地球纹理经度偏移（度；θ=0 帧看到的中心经线 = 0°+offset）')
    ap.add_argument('--no-world-flip', action='store_true',
                    help='manifest.worldFlip180=false（端侧不把上报位姿绕水平轴翻 180°）。'
                         '用途：Kit 的导出朝向随内容变，抵消过度会把模型与相机镜像错开')
    ap.add_argument('--globe-tilt', type=float, default=0.0,
                    help='球面纹理倾斜（度；默认 0 = 历史口径）。把 θ=0 帧正对的球面点从赤道'
                         '抬到该纬度 ⇒ 配合 --lon-offset 决定"画面中心是地球的哪一点"。'
                         '例：--lon-offset 105 --globe-tilt 35 ⇒ 中心 = (35°N,105°E)，'
                         '可见半球以该大陆为主（盘内陆地 25%%→53.8%%、盘心区 13.7%%→74.1%%）。'
                         '⚠️ 背景远景板不倾斜')
    ap.add_argument('--ssaa', type=int, default=2, help='超采样倍率（每边）')
    ap.add_argument('--star-cache', default='', help='背景纹理缓存路径（缺省按档位命名）')
    ap.add_argument('--bg-mode', default='plate', choices=['plate', 'sky'],
                    help='背景几何：plate（默认）= 球后远景板（相机永不在其内，'
                         '与照片路线同构）；sky = 历史天空球（**已废弃**：壳半径 12 地球半径把相机罩住，'
                         '±5° 弧只重建出残片 → 巨辉光 + 空白帧，见 render_frame 注释）')
    ap.add_argument('--bg-z', type=float, default=3.0,
                    help='远景板深度（世界单位，球半径归一 1；板在 z=−bg_z，须保证 |bg_z| > 1 且在'
                         '球之后——3.0 与照片路线"背景板≈主体身后数倍"同量级）')
    ap.add_argument('--bg-profile', default='classic', choices=sorted(BG_PROFILES.keys()),
                    help='背景壳档位（独立旋钮打包；见 BG_PROFILES）：'
                         'classic=历史档（11000 星）/ rich=加密档（30000 星 σ2.0）/ '
                         'deep_field=深空档（26000 星 + 90 软斑 + 底噪，推荐——背景可整块匹配）')
    ap.add_argument('--star-seed', type=int, default=20261005, help='背景壳纹理随机种子')
    # 单旋钮覆盖（缺省 None = 用档位值）。背景特征密度是**可匹配性 ↔ 包体**的直接换算：
    # 帧内高频特征越多越好匹配（越能给 fy 强锚点），但 JPEG 体积同步上升
    # （实测 classic 11000 星 ≈268KB/帧、deep_field 20000 星+软斑 ≈424KB/帧，120 帧即 32MB → 52MB）。
    ap.add_argument('--star-count', type=int, default=None, help='覆盖档位的星点数量')
    ap.add_argument('--star-size', type=float, default=None, help='覆盖档位的星点尺寸系数（σ 倍数）')
    ap.add_argument('--nebula-count', type=int, default=None, help='覆盖档位的大尺度软斑数量')
    ap.add_argument('--nebula-scale', type=float, default=None, help='覆盖档位的软斑尺寸系数')
    ap.add_argument('--noise-floor', type=int, default=None, help='覆盖档位的底噪幅度（0~255）')
    ap.add_argument('--star-cache-key', default='',
                    help='缓存文件名键（缺省 = --bg-profile；不同档位写不同缓存，防串档）')
    ap.add_argument('--arc-deg', type=float, default=0.0,
                    help='轨道总跨度（度）：>0 = 对称小弧 [−arc/2, +arc/2]（10 = ±5°，与照片路线'
                         '±5° 对角弧同口径）；缺省 0 = 历史整圈 −360·i/count')
    ap.add_argument('--preview-only', action='store_true', help='只渲 3 帧预览（0/60/119）不写 manifest')
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    # 单旋钮覆盖写回档位表（manifest 记录的是**最终生效**值，判据据此核对）
    bg_cfg = dict(BG_PROFILES[args.bg_profile])
    for cli_name, cfg_key in (('star_count', 'star_count'), ('star_size', 'star_size'),
                              ('nebula_count', 'nebula_count'), ('nebula_scale', 'nebula_scale'),
                              ('noise_floor', 'noise_floor')):
        v = getattr(args, cli_name)
        if v is not None:
            bg_cfg[cfg_key] = v
    BG_PROFILES[args.bg_profile] = bg_cfg
    earth_tex = np.asarray(Image.open(args.earth_tex).convert('RGB'))
    print(f'地球纹理：{earth_tex.shape[1]}x{earth_tex.shape[0]}')

    star_key = args.star_cache_key or args.bg_profile
    # classic 沿用历史文件名 star_map.png（存量数据集可复现）；其余档位各自独立缓存防串档；
    # 有单旋钮覆盖时再加参数指纹后缀——否则改了密度会误用旧缓存纹理（隐蔽串档）
    if any(getattr(args, k) is not None for k in
           ('star_count', 'star_size', 'nebula_count', 'nebula_scale', 'noise_floor')):
        star_key += '_' + hashlib.sha1(json.dumps(bg_cfg, sort_keys=True).encode()).hexdigest()[:8]
    star_name = 'star_map.png' if star_key == 'classic' else f'star_map_{star_key}.png'
    star_path = args.star_cache or os.path.join(args.out, star_name)
    if os.path.exists(star_path):
        star_tex = np.asarray(Image.open(star_path).convert('RGB'))
        print(f'背景壳纹理（缓存）：{star_tex.shape[1]}x{star_tex.shape[0]}  {star_path}')
    else:
        t0 = time.time()
        star_tex = make_star_texture(profile=args.bg_profile, seed=args.star_seed)
        Image.fromarray(star_tex).save(star_path)
        print(f'背景壳纹理：生成 {STAR_TEX_W}x{STAR_TEX_H}（档位 {args.bg_profile}，'
              f'{time.time()-t0:.1f}s）→ {star_path}')

    # 像素相机系方向网格（SSAA 分辨率；与相机位姿无关，循环外算一次）
    hh, ww = FRAME_H * args.ssaa, FRAME_W * args.ssaa
    fx = args.focal * args.ssaa
    cx, cy = ww / 2.0, hh / 2.0
    xs = (np.arange(ww) + 0.5 - cx) / fx
    ys = (cy - (np.arange(hh) + 0.5)) / fx          # 图像 y 向下 ↔ 相机 Y 向上
    uu, vv = np.meshgrid(xs, ys)                    # (hh,ww)
    cam_dirs = np.stack([uu.ravel(), vv.ravel(), -np.ones(uu.size)], axis=1).astype(np.float32)

    lon_off = math.radians(args.lon_offset)
    # 远景板足迹：按**实际轨道 + 焦距**对全部帧的四角射线求交精算（改跨度/焦距后自动适配，
    # 不必像照片路线那样维护手算常量——那条路线当年因口径变化在端点露过黑角）
    bg_half = None
    if args.bg_mode == 'plate':
        bg_half = plate_footprint(args.arc_deg, args.frames, args.radius, args.pitch,
                                  args.focal, POSE_FNS[args.orbit], args.bg_z)
        print(f'远景板：z=−{args.bg_z}（{args.bg_z} 地球半径）半宽×半高 = '
              f'{bg_half[0]:.2f}×{bg_half[1]:.2f}（按 {args.frames} 帧四角射线精算）')
    idxs = [0, args.frames // 2, args.frames - 1] if args.preview_only else range(args.frames)
    for i in idxs:
        theta = theta_of(i, args.arc_deg, args.frames)
        t0 = time.time()
        img = render_frame(theta, args.radius, args.pitch, args.focal,
                           earth_tex, star_tex, args.star_shell, lon_off,
                           cam_dirs, args.ssaa, pose_fn=POSE_FNS[args.orbit],
                           bg_mode=args.bg_mode, bg_z=args.bg_z, bg_half=bg_half,
                           globe_tilt_rad=math.radians(args.globe_tilt))
        pil = Image.fromarray(img).resize((FRAME_W, FRAME_H), Image.LANCZOS)
        name = f'f{i:04d}.jpg'
        pil.save(os.path.join(args.out, name), quality=95, subsampling=0)
        a = np.asarray(pil, dtype=np.float32)
        print(f'{name} θ={theta:+7.2f}°  mean={a.mean():6.1f} std={a.std():5.1f}  ({time.time()-t0:.1f}s)')
        if args.preview_only:
            pil.save(os.path.join(args.out, f'preview_f{i:04d}.png'))

    if args.preview_only:
        print('预览模式：不写 manifest')
        return 0

    man = {
        'version': 5,
        'name': 'earth',
        'frameW': FRAME_W,
        'frameH': FRAME_H,
        'count': args.frames,
        'focalPx': args.focal,
        'focalYRatio': args.focal_y_ratio,
        # 上报位姿的世界翻转（端侧 Service 读；只改上报位姿、不合成帧）。缺省 true = 历史口径；
        # tilt35 素材实测 Kit 导出与声明一致（不再翻）⇒ 置 false，否则模型与相机会镜像错开。
        'worldFlip180': not args.no_world_flip,
        'globeRadius': 1.0,
        'cameraRadius': args.radius,
        'poseDomain': args.orbit,
        'camPitchDeg': (args.arc_deg / 2.0 if args.orbit == 'diag-arc' and args.arc_deg > 0
                        else args.pitch),
        'orbitArcDeg': args.arc_deg,
        'source': {
            'synthetic': True,
            'generator': 'tools/depth_mesh/earth_scene.py',
            'earthTexture': os.path.basename(args.earth_tex),
            'lonOffsetDeg': args.lon_offset,
            'globeTiltDeg': args.globe_tilt,
            'bgMode': args.bg_mode,
            # 平面**实际** z（负值，在球之后）；不写"深度幅度"以免正负号歧义
            'bgPlaneZ': (-abs(args.bg_z) if args.bg_mode == 'plate' else None),
            'bgPlaneHalf': (list(bg_half) if bg_half is not None else None),
            'starShellRadius': (args.star_shell if args.bg_mode == 'sky' else None),
            'starTexture': os.path.basename(star_path),
            'bgProfile': args.bg_profile,
            'bgParams': dict(BG_PROFILES[args.bg_profile]),
            'bgSeed': args.star_seed,
            'orbit': args.orbit,
            'note': ('synthetic scene: textured unit sphere + background. '
                     'orbit = poseDomain selects the pose formula, identical on both sides '
                     '(generator <-> bridge): "diag-arc" = SrPresetDiagArcBasis (= photo route '
                     'scene=5 SrFrameBasis with Phi=Theta: camera position sweeps yaw AND pitch so '
                     'pos.y != 0 — the 坑 131 fix for "proportion compressed"); "flat-pitch" = '
                     'SrPresetFlatPitchBasis (legacy: flat ring pos.y==0 + attitude-only pitch, '
                     'measured refined fy/fx drifting to 0.119~0.474). '
                     'bgMode = background geometry: "plate" (current) = a far plate BEHIND the globe '
                     'at z=-bgPlaneZ with half-extent bgPlaneHalf, i.e. the photo route backdrop '
                     'pattern — the camera is never inside it; "sky" (deprecated) = a sky sphere of '
                     'radius starShellRadius which at 12 globe radii ENCLOSES the camera, so a ±5° '
                     'arc only rebuilds a shell patch -> giant near-field glow + blank directions '
                     '(measured: 190/299 output frames blank). '
                     'bgProfile = background texture knob pack (BG_PROFILES in the generator). '
                     'globeTiltDeg = the earth texture is sampled with the globe rotated by this '
                     'angle about the camera X axis, so the sub-camera point at theta=0 shows '
                     'latitude +globeTiltDeg (longitude still = lonOffsetDeg); the background '
                     'plate is NOT tilted.'),
        },
        'poseConvention': (
            'AR camera (X right, Y up, Z back, looks -Z); static globe at origin radius 1.0. '
            + ('diag-arc: thetaDeg sweeps the arc; the camera position itself sweeps yaw AND pitch '
               '(phi = theta, i.e. the photo route Phi=Theta): '
               'pos = cameraRadius*(sin(t)cos(t), sin(t), cos(t)cos(t)); fwd = -normalize(pos); '
               'right = normalize(fwd x upWorld); up = right x fwd; basis rows [right; up; back]. '
               'No separate attitude pitch.'
               if args.orbit == 'diag-arc' else
               'flat-pitch: camera pos = cameraRadius*(sin(t), 0, cos(t)) (pos.y == 0); '
               'fwd0 = -normalize(pos); right = normalize(fwd0 x upWorld); up0 = right x fwd0; '
               'then pitch about the right axis by camPitchDeg: up = cosb*up0 + sinb*(-fwd0), '
               'back = -sinb*up0 + cosb*(-fwd0); basis rows [right; up; back].')
            + ' thetaDeg = orbitArcDeg>0 ? (-orbitArcDeg/2 + orbitArcDeg*i/(count-1)) '
              '[symmetric small arc, same angle domain as the photo route ±5°] : -360*i/count '
              '[full ring]; the theta=0 frame faces the lon_offset meridian'),
        'knownRisks': [
            'earth texture is a community mirror of NASA Blue Marble (public-domain origin); '
            'swap to NASA original before store release',
            'star shell is CG: Kit feature matching on stars is unproven (first-round probe)',
        ],
        'frames': [
            {'file': f'f{i:04d}.jpg', 'srcFrame': i,
             'thetaDeg': round(theta_of(i, args.arc_deg, args.frames), 4)}
            for i in range(args.frames)
        ],
    }
    man_path = os.path.join(args.out, 'manifest.json')
    with open(man_path, 'w', encoding='utf-8') as f:
        json.dump(man, f, ensure_ascii=False, indent=1)
    print(f'manifest.json → {man_path}（radius={args.radius} pitch={args.pitch} fx={args.focal}）')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
