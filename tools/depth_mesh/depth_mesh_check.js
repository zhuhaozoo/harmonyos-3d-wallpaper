/**
 * 3D壁纸 · 路线B（单图深度 → 深度网格多视角合成 → Spatial Recon Kit）离线判据
 *
 * 常量口径：本工具常量必须与 `src/cpp/` 的实装常量一致（判据含源码级守卫核对）。
 * 定位：免构建、零依赖（仅 node 内置 fs/zlib），在**写任何 C++ 之前**把整条几何证清楚——
 *   "先证后写"：本文件是 C++ 实装的公式来源与验收口径。
 *
 * 场景构件（三层，绘制顺序=远景板→前景层→背景层，共享 z-buffer；等深度处先画者胜）：
 *   ① 远景背景板：常量深度平面（模糊+压暗的源图延展）——承接画幅边缘的轨道空隙
 *   ② 背景层    ：深度场=把"近区（主体）"push-pull 补洞后的背景深度，纹理=同款补洞色
 *                 ——承接主体轮廓的消隐（disocclusion），避免露出模糊板
 *   ③ 前景层    ：原深度网格（深度台阶处细分到 1px 的"正面平行小片"，**不跨台阶连接**）
 *                 ——不连接是硬要求：跨台阶的连接面在新视角下会拉成长条纹
 *
 * 两种模式：
 *   ① 合成模式（默认，零素材）：程序生成"已知深度"的测试场景，验证数学与渲染管线；
 *      同时把场景 dump 成 *.rgba.bin / *.depth.f32.bin（真实模式交换格式），并回读自检。
 *   ② 真实模式：--input=<前缀> 读外部深度产物
 *      <前缀>.rgba.bin(w*h*4) + <前缀>.depth.f32.bin(w*h*float32 相对逆深度) + <前缀>.json
 *
 * 用法：
 *   node tools/depth_mesh/depth_mesh_check.js
 *   node tools/depth_mesh/depth_mesh_check.js --input=tools/depth_mesh/out/p1
 *   node tools/depth_mesh/depth_mesh_check.js --theta=6 --k=3      # 参数扫描
 *
 * ⚠️ 位移口径（本工具建立，替代早期近似口径）：
 *   早期近似口径的位移数字 ≈ 精确投影值的 2 倍（2026-10-01 复核发现）。
 *   本工具一律用**精确针孔投影**计算同一点在两帧的落点差值，并给出：
 *     - stepNear：相邻关键帧（按 Kit 自选 5 帧估算）在**最近层**的最大像素位移（守卫对象）
 *     - slip    ：最近层与最远层的相对滑移（SfM 分出深度的证据）
 *   按同一口径重算历史实测：成功轮（主体 z=1.53，d0=2，8° 步长）峰值 ≈70px；失败轮（z=1.36）≈100px。
 *   → 守卫界取 80px（待真机标定；成功可上调、失败先降 theta）。
 *   2026-10-02 补充：下界 30px 入守卫（真机 fx=700：Θ=2°≈12px → STAGE02 no 3D points、
 *   Θ=3°≈18px 模型碎、Θ=5°≈31px 可建）。
 *
 * ⚠️ 轨道形态（坑 131，2026-10-02 定案）：**对角弧**（yaw+pitch 同步各扫 ±Θ）。
 *   纯水平弧（仅 yaw）对 (纵向尺度 Y, fy) 精确退化：v_i = cy − fy·Y/depth_i(X,Z,θ_i)，Y 只在
 *   分子 → (Y→αY, fy→fy/α) 下所有帧所有像素不变 → Kit BA 沿该方向随机漂移（真机实测
 *   refined fy/fx = 0.155 / 2.47 / 2.69，成片宽度随机压缩/拉伸/不可名状）。对角弧给相机群
 *   Y 基线与 pitch 旋转 → fy/纵向尺度可观测。实装同款（SrFrameBasis 的 pitchDeg 参数）。
 *   同轮同步：FX 1500 → 750（与产品默认焦距一致；旧 1500 口径随"750 修复"轮过时未同步）。
 */
'use strict';
const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

// ---------- 参数（与 src/cpp 实装常量对齐） ----------
const OUT_W = 1080, OUT_H = 1440;          // Kit 硬约束（官方：仅 1080×1440）
const FX = 750, FY = 750;                  // 虚拟相机内参（px；2026-10-02 起 = 产品默认焦距，旧 1500 口径停用）
const CX = OUT_W / 2, CY = OUT_H / 2;
const Z_NEAR = 1.0;                        // 最近内容世界深度（沿参考相机光轴）
const K_RATIO = 3.0;                       // 远/近比（k）；远层 = k·zNear
const D0_DEF = (Z_NEAR + Z_NEAR * K_RATIO) / 2;
const THETA_DEG = 5.0;                     // 默认轨道半角（±Θ；与 scene=5 真机成功基线一致）
const N_FRAMES = 73;                       // 帧数（奇数 → 含 θ=0 参考帧）
const KF_COUNT = 5;                        // Kit 自选关键帧数（保守估算）
const STRIDE = 4;                          // 网格抽稀步长（px）
const SKIRT = 80;                          // 照片网格向画幅外的"裙边"延展（px；采样钳制=边界像素按深度延续，
                                           //   用于承接轨道端点的画幅边缘空隙——否则露出模糊远景板）
const REFINE_RATIO = 1.15;                 // 深度台阶判定：四角 z 比 > 该值 → 细分到 1px 小片
const NEAR_SPLIT_S = 0.5;                  // 背景层补洞的"近区"判定：s > 该值（s=1 最近）
const BG_Z_FACTOR = 2.5;                   // 远景背景板深度 = 2.5·zFar
const BG_HALF_W = 7.4, BG_HALF_H = 9.6;    // 背景板半尺寸（世界单位；2026-10-02 随 FX 750 放大：
                                           //   fx=750 广角 + 对角弧俯仰的**角落对角射线**精确需求
                                           //   7.24×9.43（四角×两端点位姿枚举算出），旧 4.4×7.2 是
                                           //   fx=1500 窄视场口径（需 3.46），端点对角象限会露黑角 →
                                           //   判据"覆盖率 100%"FAIL；与 bridge kDepthPlaneHX/HY 同值）
const GUARD_STEP_PX = 80;                  // 最近层相邻关键帧位移守卫上界（px）
const GUARD_STEP_MIN_PX = 30;              // 位移下界（px；真机 fx700：Θ2°≈12px 失败 / Θ5°≈31px 可建，坑 131 标定）
const GUARD_SLIP_MIN = 20;                 // 近远层相对滑移下界（px；2026-10-02 随 FX 750/对角弧重标定：
                                           //   旧 25px 系 fx=1500 纯水平弧口径（slip≈44）；深度可分性由
                                           //   (z−d0)/z 杠杆决定、与轨道形态无关，新口径默认配置 slip≈23.3）
const GUARD_PHOTO_RATIO = 0.70;            // 轨道端点前景照片占比下限
const P_NORM_LO = 0.01, P_NORM_HI = 0.99;  // 深度稳健归一化分位

const args = {};
process.argv.slice(2).forEach((a) => { const m = a.match(/^--([^=]+)=?(.*)$/); if (m) args[m[1]] = m[2]; });
const OUT_DIR = args.out || 'tools/ui_shots/depth_mesh';
const INPUT = args.input || '';
const SKY_SKIP = !!args.skyskip;
const THETA = ((parseFloat(args.theta) || THETA_DEG)) * Math.PI / 180;
const PHI = THETA;                         // 对角弧俯仰半角 Φ=Θ（与实装耦合一致：pitchDeg = orbitDeg，坑 131）
const K_R = (parseFloat(args.k) || K_RATIO);
const ZF = Z_NEAR * K_R;
const D0_USE = (Z_NEAR + ZF) / 2;
const BG_Z = BG_Z_FACTOR * ZF;

// ---------- 基础工具 ----------
const v3 = (x, y, z) => ({ x, y, z });
const sub = (a, b) => v3(a.x - b.x, a.y - b.y, a.z - b.z);
const cross = (a, b) => v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
const dot3 = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const norm3 = (a) => { const l = Math.hypot(a.x, a.y, a.z); return l > 1e-12 ? v3(a.x / l, a.y / l, a.z / l) : v3(0, 0, 0); };

/**
 * 相机位姿：绕中心**对角弧**轨道，相机始终看向中心。θ=φ=0 → C=(0,0,0)、R=I，
 * 与 C++ SrFrameBasis 同构（look-at 构造：fwd=normalize(center−C)、right=normalize(fwd×worldUp)、
 * up=right×fwd；offset = d0·(sinθcosφ, sinφ, cosθcosφ)）。φ=0 时与旧闭式
 * （C=(d0 sinθ,0,−d0(1−cosθ))，R=yaw 旋转）逐位一致（负测试：数值核对过）。
 * ⚠️ 坑 131（2026-10-02 定案）：纯水平弧（仅 yaw）对 (纵向尺度 Y, fy) 精确退化——
 * v_i = cy − fy·Y/depth_i(X,Z,θ_i)，Y 只在分子 → (Y→αY, fy→fy/α) 下所有像素不变，
 * Kit BA 沿该方向随机漂移（真机 refined fy/fx = 0.155/2.47/2.69 → 宽度压缩/拉伸/碎）。
 * 对角弧（yaw+pitch 同步 ±Θ）给相机群 Y 基线与 pitch → fy/纵向尺度可观测。
 */
function poseOf(theta, phi, d0) {
  const D = d0 === undefined ? D0_USE : d0;
  const ph = (phi === undefined) ? 0 : phi;
  const st = Math.sin(theta), ct = Math.cos(theta), sp = Math.sin(ph), cp = Math.cos(ph);
  const C = v3(D * st * cp, D * sp, -D * (1 - ct * cp));
  const fwd = norm3(sub(v3(0, 0, -D), C));
  const right = norm3(cross(fwd, v3(0, 1, 0)));
  const up = cross(right, fwd);
  const R = (p) => v3(dot3(right, p), dot3(up, p), -dot3(fwd, p));
  return { theta, phi: ph, C, R };
}
/** 世界点 → 像素（§3.1：u = cx + fx·p.x/(-p.z)，v = cy - fy·p.y/(-p.z)）
 *  fyOverride：可选覆盖 fy（坑 131 投影级退化验证用——测试 (Y→αY, fy→fy/α) 变换） */
function project(P, pose, fyOverride) {
  const fyUse = (fyOverride === undefined) ? FY : fyOverride;
  const q = pose.R(sub(P, pose.C));
  const depth = -q.z;
  if (depth <= 1e-9) return null;
  return { u: CX + FX * q.x / depth, v: CY - fyUse * q.y / depth, depth };
}
/** 像素（连续坐标）→ 世界射线方向（前进分量 1） */
function rayDir(x, y) {
  const a = (x - CX) / FX;
  const b = -(y - CY) / FY;        // 行向下 ↔ 相机 Y 向上（写成 +b 即垂直镜像，坑 119）
  return v3(a, b, -1);
}
function unproject(x, y, z) { const d = rayDir(x, y); return v3(d.x * z, d.y * z, -z); }

// ---------- 深度映射（§3.2 逆深度线性） ----------
function zOfS(s) { return 1 / (s / Z_NEAR + (1 - s) / ZF); }
function normalizeDepth(raw, n) {
  const sorted = Float32Array.from(raw); sorted.sort();
  const p1 = sorted[Math.floor(P_NORM_LO * (n - 1))];
  const p99 = sorted[Math.floor(P_NORM_HI * (n - 1))];
  const span = p99 - p1;
  const s = new Float32Array(n);
  if (!(span > 1e-9)) return { s, p1, p99, span, valid: false };
  for (let i = 0; i < n; i++) s[i] = Math.min(1, Math.max(0, (raw[i] - p1) / span));
  return { s, p1, p99, span, valid: true };
}

// ---------- 采样（连续坐标 (x,y)：像素 px 的中心在 px+0.5） ----------
function sampleRgb(img, w, h, x, y) {
  const fx = Math.min(w - 1.001, Math.max(0, x - 0.5));
  const fy = Math.min(h - 1.001, Math.max(0, y - 0.5));
  const x0 = Math.floor(fx), y0 = Math.floor(fy);
  const tx = fx - x0, ty = fy - y0;
  const x1 = Math.min(w - 1, x0 + 1), y1 = Math.min(h - 1, y0 + 1);
  const out = [0, 0, 0];
  for (let c = 0; c < 3; c++) {
    const v00 = img[(y0 * w + x0) * 3 + c], v10 = img[(y0 * w + x1) * 3 + c];
    const v01 = img[(y1 * w + x0) * 3 + c], v11 = img[(y1 * w + x1) * 3 + c];
    out[c] = (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
  }
  return out;
}
function sampleS(sField, w, h, x, y) {
  const fx = Math.min(w - 1.001, Math.max(0, x - 0.5));
  const fy = Math.min(h - 1.001, Math.max(0, y - 0.5));
  const x0 = Math.floor(fx), y0 = Math.floor(fy);
  const tx = fx - x0, ty = fy - y0;
  const x1 = Math.min(w - 1, x0 + 1), y1 = Math.min(h - 1, y0 + 1);
  const v00 = sField[y0 * w + x0], v10 = sField[y0 * w + x1];
  const v01 = sField[y1 * w + x0], v11 = sField[y1 * w + x1];
  return (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
}
/** 背景板纹理 = 源图 1/16 盒式模糊 */
function makeBlur(src, w, h, f) {
  const bw = Math.max(1, Math.ceil(w / f)), bh = Math.max(1, Math.ceil(h / f));
  const out = new Uint8Array(bw * bh * 3);
  for (let by = 0; by < bh; by++) {
    for (let bx = 0; bx < bw; bx++) {
      let r = 0, g = 0, b = 0, n = 0;
      for (let y = by * f; y < Math.min(h, (by + 1) * f); y++) {
        for (let x = bx * f; x < Math.min(w, (bx + 1) * f); x++) {
          const o = (y * w + x) * 3;
          r += src[o]; g += src[o + 1]; b += src[o + 2]; n++;
        }
      }
      const o = (by * bw + bx) * 3;
      out[o] = r / n; out[o + 1] = g / n; out[o + 2] = b / n;
    }
  }
  return { img: out, w: bw, h: bh, f };
}

// ---------- push-pull 补洞（把 mask=1 的区域用周围非 mask 内容填充；多通道） ----------
function pushPullFill(field, ch, mask, w, h) {
  // field: Float32Array(w*h*ch)；mask: Uint8Array(w*h)（1 = 待填）
  const levs = [];
  let cw = w, chh = h;
  const v0 = Float32Array.from(field);
  const wt0 = new Float32Array(w * h);
  for (let i = 0; i < w * h; i++) wt0[i] = mask[i] ? 0 : 1;
  levs.push({ v: v0, wt: wt0, w: cw, h: chh });
  while (cw > 4 && chh > 4) {
    const cur = levs[levs.length - 1];
    const nw = Math.ceil(cw / 2), nh = Math.ceil(chh / 2);
    const nv = new Float32Array(nw * nh * ch), nwt = new Float32Array(nw * nh);
    for (let y = 0; y < nh; y++) for (let x = 0; x < nw; x++) {
      let wt = 0;
      const acc = new Float32Array(ch);
      for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) {
        const sx = Math.min(cw - 1, x * 2 + dx), sy = Math.min(chh - 1, y * 2 + dy);
        const k = sy * cw + sx, wk = cur.wt[k];
        if (wk <= 0) continue;
        for (let c = 0; c < ch; c++) acc[c] += cur.v[k * ch + c] * wk;
        wt += wk;
      }
      const k2 = y * nw + x;
      if (wt > 0) { for (let c = 0; c < ch; c++) nv[k2 * ch + c] = acc[c] / wt; nwt[k2] = wt; }
    }
    levs.push({ v: nv, wt: nwt, w: nw, h: nh });
    cw = nw; chh = nh;
  }
  // 顶层兜底（关键）：金字塔可能在 4px 就停，主体内部还存在"整块零权重"的顶层像素——
  // 不处理则 push 时 `up.w <= 0` 直接 continue，主体区域的补洞值原样保留源图颜色（消隐带会露鬼影）。
  // 两步兜底：① 顶层用最近有效邻居迭代播种；② 仍为零的用全局加权均值。
  const top = levs[levs.length - 1];
  const gmean = new Float32Array(ch);
  {
    const L0 = levs[0];
    let gwt = 0;
    for (let i = 0; i < L0.w * L0.h; i++) {
      const wk = L0.wt[i];
      if (wk <= 0) continue;
      for (let c = 0; c < ch; c++) gmean[c] += L0.v[i * ch + c] * wk;
      gwt += wk;
    }
    if (gwt > 0) for (let c = 0; c < ch; c++) gmean[c] /= gwt;
  }
  for (let pass = 0; pass < 16; pass++) {
    let changed = 0;
    const pv = Float32Array.from(top.v), pw = Float32Array.from(top.wt);
    for (let y = 0; y < top.h; y++) for (let x = 0; x < top.w; x++) {
      const k = y * top.w + x;
      if (pw[k] > 0) continue;
      let wt = 0;
      const acc = new Float32Array(ch);
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= top.w || yy >= top.h) continue;
        const kk = yy * top.w + xx;
        if (pw[kk] <= 0) continue;
        for (let c = 0; c < ch; c++) acc[c] += pv[kk * ch + c];
        wt++;
      }
      if (wt > 0) { for (let c = 0; c < ch; c++) top.v[k * ch + c] = acc[c] / wt; top.wt[k] = 1e-6; changed++; }
    }
    if (!changed) break;
  }
  for (let i = 0; i < top.w * top.h; i++) {
    if (top.wt[i] > 0) continue;
    for (let c = 0; c < ch; c++) top.v[i * ch + c] = gmean[c];
    top.wt[i] = 1e-6;
  }
  // push：自顶向下把父层值填入无权重像素
  for (let L = levs.length - 1; L > 0; L--) {
    const up = levs[L], dn = levs[L - 1];
    for (let y = 0; y < up.h; y++) for (let x = 0; x < up.w; x++) {
      const ku = y * up.w + x;
      if (up.wt[ku] <= 0) continue;
      for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) {
        const sx = Math.min(dn.w - 1, x * 2 + dx), sy = Math.min(dn.h - 1, y * 2 + dy);
        const kd = sy * dn.w + sx;
        if (dn.wt[kd] > 0) continue;
        // 双线性采样父层（比 2×2 块拷贝平滑；与既有 spatial_recon_layers.cpp 的"双线性升采样"同款）
        const px = Math.min(up.w - 1.001, Math.max(0, (sx + 0.5) / 2 - 0.5));
        const py = Math.min(up.h - 1.001, Math.max(0, (sy + 0.5) / 2 - 0.5));
        const x0 = Math.floor(px), y0 = Math.floor(py), tx = px - x0, ty = py - y0;
        const x1 = Math.min(up.w - 1, x0 + 1), y1 = Math.min(up.h - 1, y0 + 1);
        for (let c = 0; c < ch; c++) {
          const v00 = up.v[(y0 * up.w + x0) * ch + c], v10 = up.v[(y0 * up.w + x1) * ch + c];
          const v01 = up.v[(y1 * up.w + x0) * ch + c], v11 = up.v[(y1 * up.w + x1) * ch + c];
          dn.v[kd * ch + c] = (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
        }
        dn.wt[kd] = 1e-6;
      }
    }
  }
  return levs[0].v;
}

// ---------- 网格构建 ----------
// 平滑格（四角 z 比 ≤ REFINE_RATIO）：单 quad，四角各自深度（真正的斜面）
// 台阶格：细分到 1px 的"正面平行小片"——四点同深（取格心），**绝不跨台阶连接**
function buildMesh(sField, w, h, stride, emitMask) {
  const quads = [];
  const pushV = (x, y, z) => {
    const d = rayDir(x, y);
    quads.push(d.x * z, d.y * z, -z, x, y);
  };
  const emitQuad = (x0, y0, x1, y1, z) => { pushV(x0, y0, z); pushV(x1, y0, z); pushV(x1, y1, z); pushV(x0, y1, z); };
  let coarse = 0, refined = 0, maxRatio = 1;
  for (let j = -SKIRT; j + stride <= h + SKIRT; j += stride) {
    for (let i = -SKIRT; i + stride <= w + SKIRT; i += stride) {
      if (emitMask) {   // 背景层只在"近区（含膨胀）"生成：掩膜外与前层完全同位，生成只会造成等深度交叠
        let any = 0;
        for (let yy = j; yy <= j + stride; yy += stride) for (let xx = i; xx <= i + stride; xx += stride) {
          if (emitMask[Math.min(h - 1, yy) * w + Math.min(w - 1, xx)]) { any = 1; break; }
        }
        if (!any) continue;
      }
      const zs = [[i, j], [i + stride, j], [i + stride, j + stride], [i, j + stride]]
        .map(([x, y]) => zOfS(sampleS(sField, w, h, x, y)));
      const zmin = Math.min(...zs), zmax = Math.max(...zs);
      const ratio = zmax / Math.max(1e-9, zmin);
      if (ratio > REFINE_RATIO) {
        for (let yy = j; yy < j + stride; yy++) {
          for (let xx = i; xx < i + stride; xx++) {
            const zc = zOfS(sampleS(sField, w, h, xx + 0.5, yy + 0.5));
            emitQuad(xx, yy, xx + 1, yy + 1, zc);
            refined++;
          }
        }
      } else {
        pushV(i, j, zs[0]); pushV(i + stride, j, zs[1]); pushV(i + stride, j + stride, zs[2]); pushV(i, j + stride, zs[3]);
        coarse++; maxRatio = Math.max(maxRatio, ratio);
      }
    }
  }
  return { quads: Float32Array.from(quads), count: quads.length / 20, coarse, refined, stride, maxRatio };
}

// ---------- 光栅化（远→近：背景板 → 背景层 → 前景层；共享 z-buffer；透视正确 UV） ----------
function rasterize(pose, layers, blur, w, h, scale) {
  // layers: [{mesh, img}...] 由远到近
  const RW = Math.round(w * scale), RH = Math.round(h * scale);
  const rgb = new Uint8Array(RW * RH * 3);
  const label = new Uint8Array(RW * RH);      // 0=背景板 2=背景层 1=前景层
  const written = new Uint8Array(RW * RH);
  const zbuf = new Float32Array(RW * RH);
  const fx = FX * scale, fy = FY * scale, cx = CX * scale, cy = CY * scale;

  const rasterTri = (p0, p1, p2, uv0, uv1, uv2, lab) => {
    const dz = [1 / p0.d, 1 / p1.d, 1 / p2.d];
    const area = (p1.u - p0.u) * (p2.v - p0.v) - (p2.u - p0.u) * (p1.v - p0.v);
    if (Math.abs(area) < 1e-9) return;
    const inv = 1 / area;
    const minX = Math.max(0, Math.floor(Math.min(p0.u, p1.u, p2.u)));
    const maxX = Math.min(RW - 1, Math.ceil(Math.max(p0.u, p1.u, p2.u)));
    const minY = Math.max(0, Math.floor(Math.min(p0.v, p1.v, p2.v)));
    const maxY = Math.min(RH - 1, Math.ceil(Math.max(p0.v, p1.v, p2.v)));
    for (let y = minY; y <= maxY; y++) {
      const sy = y + 0.5;
      for (let x = minX; x <= maxX; x++) {
        const sx = x + 0.5;
        const l0 = ((p1.u - sx) * (p2.v - sy) - (p2.u - sx) * (p1.v - sy)) * inv;
        if (l0 < -1e-6) continue;
        const l1 = ((p2.u - sx) * (p0.v - sy) - (p0.u - sx) * (p2.v - sy)) * inv;
        if (l1 < -1e-6) continue;
        const l2 = 1 - l0 - l1;
        if (l2 < -1e-6) continue;
        const zInv = l0 * dz[0] + l1 * dz[1] + l2 * dz[2];
        const idx = y * RW + x;
        if (zInv <= zbuf[idx]) continue;
        zbuf[idx] = zInv;
        written[idx] = 1;
        const U = (l0 * uv0[0] * dz[0] + l1 * uv1[0] * dz[1] + l2 * uv2[0] * dz[2]) / zInv;
        const Vv = (l0 * uv0[1] * dz[0] + l1 * uv1[1] * dz[1] + l2 * uv2[1] * dz[2]) / zInv;
        let col;
        if (lab === 0) {
          col = sampleRgb(blur.img, blur.w, blur.h, U / blur.f, Vv / blur.f);
          col = [col[0] * 0.72, col[1] * 0.72, col[2] * 0.75];
        } else {
          const ly = layers.find((L) => L.label === lab);
          col = sampleRgb(ly.img, w, h, U, Vv);
        }
        rgb[idx * 3] = col[0]; rgb[idx * 3 + 1] = col[1]; rgb[idx * 3 + 2] = col[2];
        label[idx] = lab;
      }
    }
  };

  // ① 背景板
  {
    const hw = BG_HALF_W, hh = BG_HALF_H, sUV = FX / BG_Z, sUVy = FY / BG_Z;
    const corners = [v3(-hw, hh, -BG_Z), v3(hw, hh, -BG_Z), v3(hw, -hh, -BG_Z), v3(-hw, -hh, -BG_Z)];
    const uvs = corners.map((P) => [CX + P.x * sUV, CY - P.y * sUVy]);
    const scr = corners.map((P) => {
      const q = pose.R(sub(P, pose.C)); const d = -q.z;
      return { u: cx + fx * q.x / d, v: cy - fy * q.y / d, d };
    });
    rasterTri(scr[0], scr[1], scr[2], uvs[0], uvs[1], uvs[2], 0);
    rasterTri(scr[0], scr[2], scr[3], uvs[0], uvs[2], uvs[3], 0);
  }
  // ②③ 各层网格
  for (const L of layers) {
    const q = L.mesh.quads;
    for (let qi = 0; qi < L.mesh.count; qi++) {
      const o = qi * 20;
      const scr = [];
      let ok = true;
      for (let vi = 0; vi < 4; vi++) {
        const P = v3(q[o + vi * 5], q[o + vi * 5 + 1], q[o + vi * 5 + 2]);
        const c = pose.R(sub(P, pose.C)); const d = -c.z;
        if (d <= 1e-9) { ok = false; break; }
        scr.push({ u: cx + fx * c.x / d, v: cy - fy * c.y / d, d });
      }
      if (!ok) continue;
      const T = [0, 1, 2, 3].map((vi) => [q[o + vi * 5 + 3], q[o + vi * 5 + 4]]);
      rasterTri(scr[0], scr[1], scr[2], T[0], T[1], T[2], L.label);
      rasterTri(scr[0], scr[2], scr[3], T[0], T[2], T[3], L.label);
    }
  }
  return { rgb, label, written, W: RW, H: RH };
}

// ---------- PNG 输出（与 tools/sr_frames_check.js 同款写法） ----------
function crc32(buf) {
  let c, table = crc32.table;
  if (!table) {
    table = crc32.table = new Int32Array(256);
    for (let n = 0; n < 256; n++) { c = n; for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1); table[n] = c; }
  }
  let crc = -1;
  for (let i = 0; i < buf.length; i++) crc = (crc >>> 8) ^ table[(crc ^ buf[i]) & 0xFF];
  return (crc ^ -1) >>> 0;
}
function chunk(type, data) {
  const len = Buffer.alloc(4); len.writeUInt32BE(data.length, 0);
  const td = Buffer.concat([Buffer.from(type, 'ascii'), data]);
  const crc = Buffer.alloc(4); crc.writeUInt32BE(crc32(td), 0);
  return Buffer.concat([len, td, crc]);
}
function writePng(file, w, h, rgb) {
  const raw = Buffer.alloc((w * 3 + 1) * h);
  const src = Buffer.from(rgb.buffer, rgb.byteOffset, w * h * 3);
  for (let y = 0; y < h; y++) {
    raw[y * (w * 3 + 1)] = 0;
    src.copy(raw, y * (w * 3 + 1) + 1, y * w * 3, (y + 1) * w * 3);
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4);
  ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
  fs.writeFileSync(file, Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]),
    chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0)),
  ]));
}

// ---------- 判据框架 ----------
let pass = 0, fail = 0;
function check(name, ok, detail) {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  ok ? pass++ : fail++;
}
function skip(name, why) { console.log(`SKIP  ${name}  ${why}`); }

// ---------- 合成场景（已知深度） ----------
function makeSynthetic(w, h) {
  const rgb = new Uint8Array(w * h * 3);
  const raw = new Float32Array(w * h);
  const rnd = (x, y) => { let n = (x * 374761393 + y * 668265263) | 0; n = (n ^ (n >> 13)) * 1274126177 | 0; return ((n ^ (n >> 16)) >>> 0) / 4294967295; };
  const MARK = { x0: 500, x1: 540, y0: 686, y1: 726 };
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const o = (y * w + x) * 3, i = y * w + x;
      const fx = x / w, fy = y / h;
      let s = 0.08;
      const groundTop = 0.78;
      if (fy > groundTop) s = 0.08 + (fy - groundTop) / (1 - groundTop) * 0.34;   // 地面斜坡（峰值 < NEAR_SPLIT_S）
      const inPanel = (fx > 0.30 && fx < 0.70 && fy > 0.26 && fy < 0.74);
      const inEllipse = Math.hypot((fx - 0.5) / 0.16, (fy - 0.55) / 0.20) < 1;
      if (inPanel || inEllipse) s = 1.0;
      raw[i] = s * 2.5 + 0.4;
      const sp = rnd(x, y) * 38;
      if (inEllipse) {
        const c = ((Math.floor(x / 14) + Math.floor(y / 14)) & 1) ? [235, 90, 70] : [70, 120, 235];
        rgb[o] = Math.min(255, c[0] * 0.8 + sp); rgb[o + 1] = Math.min(255, c[1] * 0.8 + sp); rgb[o + 2] = Math.min(255, c[2] * 0.8 + sp);
      } else if (inPanel) {
        rgb[o] = Math.min(255, 60 + sp + 40 * Math.sin(fx * 40)); rgb[o + 1] = Math.min(255, 150 + sp); rgb[o + 2] = Math.min(255, 120 + sp + 40 * Math.cos(fy * 30));
      } else if (fy > groundTop) {
        const t = (fy - groundTop) / (1 - groundTop);
        rgb[o] = Math.min(255, 90 + 60 * t + sp); rgb[o + 1] = Math.min(255, 80 + 50 * t + sp); rgb[o + 2] = Math.min(255, 60 + 40 * t + sp);
      } else {
        rgb[o] = Math.min(255, 120 + 60 * (1 - fy) + sp); rgb[o + 1] = Math.min(255, 140 + 50 * (1 - fy) + sp); rgb[o + 2] = Math.min(255, 180 + sp);
      }
      if (x >= MARK.x0 && x < MARK.x1 && y >= MARK.y0 && y < MARK.y1) {
        const chk = ((Math.floor((x - MARK.x0) / 10) + Math.floor((y - MARK.y0) / 10)) & 1);
        rgb[o] = chk ? 255 : 0; rgb[o + 1] = chk ? 0 : 255; rgb[o + 2] = chk ? 255 : 0;
      }
    }
  }
  return { rgb, raw, marker: MARK };
}

// ---------- 真实数据读取（深度产物交换格式） ----------
function loadInputs(prefix) {
  const meta = JSON.parse(fs.readFileSync(prefix + '.json', 'utf8'));
  const w = meta.w, h = meta.h;
  const rgbaBuf = fs.readFileSync(prefix + '.rgba.bin');
  const rgba = new Uint8Array(rgbaBuf.buffer, rgbaBuf.byteOffset, w * h * 4);
  const rawBuf = fs.readFileSync(prefix + '.depth.f32.bin');
  const raw = new Float32Array(rawBuf.buffer, rawBuf.byteOffset, w * h);
  const rgb = new Uint8Array(w * h * 3);
  for (let i = 0; i < w * h; i++) { rgb[i * 3] = rgba[i * 4]; rgb[i * 3 + 1] = rgba[i * 4 + 1]; rgb[i * 3 + 2] = rgba[i * 4 + 2]; }
  return { w, h, rgb, raw, meta };
}

// ---------- 位移（精确投影；对角弧：相邻关键帧 (θ,φ) 同步等步进） ----------
function kfAngles(half) {
  const out = [];
  for (let i = 0; i < KF_COUNT; i++) out.push({ th: -half + i * (2 * half) / (KF_COUNT - 1), ph: -half + i * (2 * half) / (KF_COUNT - 1) });
  return out;
}
function offsetOf(x0, y0, z0, angA, angB) {
  const P = unproject(x0, y0, z0);
  const a = project(P, poseOf(angA.th, angA.ph)), b = project(P, poseOf(angB.th, angB.ph));
  if (!a || !b) return null;
  return { du: b.u - a.u, dv: b.v - a.v };
}
function stepScan(z, thetaHalf) {
  const angs = kfAngles(thetaHalf);
  let worst = 0;
  for (let gy = 0.10; gy <= 0.90; gy += 0.10) {
    for (let gx = 0.05; gx <= 0.95; gx += 0.10) {
      for (let i = 0; i + 1 < angs.length; i++) {
        const d = offsetOf(gx * OUT_W, gy * OUT_H, z, angs[i], angs[i + 1]);
        if (d) worst = Math.max(worst, Math.hypot(d.du, d.dv));
      }
    }
  }
  return worst;
}

// ---------- 主流程 ----------
const t0 = Date.now();
fs.mkdirSync(OUT_DIR, { recursive: true });
let W = OUT_W, H = OUT_H, rgb, raw, marker = null, meta = null;
if (!INPUT) {
  const sc = makeSynthetic(OUT_W, OUT_H);
  rgb = sc.rgb; raw = sc.raw; marker = sc.marker;
  console.log(`模式：合成场景（已知深度）  ${W}x${H}`);
} else {
  const li = loadInputs(INPUT);
  W = li.w; H = li.h; rgb = li.rgb; raw = li.raw; meta = li.meta;
  console.log(`模式：真实数据 ${INPUT}  ${W}x${H}  meta=${JSON.stringify({ src: meta.src, onnx: meta.onnx_sha256 ? String(meta.onnx_sha256).slice(0, 12) : null, input: meta.input_shape })}`);
  if (W !== OUT_W || H !== OUT_H) console.log(`  ⚠️ 输入尺寸 ${W}x${H} ≠ Kit 约束 ${OUT_W}x${OUT_H}（深度产物应为 ${OUT_W}x${OUT_H}）`);
}
console.log(`参数：fx=${FX} zNear=${Z_NEAR} k=${K_R} zFar=${ZF.toFixed(2)} d0=${D0_USE.toFixed(2)} Θ=±${(THETA * 180 / Math.PI).toFixed(1)}° Φ=±${(PHI * 180 / Math.PI).toFixed(1)}°（对角弧） N=${N_FRAMES} stride=${STRIDE} 关键帧步长=${(THETA * 180 / Math.PI * 2 / (KF_COUNT - 1)).toFixed(2)}°`);

// ---------- [1] 深度方向与分布 ----------
console.log('\n=== [1] 深度方向（相对逆深度：越大越近）与分布 ===');
const norm = normalizeDepth(raw, W * H);
const s = norm.s;
console.log(`  raw p1=${norm.p1.toFixed(3)} p99=${norm.p99.toFixed(3)} span=${norm.span.toFixed(3)}`);
if (marker) {
  const sPanel = s[Math.floor(H * 0.5) * W + Math.floor(W * 0.5)];
  const sWall = s[Math.floor(H * 0.08) * W + Math.floor(W * 0.08)];
  check('合成场景：主体（近）深度 > 远墙深度', sPanel > sWall + 0.4, `panel=${sPanel.toFixed(2)} wall=${sWall.toFixed(2)}`);
} else if (SKY_SKIP) {
  skip('天空区 < 地面区（深度方向）', '--skyskip 指定跳过（无天空的照片）');
} else {
  const med = (arr) => { const a = Float32Array.from(arr); a.sort(); return a[Math.floor(a.length / 2)]; };
  const topRows = [], botRows = [];
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x += 4) {
    if (y < H * 0.12) topRows.push(s[y * W + x]);
    if (y > H * 0.80) botRows.push(s[y * W + x]);
  }
  const mTop = med(topRows), mBot = med(botRows);
  check('天空区（上 12%）中位深度 < 地面区（下 20%）中位深度', mTop < mBot, `top=${mTop.toFixed(3)} bottom=${mBot.toFixed(3)}（无天空照片请加 --skyskip=1）`);
}
check('深度有效（对比度足够）', norm.valid, `span=${norm.span.toFixed(3)}`);

// ---------- 网格 + 三层场景 ----------
console.log('\n=== [2] 参考位姿恒等 + 网格结构守卫 ===');
const meshFront = buildMesh(s, W, H, STRIDE);
console.log(`  前景层网格：粗格 ${meshFront.coarse} + 台阶细分 ${meshFront.refined}（共 ${meshFront.count} quads）`);
check('网格结构守卫：无跨深度台阶的连接面（quad 四角 z 比 ≤ 1.2）', meshFront.maxRatio <= 1.2,
  `maxRatio=${meshFront.maxRatio.toFixed(3)}（>${REFINE_RATIO} 的格子已细分到 1px 小片，四点同深）`);

// 背景层：近区（s > NEAR_SPLIT_S）用 push-pull 从周围背景补洞（深度 + 颜色）
const nearMask = new Uint8Array(W * H);
let nearCnt = 0;
for (let i = 0; i < W * H; i++) { if (s[i] > NEAR_SPLIT_S) { nearMask[i] = 1; nearCnt++; } }
const sFill = pushPullFill(s, 1, nearMask, W, H);
const rgbF = new Float32Array(W * H * 3);
for (let i = 0; i < W * H; i++) { rgbF[i * 3] = rgb[i * 3]; rgbF[i * 3 + 1] = rgb[i * 3 + 1]; rgbF[i * 3 + 2] = rgb[i * 3 + 2]; }
const colFill = pushPullFill(rgbF, 3, nearMask, W, H);
const rgbBg = new Uint8Array(W * H * 3);
for (let i = 0; i < W * H; i++) { rgbBg[i * 3] = colFill[i * 3]; rgbBg[i * 3 + 1] = colFill[i * 3 + 1]; rgbBg[i * 3 + 2] = colFill[i * 3 + 2]; }
// 背景层：① 只在"近区膨胀 24px"内生成（掩膜外与前层同位，生成只会造成等深度交叠）
//        ② 深度乘 (1-2e-3) 做 tie 破断（与前层重叠处让前层稳定胜出）
const emitMask = Uint8Array.from(nearMask);
for (let rep = 0; rep < 24; rep++) {
  const prev = Uint8Array.from(emitMask);
  for (let y = 1; y < H - 1; y++) for (let x = 1; x < W - 1; x++) {
    const i = y * W + x;
    if (!prev[i] && (prev[i - 1] || prev[i + 1] || prev[i - W] || prev[i + W])) emitMask[i] = 1;
  }
}
const sFillBias = new Float32Array(W * H);
for (let i = 0; i < W * H; i++) sFillBias[i] = sFill[i] * (1 - 2e-3);
const meshBg = buildMesh(sFillBias, W, H, STRIDE, emitMask);
console.log(`  背景层：近区占比 ${(nearCnt / (W * H) * 100).toFixed(1)}%（膨胀 24px 后生成 ${meshBg.count} quads）`);

const blur = makeBlur(rgb, W, H, 16);
// 绘制顺序（z-test 之外，顺序只决定"等深度"时像素归属：先画者胜）
//   远景板 → 前景层 → 背景层：等深度处归前景层；前景层没覆盖的（消隐带、画幅边缘）由背景层/远景板承接
const layersAt = (pose, w, h, scale) => rasterize(pose, [{ mesh: meshFront, img: rgb, label: 1 }, { mesh: meshBg, img: rgbBg, label: 2 }], blur, w, h, scale);
const r0 = layersAt(poseOf(0, 0), W, H, 1);
const band = new Uint8Array(W * H);
for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
  const gxv = Math.abs(s[y * W + Math.min(W - 1, x + 1)] - s[y * W + Math.max(0, x - 1)]);
  const gyv = Math.abs(s[Math.min(H - 1, y + 1) * W + x] - s[Math.max(0, y - 1) * W + x]);
  if (gxv > 0.04 || gyv > 0.04) band[y * W + x] = 1;
}
{
  const b2 = Uint8Array.from(band);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) if (band[y * W + x]) {
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
      b2[Math.min(H - 1, Math.max(0, y + dy)) * W + Math.min(W - 1, Math.max(0, x + dx))] = 1;
    }
  }
  band.set(b2);
}
let exact = 0, nearPx = 0, maxd = 0, unwritten = 0;
let offBandTot = 0, offBandNear = 0, bandTot = 0, bandBad = 0;
for (let i = 0; i < W * H; i++) {
  const d = Math.max(Math.abs(r0.rgb[i * 3] - rgb[i * 3]), Math.abs(r0.rgb[i * 3 + 1] - rgb[i * 3 + 1]), Math.abs(r0.rgb[i * 3 + 2] - rgb[i * 3 + 2]));
  if (d === 0) exact++; if (d <= 1) nearPx++; if (d > maxd) maxd = d;
  if (!r0.written[i]) unwritten++;
  if (band[i]) { bandTot++; if (d > 1) bandBad++; }
  else { offBandTot++; if (d <= 1) offBandNear++; }
}
const offBandPct = offBandNear / offBandTot * 100;
console.log(`  逐像素比对：完全一致 ${(exact / (W * H) * 100).toFixed(2)}%，|Δ|≤1 ${(nearPx / (W * H) * 100).toFixed(3)}%，maxΔ=${maxd}，未写入=${unwritten}`);
console.log(`  台阶带：${bandTot}px（${(bandTot / (W * H) * 100).toFixed(2)}%，带内 |Δ|>1 的 ${bandBad}px）；带外恒等率 ${offBandPct.toFixed(3)}%`);
check('参考位姿恒等：台阶带外 |Δ|≤1 占比 ≥ 99.9%', offBandPct >= 99.9, `${offBandPct.toFixed(3)}%`);
check('台阶带占比 ≤ 3%', bandTot / (W * H) <= 0.03, `${(bandTot / (W * H) * 100).toFixed(2)}%`);

// ---------- [3] 位移量级守卫 ----------
console.log(`\n=== [3] 位移量级守卫（最近层 / 相邻关键帧 / 轨道总位移） ===`);
const stepNear = stepScan(Z_NEAR, THETA);
const stepFar = stepScan(ZF, THETA);
const slip = Math.abs(stepNear - stepFar);
console.log(`  stepNear=${stepNear.toFixed(1)}px  stepFar=${stepFar.toFixed(1)}px  slip=${slip.toFixed(1)}px`);
console.log(`  历史精确换算：成功轮(主体 z=1.53, 8° 步长) ≈70px；失败轮(z=1.36) ≈100px → 上界 ${GUARD_STEP_PX}px`);
console.log(`  下界标定（坑 131，2026-10-02 真机 fx=700）：Θ2°≈12px → no 3D points；Θ3°≈18px 模型碎；Θ5°≈31px 可建 → 下界 ${GUARD_STEP_MIN_PX}px`);
check(`最近层相邻关键帧总位移 ≤ ${GUARD_STEP_PX}px`, stepNear <= GUARD_STEP_PX, `${stepNear.toFixed(1)}px`);
check(`最近层相邻关键帧总位移 ≥ ${GUARD_STEP_MIN_PX}px（匹配下界，坑 131）`, stepNear >= GUARD_STEP_MIN_PX, `${stepNear.toFixed(1)}px`);
check(`近远层相对滑移 ≥ ${GUARD_SLIP_MIN}px（SfM 可分出两层）`, slip >= GUARD_SLIP_MIN, `${slip.toFixed(1)}px`);
// 坑 133（2026-10-02）：守护深度 = s 的 p90（面积自适应）。近层面积过小的图在 zNear 处位移
// 达标不代表实际内容达标；实装侧 Θ 钳制按此守护深度取 coef（近层充足时 ≈ zNear 口径，
// 行为不变）。该守卫为稳健性设计，必要性尚未被真机证伪/证实（见坑文档坑 133"轮 4 修正注记"）。
{
  const sortedS = Float32Array.from(s); sortedS.sort();
  const sP90 = sortedS[Math.floor(0.90 * (sortedS.length - 1))];
  const zGuard = 1 / (sP90 / Z_NEAR + (1 - sP90) / ZF);
  const coef = Math.max(0.05, 0.5 * Math.abs(zGuard - D0_USE) / zGuard);
  const stepGuard = Math.SQRT2 * coef * FX * THETA;
  console.log(`  守护深度：sP90=${sP90.toFixed(3)} → z=${zGuard.toFixed(3)}，coef=${coef.toFixed(3)}，位移=${stepGuard.toFixed(1)}px（实装 Θ 钳制同口径）`);
  check(`守护深度(sP90)位移 ≥ ${GUARD_STEP_MIN_PX}px（坑 133 面积自适应）`, stepGuard >= GUARD_STEP_MIN_PX, `${stepGuard.toFixed(1)}px`);
}
if (marker) {
  const findMarker = (rr) => {
    let sx = 0, sy = 0, cnt = 0;
    for (let y = 0; y < rr.H; y++) for (let x = 0; x < rr.W; x++) {
      const o = (y * rr.W + x) * 3;
      const r = rr.rgb[o], g = rr.rgb[o + 1], b = rr.rgb[o + 2];
      if ((r > 200 && g < 90 && b > 200) || (r < 90 && g > 200 && b < 90)) { sx += x + 0.5; sy += y + 0.5; cnt++; }
    }
    return cnt > 100 ? { x: sx / cnt, y: sy / cnt, cnt } : null;
  };
  const c0 = findMarker(r0);
  const dth = 2 * THETA / (KF_COUNT - 1);
  const rS = layersAt(poseOf(dth, dth), W, H, 1);
  const c1 = findMarker(rS);
  const pred = c0 ? offsetOf(c0.x, c0.y, Z_NEAR, { th: 0, ph: 0 }, { th: dth, ph: dth }) : null;
  if (c0 && c1 && pred) {
    const err = Math.hypot((c1.x - c0.x) - pred.du, (c1.y - c0.y) - pred.dv);
    console.log(`  标记位移：实测 (${(c1.x - c0.x).toFixed(1)}, ${(c1.y - c0.y).toFixed(1)})  公式 (${pred.du.toFixed(1)}, ${pred.dv.toFixed(1)})`);
    check('渲染器位移 == 投影公式（误差 ≤ 3px）', err <= 3, `err=${err.toFixed(2)}px`);
  } else { check('渲染器位移 == 投影公式', false, '标记未找到'); }
} else {
  skip('渲染器位移 == 投影公式', '真实模式无定位标记（由真机标定覆盖）');
}

// ---------- [4] 非共面 + 非退化守卫 ----------
console.log('\n=== [4] 非共面 / 非退化守卫（SfM 硬前提） ===');
check(`k=${K_R} ≥ 2.0`, K_R >= 2.0, `k=${K_R}`);
check('远层仍有可匹配位移（stepFar ≥ 5px）', stepFar >= 5, `${stepFar.toFixed(1)}px`);
// 坑 131（2026-10-02）：纯水平轨道 → (Y, fy) 精确不可观测 → Kit BA 各向异性漂移
// （真机 refined fy/fx = 0.155/2.47/2.69，成片宽度随机压缩/拉伸）。守卫一 = 相机中心必须
// 离开 y=0 平面（对角弧俯仰分量），数值上要求 Y 跨度达到 d0·sinΦ 的 90% 以上。
{
  const angs = kfAngles(THETA);
  const yMax = Math.max(...angs.map((a) => Math.abs(poseOf(a.th, a.ph).C.y)));
  const yNeed = 0.9 * D0_USE * Math.sin(PHI);
  check('轨道非退化：相机中心离开 y=0 平面（坑 131）', yMax > yNeed, `yMax=${yMax.toFixed(4)} ≥ ${yNeed.toFixed(4)} (0.9·d0·sinΦ)`);
}
// 守卫二（投影级，比"Y 跨度"更本质）：对世界做 (Y→αY, fy→fy/α) 变换后重投影——
// 纯水平弧下**所有帧所有点逐像素不变**（精确退化，已数值复现 0.00px）；对角弧下显著变化
// （fy/纵向尺度可观测）。alpha=2、阈 5px。
{
  const alpha = 2.0;
  const pts = [v3(0.3, 0.4, -1.5), v3(-0.5, 0.9, -2.5), v3(0.8, -0.6, -1.2)];
  const scanDeg = (withPhi) => {
    let maxDiff = 0;
    for (const a of kfAngles(THETA)) {
      const pose = poseOf(a.th, withPhi ? a.ph : 0);
      for (const P of pts) {
        const p1 = project(P, pose, FY), p2 = project(v3(P.x, alpha * P.y, P.z), pose, FY / alpha);
        if (p1 && p2) maxDiff = Math.max(maxDiff, Math.hypot(p2.u - p1.u, p2.v - p1.v));
      }
    }
    return maxDiff;
  };
  const diagDiff = scanDeg(true);
  const horizDiff = scanDeg(false);
  check('轨道非退化（投影级）：(Y→2Y, fy→fy/2) 变换下像素显著变化（坑 131）', diagDiff > 5,
    `对角弧 Δ=${diagDiff.toFixed(1)}px（纯水平弧恒 ${horizDiff.toFixed(2)}px，精确退化）`);
}

// ---------- [5][6] 轨道端点：三层占比 / 覆盖率 ----------
console.log('\n=== [5][6] 轨道端点帧：三层占比 / 覆盖率 / 帧缓冲口径 ===');
const SC = 0.5;
for (const [tag, th, ph] of [['+Θ', +THETA, +PHI], ['-Θ', -THETA, -PHI]]) {
  const rr = layersAt(poseOf(th, ph), W, H, SC);
  let front = 0, under = 0, bg = 0, unwritten = 0;
  for (let i = 0; i < rr.W * rr.H; i++) {
    if (!rr.written[i]) unwritten++;
    if (rr.label[i] === 1) front++; else if (rr.label[i] === 2) under++; else bg++;
  }
  const n = rr.W * rr.H;
  console.log(`  ${tag} 帧（${rr.W}x${rr.H}）：前景 ${(front / n * 100).toFixed(1)}% / 背景层 ${(under / n * 100).toFixed(1)}% / 远景板 ${(bg / n * 100).toFixed(1)}%，未写入 ${unwritten}`);
  check(`${tag} 端点：前景照片占比 ≥ ${(GUARD_PHOTO_RATIO * 100).toFixed(0)}%`, front / n >= GUARD_PHOTO_RATIO, `${(front / n * 100).toFixed(1)}%`);
  check(`${tag} 端点：消隐带主要由背景层承接（背景层 ≥ 1% 且 远景板 ≤ 10%）`, under / n >= 0.01 && bg / n <= 0.10, `under=${(under / n * 100).toFixed(2)}% bg=${(bg / n * 100).toFixed(2)}%`);
  check(`${tag} 端点：覆盖率 100%`, unwritten === 0, `unwritten=${unwritten}`);
  writePng(path.join(OUT_DIR, tag === '+Θ' ? 'dm_end_pos.png' : 'dm_end_neg.png'), rr.W, rr.H, rr.rgb);
  if (tag === '+Θ') {   // 三层归属粗网格（诊断：# 前景 / + 背景层 / . 远景板）
    const gw = 40, gh = 40, rows = Math.floor(rr.H / gh), cols = Math.floor(rr.W / gw);
    let map = '';
    for (let gy = 0; gy < rows; gy++) {
      for (let gx = 0; gx < cols; gx++) {
        let f = 0, u = 0, b = 0;
        for (let y = gy * gh; y < (gy + 1) * gh; y++) for (let x = gx * gw; x < (gx + 1) * gw; x++) {
          const l = rr.label[y * rr.W + x];
          if (l === 1) f++; else if (l === 2) u++; else b++;
        }
        map += f > u && f > b ? '#' : (u >= f && u > b ? '+' : '.');
      }
      map += '\n    ';
    }
    console.log(`  归属图（每格 ${gw}x${gh}px，# 前景 / + 背景层 / . 远景板）：\n    ${map}`);
  }
}
check('帧缓冲口径：紧凑 RGB、长度 == 宽×高×3（C++ 侧同款断言：行跨距 == 宽×3）',
  r0.rgb.length === OUT_W * OUT_H * 3, `${OUT_W}×3=${OUT_W * 3}`);

// ---------- [7] 实装一致性（源码级；C++ 未实装时 PENDING） ----------
console.log('\n=== [7] 实装一致性（源码级守卫；C++ 未实装时 PENDING） ===');
const cppPath = path.join(__dirname, '..', '..', 'src', 'cpp', 'spatial_recon_bridge.cpp');
if (fs.existsSync(cppPath)) {
  const cpp = fs.readFileSync(cppPath, 'utf8');
  const has = (re) => re.test(cpp);
  if (!(has(/kDepthZNear/) || has(/SrRasterizeDepthMesh/))) {
    skip('cpp 源码守卫', 'PENDING：bridge 存在但深度网格尚未实装（阶段2 实装后本判据自动生效）');
  } else {
    check('cpp 含深度网格常量 kDepthZNear/kDepthRatio/kDepthTheta', has(/kDepthZNear/) && has(/kDepthRatio/) && has(/kDepthTheta/), '');
    check('cpp 深度网格实装入口存在（SrRasterizeDepthMesh 等）', has(/SrRasterizeDepthMesh|depthMeshRasterize/), '');
    check('cpp 轨道含俯仰自由度（SrFrameBasis 带 pitchDeg，坑 131 对角弧）',
      /void SrFrameBasis\(int32_t i, int32_t n, float orbitDeg, float pitchDeg/.test(cpp), '');
    check('cpp scene=5 调用点俯仰接线（Φ=Θ）',
      /\(scene == 5\) \? orbitDeg : 0\.0f/.test(cpp), '');
    check('cpp Θ 钳制取 sP90 守护深度（坑 133）', /sP90/.test(cpp) && /SRCHK orbit clamp/.test(cpp), '');
    if (has(/kDepthRatio\s*=\s*([0-9.]+)/)) {
      const m = cpp.match(/kDepthRatio\s*=\s*([0-9.]+)/);
      check('cpp kDepthRatio == 判据脚本 K_RATIO', Math.abs(parseFloat(m[1]) - K_R) < 1e-6, `cpp=${m[1]} js=${K_R}`);
    }
    if (has(/kDepthZNear\s*=\s*([0-9.]+)/)) {
      const m = cpp.match(/kDepthZNear\s*=\s*([0-9.]+)/);
      check('cpp kDepthZNear == 判据脚本 Z_NEAR', Math.abs(parseFloat(m[1]) - Z_NEAR) < 1e-6, `cpp=${m[1]} js=${Z_NEAR}`);
    }
    if (has(/kDepthThetaDeg\s*=\s*([0-9.]+)/)) {
      const m = cpp.match(/kDepthThetaDeg\s*=\s*([0-9.]+)/);
      check('cpp kDepthThetaDeg == 判据脚本 Θ', Math.abs(parseFloat(m[1]) - THETA_DEG) < 1e-6, `cpp=${m[1]} js=${THETA_DEG}`);
    }
    // 坑 129 守卫（2026-10-01 真机首测失败的根因；两条一并守）：
    //   ① 世界点必须"一次性固化"：SrBakeDepthMeshWorld 在构建时按参考位姿反投影、逐帧只做投影。
    //      若在逐帧渲染里用**当前帧** basis 重建世界点 → 顶点投影恒等于其像素坐标（u ≡ q.x，与 θ 无关）
    //      → 72 帧画面逐帧完全相同（零视差）、只有上报位姿在动 → 特征匹配"有图像无基线"→ 三角化 0 点
    //      （真机 `[GSRECON][STAGE02] no 3D points` → `STAGE03 Adapting failed` → 降级 scene=1）。
    //   ② 网格坐标必须映射到画布：SrBuildDepthMesh 要传 kOutW/kOutH（深度场 546×728 ≠ 画布 1080×1440，
    //      否则照片缩在画幅左上角、参考位姿不恒等——`SRCHK fmid` ASCII 一判即知）。
    check('cpp 世界点一次性固化（SrBakeDepthMeshWorld 存在）', has(/SrBakeDepthMeshWorld/), '');
    check('cpp 深度网格构建传入画布尺寸（网格坐标映射到画布空间）',
      /SrBuildDepthMesh\([^;]*kOutW[\s\S]{0,40}kOutH/.test(cpp), '');
    {
      const m = cpp.match(/const auto quadToScreen[\s\S]*?\n    \};/);
      const body = m ? m[0] : '';
      check('cpp 逐帧网格变换只做投影、不得用 basis 重建世界点（坑 129）',
        body.length > 0 && !/basis/.test(body), body.length > 0 ? '' : '未找到 quadToScreen');
    }
  }
} else {
  skip('cpp 源码守卫', 'PENDING：spatial_recon_bridge.cpp 尚未实装深度网格（阶段2 实装后本判据自动生效）');
}
// 单源纪律守卫（阶段2）：纯 2D 单元 spatial_recon_depth_mesh.cpp 只做"深度场→四边形/光栅化"，
// **不得出现第二份相机/射线/内参数学**（相机数学单源纪律）
{
  const dmPath = path.join(__dirname, '..', '..', 'src', 'cpp', 'spatial_recon_depth_mesh.cpp');
  if (fs.existsSync(dmPath)) {
    const dm = fs.readFileSync(dmPath, 'utf8');
    check('depth_mesh 单元含 SrBuildDepthMesh / SrRasterizeQuads',
      /SrBuildDepthMesh/.test(dm) && /SrRasterizeQuads/.test(dm));
    const leakTokens = dm.match(/rayDir|frameBasis|basis\[|SrPixelRay|kFaceZ|kOutW/g) || [];
    check('depth_mesh 单元无第二份相机/射线数学（单源纪律）', leakTokens.length === 0,
      leakTokens.length > 0 ? `检测到：${[...new Set(leakTokens)].join(',')}` : '');
  } else {
    skip('depth_mesh 单源纪律守卫', 'PENDING：spatial_recon_depth_mesh.cpp 尚未落地');
  }
}

// ---------- [8] 出图 ----------
console.log('\n=== [8] 出图（肉眼核验：视差方向 / 无条纹 / 无拉伸） ===');
writePng(path.join(OUT_DIR, 'dm_ref.png'), W, H, r0.rgb);
{
  const dcol = new Uint8Array(W * H * 3);
  for (let i = 0; i < W * H; i++) { const v = Math.round(s[i] * 255); dcol[i * 3] = v; dcol[i * 3 + 1] = v; dcol[i * 3 + 2] = v; }
  writePng(path.join(OUT_DIR, 'dm_depth.png'), W, H, dcol);
  const bcol = new Uint8Array(W * H * 3);
  for (let i = 0; i < W * H; i++) { bcol[i * 3] = rgbBg[i * 3]; bcol[i * 3 + 1] = rgbBg[i * 3 + 1]; bcol[i * 3 + 2] = rgbBg[i * 3 + 2]; }
  writePng(path.join(OUT_DIR, 'dm_bg_fill.png'), W, H, bcol);   // 背景层补洞结果（消隐带将显示的内容）
}
console.log(`  已出图：${OUT_DIR}/dm_ref.png（θ=0）/ dm_depth.png（深度，近=亮）/ dm_bg_fill.png（背景层补洞）/ dm_end_pos.png / dm_end_neg.png（两端对翻看视差）`);

// ---------- 参数扫描 ----------
console.log(`\n=== 参数扫描（对角弧精确投影口径；位移带 ${GUARD_STEP_MIN_PX}~${GUARD_STEP_PX}px / 裙边 ${SKIRT}px） ===`);
for (const thDeg of [3, 4, 5, 6, 8]) {
  const rad = thDeg * Math.PI / 180;
  const sn = stepScan(Z_NEAR, rad), sf = stepScan(ZF, rad);
  const span = 2 * Math.abs(offsetOf(CX, CY, Z_NEAR, { th: 0, ph: 0 }, { th: rad, ph: rad }).du);
  const endShift = FX * rad;   // 近层"中心→端点"单轴位移 ≈ fx·Θ_rad（裙边承接上限）
  const okScan = sn >= GUARD_STEP_MIN_PX && sn <= GUARD_STEP_PX && endShift <= SKIRT;
  console.log(`  Θ=±${thDeg}°(Φ=Θ)：stepNear=${sn.toFixed(1)}px  stepFar=${sf.toFixed(1)}px  slip=${Math.abs(sn - sf).toFixed(1)}px  端点单轴位移≈${endShift.toFixed(0)}px  全轨道近层总位移≈${span.toFixed(0)}px  ${okScan ? '✅' : '❌ 超界'}`);
}

// ---------- 合成模式：dump + 回读自检 ----------
if (marker) {
  console.log('\n=== [附加] dump 交换格式（真实模式同款）并回读自检 ===');
  fs.mkdirSync(path.join(__dirname, 'out'), { recursive: true });
  const p = path.join(__dirname, 'out', 'synth');
  fs.writeFileSync(p + '.depth.f32.bin', Buffer.from(raw.buffer, raw.byteOffset, raw.length * 4));
  const rgba = new Uint8Array(W * H * 4);
  for (let i = 0; i < W * H; i++) { rgba[i * 4] = rgb[i * 3]; rgba[i * 4 + 1] = rgb[i * 3 + 1]; rgba[i * 4 + 2] = rgb[i * 3 + 2]; rgba[i * 4 + 3] = 255; }
  fs.writeFileSync(p + '.rgba.bin', Buffer.from(rgba.buffer));
  fs.writeFileSync(p + '.json', JSON.stringify({ w: W, h: H, src: 'synthetic', onnx_sha256: null, input_shape: [1, 3, 728, 546] }, null, 2));
  const li = loadInputs(p);
  let same = true;
  for (let i = 0; i < W * H; i++) if (Math.abs(li.raw[i] - raw[i]) > 1e-9) { same = false; break; }
  check('回读一致（*.depth.f32.bin / *.rgba.bin 交换格式可用）', same && li.w === W && li.h === H, `${p}.*`);
  console.log(`  真实模式用法：node tools/depth_mesh/depth_mesh_check.js --input=${p}`);
}

console.log(`\n=== 汇总：PASS ${pass} / FAIL ${fail} ===（耗时 ${((Date.now() - t0) / 1000).toFixed(1)}s）`);
process.exit(fail ? 1 : 0);
