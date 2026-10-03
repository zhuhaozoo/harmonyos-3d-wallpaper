'use strict';
/**
 * scene5_render_check.js —— 路线B（scene=5 深度网格）合成帧的离线复现与核验工具。
 *
 * 用途（一份脚本两个模式）：
 *   cpp 模式：复刻 **2026-10-01 首测时的 native 实装语义**（spatial_recon_depth_mesh.cpp +
 *             bridge SrRasterizeDepthMesh：深度场坐标直落画布 + 逐帧按当前机位重建世界点）。
 *             该语义已被判定为坑 129（零视差 + 参考位姿不恒等），**代码已修**——本模式保留用于
 *             回归取证（对着首测日志复现：设备 f000/fmid 与本模式 1499/1500 格吻合）。
 *   fix 模式：按离线判据 depth_mesh_check.js 的口径渲染（网格像素坐标 → 画布坐标缩放；
 *             世界点由参考位姿一次性反投影得到，逐帧只做投影）——即**修复后的实装语义**。用于验证：
 *               ① 参考位姿恒等（θ=0 帧 == 源照片）
 *               ② 真实视差存在（θ=±Θ 帧 ≠ θ=0 帧）
 *               ③ 位移量级与判据守卫一致
 *
 * 用法：
 *   node tools/depth_mesh/scene5_render_check.js \
 *     --depth=tools/depth_mesh/out/dev5.depth.f32.bin \
 *     --rgba=tools/depth_mesh/out/dev5.rgba.bin \
 *     --frame=tools/depth_mesh/out/dev5f.rgba.bin \
 *     [--log=<hilog 文本路径>]
 *
 * 说明：`--depth/--rgba` 是端侧同口径（546x728）的深度/贴图；`--frame` 是画布口径（1080x1440）
 * 的源照片（fix 模式的前景贴图）；`--log` 给了就做设备比对。
 */
const fs = require('fs');
const path = require('path');

const args = {};
process.argv.slice(2).forEach((a) => { const m = a.match(/^--([^=]+)=?(.*)$/); if (m) args[m[1]] = m[2]; });
const P_DEPTH = args.depth || 'tools/depth_mesh/out/dev5.depth.f32.bin';
const P_RGBA = args.rgba || 'tools/depth_mesh/out/dev5.rgba.bin';
const P_FRAME = args.frame || 'tools/depth_mesh/out/dev5f.rgba.bin';
const P_LOG = args.log || '';
const N_FRAMES = parseInt(args.n || '72', 10);
const THETA_DEG = parseFloat(args.theta || '5');

// ---------- 常量（与实现文档 §附录B / spatial_recon_bridge.cpp 对齐） ----------
const OUT_W = 1080, OUT_H = 1440;
const CX = OUT_W / 2, CY = OUT_H / 2;
const FX = 1500, FY = 1500;
const Z_NEAR = 1.0, K_RATIO = 3.0, ZF = Z_NEAR * K_RATIO, D0 = (Z_NEAR + ZF) / 2;
const BG_Z_FACTOR = 2.5, BG_Z = BG_Z_FACTOR * ZF;
const BG_HALF_W = 4.4, BG_HALF_H = 7.2;
const STRIDE = 4, SKIRT = 80, REFINE_RATIO = 1.15, NEAR_SPLIT_S = 0.5;
const DILATE_RADIUS = 24, BLUR_F = 16;

// ---------- IO ----------
function readF32(p) { const b = fs.readFileSync(p); return new Float32Array(b.buffer, b.byteOffset, b.length / 4); }
function readU8(p) { return new Uint8Array(fs.readFileSync(p)); }
function rgbaToRgb(u8, w, h) {
  const out = new Uint8Array(w * h * 3);
  for (let i = 0, o = 0; i < w * h; i++, o += 4) { out[i * 3] = u8[o]; out[i * 3 + 1] = u8[o + 1]; out[i * 3 + 2] = u8[o + 2]; }
  return out;
}

// ---------- 深度归一化（与 spatial_recon_depth_mesh.cpp 的 Percentiles 同口径：4096 直方图） ----------
function percentilesCpp(a) {
  let mn = a[0], mx = a[0];
  for (let i = 1; i < a.length; i++) { if (a[i] < mn) mn = a[i]; if (a[i] > mx) mx = a[i]; }
  const BINS = 4096;
  const hist = new Int32Array(BINS);
  const span = (mx - mn) > 1e-9 ? (mx - mn) : 1.0;
  for (let i = 0; i < a.length; i++) {
    let b = Math.floor((a[i] - mn) / span * (BINS - 1));
    if (b < 0) b = 0; if (b > BINS - 1) b = BINS - 1;
    hist[b]++;
  }
  const lo = Math.floor(0.01 * a.length), hi = Math.floor(0.99 * a.length);
  let acc = 0, p1 = mn, p99 = mx;
  for (let b = 0; b < BINS; b++) {
    acc += hist[b];
    const val = mn + span * (b + 0.5) / BINS;
    if (acc >= lo && p1 === mn) p1 = val;
    if (acc >= hi) { p99 = val; break; }
  }
  return { p1, p99 };
}
const zOfS = (s) => 1 / (s / Z_NEAR + (1 - s) / ZF);

// ---------- 双线性采样（连续坐标：像素中心在 px+0.5；越界钳制） ----------
function sampleS(s, w, h, x, y) {
  const fx = Math.min(w - 1.001, Math.max(0, x - 0.5));
  const fy = Math.min(h - 1.001, Math.max(0, y - 0.5));
  const x0 = Math.floor(fx), y0 = Math.floor(fy);
  const tx = fx - x0, ty = fy - y0;
  const x1 = Math.min(w - 1, x0 + 1), y1 = Math.min(h - 1, y0 + 1);
  return (s[y0 * w + x0] * (1 - tx) + s[y0 * w + x1] * tx) * (1 - ty) +
         (s[y1 * w + x0] * (1 - tx) + s[y1 * w + x1] * tx) * ty;
}
function sampleRgb(img, w, h, x, y, ch, out) {
  const fx = Math.min(w - 1.001, Math.max(0, x - 0.5));
  const fy = Math.min(h - 1.001, Math.max(0, y - 0.5));
  const x0 = Math.floor(fx), y0 = Math.floor(fy);
  const tx = fx - x0, ty = fy - y0;
  const x1 = Math.min(w - 1, x0 + 1), y1 = Math.min(h - 1, y0 + 1);
  for (let c = 0; c < 3; c++) {
    const v00 = img[(y0 * w + x0) * ch + c], v10 = img[(y0 * w + x1) * ch + c];
    const v01 = img[(y1 * w + x0) * ch + c], v11 = img[(y1 * w + x1) * ch + c];
    out[c] = (v00 * (1 - tx) + v10 * tx) * (1 - ty) + (v01 * (1 - tx) + v11 * tx) * ty;
  }
}

// ---------- push-pull 补洞（与 native/判据同款：金字塔 + 顶层播种/均值 + 双线性 push） ----------
function pushPullFill(field, ch, mask, w, h) {
  const levs = [];
  {
    const wt = new Float32Array(w * h);
    for (let i = 0; i < w * h; i++) wt[i] = mask[i] ? 0 : 1;
    levs.push({ v: Float32Array.from(field), wt, w, h });
  }
  let cw = w, chh = h;
  while (cw > 4 && chh > 4) {
    const cur = levs[levs.length - 1];
    const nw = (cw + 1) >> 1, nh = (chh + 1) >> 1;
    const nx = { v: new Float32Array(nw * nh * ch), wt: new Float32Array(nw * nh), w: nw, h: nh };
    for (let y = 0; y < nh; y++) for (let x = 0; x < nw; x++) {
      let wt = 0; const acc = [0, 0, 0];
      for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) {
        const sx = Math.min(cw - 1, x * 2 + dx), sy = Math.min(chh - 1, y * 2 + dy);
        const k = sy * cw + sx, wk = cur.wt[k];
        if (wk <= 0) continue;
        for (let c = 0; c < ch; c++) acc[c] += cur.v[k * ch + c] * wk;
        wt += wk;
      }
      const k2 = y * nw + x;
      if (wt > 0) { for (let c = 0; c < ch; c++) nx.v[k2 * ch + c] = acc[c] / wt; nx.wt[k2] = wt; }
    }
    levs.push(nx); cw = nw; chh = nh;
  }
  const top = levs[levs.length - 1];
  const gmean = [0, 0, 0];
  {
    const l0 = levs[0]; let gwt = 0;
    for (let i = 0; i < l0.wt.length; i++) {
      const wk = l0.wt[i]; if (wk <= 0) continue;
      for (let c = 0; c < ch; c++) gmean[c] += l0.v[i * ch + c] * wk;
      gwt += wk;
    }
    if (gwt > 0) for (let c = 0; c < ch; c++) gmean[c] /= gwt;
  }
  for (let pass = 0; pass < 16; pass++) {
    let changed = 0;
    const pv = Float32Array.from(top.v), pw = Float32Array.from(top.wt);
    for (let y = 0; y < top.h; y++) for (let x = 0; x < top.w; x++) {
      const k = y * top.w + x; if (pw[k] > 0) continue;
      let wt = 0; const acc = [0, 0, 0];
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
        const xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= top.w || yy >= top.h) continue;
        const kk = yy * top.w + xx; if (pw[kk] <= 0) continue;
        for (let c = 0; c < ch; c++) acc[c] += pv[kk * ch + c];
        wt++;
      }
      if (wt > 0) { for (let c = 0; c < ch; c++) top.v[k * ch + c] = acc[c] / wt; top.wt[k] = 1e-6; changed++; }
    }
    if (!changed) break;
  }
  for (let i = 0; i < top.wt.length; i++) {
    if (top.wt[i] > 0) continue;
    for (let c = 0; c < ch; c++) top.v[i * ch + c] = gmean[c];
    top.wt[i] = 1e-6;
  }
  for (let L = levs.length - 1; L > 0; L--) {
    const up = levs[L], dn = levs[L - 1];
    for (let y = 0; y < up.h; y++) for (let x = 0; x < up.w; x++) {
      const ku = y * up.w + x; if (up.wt[ku] <= 0) continue;
      for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) {
        const sx = Math.min(dn.w - 1, x * 2 + dx), sy = Math.min(dn.h - 1, y * 2 + dy);
        const kd = sy * dn.w + sx; if (dn.wt[kd] > 0) continue;
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
  field.set(levs[0].v);
}
function dilateMask(mask, w, h, radius) {
  const tmp = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    let v = 0;
    for (let xx = Math.max(0, x - radius); xx <= Math.min(w - 1, x + radius) && !v; xx++) v = mask[y * w + xx];
    tmp[y * w + x] = v;
  }
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    const y0 = Math.max(0, y - radius), y1 = Math.min(h - 1, y + radius);
    for (let x = 0; x < w; x++) {
      let v = 0;
      for (let yy = y0; yy <= y1 && !v; yy++) v = tmp[yy * w + x];
      out[y * w + x] = v;
    }
  }
  return out;
}
function makeBlur(src, w, h, f) {
  const bw = Math.max(1, Math.ceil(w / f)), bh = Math.max(1, Math.ceil(h / f));
  const out = new Uint8Array(bw * bh * 3);
  for (let by = 0; by < bh; by++) for (let bx = 0; bx < bw; bx++) {
    let r = 0, g = 0, b = 0, n = 0;
    for (let y = by * f; y < Math.min(h, (by + 1) * f); y++) for (let x = bx * f; x < Math.min(w, (bx + 1) * f); x++) {
      const o = (y * w + x) * 3; r += src[o]; g += src[o + 1]; b += src[o + 2]; n++;
    }
    const o = (by * bw + bx) * 3;
    if (n > 0) { out[o] = r / n | 0; out[o + 1] = g / n | 0; out[o + 2] = b / n | 0; }
  }
  return { img: out, w: bw, h: bh };
}
/** 双线性升采样（texel 中心映射；与 native SampleRgb 语义一致） */
function upsampleRgb(src, w, h, ow, oh) {
  const out = new Uint8Array(ow * oh * 3);
  const col = new Float32Array(3);
  for (let y = 0; y < oh; y++) for (let x = 0; x < ow; x++) {
    sampleRgb(src, w, h, (x + 0.5) * w / ow, (y + 0.5) * h / oh, 3, col);
    const o = (y * ow + x) * 3;
    out[o] = Math.max(0, Math.min(255, Math.round(col[0])));
    out[o + 1] = Math.max(0, Math.min(255, Math.round(col[1])));
    out[o + 2] = Math.max(0, Math.min(255, Math.round(col[2])));
  }
  return out;
}

// ---------- 网格构建（与 spatial_recon_depth_mesh.cpp 同构；四元组 = px,py,z,u,v） ----------
function buildMesh(sField, w, h) {
  const front = [], bg = [];
  const pushV = (arr, x, y, z) => { arr.push(x, y, z, x, y); };
  let coarse = 0, refined = 0, maxRatio = 1;
  for (let j = -SKIRT; j + STRIDE <= h + SKIRT; j += STRIDE) {
    for (let i = -SKIRT; i + STRIDE <= w + SKIRT; i += STRIDE) {
      const zs = [[i, j], [i + STRIDE, j], [i + STRIDE, j + STRIDE], [i, j + STRIDE]]
        .map(([x, y]) => zOfS(sampleS(sField, w, h, x, y)));
      const zmin = Math.min(...zs), zmax = Math.max(...zs);
      const ratio = zmax / Math.max(1e-9, zmin);
      if (ratio > REFINE_RATIO) {
        for (let yy = j; yy < j + STRIDE; yy++) for (let xx = i; xx < i + STRIDE; xx++) {
          const zc = zOfS(sampleS(sField, w, h, xx + 0.5, yy + 0.5));
          pushV(front, xx, yy, zc); pushV(front, xx + 1, yy, zc); pushV(front, xx + 1, yy + 1, zc); pushV(front, xx, yy + 1, zc);
          refined++;
        }
      } else {
        pushV(front, i, j, zs[0]); pushV(front, i + STRIDE, j, zs[1]);
        pushV(front, i + STRIDE, j + STRIDE, zs[2]); pushV(front, i, j + STRIDE, zs[3]);
        coarse++; maxRatio = Math.max(maxRatio, ratio);
      }
    }
  }
  // 背景层：近区 push-pull 补洞 + 膨胀范围生成 + tie 破断
  const nearMask = new Uint8Array(w * h);
  let nearCnt = 0;
  for (let i = 0; i < w * h; i++) if (sField[i] > NEAR_SPLIT_S) { nearMask[i] = 1; nearCnt++; }
  const sFill = Float32Array.from(sField);
  pushPullFill(sFill, 1, nearMask, w, h);
  for (let i = 0; i < w * h; i++) sFill[i] *= (1 - 2e-3);
  const emitMask = dilateMask(nearMask, w, h, DILATE_RADIUS);
  for (let j = 0; j + STRIDE <= h; j += STRIDE) {
    for (let i = 0; i + STRIDE <= w; i += STRIDE) {
      let any = 0;
      for (let yy = j; yy <= j + STRIDE && !any; yy += STRIDE) for (let xx = i; xx <= i + STRIDE && !any; xx += STRIDE) {
        if (emitMask[Math.min(h - 1, yy) * w + Math.min(w - 1, xx)]) any = 1;
      }
      if (!any) continue;
      pushV(bg, i, j, zOfS(sampleS(sFill, w, h, i, j)));
      pushV(bg, i + STRIDE, j, zOfS(sampleS(sFill, w, h, i + STRIDE, j)));
      pushV(bg, i + STRIDE, j + STRIDE, zOfS(sampleS(sFill, w, h, i + STRIDE, j + STRIDE)));
      pushV(bg, i, j + STRIDE, zOfS(sampleS(sFill, w, h, i, j + STRIDE)));
    }
  }
  return { front, bg, coarse, refined, nearCnt, maxRatio };
}

// ---------- 相机（与 SrFrameBasis 同源：列 = [right | up | -fwd]） ----------
function poseOf(theta) {
  const D = D0, s = Math.sin(theta), c = Math.cos(theta);
  return { C: [D * s, 0, -D * (1 - c)], s, c };
}
function project(P, pose) {
  const dx = P[0] - pose.C[0], dy = P[1] - pose.C[1], dz = P[2] - pose.C[2];
  const px = pose.c * dx - pose.s * dz, py = dy, pz = pose.s * dx + pose.c * dz;
  const d = -pz;
  if (d <= 1e-9) return null;
  return { u: CX + FX * px / d, v: CY - FY * py / d, d };
}
/** 逐帧世界点：cpp 模式 = 用当前帧机位重建（实装现状）；fix 模式 = 参考位姿一次性反投影（画布坐标） */
function worldPointCpp(px, py, z, pose) {
  const a = (px - CX) / FX, b = -(py - CY) / FY;
  const fwd = [-pose.s, 0, -pose.c];            // = -Z_cam
  const X = [pose.c, 0, -pose.s], Y = [0, 1, 0]; // 列 = [right, up]
  return [pose.C[0] + (a * X[0] + b * Y[0] + fwd[0]) * z,
          pose.C[1] + (a * X[1] + b * Y[1] + fwd[1]) * z,
          pose.C[2] + (a * X[2] + b * Y[2] + fwd[2]) * z];
}
function worldPointFix(px, py, z, sx, sy) {
  const a = (px * sx - CX) / FX, b = -(py * sy - CY) / FY;
  return [a * z, b * z, -z];
}

// ---------- 光栅化（三层：远景板 → 前景层 → 背景层；共享 z-buffer；等深度先画者胜） ----------
function rasterizeTri(p0, p1, p2, uv0, uv1, uv2, tex, mode, rgb, zbuf, written) {
  const dz = [1 / p0.d, 1 / p1.d, 1 / p2.d];
  const area = (p1.u - p0.u) * (p2.v - p0.v) - (p2.u - p0.u) * (p1.v - p0.v);
  if (Math.abs(area) < 1e-9) return;
  const inv = 1 / area;
  const minX = Math.max(0, Math.floor(Math.min(p0.u, p1.u, p2.u)));
  const maxX = Math.min(OUT_W - 1, Math.ceil(Math.max(p0.u, p1.u, p2.u)));
  const minY = Math.max(0, Math.floor(Math.min(p0.v, p1.v, p2.v)));
  const maxY = Math.min(OUT_H - 1, Math.ceil(Math.max(p0.v, p1.v, p2.v)));
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
      const idx = y * OUT_W + x;
      if (zInv <= zbuf[idx]) continue;
      zbuf[idx] = zInv;
      written[idx] = mode;   // 1=前景 / 2=背景层 / 0=远景板
      const U = (l0 * uv0[0] * dz[0] + l1 * uv1[0] * dz[1] + l2 * uv2[0] * dz[2]) / zInv;
      const V = (l0 * uv0[1] * dz[0] + l1 * uv1[1] * dz[1] + l2 * uv2[1] * dz[2]) / zInv;
      const col = [0, 0, 0];
      if (mode === 0) {
        const t = tex.blur;
        sampleRgb(t.img, t.w, t.h, U / tex.blurDiv, V / tex.blurDiv, 3, col);
        col[0] *= 0.72; col[1] *= 0.72; col[2] *= 0.75;
      } else if (mode === 1) {
        sampleRgb(tex.front.img, tex.front.w, tex.front.h, U, V, 3, col);
      } else {
        sampleRgb(tex.bg.img, tex.bg.w, tex.bg.h, U, V, 3, col);
      }
      rgb[idx * 3] = col[0]; rgb[idx * 3 + 1] = col[1]; rgb[idx * 3 + 2] = col[2];
    }
  }
}

function renderFrame(i, n, orbitDeg, mesh, tex, cppMode) {
  const theta = (orbitDeg * Math.PI / 180) * (2 * i / (n - 1) - 1);
  const pose = poseOf(theta);
  const sx = OUT_W / mesh.w, sy = OUT_H / mesh.h;
  const rgb = new Uint8Array(OUT_W * OUT_H * 3);
  const zbuf = new Float32Array(OUT_W * OUT_H);
  const written = new Uint8Array(OUT_W * OUT_H);
  // ① 远景板
  {
    const corners = [[-BG_HALF_W, BG_HALF_H, -BG_Z], [BG_HALF_W, BG_HALF_H, -BG_Z],
                     [BG_HALF_W, -BG_HALF_H, -BG_Z], [-BG_HALF_W, -BG_HALF_H, -BG_Z]];
    const uv = corners.map((P) => [CX + P[0] * (FX / BG_Z), CY - P[1] * (FY / BG_Z)]);
    const scr = corners.map((P) => project(P, pose));
    if (scr.every((v) => v)) {
      rasterizeTri(scr[0], scr[1], scr[2], uv[0], uv[1], uv[2], tex, 0, rgb, zbuf, written);
      rasterizeTri(scr[0], scr[2], scr[3], uv[0], uv[2], uv[3], tex, 0, rgb, zbuf, written);
    }
  }
  // ②③ 网格层
  const draw = (quads, layerId) => {
    for (let qi = 0; qi + 19 < quads.length; qi += 20) {
      const scr = [], uv = [];
      let ok = true;
      for (let vi = 0; vi < 4; vi++) {
        const o = qi + vi * 5;
        const P = cppMode ? worldPointCpp(quads[o], quads[o + 1], quads[o + 2], pose)
                          : worldPointFix(quads[o], quads[o + 1], quads[o + 2], sx, sy);
        const s = project(P, pose);
        if (!s) { ok = false; break; }
        scr.push(s);
        uv.push(cppMode ? [quads[o + 3], quads[o + 4]]
                        : [quads[o + 3] * sx, quads[o + 4] * sy]);   // fix：画布坐标 → 画布贴图
      }
      if (!ok) continue;
      rasterizeTri(scr[0], scr[1], scr[2], uv[0], uv[1], uv[2], tex, layerId, rgb, zbuf, written);
      rasterizeTri(scr[0], scr[2], scr[3], uv[0], uv[2], uv[3], tex, layerId, rgb, zbuf, written);
    }
  };
  draw(mesh.front, 1);
  draw(mesh.bg, 2);
  let miss = 0;
  for (let k = 0; k < written.length; k++) if (written[k] === 0) miss++;   // 远景板 = "无几何命中"（与 native 契约近似）
  return { rgb, written, theta };
}

// ---------- ASCII 缩略图（与 SrLogAscii 同口径：48x32 点采样 + 10 级 ramp） ----------
const RAMP = ' .:-=+*#%@';
function asciiOf(rgb, w, h, ch) {
  const rows = [];
  for (let r = 0; r < 32; r++) {
    let line = '';
    for (let c = 0; c < 48; c++) {
      const x = Math.floor(c * w / 48), y = Math.floor(r * h / 32);
      const o = (y * w + x) * ch;
      const lum = Math.floor((rgb[o] * 30 + rgb[o + 1] * 59 + rgb[o + 2] * 11) / 100);
      line += RAMP[Math.floor(lum * 9 / 255)];
    }
    rows.push(line);
  }
  return rows;
}
function printAscii(tag, rows) { rows.forEach((l, r) => console.log(`  ${tag} r${String(r).padStart(2, '0')} |${l}|`)); }
function diffAscii(a, b) {
  let same = 0, d1 = 0, d2 = 0; const map = [];
  for (let r = 0; r < 32; r++) {
    let line = '';
    for (let c = 0; c < 48; c++) {
      const d = Math.abs(RAMP.indexOf(a[r][c]) - RAMP.indexOf(b[r][c]));
      if (d === 0) { same++; line += '.'; } else if (d === 1) { d1++; line += '1'; } else { d2++; line += 'X'; }
    }
    map.push(line);
  }
  return { same, d1, d2, map };
}

// ---------- 设备日志解析 ----------
/** 取**首次**出现的 tag（同一 log 里 scene=5 与降级 scene=1 各有一套 f000/fmid） */
function logAscii(logPath, tag) {
  const lines = fs.readFileSync(logPath, 'latin1').split('\n');
  const rows = [];
  for (const line of lines) {
    const i = line.indexOf('SRCHK ' + tag + ' r');
    if (i < 0) continue;
    const r = parseInt(line.substr(i + 6 + tag.length + 2, 2), 10);
    if (rows[r] !== undefined) continue;   // 首次出现优先
    const a = line.indexOf('|', i), b = line.lastIndexOf('|');
    if (a < 0 || b <= a) continue;
    rows[r] = line.substring(a + 1, b);
  }
  return rows.filter((x) => x !== undefined).length === 32 ? rows : null;
}

// ---------- 主流程 ----------
const t0 = Date.now();
const raw = readF32(P_DEPTH);
const W = parseInt(args.dw || '0', 10) || Math.round(Math.sqrt(raw.length * 3 / 4));
const H = Math.round(raw.length / W);
console.log(`深度场 ${W}x${H}（${raw.length} px）  贴图(网格口径) ${P_RGBA}  贴图(画布口径) ${P_FRAME}`);
const { p1, p99 } = percentilesCpp(raw);
const span = p99 - p1;
console.log(`稳健归一化 p1=${p1.toFixed(3)} p99=${p99.toFixed(3)} span=${span.toFixed(3)}`);
if (!(span > 1e-6)) { console.error('深度无效（span 过小）'); process.exit(2); }
const s = new Float32Array(W * H);
for (let i = 0; i < raw.length; i++) s[i] = Math.min(1, Math.max(0, (raw[i] - p1) / span));

const mesh = buildMesh(s, W, H);
mesh.w = W; mesh.h = H;
console.log(`网格 front=${mesh.front.length / 20}（coarse=${mesh.coarse} refined=${mesh.refined} maxZRatio=${mesh.maxRatio.toFixed(3)}）bg=${mesh.bg.length / 20} near=${mesh.nearCnt}/${W * H}`);

const texMeshRgb = rgbaToRgb(readU8(P_RGBA), W, H);
const texFrameRgb = rgbaToRgb(readU8(P_FRAME), OUT_W, OUT_H);
const bgFillRgb = new Uint8Array(W * H * 3);
{
  const mask = new Uint8Array(W * H);
  for (let i = 0; i < W * H; i++) mask[i] = s[i] > NEAR_SPLIT_S ? 1 : 0;
  const rgbF = new Float32Array(W * H * 3);
  for (let i = 0; i < W * H * 3; i++) rgbF[i] = texMeshRgb[i];
  pushPullFill(rgbF, 3, mask, W, H);
  for (let i = 0; i < W * H * 3; i++) bgFillRgb[i] = Math.max(0, Math.min(255, Math.round(rgbF[i])));
}
const bgFrameRgb = upsampleRgb(bgFillRgb, W, H, OUT_W, OUT_H);
const blurMesh = makeBlur(texMeshRgb, W, H, BLUR_F);
const blurFrame = makeBlur(texFrameRgb, OUT_W, OUT_H, BLUR_F);

function mkTex(cppMode) {
  return cppMode
    ? { front: { img: texMeshRgb, w: W, h: H }, bg: { img: bgFillRgb, w: W, h: H },
        blur: blurMesh, blurDiv: 1 }                        // 现状：UV 未除 blur 因子
    : { front: { img: texFrameRgb, w: OUT_W, h: OUT_H }, bg: { img: bgFrameRgb, w: OUT_W, h: OUT_H },
        blur: blurFrame, blurDiv: BLUR_F };                 // 修复：与离线判据同款（UV / blur.f）
}

const iMid = Math.floor(N_FRAMES / 2);
const R = {};
for (const mode of ['cpp', 'fix']) {
  const cppMode = mode === 'cpp';
  const f0 = renderFrame(0, N_FRAMES, THETA_DEG, mesh, mkTex(cppMode), cppMode);
  const fm = renderFrame(iMid, N_FRAMES, THETA_DEG, mesh, mkTex(cppMode), cppMode);
  R[mode] = { f0: { ascii: asciiOf(f0.rgb, OUT_W, OUT_H, 3), theta: f0.theta },
              fm: { ascii: asciiOf(fm.rgb, OUT_W, OUT_H, 3), theta: fm.theta } };
  const fx = renderFrame(N_FRAMES - 1, N_FRAMES, THETA_DEG, mesh, mkTex(cppMode), cppMode);
  R[mode].fEnd = { ascii: asciiOf(fx.rgb, OUT_W, OUT_H, 3), theta: fx.theta };
}

console.log(`\n===== [A] 现状语义（cpp）f000（θ=${(R.cpp.f0.theta * 180 / Math.PI).toFixed(3)}°）=====`);
printAscii('cpp f000', R.cpp.f0.ascii);
console.log(`\n===== [A] 现状语义（cpp）fmid（θ=${(R.cpp.fm.theta * 180 / Math.PI).toFixed(3)}°）=====`);
printAscii('cpp fmid', R.cpp.fm.ascii);
console.log(`\n  cpp f000 vs cpp fmid 差异：` + JSON.stringify(diffAscii(R.cpp.f0.ascii, R.cpp.fm.ascii)));
console.log(`  cpp fmid vs cpp fEnd 差异：` + JSON.stringify(diffAscii(R.cpp.fm.ascii, R.cpp.fEnd.ascii)));

console.log(`\n===== [B] 修复语义（fix）fmid（θ=${(R.fix.fm.theta * 180 / Math.PI).toFixed(3)}°）=====`);
printAscii('fix fmid', R.fix.fm.ascii);
const srcAscii = asciiOf(texFrameRgb, OUT_W, OUT_H, 3);
console.log(`\n  fix fmid vs 源照片（画布口径）差异：` + JSON.stringify(diffAscii(R.fix.fm.ascii, srcAscii)));
console.log(`  fix f000 vs fix fmid 差异（应≈0=无视差；>0 有视差）：` + JSON.stringify(diffAscii(R.fix.f0.ascii, R.fix.fm.ascii)));
console.log(`  fix fmid vs fix fEnd 差异（应>0=有视差）：` + JSON.stringify(diffAscii(R.fix.fm.ascii, R.fix.fEnd.ascii)));

if (P_LOG) {
  for (const tag of ['f000', 'fmid']) {
    const dev = logAscii(P_LOG, tag);
    if (!dev) { console.log(`\n[日志] 未取到 ${tag}`); continue; }
    const cmpCpp = diffAscii(dev, R.cpp[tag === 'f000' ? 'f0' : 'fm'].ascii);
    const cmpFix = diffAscii(dev, R.fix[tag === 'f000' ? 'f0' : 'fm'].ascii);
    console.log(`\n===== [C] 设备日志 ${tag} 比对 =====`);
    console.log(`  vs cpp 现状：same=${cmpCpp.same}/1536  Δ1=${cmpCpp.d1}  Δ≥2=${cmpCpp.d2}`);
    console.log(`  vs fix 修复：same=${cmpFix.same}/1536  Δ1=${cmpFix.d1}  Δ≥2=${cmpFix.d2}`);
    if (cmpCpp.d2 > 0) {
      console.log('  设备 vs cpp 差异图（.=同 1=Δ1 X=Δ≥2）：');
      cmpCpp.map.forEach((l, r) => console.log(`    r${String(r).padStart(2, '0')} ${l}`));
    }
  }
  // [D] 设备内部双检（不依赖离线模型）：§9 验收两条硬判据的直接读法
  //   ① 参考位姿恒等：`fmid`（θ≈0）≈ `src`（源照片，按画布口径逐格取点）
  //   ② 视差存在：`f000`（θ=−Θ）≠ `fmid`
  const devSrc = logAscii(P_LOG, 'src');
  const devF0 = logAscii(P_LOG, 'f000');
  const devFm = logAscii(P_LOG, 'fmid');
  if (devSrc && devF0 && devFm) {
    const ident = diffAscii(devFm, devSrc);
    const para = diffAscii(devF0, devFm);
    console.log('\n===== [D] 设备内部双检（无需离线模型） =====');
    console.log(`  ① 参考位姿恒等  fmid vs src ：` +
                `same=${ident.same}/1536  Δ1=${ident.d1}  Δ≥2=${ident.d2}` +
                `  → Δ≤1 占比 ${(((ident.same + ident.d1) / 1536) * 100).toFixed(1)}%（恒等恢复应 ≥ 90%，理想 ≥ 95%）`);
    console.log(`  ② 视差存在      f000 vs fmid：same=${para.same}/1536  Δ1=${para.d1}  Δ≥2=${para.d2}` +
                `  → 不同格数 ${((para.d1 + para.d2) / 1536 * 100).toFixed(1)}%（>10% = 视差正常；≈0% = 零视差，scene=5 必失败）`);
    if (ident.d2 > 0) {
      console.log('  恒等差异图（.=同 1=Δ1 X=Δ≥2）：');
      ident.map.forEach((l, r) => console.log(`    r${String(r).padStart(2, '0')} ${l}`));
    }
  } else {
    console.log('\n[日志] 设备内部双检跳过（缺 src / f000 / fmid 之一）');
  }
}

// ---------- 位移守卫（与判据 offsetOf/stepScan 同口径） ----------
function unproject(x, y, z) { return [(x - CX) / FX * z, -(y - CY) / FY * z, -z]; }
function offsetOf(x0, y0, z0, thA, thB) {
  const P = unproject(x0, y0, z0);
  const a = project(P, poseOf(thA)), b = project(P, poseOf(thB));
  return { du: b.u - a.u, dv: b.v - a.v };
}
for (const z of [Z_NEAR, ZF]) {
  const d = offsetOf(CX, CY, z, 0, 2 * THETA_DEG / 180 * Math.PI / 4);   // 相邻关键帧步长（KF=5）
  console.log(`\n[守卫] z=${z} 相邻关键帧(Δθ=${(THETA_DEG / 2).toFixed(2)}°)位移 = ${Math.hypot(d.du, d.dv).toFixed(1)}px`);
}
console.log(`\n完成（${((Date.now() - t0) / 1000).toFixed(1)}s）`);
