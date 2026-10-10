/**
 * 3D 壁纸 · 深度场质量算法判据（质量优化第二档：D1 边缘保持上采样 / D4 保边预平滑 /
 *   D3 自适应纵深 k / D2 主体蒙版对齐）
 *
 * 定位：免构建、零依赖（node 内置 fs/zlib），在写任何 C++ 之前把算法证清楚（先证后写，
 *   坑 120/124/129 的共同教训）；C++ 落地后本脚本兼任**源码级守卫**（防实装走样）与
 *   **真实数据分析台**（--input 模式）。
 *
 * 算法口径（与 C++ 实装严格同源，改一处必须同步）：
 *   D4 保边预平滑：s 场（546×728，p1/p99 归一化后）单遍 bilateral（半径 r=2，σs=r/1.5，
 *     σr=0.06，s∈[0,1] 值域）——消细碎台阶（refinedQuads 代理计数下降），台阶边缘不跨糊。
 *   D1 RGB 引导上采样（JBU，joint bilateral upsample）：s 低分辨场 → 画布分辨率，引导图 =
 *     画布分辨率源图；窗口半径 r=2（低分辨率采样点），σs=1.6（低分辨率像素），
 *     σr=0.10（RMS 通道差 0~1）；引导图在 tap 处取低分辨率网格预采样 G[q]，
 *     值域权重 = exp(−|I_p − G[q]|²/(2σr²))；Σw < 1e-6 时回退双线性。
 *   D3 自适应 k（d0 恒 2 公式：zNear=4/(1+k)、zFar=4k/(1+k)，相机数学/位移守卫零改动）：
 *     s 场 64 桶直方图 Otsu 两分 → 分离度 sep = |μ1−μ2| · 2·n1·n0/n²（类平衡加权）；
 *     k = 2.2 + 1.6 · clamp((sep − 0.22)/(0.50 − 0.22), 0, 1) ∈ [2.2, 3.8]。
 *   D2 主体对齐：蒙版（重采样到深度场分辨率，≥128 为主体）内做 2 遍蒙版门控 bilateral
 *     （半径 2，σr=0.08），蒙版外逐字节不变；主体内部噪声下降、轮廓（蒙版边界）不糊。
 *
 * 用法：
 *   node tools/depth_mesh/depth_edge_check.js                      # 合成判据 + 源码守卫
 *   node tools/depth_mesh/depth_edge_check.js --input=tools/depth_mesh/out/dev5f
 *                                                                  # 真实数据分析（金标=画布分辨率
 *                                                                  # 参考深度；端侧场=金标降采样
 *                                                                  # 的理想化模拟）+ PNG 对比图
 * 产物（--input 模式）：tools/ui_shots/sr/depth_truth|bilinear|jbu[_sm].png
 */
'use strict';
const fs = require('fs');
const path = require('path');

const args = {};
process.argv.slice(2).forEach((a) => {
  const m = a.match(/^--([^=]+)=?(.*)$/);
  if (m) args[m[1]] = m[2];
});
const INPUT = args.input || '';
const OUT_DIR = args.out || 'tools/ui_shots/sr';

let pass = 0;
let fail = 0;
function check(name, ok, detail) {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  ok ? pass++ : fail++;
}

// ---------- PNG 输出（与 depth_mesh_check.js 同款编码器） ----------
const CRC_T = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
    t[n] = c >>> 0;
  }
  return t;
})();
function crc32(buf) {
  let c = 0xFFFFFFFF;
  for (let i = 0; i < buf.length; i++) c = CRC_T[(c ^ buf[i]) & 0xFF] ^ (c >>> 8);
  return (c ^ 0xFFFFFFFF) >>> 0;
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
  ihdr[8] = 8; ihdr[9] = 2;
  fs.writeFileSync(file, Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]),
    chunk('IHDR', ihdr), chunk('IDAT', require('zlib').deflateSync(raw)), chunk('IEND', Buffer.alloc(0)),
  ]));
}

// ---------- 常量（与 C++ 同源；改这里必须同步 spatial_recon_depth_mesh.cpp / bridge） ----------
const OPTS = {
  preSmoothR: 2,        // D4 半径（深度场像素）
  preSmoothSigmaR: 0.10,// D4 值域 σ（s∈[0,1]；σr=0.10：消 ±0.1 级噪声，Δ≥0.3 的台阶跨边权重 ~e^-11 仍清晰分离）
  jbuRadius: 2,         // D1 窗口半径（低分辨率采样点）
  jbuSigmaS: 1.6,       // D1 空间域 σ（低分辨率像素）
  jbuSigmaR: 0.10,      // D1 值域 σ（RMS 通道差 0~1）
  subjectSigmaR: 0.08,  // D2 值域 σ
  kMin: 2.2, kMax: 3.8, // D3 k 值域（d0 恒 2 公式）
  sepLo: 0.10, sepHi: 0.30, // D3 分离度→k 映射端点（平衡加权 sep 上限 ≈0.3：人像/分层强 ≈0.25~0.30、缓变 ≈0.1、平坦 <0.08）
};

// ---------- 基础采样/工具（与 C++ SampleS 同语义：连续像素坐标、越界钳制、-0.5 像素中心） ----------
function clamp(v, lo, hi) { return v < lo ? lo : (v > hi ? hi : v); }
function sampleBilinear(f, w, h, x, y) {
  const fx = clamp(x - 0.5, 0, w - 1.001), fy = clamp(y - 0.5, 0, h - 1.001);
  const x0 = fx | 0, y0 = fy | 0, tx = fx - x0, ty = fy - y0;
  const x1 = Math.min(w - 1, x0 + 1), y1 = Math.min(h - 1, y0 + 1);
  return (f[y0 * w + x0] * (1 - tx) + f[y0 * w + x1] * tx) * (1 - ty) +
         (f[y1 * w + x0] * (1 - tx) + f[y1 * w + x1] * tx) * ty;
}
function sampleRgbBilinear(rgb, w, h, x, y, out3) {
  const fx = clamp(x - 0.5, 0, w - 1.001), fy = clamp(y - 0.5, 0, h - 1.001);
  const x0 = fx | 0, y0 = fy | 0, tx = fx - x0, ty = fy - y0;
  const x1 = Math.min(w - 1, x0 + 1), y1 = Math.min(h - 1, y0 + 1);
  for (let c = 0; c < 3; c++) {
    out3[c] = (rgb[(y0 * w + x0) * 3 + c] * (1 - tx) + rgb[(y0 * w + x1) * 3 + c] * tx) * (1 - ty) +
              (rgb[(y1 * w + x0) * 3 + c] * (1 - tx) + rgb[(y1 * w + x1) * 3 + c] * tx) * ty;
  }
}
/** 直方图分位（与 C++ Percentiles 同款：4096 桶） */
function percentiles(a, plo, phi) {
  let mn = a[0], mx = a[0];
  for (let i = 1; i < a.length; i++) { if (a[i] < mn) mn = a[i]; if (a[i] > mx) mx = a[i]; }
  const B = 4096, hist = new Uint32Array(B);
  const span = (mx - mn) > 1e-9 ? (mx - mn) : 1;
  for (let i = 0; i < a.length; i++) {
    const b = clamp(Math.floor((a[i] - mn) / span * (B - 1)), 0, B - 1);
    hist[b]++;
  }
  const lo = Math.floor(plo * a.length), hi = Math.floor(phi * a.length);
  let acc = 0, p1 = mn, p99 = mx;
  for (let b = 0; b < B; b++) {
    acc += hist[b];
    const val = mn + span * (b + 0.5) / B;
    if (acc >= lo && p1 === mn) p1 = val;
    if (acc >= hi) { p99 = val; break; }
  }
  return [p1, p99];
}

// ---------- D4：保边预平滑（单遍 bilateral） ----------
function bilateralSmooth(s, w, h, r, sigmaR) {
  const out = new Float32Array(s.length);
  const sigmaS = r / 1.5;
  const spatial = [];
  for (let dy = -r; dy <= r; dy++) for (let dx = -r; dx <= r; dx++) {
    spatial.push([dx, dy, Math.exp(-(dx * dx + dy * dy) / (2 * sigmaS * sigmaS))]);
  }
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      let acc = 0, wsum = 0;
      const v0 = s[y * w + x];
      for (const [dx, dy, ws] of spatial) {
        const xx = x + dx, yy = y + dy;
        if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
        const v = s[yy * w + xx];
        const wr = Math.exp(-(v - v0) * (v - v0) / (2 * sigmaR * sigmaR));
        acc += v * ws * wr; wsum += ws * wr;
      }
      out[y * w + x] = wsum > 1e-9 ? acc / wsum : v0;
    }
  }
  return out;
}

// ---------- D2：蒙版门控保边平滑（2 遍；蒙版外逐字节不变） ----------
function subjectAlignSmooth(s, w, h, mask, sigmaR) {
  let cur = s;
  for (let pass = 0; pass < 2; pass++) {
    const out = Float32Array.from(cur);
    const r = 2, sigmaS = r / 1.5;
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const k = y * w + x;
        if (mask[k] < 128) continue;   // 蒙版外不动
        let acc = 0, wsum = 0;
        const v0 = cur[k];
        for (let dy = -r; dy <= r; dy++) {
          for (let dx = -r; dx <= r; dx++) {
            const xx = x + dx, yy = y + dy;
            if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
            const kk = yy * w + xx;
            if (mask[kk] < 128) continue;   // 只聚合主体内邻居（蒙版门控）
            const v = cur[kk];
            const ws = Math.exp(-(dx * dx + dy * dy) / (2 * sigmaS * sigmaS));
            const wr = Math.exp(-(v - v0) * (v - v0) / (2 * sigmaR * sigmaR));
            acc += v * ws * wr; wsum += ws * wr;
          }
        }
        if (wsum > 1e-9) out[k] = acc / wsum;
      }
    }
    cur = out;
  }
  return cur;
}

// ---------- D1：JBU（RGB 引导上采样；guide = 画布分辨率 RGB） ----------
function jbuUpsample(s, dw, dh, guideRgb, W, H, o) {
  const r = o.jbuRadius, sigmaS = o.jbuSigmaS, sigmaR = o.jbuSigmaR;
  // 引导图在低分辨率 tap 处的预采样 G[q]（dw×dh×3）
  const G = new Float32Array(dw * dh * 3);
  const tmp = [0, 0, 0];
  for (let q = 0; q < dw * dh; q++) {
    const qx = q % dw, qy = (q / dw) | 0;
    sampleRgbBilinear(guideRgb, W, H, (qx + 0.5) * W / dw, (qy + 0.5) * H / dh, tmp);
    G[q * 3] = tmp[0]; G[q * 3 + 1] = tmp[1]; G[q * 3 + 2] = tmp[2];
  }
  const inv2sr2 = 1 / (2 * sigmaR * sigmaR);
  const inv2ss2 = 1 / (2 * sigmaS * sigmaS);
  const out = new Float32Array(W * H);
  for (let y = 0; y < H; y++) {
    const lpy = (y + 0.5) * dh / H;
    for (let x = 0; x < W; x++) {
      const lpx = (x + 0.5) * dw / W;
      const p = y * W + x;
      const qx0 = Math.max(0, Math.min(dw - 1, Math.round(lpx)));
      const qy0 = Math.max(0, Math.min(dh - 1, Math.round(lpy)));
      const pr = guideRgb[p * 3] / 255, pg = guideRgb[p * 3 + 1] / 255, pb = guideRgb[p * 3 + 2] / 255;
      let acc = 0, wsum = 0;
      for (let dy = -r; dy <= r; dy++) {
        const qy = qy0 + dy;
        if (qy < 0 || qy >= dh) continue;
        for (let dx = -r; dx <= r; dx++) {
          const qx = qx0 + dx;
          if (qx < 0 || qx >= dw) continue;
          const q = qy * dw + qx;
          const ex = lpx - qx, ey = lpy - qy;
          const dr = pr - G[q * 3] / 255, dg = pg - G[q * 3 + 1] / 255, db = pb - G[q * 3 + 2] / 255;
          const d2 = (dr * dr + dg * dg + db * db) / 3;   // RMS 通道差平方
          const wgt = Math.exp(-(ex * ex + ey * ey) * inv2ss2 - d2 * inv2sr2);
          acc += s[q] * wgt; wsum += wgt;
        }
      }
      out[p] = wsum > 1e-6 ? acc / wsum : sampleBilinear(s, dw, dh, lpx + 0.5, lpy + 0.5);
    }
  }
  return out;
}

// ---------- D3：自适应 k（Otsu 分离度 → k；d0 恒 2 公式） ----------
function adaptiveK(s, o) {
  const B = 64;
  const hist = new Float64Array(B);
  for (let i = 0; i < s.length; i++) hist[clamp(Math.floor(s[i] * B), 0, B - 1)]++;
  const n = s.length;
  let best = -1, bestT = 0;
  let sumAll = 0;
  for (let b = 0; b < B; b++) sumAll += hist[b] * (b + 0.5) / B;
  let w0 = 0, sum0 = 0;
  for (let b = 0; b < B - 1; b++) {
    w0 += hist[b]; sum0 += hist[b] * (b + 0.5) / B;
    const w1 = n - w0;
    if (w0 < 0.05 * n || w1 < 0.05 * n) continue;   // 两类都须 ≥5% 面积
    const m0 = sum0 / w0, m1 = (sumAll - sum0) / w1;
    const v = w0 * w1 * (m0 - m1) * (m0 - m1);
    if (v > best) { best = v; bestT = (b + 1) / B; }
  }
  if (best < 0) return { k: (o.kMin + o.kMax) / 2, sep: 0, t: 0.5, otsuOk: false };
  // 类均值（按最优阈值重算精确值）
  let a0 = 0, c0 = 0, a1 = 0, c1 = 0;
  for (let b = 0; b < B; b++) {
    const v = (b + 0.5) / B;
    if (v < bestT) { a0 += hist[b] * v; c0 += hist[b]; } else { a1 += hist[b] * v; c1 += hist[b]; }
  }
  const sep = Math.abs(a1 / Math.max(1, c1) - a0 / Math.max(1, c0)) * (2 * c0 * c1) / (n * n);
  const k = o.kMin + (o.kMax - o.kMin) * clamp((sep - o.sepLo) / (o.sepHi - o.sepLo), 0, 1);
  return { k, sep, t: bestT, otsuOk: true };
}

// ---------- 指标：颜色强边缘 ↔ 深度边缘对齐 ----------
function grayOf(rgb, i) { return 0.299 * rgb[i * 3] + 0.587 * rgb[i * 3 + 1] + 0.114 * rgb[i * 3 + 2]; }
function strongColorEdges(rgb, W, H, q = 0.99) {
  const mag = new Float32Array(W * H);
  for (let y = 1; y < H - 1; y++) {
    for (let x = 1; x < W - 1; x++) {
      const p = y * W + x;
      const gx = -grayOf(rgb, p - W - 1) - 2 * grayOf(rgb, p - 1) - grayOf(rgb, p + W - 1) +
                  grayOf(rgb, p - W + 1) + 2 * grayOf(rgb, p + 1) + grayOf(rgb, p + W + 1);
      const gy = -grayOf(rgb, p - W - 1) - 2 * grayOf(rgb, p - W) - grayOf(rgb, p - W + 1) +
                  grayOf(rgb, p + W - 1) + 2 * grayOf(rgb, p + W) + grayOf(rgb, p + W + 1);
      mag[p] = Math.hypot(gx, gy);
    }
  }
  const sorted = Float32Array.from(mag).sort();
  const thr = sorted[Math.floor(q * (W * H - 1))];
  const edge = new Uint8Array(W * H);
  for (let y = 1; y < H - 1; y++) {
    for (let x = 1; x < W - 1; x++) {
      const p = y * W + x;
      if (mag[p] < thr) continue;
      if (mag[p] >= mag[p - 1] && mag[p] >= mag[p + 1] && mag[p] >= mag[p - W] && mag[p] >= mag[p + W]) edge[p] = 1;
    }
  }
  return edge;
}
function depthEdgeMask(sUp, W, H, q = 0.985) {
  const g = new Float32Array(W * H);
  for (let y = 1; y < H - 1; y++) {
    for (let x = 1; x < W - 1; x++) {
      const p = y * W + x;
      g[p] = Math.max(Math.abs(sUp[p + 1] - sUp[p - 1]), Math.abs(sUp[p + W] - sUp[p - W]));
    }
  }
  const sorted = Float32Array.from(g).sort();
  const thr = sorted[Math.floor(q * (W * H - 1))];
  const edge = new Uint8Array(W * H);
  for (let y = 1; y < H - 1; y++) {
    for (let x = 1; x < W - 1; x++) {
      const p = y * W + x;
      if (g[p] < thr) continue;
      if (g[p] >= g[p - 1] && g[p] >= g[p + 1] && g[p] >= g[p - W] && g[p] >= g[p + W]) edge[p] = 1;
    }
  }
  return edge;
}
/** 对齐分（颜色强边缘 1px 内有深度边缘的比例）与渗色带宽（颜色边缘→最近深度边缘距离，封顶10；按环精确取最近） */
function edgeAlignment(colorEdge, depthEdge, W, H) {
  let nC = 0, hit = 0, distSum = 0;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      if (!colorEdge[y * W + x]) continue;
      nC++;
      let best = 10;   // 封顶
      outer:
      for (let r = 0; r <= 10; r++) {
        for (let dy = -r; dy <= r; dy++) {
          for (let dx = -r; dx <= r; dx++) {
            if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;   // 只扫当前环
            const xx = x + dx, yy = y + dy;
            if (xx < 0 || yy < 0 || xx >= W || yy >= H) continue;
            if (depthEdge[yy * W + xx]) { best = r; break outer; }
          }
        }
      }
      if (best <= 1) hit++;
      distSum += best;
    }
  }
  return { align: nC > 0 ? hit / nC : 0, bleed: nC > 0 ? distSum / nC : 10 };
}

// ---------- 台阶细分代理（4px 格四角 z 比 > 1.15 计数；zOfS k=3 与 C++ 同式） ----------
function refinedProxy(s, w, h, stride = 4) {
  const zNear = 1, zFar = 3;
  const zOf = (sv) => 1 / (sv / zNear + (1 - sv) / zFar);
  const sample = (x, y) => sampleBilinear(s, w, h, x, y);
  let cnt = 0, cells = 0;
  for (let j = 0; j + stride <= h; j += stride) {
    for (let i = 0; i + stride <= w; i += stride) {
      cells++;
      const zs = [sample(i, j), sample(i + stride, j), sample(i + stride, j + stride), sample(i, j + stride)].map(zOf);
      const ratio = Math.max(...zs) / Math.max(1e-9, Math.min(...zs));
      if (ratio > 1.15) cnt++;
    }
  }
  return { cnt, cells, pct: cells > 0 ? cnt / cells : 0 };
}

// ---------- 归一化 ----------
function normalizeField(raw) {
  const [p1, p99] = percentiles(raw, 0.01, 0.99);
  const span = p99 - p1;
  const s = new Float32Array(raw.length);
  for (let i = 0; i < raw.length; i++) s[i] = clamp((raw[i] - p1) / span, 0, 1);
  return { s, p1, p99 };
}

// ================================================================
// 模式一：--input 真实数据分析（金标 = 画布分辨率 PC 参考深度）
// ================================================================
if (INPUT) {
  const json = JSON.parse(fs.readFileSync(`${INPUT}.json`, 'utf8'));
  const W = json.w, H = json.h;
  const truthRaw = new Float32Array(fs.readFileSync(`${INPUT}.depth.f32.bin`).buffer.slice(0, W * H * 4));
  const rgbaRaw = new Uint8Array(fs.readFileSync(`${INPUT}.rgba.bin`).buffer.slice(0, W * H * 4));
  if (truthRaw.length !== W * H || rgbaRaw.length !== W * H * 4) {
    console.error(`尺寸不符：depth ${truthRaw.length} / rgba ${rgbaRaw.length} vs ${W}x${H}`); process.exit(1);
  }
  // RGBA(4B/px) → 紧凑 RGB(3B/px)：本脚本的灰度/引导图/JBU 全按 3bpp 寻址
  const rgba = new Uint8Array(W * H * 3);
  for (let i = 0; i < W * H; i++) {
    rgba[i * 3] = rgbaRaw[i * 4]; rgba[i * 3 + 1] = rgbaRaw[i * 4 + 1]; rgba[i * 3 + 2] = rgbaRaw[i * 4 + 2];
  }
  const dw = 546, dh = 728;   // 端侧模型输出分辨率（与 runner 同）
  // 端侧场 = 金标降采样的理想化模拟（真机行为以 scene5_render_check 口径另行复核）
  const devRaw = new Float32Array(dw * dh);
  for (let y = 0; y < dh; y++) {
    for (let x = 0; x < dw; x++) {
      devRaw[y * dw + x] = sampleBilinear(truthRaw, W, H, (x + 0.5) * W / dw, (y + 0.5) * H / dh);
    }
  }
  const { s } = normalizeField(devRaw);
  const truthS = normalizeField(truthRaw);
  const sSm = bilateralSmooth(s, dw, dh, OPTS.preSmoothR, OPTS.preSmoothSigmaR);
  const colorEdge = strongColorEdges(rgba, W, H);

  const variants = [];
  const bilinearUp = (field) => {
    const up = new Float32Array(W * H);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      up[y * W + x] = sampleBilinear(field, dw, dh, (x + 0.5) * dw / W + 0.5, (y + 0.5) * dh / H + 0.5);
    }
    return up;
  };
  const t0 = Date.now();
  variants.push(['bilinear(旧行为)', bilinearUp(s)]);
  variants.push(['JBU(D1)', jbuUpsample(s, dw, dh, rgba, W, H, OPTS)]);
  variants.push(['bilinear+D4', bilinearUp(sSm)]);
  variants.push(['JBU+D4(完整档)', jbuUpsample(sSm, dw, dh, rgba, W, H, OPTS)]);
  const ms = Date.now() - t0;

  // 金标对齐（truthS 用 devRaw 的 p1/p99 同口径归一，保证可比）
  const truthNorm = new Float32Array(W * H);
  const { p1, p99 } = normalizeField(devRaw);
  for (let i = 0; i < W * H; i++) truthNorm[i] = clamp((truthRaw[i] - p1) / (p99 - p1), 0, 1);

  console.log(`== 真实数据 ${INPUT}（canvas ${W}x${H}，端侧场 ${dw}x${dh}，JBU×2 共 ${ms}ms） ==`);
  console.log('变体                对齐分↑   渗色带宽↓   MAE(vs金标)↓');
  for (const [name, up] of variants) {
    const dEdge = depthEdgeMask(up, W, H);
    const { align, bleed } = edgeAlignment(colorEdge, dEdge, W, H);
    let mae = 0;
    for (let i = 0; i < W * H; i++) mae += Math.abs(up[i] - truthNorm[i]);
    console.log(`${name.padEnd(18)} ${align.toFixed(3)}     ${bleed.toFixed(2)}px      ${(mae / W / H).toFixed(4)}`);
  }
  const rp0 = refinedProxy(s, dw, dh);
  const rp1 = refinedProxy(sSm, dw, dh);
  console.log(`D4 细碎台阶代理：raw ${(rp0.pct * 100).toFixed(2)}% (${rp0.cnt}/${rp0.cells}) -> 平滑后 ${(rp1.pct * 100).toFixed(2)}% (${rp1.cnt}/${rp1.cells})`);
  const ak = adaptiveK(s, OPTS);
  console.log(`D3 自适应 k：sep=${ak.sep.toFixed(3)} t=${ak.t.toFixed(2)} -> k=${ak.k.toFixed(2)}（d0=2 -> zNear=${(4 / (1 + ak.k)).toFixed(3)} zFar=${(4 * ak.k / (1 + ak.k)).toFixed(3)}）`);

  fs.mkdirSync(OUT_DIR, { recursive: true });
  const toGray = (f) => { const g = new Uint8Array(W * H * 3); for (let i = 0; i < W * H; i++) { const v = Math.round(clamp(f[i], 0, 1) * 255); g[i * 3] = g[i * 3 + 1] = g[i * 3 + 2] = v; } return g; };
  writePng(path.join(OUT_DIR, 'depth_truth.png'), W, H, toGray(truthNorm));
  writePng(path.join(OUT_DIR, 'depth_bilinear.png'), W, H, toGray(variants[0][1]));
  writePng(path.join(OUT_DIR, 'depth_jbu_sm.png'), W, H, toGray(variants[3][1]));
  console.log(`PNG：${OUT_DIR}/depth_truth|bilinear|jbu_sm.png（肉眼对比边缘贴合）`);
  process.exit(0);
}

// ================================================================
// 模式二：合成判据 + 源码守卫（默认）
// ================================================================
console.log('=== [1] D1 · JBU 边缘锐化（合成：深度台阶糊在颜色边缘处，JBU 收紧过渡带 + 小偏移回拉） ===');
{
  const W = 135, H = 180, dw = 68, dh = 90;   // 1/8 画布的等比缩
  const rgba = new Uint8Array(W * H * 3);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const v = x >= 90 ? 200 : 80;   // 颜色边界 x=90（低分辨率 45.3）
    const p = (y * W + x) * 3; rgba[p] = v; rgba[p + 1] = v; rgba[p + 2] = v;
  }
  // 用例 A（真实病灶模型）：低分辨率深度台阶中心在颜色边缘处（低分辨 45），但被模型涂糊成 3~4 格宽
  const rampCenter = 45, rampHalf = 2;
  const sA = new Float32Array(dw * dh);
  for (let y = 0; y < dh; y++) for (let x = 0; x < dw; x++) {
    const t = clamp((x - (rampCenter - rampHalf)) / (2 * rampHalf), 0, 1);
    sA[y * dw + x] = 0.35 + 0.5 * t;
  }
  const upAB = new Float32Array(W * H);
  const upAJ = jbuUpsample(sA, dw, dh, rgba, W, H, OPTS);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    upAB[y * W + x] = sampleBilinear(sA, dw, dh, (x + 0.5) * dw / W + 0.5, (y + 0.5) * dh / H + 0.5);
  }
  const widthAt = (up, y) => {   // 该行过渡带宽度（0.45 < s < 0.75 的连续跨度）
    let w = 0;
    for (let x = 0; x < W; x++) if (up[y * W + x] > 0.45 && up[y * W + x] < 0.75) w++;
    return w;
  };
  let wB = 0, wJ = 0, rows = 0;
  for (let y = 10; y < H - 10; y += 7) { wB += widthAt(upAB, y); wJ += widthAt(upAJ, y); rows++; }
  wB /= rows; wJ /= rows;
  check('JBU 把糊台阶收紧到 ≤ 双线性的一半', wJ < wB * 0.5, `JBU ${wJ.toFixed(1)}px vs bilinear ${wB.toFixed(1)}px`);
  // 用例 B（小偏移回拉）：低分辨硬台阶中心在 43.5（画布 ≈87.7），颜色边缘 90 —— JBU 应把
  // 过渡带中心拉向 90（两侧都有低分辨证据时 JBU 才可能移动边缘；此处 43.5~45.3 有样本可用）
  const sB = new Float32Array(dw * dh);
  for (let y = 0; y < dh; y++) for (let x = 0; x < dw; x++) sB[y * dw + x] = x >= 43.5 ? 0.85 : 0.35;
  const upBB = new Float32Array(W * H);
  const upBJ = jbuUpsample(sB, dw, dh, rgba, W, H, OPTS);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    upBB[y * W + x] = sampleBilinear(sB, dw, dh, (x + 0.5) * dw / W + 0.5, (y + 0.5) * dh / H + 0.5);
  }
  const centerAt = (up, y) => {
    let lastLow = -1, firstHigh = -1;
    for (let x = 0; x < W; x++) {
      if (up[y * W + x] < 0.6) lastLow = x;
      if (up[y * W + x] > 0.6 && firstHigh < 0) firstHigh = x;
    }
    return (lastLow + firstHigh) / 2;
  };
  let cB = 0, cJ = 0;
  rows = 0;
  for (let y = 10; y < H - 10; y += 7) { cB += Math.abs(centerAt(upBB, y) - 90); cJ += Math.abs(centerAt(upBJ, y) - 90); rows++; }
  cB /= rows; cJ /= rows;
  check('小偏移台阶被 JBU 显著拉向颜色边缘（误差 ≤ 双线性的 65%；JBU 不创造低分辨场不存在的边缘，只能用已有证据部分回拉）', cJ < cB * 0.65, `JBU ${cJ.toFixed(2)}px vs bilinear ${cB.toFixed(2)}px`);
  // 平坦区无畸变
  let flatErr = 0;
  for (let y = 20; y < 40; y++) for (let x = 100; x < 120; x++) flatErr = Math.max(flatErr, Math.abs(upAJ[y * W + x] - 0.85));
  check('JBU 同色平坦区不引入起伏（|Δs| < 0.02）', flatErr < 0.02, `max|Δs|=${flatErr.toFixed(4)}`);
}
console.log('=== [2] D4 · 保边预平滑（消细碎台阶，不跨糊真台阶） ===');
{
  const dw = 68, dh = 90;
  const clean = new Float32Array(dw * dh);
  for (let y = 0; y < dh; y++) for (let x = 0; x < dw; x++) clean[y * dw + x] = 0.4 + 0.1 * (y / dh);
  const noisy = Float32Array.from(clean);
  let seed = 42;
  const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF; return seed / 0x7FFFFFFF - 0.5; };
  for (let i = 0; i < noisy.length; i++) noisy[i] += rnd() * 0.36;   // 模拟端侧噪声：足以触发台阶细分
  const sm = bilateralSmooth(noisy, dw, dh, OPTS.preSmoothR, OPTS.preSmoothSigmaR);
  const rpN = refinedProxy(noisy, dw, dh), rpS = refinedProxy(sm, dw, dh);
  check('预平滑消细碎台阶（细分格占比显著下降）', rpS.cnt < rpN.cnt * 0.5, `${(rpN.pct * 100).toFixed(1)}% -> ${(rpS.pct * 100).toFixed(1)}%`);
  let maeN = 0, maeS = 0;
  for (let i = 0; i < clean.length; i++) { maeN += Math.abs(noisy[i] - clean[i]); maeS += Math.abs(sm[i] - clean[i]); }
  check('预平滑向干净场收敛（MAE 下降）', maeS < maeN, `${(maeN / clean.length).toFixed(4)} -> ${(maeS / clean.length).toFixed(4)}`);
  // 真台阶不被跨糊
  const step = new Float32Array(dw * dh);
  for (let y = 0; y < dh; y++) for (let x = 0; x < dw; x++) step[y * dw + x] = x >= 34 ? 0.8 : 0.3;
  const smStep = bilateralSmooth(step, dw, dh, OPTS.preSmoothR, OPTS.preSmoothSigmaR);
  let cross = 0;
  for (let y = 0; y < dh; y++) {
    const lo = smStep[y * dw + 30], hi = smStep[y * dw + 38];
    if (lo > 0.45 || hi < 0.65) cross++;
  }
  check('真台阶两侧保持分离（不跨糊）', cross === 0, `越界行 ${cross}`);
}
console.log('=== [3] D3 · 自适应 k（d0 恒 2 公式 + 分离度单调） ===');
{
  const mk = (fill) => { const f = new Float32Array(546 * 728); for (let i = 0; i < f.length; i++) f[i] = fill(i); return f; };
  const W = 546, H = 728;
  const flat = mk((i) => 0.5 + 0.02 * Math.sin((i % W) * 0.05));                       // 近乎无分离
  const ramp = mk((i) => (i / (W * H)));                                              // 单调渐变（无两层）
  const bimodalWeak = mk((i) => (i % W) / W < 0.5 ? 0.45 : 0.62);                     // 弱两层
  const bimodalStrong = mk((i) => ((i / W | 0) / H) < 0.4 ? 0.25 : 0.80);             // 强两层（主体 40% 面积）
  const a1 = adaptiveK(flat, OPTS), a2 = adaptiveK(ramp, OPTS), a3 = adaptiveK(bimodalWeak, OPTS), a4 = adaptiveK(bimodalStrong, OPTS);
  check('平坦场 → 低 k（≤ 2.45）', a1.k <= 2.45, `k=${a1.k.toFixed(2)} sep=${a1.sep.toFixed(3)}`);
  check('渐变场（大纵深墙面）→ 中高 k（2.9~3.8，"大纵深更立体"属预期）', a2.k >= 2.9 && a2.k <= 3.8, `k=${a2.k.toFixed(2)} sep=${a2.sep.toFixed(3)}`);
  check('强两层 → 高 k（≥ 3.2）', a4.k >= 3.2, `k=${a4.k.toFixed(2)} sep=${a4.sep.toFixed(3)}`);
  check('分离度单调（flat ≤ weak ≤ strong）', a1.sep <= a3.sep + 1e-9 && a3.sep <= a4.sep + 1e-9,
    `${a1.sep.toFixed(3)} ≤ ${a3.sep.toFixed(3)} ≤ ${a4.sep.toFixed(3)}`);
  check('k 全程落在 [2.2, 3.8]（远景板 7.4×9.6 覆盖域内）',
    [a1, a2, a3, a4].every((a) => a.k >= OPTS.kMin - 1e-9 && a.k <= OPTS.kMax + 1e-9));
  const zN = 4 / (1 + a4.k), zF = 4 * a4.k / (1 + a4.k);
  check('d0 恒 2（(zNear+zFar)/2 = 2，相机数学零改动）', Math.abs((zN + zF) / 2 - 2) < 1e-9, `zNear=${zN.toFixed(3)} zFar=${zF.toFixed(3)}`);
}
console.log('=== [4] D2 · 主体蒙版对齐（蒙版内抑噪、蒙版外逐字节不变） ===');
{
  const dw = 68, dh = 90;
  const mask = new Uint8Array(dw * dh);
  for (let y = 20; y < 70; y++) for (let x = 20; x < 50; x++) mask[y * dw + x] = 255;
  let seed = 7;
  const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF; return seed / 0x7FFFFFFF - 0.5; };
  const s = new Float32Array(dw * dh);
  for (let i = 0; i < s.length; i++) s[i] = 0.7 + rnd() * 0.10;
  const out = subjectAlignSmooth(s, dw, dh, mask, OPTS.subjectSigmaR);
  let inVar0 = 0, inVar1 = 0, outDiff = 0, n = 0;
  for (let y = 0; y < dh; y++) for (let x = 0; x < dw; x++) {
    const k = y * dw + x;
    if (mask[k] >= 128) { inVar0 += Math.abs(s[k] - 0.7); inVar1 += Math.abs(out[k] - 0.7); n++; }
    else if (s[k] !== out[k]) outDiff++;
  }
  check('主体内噪声下降（≥ 40%）', inVar1 < inVar0 * 0.6, `${(inVar0 / n).toFixed(4)} -> ${(inVar1 / n).toFixed(4)}`);
  check('蒙版外逐字节不变', outDiff === 0, `diff=${outDiff}`);
  // 蒙版边界：主体内边缘像素不被背景深度稀释（边界内 1px 处均值仍接近主体中心值）
  let bsum = 0, bn = 0;
  for (let y = 20; y < 70; y++) { const k = y * dw + 51 - 1; bsum += out[y * dw + 49]; bn++; }
  check('蒙版边界内侧不被背景稀释（均值 ≈ 主体值）', Math.abs(bsum / bn - 0.7) < 0.02, `${(bsum / bn).toFixed(3)}`);
}
console.log('=== [5] 关=旧行为（默认参数下字节级等价的结构守卫） ===');
{
  const ROOT = path.join(__dirname, '..', '..');
  const read = (p) => fs.readFileSync(path.join(ROOT, p), 'utf8');
  const hCpp = read('src/cpp/spatial_recon_depth_mesh.h');
  const cCpp = read('src/cpp/spatial_recon_depth_mesh.cpp');
  const bridge = read('src/cpp/spatial_recon_bridge.cpp');
  const bridgeH = read('src/cpp/spatial_recon_bridge.h');
  const dts = read('src/ets/libglassrender-3d.d.ts');
  const svc = read('src/ets/SpatialReconService.ets');

  check('mesh 单元声明 SrDepthMeshOpts（edgeMode 预平滑/JBU/蒙版参数）', /struct SrDepthMeshOpts/.test(hCpp) && /edgeMode/.test(hCpp));
  check('mesh 单元声明 SrAdaptiveDepthK（纯 2D，D3 分析）', /float SrAdaptiveDepthK\(/.test(hCpp));
  check('C++ 实装：D4 bilateral / D1 JBU / D2 蒙版门控 / Otsu 分离度四算法在位',
    /bilateral/i.test(cCpp) && /Jbu|JBU/.test(cCpp) && /Otsu/.test(cCpp) && cCpp.includes('subjectMask'));
  check('默认档：SrBuildDepthMesh 旧签名路径不变（mesh 单元自身缺省；产品档由调用方传 opts 决定）',
    /const SrDepthMeshOpts &opts = SrDepthMeshOpts\(\)/.test(hCpp));
  check('bridge：D1/D2 经 opts 传入，D3 在 zFar 计算前自适应（d0 恒 2 公式）',
    /SrAdaptiveDepthK/.test(bridge) && /4\.0f \/ \(1\.0f \+ k/.test(bridge));
  check('bridge：自适应 k 日志（SRCHK adaptive k=…）', /SRCHK adaptive k=/.test(bridge));
  check('mesh 单元：边缘模式日志（SRDM tag「edge mode=」真机对账）', /edge mode=%\{public\}d/.test(cCpp));
  check('params：depthEdge/adaptiveK/subjectAlign 三字段默认 1（质量增强档；旧"默认 0"作废）',
    /int32_t depthEdge = 1;/.test(bridgeH) && /int32_t adaptiveK = 1;/.test(bridgeH) && /int32_t subjectAlign = 1;/.test(bridgeH));
  check('d.ts：三可选字段声明', /depthEdge\?: number;/.test(dts) && /adaptiveK\?: number;/.test(dts) && /subjectAlign\?: number;/.test(dts));
  check('Service：AdaptOptions 三字段 + 透传（缺省 = 增强档 1；adaptiveK 走 effAdaptiveK）',
    /depthEdge\?: number;/.test(svc) && /adaptiveK\?: number;/.test(svc) && /subjectAlign\?: number;/.test(svc) &&
    /depthEdge: adapt\?\.depthEdge \?\? 1/.test(svc) &&
    /let effAdaptiveK: number = adapt\?\.adaptiveK \?\? 1;/.test(svc) &&
    /adaptiveK: effAdaptiveK,/.test(svc) &&
    /subjectAlign: adapt\?\.subjectAlign \?\? 1/.test(svc));
  check('Service：scene=5 + subjectAlign 时构建主体蒙版（复用 buildSubjectMask）',
    /wantedScene === 5 && adapt\?\.subjectAlign === 1/.test(svc));
}

console.log(`\n=== 汇总：PASS ${pass} / FAIL ${fail} ===`);
process.exit(fail === 0 ? 0 : 1);
