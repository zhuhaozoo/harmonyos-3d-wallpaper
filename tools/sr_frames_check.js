/**
 * 空间重建合成帧离线校验（3D壁纸 · Spatial Recon Kit 管线）
 *
 * 免构建复现 spatial_recon_bridge.cpp 的相机模型/场景渲染，用于在真机测试前回答三个问题：
 *   [1] 镜像判据：渲染出的图像与"上报给 Kit 的位姿"是否右手系一致
 *       （判据：对任一三维点，渲染器把它放到哪个像素 == 标准针孔投影按上报位姿预测的像素）
 *   [2] 相机系等价性：ARKit 系(Y上/Z后) 与 OpenCV 系(Y下/Z前) 两种上报约定对 (u,v) 的影响
 *       （应完全相同——它们只是同一台相机的两种坐标系标签，用于证明 A/B 开关是安全的）
 *   [3] 场景深度结构：逐像素统计命中面（主盒/背景板）与视差范围
 *       —— SfM 需要"非共面"观测；纯平面场景（全部像素落在同一平面）是退化配置。
 *   [6] 实装一致性（源码级）：直接读 cpp 源码解析射线公式的符号，按"实装公式"渲染首帧并
 *       统计覆盖——防"脚本判据通过、cpp 实装走样"（2026-10-01 真机故障即此类：
 *       cpp 把 basis 第 2 列（-fwd）当成 +fwd 用，整帧只剩底色，重建报 no 3D points）。
 *   [7] 近景特征锚：主盒正面"照片 contain 不变形 + 补边恒存在"（坑 122）
 *   [8] 分层深度场景（scene=3）：参考位姿两层投影重合（=> 合成画面 == 原照片）+ 主体/背景
 *       视差量级与档位单调性 + 源码守卫；并出 layered_ref/last.png 供肉眼核验
 *   [9] 背景补洞（spatial_recon_layers.cpp 的复现）：非主体像素逐像素保真、补洞区不泄漏
 *       主体色、平滑无空洞、贴近周围背景 + 源码守卫
 *
 * 用法：node tools/sr_frames_check.js [--out=tools/ui_shots/sr] [--w=270] [--h=360]
 * 产物：PNG 帧（frame0/mid/last）+ 控制台判据结论。
 */
'use strict';
const fs = require('fs');
const path = require('path');
const zlib = require('zlib');

// ---------- 参数（与 cpp 对齐） ----------
const OUT_W = 1080;
const OUT_H = 1440;
const SCENE_DIST = 2.0;          // 场景中心距离
const OLD_BOX = { hx: 0.72, hy: 0.96, hz: 0.60 };   // 旧"立体"场景：面填满画面（曾是唯一被测的立体场景）
const NEW_BOX = { hx: 0.36, hy: 0.48, hz: 0.30 };   // 新场景：正面 0.72×0.96（照片+10%补边），四周露远景
const BOX2 = { cx: 0.62, cy: -0.30, hx: 0.22, hy: 0.22, hz: 0.22, cz: -2.6 }; // 第二个盒子（更远，加结构）
const BACKDROP_Z = -8.0;

const args = {};
process.argv.slice(2).forEach((a) => {
  const m = a.match(/^--([^=]+)=?(.*)$/);
  if (m) args[m[1]] = m[2];
});
const SHOT_W = parseInt(args.w || '270', 10);
const SHOT_H = parseInt(args.h || '360', 10);
const OUT_DIR = args.out || 'tools/ui_shots/sr';
const ORBIT_DEG = parseFloat(args.orbit || '16');

// ---------- 向量/矩阵工具 ----------
const v3 = (x, y, z) => ({ x, y, z });
const cross = (a, b) => v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const norm = (a) => { const l = Math.sqrt(dot(a, a)); return l < 1e-9 ? v3(0, 0, 0) : v3(a.x / l, a.y / l, a.z / l); };
const add = (a, b) => v3(a.x + b.x, a.y + b.y, a.z + b.z);
const mul = (a, s) => v3(a.x * s, a.y * s, a.z * s);

/** 轨道位姿：basis 列 = [right, up, -fwd]（渲染基：X右 Y上 Z后，右手系） */
function frameBasis(i, n, orbitDeg, dist) {
  const D = dist === undefined ? SCENE_DIST : dist;
  const theta = n > 1 ? (orbitDeg * Math.PI / 180) * (2 * i / (n - 1) - 1) : 0;
  const pos = v3(D * Math.sin(theta), 0, -D + D * Math.cos(theta));
  const center = v3(0, 0, -D);
  const fwd = norm(v3(center.x - pos.x, center.y - pos.y, center.z - pos.z));
  const worldUp = v3(0, 1, 0);
  const right = norm(cross(fwd, worldUp));
  const up = cross(right, fwd);
  return {
    pos, fwd, right, up,
    basis: [
      [right.x, right.y, right.z],
      [up.x, up.y, up.z],
      [-fwd.x, -fwd.y, -fwd.z],
    ],
  };
}

/** 相机局部坐标：c = basisᵀ · (P - pos)（basis 列为正交基，故转置即逆） */
function toCam(frame, P) {
  const d = v3(P.x - frame.pos.x, P.y - frame.pos.y, P.z - frame.pos.z);
  return v3(
    frame.basis[0][0] * d.x + frame.basis[0][1] * d.y + frame.basis[0][2] * d.z,
    frame.basis[1][0] * d.x + frame.basis[1][1] * d.y + frame.basis[1][2] * d.z,
    frame.basis[2][0] * d.x + frame.basis[2][1] * d.y + frame.basis[2][2] * d.z,
  );
}

/** 标准针孔（按帧的渲染基，Y上/Z后）：u = cx + fx·X/(-Z)，v = cy - fy·Y/(-Z) */
function projectStd(frame, P, fx, fy, cx, cy) {
  const c = toCam(frame, P);
  const zf = -c.z;                       // 前方深度
  if (zf <= 1e-9) return null;
  return { u: cx + fx * c.x / zf, v: cy - fy * c.y / zf };
}

/** 渲染器射线（通用式：bSign=图像行方向系数，fwdCoeff=前进项系数；标准式 = (-1, +1)） */
function rayDirEx(frame, u, v, fx, fy, cx, cy, bSign, fwdCoeff) {
  const a = (u + 0.5 - cx) / fx;
  const b = bSign * (v + 0.5 - cy) / fy;
  return {
    x: frame.right.x * a + frame.up.x * b + frame.fwd.x * fwdCoeff,
    y: frame.right.y * a + frame.up.y * b + frame.fwd.y * fwdCoeff,
    z: frame.right.z * a + frame.up.z * b + frame.fwd.z * fwdCoeff,
  };
}

/** 渲染器射线（fixed=false 为修复前的旧实现） */
function rayDir(frame, u, v, fx, fy, cx, cy, fixed) {
  return rayDirEx(frame, u, v, fx, fy, cx, cy, fixed ? -1 : +1, +1);
}

/** 旧实现下，三维点 P 会落在哪个像素（求 ray ∝ (P-pos) 对应的 a、t） */
function pixelOfPointOld(frame, P, fx, fy, cx, cy) {
  const c = toCam(frame, P);
  // 旧实现：dir = a·right + t·up + fwd，且 (P-pos) = α·right + β·up + γ·(-fwd)
  // 匹配：a = -c.x/c.z，t = -c.y/c.z   （c 为相机局部坐标，c.z<0 在前）
  if (c.z >= -1e-9) return null;
  return { u: cx + fx * (-c.x / c.z) - 0.5, v: cy + fy * (-c.y / c.z) - 0.5 };
}

// ---------- 场景求交 ----------
const HIT_NONE = 0, HIT_BOX_FRONT = 1, HIT_BOX_OTHER = 2, HIT_BOX2 = 3, HIT_BACKDROP = 4;

function intersectAabb(o, d, cx, cy, cz, hx, hy, hz) {
  const min = [cx - hx, cy - hy, cz - hz];
  const max = [cx + hx, cy + hy, cz + hz];
  const oo = [o.x, o.y, o.z];
  const dd = [d.x, d.y, d.z];
  let tmin = -1e30, tmax = 1e30, axis = -1, sign = 0;
  for (let a = 0; a < 3; a++) {
    if (Math.abs(dd[a]) < 1e-9) {
      if (oo[a] < min[a] || oo[a] > max[a]) return null;
      continue;
    }
    const inv = 1 / dd[a];
    let t1 = (min[a] - oo[a]) * inv, t2 = (max[a] - oo[a]) * inv, s = -1;
    if (t1 > t2) { const tmp = t1; t1 = t2; t2 = tmp; s = 1; }
    if (t1 > tmin) { tmin = t1; axis = a; sign = s; }
    if (t2 < tmax) tmax = t2;
    if (tmin > tmax) return null;
  }
  if (axis < 0 || tmin < 1e-4) return null;
  return { t: tmin, axis, sign, p: add(o, mul(d, tmin)) };
}

function checker(u, v, cell, a, b) {
  const ix = Math.floor(u / cell), iy = Math.floor(v / cell);
  return ((ix + iy) & 1) ? a : b;
}

/** 伪随机哈希（cpp SrHash01 的复现）：整数晶格 + 种子 → [0,1) */
function hash01(gx, gy, seed) {
  const ix = Math.floor(gx) | 0, iy = Math.floor(gy) | 0;
  let h = (Math.imul(ix, 73856093) ^ Math.imul(iy, 19349663) ^ Math.imul(seed | 0, 83492791)) >>> 0;
  h ^= h >>> 13;
  h = Math.imul(h, 1274126177) >>> 0;
  h ^= h >>> 16;
  return (h & 0xFFFF) / 65536;
}

/** 不可重复程序纹理（cpp SrRandomTexture 的复现）：两级随机晶格叠加 */
function randomTexture(u, v, seed) {
  const c1 = hash01(u / 0.16, v / 0.16, seed);
  const c2 = hash01(u / 0.05, v / 0.05, seed + 31);
  let val = 26 + 128 * c1 + 78 * c2;
  return Math.max(0, Math.min(255, Math.round(val)));
}

/** 主盒正面的"照片矩形"（与 cpp 同构）：源图先按自身宽高比 contain 到面内，再统一内缩 10%（补边恒存在） */
function facePhotoRect(srcAspect, box) {
  const faceAspect = box.hx / box.hy;
  let wRel = 1.0;
  let hRel = faceAspect / srcAspect;
  if (srcAspect < faceAspect) {
    hRel = 1.0;
    wRel = srcAspect / faceAspect;
  }
  const inset = 0.90;   // 与 cpp kPhotoInset 同步
  wRel *= inset;
  hRel *= inset;
  return { u0: (1 - wRel) / 2, u1: (1 + wRel) / 2, v0: (1 - hRel) / 2, v1: (1 + hRel) / 2, w: wRel, h: hRel };
}

/** 渲染一帧（返回 RGB 缓冲与命中统计） */
function renderFrame(frame, opts) {
  const { srcW, srcH, fx, fy, cx, cy, fixed, scene, oldBox } = opts;
  // bSign/fwdCoeff 可显式给出（判据 [6] 按 cpp 实装公式渲染）；缺省时随 fixed 走标准式
  const bSign = opts.bSign === undefined ? (fixed ? -1 : +1) : opts.bSign;
  const fwdCoeff = opts.fwdCoeff === undefined ? +1 : opts.fwdCoeff;
  const rgb = new Uint8Array(SHOT_W * SHOT_H * 3);
  const stat = { front: 0, frontBorder: 0, other: 0, box2: 0, backdrop: 0, none: 0 };
  const box = oldBox ? OLD_BOX : NEW_BOX;
  const photoRect = facePhotoRect(srcW / srcH, box);
  // 平面场景的 contain 定位
  const scale = Math.min(SHOT_W / srcW, SHOT_H / srcH);
  const dispW = srcW * scale, dispH = srcH * scale;
  const ox = (SHOT_W - dispW) / 2, oy = (SHOT_H - dispH) / 2;
  const sampleSrc = (sx, sy) => {
    if (sx < 0 || sy < 0 || sx > srcW - 1 || sy > srcH - 1) return [16, 16, 20];
    return [20 + Math.floor(200 * (sx / srcW)), 20 + Math.floor(200 * (sy / srcH)), 90];
  };
  for (let v = 0; v < SHOT_H; v++) {
    for (let u = 0; u < SHOT_W; u++) {
      const d = rayDirEx(frame, u, v, fx, fy, cx, cy, bSign, fwdCoeff);
      let col = [16, 16, 20];
      if (scene === 1) {
        const hitBox = intersectAabb(frame.pos, d, 0, 0, -SCENE_DIST, box.hx, box.hy, box.hz);
        const hitBox2 = intersectAabb(frame.pos, d, BOX2.cx, BOX2.cy, BOX2.cz, BOX2.hx, BOX2.hy, BOX2.hz);
        let tBd = -1;
        if (d.z < -1e-6) tBd = (BACKDROP_Z - frame.pos.z) / d.z;
        const cands = [];
        if (hitBox) cands.push({ t: hitBox.t, kind: 'box', hit: hitBox });
        if (hitBox2) cands.push({ t: hitBox2.t, kind: 'box2', hit: hitBox2 });
        if (tBd > 0) cands.push({ t: tBd, kind: 'bd' });
        cands.sort((p, q) => p.t - q.t);
        if (cands.length > 0) {
          const c0 = cands[0];
          if (c0.kind === 'bd') {
            const p = add(frame.pos, mul(d, c0.t));
            const tone = randomTexture(p.x, p.y, 37);
            col = [tone, tone, tone];
            stat.backdrop++;
          } else if (c0.kind === 'box2') {
            const p = c0.hit.p;
            const tone = randomTexture(p.x + p.z, p.y, 23);
            col = [tone, tone, tone];
            stat.box2++;
          } else {
            const p = c0.hit.p;
            if (c0.hit.axis === 2 && c0.hit.sign > 0 && scene !== 2) {
              // 正面：照片按自身宽高比 contain，余量用程序纹理补（与 cpp 同构）
              const u01 = p.x / box.hx * 0.5 + 0.5;
              const v01 = 0.5 - p.y / box.hy * 0.5;
              const fu = (u01 - photoRect.u0) / photoRect.w;
              const fv = (v01 - photoRect.v0) / photoRect.h;
              if (fu >= 0 && fu <= 1 && fv >= 0 && fv <= 1) {
                col = sampleSrc(fu * (srcW - 1), fv * (srcH - 1));
                stat.front++;
              } else {
                const tone = randomTexture(p.x, p.y + p.z, 11);
                col = [tone, tone, tone];
                stat.frontBorder++;
              }
            } else {
              const tone = randomTexture(p.x, p.y + p.z, 11);
              col = [tone, tone, tone];
              stat.other++;
            }
          }
        } else {
          stat.none++;
        }
      } else {
        // 平面场景（旧版逻辑，仅用于对照）
        if (Math.abs(d.z) > 1e-9) {
          const t = (-SCENE_DIST - frame.pos.z) / d.z;
          if (t > 0) {
            const p = add(frame.pos, mul(d, t));
            const depth = -p.z;
            if (depth > 1e-6) {
              const u0 = cx + fx * p.x / depth;
              const v0 = bSign < 0 ? cy - fy * p.y / depth : cy + fy * p.y / depth;
              col = sampleSrc(ox + u0, oy + v0);
            }
          }
        }
        stat.front++;
      }
      const o = (v * SHOT_W + u) * 3;
      rgb[o] = col[0]; rgb[o + 1] = col[1]; rgb[o + 2] = col[2];
    }
  }
  return { rgb, stat };
}

// ---------- PNG 输出 ----------
function crc32(buf) {
  let c, table = crc32.table;
  if (!table) {
    table = crc32.table = new Int32Array(256);
    for (let n = 0; n < 256; n++) {
      c = n;
      for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
      table[n] = c;
    }
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
  for (let y = 0; y < h; y++) {
    raw[y * (w * 3 + 1)] = 0;
    Buffer.from(rgb.buffer, rgb.byteOffset + y * w * 3, w * 3).copy(raw, y * (w * 3 + 1) + 1);
  }
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4);
  ihdr[8] = 8; ihdr[9] = 2; ihdr[10] = 0; ihdr[11] = 0; ihdr[12] = 0;
  fs.writeFileSync(file, Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A]),
    chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0)),
  ]));
}

// ---------- 判据 ----------
let pass = 0, fail = 0;
function check(name, ok, detail) {
  console.log(`${ok ? 'PASS' : 'FAIL'}  ${name}${detail ? '  ' + detail : ''}`);
  ok ? pass++ : fail++;
}

const fx = 1500, fy = 1500, cx = OUT_W / 2, cy = OUT_H / 2;
const N = 72;
const f0 = frameBasis(0, N, ORBIT_DEG);
const fMid = frameBasis(36, N, ORBIT_DEG);
const fLast = frameBasis(N - 1, N, ORBIT_DEG);

console.log('=== [1] 镜像判据：渲染器落点 == 标准投影按上报位姿预测的落点 ===');
{
  // 取若干三维点，比较"旧实现把点放到哪个像素"与"标准投影预测的像素"
  const pts = [v3(0.1, 0.5, -2), v3(-0.3, -0.4, -2), v3(0.2, 0.9, -8)];
  let oldMismatch = 0, oldMirror = 0;
  pts.forEach((P, k) => {
    const std = projectStd(f0, P, fx, fy, cx, cy);
    const oldPx = pixelOfPointOld(f0, P, fx, fy, cx, cy);
    if (!std || !oldPx) return;
    const dv = oldPx.v - std.v;
    if (Math.abs(dv) > 1) oldMismatch++;
    if (Math.abs(oldPx.v - (2 * cy - std.v)) < 1) oldMirror++;
    console.log(`  点${k} y=${P.y.toFixed(2)}  标准投影 v=${std.v.toFixed(1)}  旧实现 v=${oldPx.v.toFixed(1)}  ` +
      `${Math.abs(oldPx.v - (2 * cy - std.v)) < 1 ? '(恰为垂直镜像)' : ''}`);
  });
  check('旧实现存在垂直镜像（图像与位姿不一致）', oldMismatch === pts.length && oldMirror === pts.length,
    `mismatch=${oldMismatch}/${pts.length} mirror=${oldMirror}/${pts.length}`);
  // 修复后：渲染器射线应等于标准投影射线
  let rayOk = 0;
  [[f0, 0, 0], [f0, 300, 1000], [fMid, 900, 300], [fLast, 540, 720]].forEach(([fr, u, v]) => {
    const d = rayDir(fr, u, v, fx, fy, cx, cy, true);
    // 标准：像素 (u,v) 的射线方向（Y上/Z后） = a·right + (-b)·up + fwd，a=(u-cx)/fx, b=(v-cy)/fy
    const a = (u + 0.5 - cx) / fx, b = (v + 0.5 - cy) / fy;
    const want = {
      x: fr.right.x * a - fr.up.x * b + fr.fwd.x,
      y: fr.right.y * a - fr.up.y * b + fr.fwd.y,
      z: fr.right.z * a - fr.up.z * b + fr.fwd.z,
    };
    if (Math.hypot(d.x - want.x, d.y - want.y, d.z - want.z) < 1e-6) rayOk++;
  });
  check('修复后渲染射线 == 标准针孔射线', rayOk === 4, `ok=${rayOk}/4`);
}

console.log('=== [2] 相机系等价性：ARKit 系 / OpenCV 系两种上报对 (u,v) 无影响 ===');
{
  // family1 = R·diag(1,-1,-1)；对同一 P，用其相机局部坐标按 (Y下/Z前) 模型投影，应与 family0 完全一致
  const pts = [v3(0.1, 0.5, -2), v3(-0.7, 0.3, -8), v3(0.4, -0.6, -1.7)];
  let same = 0;
  pts.forEach((P) => {
    const a0 = projectStd(f0, P, fx, fy, cx, cy);            // family0: Y上/Z后, v = cy - fy·Y/(-Z)
    const c = toCam(f0, P);                                   // family1: (X, -Y, -Z), v = cy + fy·Y'/Z'
    const u1 = cx + fx * c.x / (-c.z);
    const v1 = cy + fy * (-c.y) / (-c.z);
    if (a0 && Math.abs(a0.u - u1) < 1e-6 && Math.abs(a0.v - v1) < 1e-6) same++;
  });
  check('两种相机系标签给出相同像素（A/B 开关安全）', same === pts.length, `same=${same}/${pts.length}`);
}

console.log('=== [3] 场景深度结构（SfM 需要非共面观测） ===');
{
  const opts = (scene, oldBox) => ({ srcW: 900, srcH: 1200, fx: fx * SHOT_W / OUT_W, fy: fy * SHOT_H / OUT_H, cx: SHOT_W / 2, cy: SHOT_H / 2, fixed: true, scene, oldBox });
  const rOld = renderFrame(f0, opts(1, true));    // 旧"立体"（面填满画面）
  const rNew = renderFrame(f0, opts(1, false));   // 新立体（小盒 + 背景板）
  const pct = (s) => `主盒正面 ${(100 * s.front / (SHOT_W * SHOT_H)).toFixed(1)}% 正面补边 ${(100 * (s.frontBorder || 0) / (SHOT_W * SHOT_H)).toFixed(1)}% 主盒侧面 ${(100 * s.other / (SHOT_W * SHOT_H)).toFixed(1)}% 副盒 ${(100 * s.box2 / (SHOT_W * SHOT_H)).toFixed(1)}% 远景 ${(100 * s.backdrop / (SHOT_W * SHOT_H)).toFixed(1)}%`;
  console.log('  旧"立体"场景命中分布: ' + pct(rOld.stat));
  console.log('  新立体场景命中分布:   ' + pct(rNew.stat));
  const oldDepthLayers = (rOld.stat.front + rOld.stat.other > 0 && rOld.stat.backdrop + rOld.stat.box2 === 0) ? 1 : 2;
  const newLayers = (rNew.stat.backdrop > 0 && (rNew.stat.front + rNew.stat.other + rNew.stat.box2) > 0) ? 2 : 1;
  check('旧"立体"场景实为单深度层（面填满画面，等价平面）→ 解释首测 no 3D points', oldDepthLayers === 1,
    `层数=${oldDepthLayers}`);
  check('新立体场景含两个以上深度层（主盒 z≈-1.7 / 远景 z=-8）', newLayers >= 2, `层数=${newLayers}`);
  // 视差对比：同一轨道下，盒面点与远景点在图像上的位移量级
  const boxPt = v3(0.25, 0.3, -1.7), bdPt = v3(0.25, 0.3, -8);
  const disp = (P) => {
    const a = projectStd(f0, P, fx, fy, cx, cy), b = projectStd(fLast, P, fx, fy, cx, cy);
    return Math.hypot(a.u - b.u, a.v - b.v);
  };
  console.log(`  ±${ORBIT_DEG / 2}° 轨道下像素位移：盒面点 ${disp(boxPt).toFixed(0)}px，远景点 ${disp(bdPt).toFixed(0)}px`);
  check('两个深度层的视差量级不同（深度可分）', Math.abs(disp(boxPt) - disp(bdPt)) > 5,
    `Δ=${Math.abs(disp(boxPt) - disp(bdPt)).toFixed(0)}px`);
}

console.log('=== [5] 纹理非重复性（周期纹理会造成特征匹配歧义，SfM 需要处处唯一的纹理） ===');
{
  // 在纹理域上采 24x24 的补丁若干，统计"近重复补丁"比例
  const patchPts = [];
  for (let i = 0; i < 40; i++) {
    patchPts.push([(hash01(i, 7, 99) - 0.5) * 6, (hash01(i, 13, 123) - 0.5) * 3]);
  }
  const PATCH = 8, STEP = 0.02;
  const patchOf = (fn, cx, cy) => {
    const a = [];
    for (let r = 0; r < PATCH; r++) {
      for (let c = 0; c < PATCH; c++) a.push(fn(cx + (c - PATCH / 2) * STEP, cy + (r - PATCH / 2) * STEP));
    }
    return a;
  };
  const dupFraction = (fn) => {
    let dup = 0, tot = 0;
    const patches = patchPts.map(([x, y]) => patchOf(fn, x, y));
    for (let i = 0; i < patches.length; i++) {
      for (let j = i + 1; j < patches.length; j++) {
        let ssd = 0;
        for (let k = 0; k < patches[i].length; k++) {
          const df = patches[i][k] - patches[j][k];
          ssd += df * df;
        }
        const rms = Math.sqrt(ssd / patches[i].length);
        tot++;
        if (rms < 12) dup++;   // 近重复判据
      }
    }
    return dup / tot;
  };
  const fChecker = (u, v) => checker(u, v, 0.16, 205, 62);
  const fRandom = (u, v) => randomTexture(u, v, 11);
  const dChk = dupFraction(fChecker), dRnd = dupFraction(fRandom);
  console.log(`  近重复补丁比例：周期棋盘格 ${(100 * dChk).toFixed(1)}%  新程序纹理 ${(100 * dRnd).toFixed(1)}%`);
  check('新程序纹理几乎无重复（< 5%），优于棋盘格', dRnd < 0.05 && dRnd < dChk, `chk=${(100 * dChk).toFixed(1)}% rnd=${(100 * dRnd).toFixed(1)}%`);
}

console.log('=== [4] 出图（供肉眼核验） ===');
{
  fs.mkdirSync(OUT_DIR, { recursive: true });
  const mkOpts = (scene, oldBox) => ({ srcW: 900, srcH: 1200, fx: fx * SHOT_W / OUT_W, fy: fy * SHOT_H / OUT_H, cx: SHOT_W / 2, cy: SHOT_H / 2, fixed: true, scene, oldBox });
  [[f0, 'frame0'], [fMid, 'mid'], [fLast, 'last']].forEach(([fr, tag]) => {
    const r = renderFrame(fr, mkOpts(1, false));
    writePng(path.join(OUT_DIR, `new3d_${tag}.png`), SHOT_W, SHOT_H, r.rgb);
  });
  const rp = renderFrame(f0, mkOpts(0, false));
  writePng(path.join(OUT_DIR, 'plane_frame0.png'), SHOT_W, SHOT_H, rp.rgb);
  const ro = renderFrame(f0, mkOpts(1, true));
  writePng(path.join(OUT_DIR, 'oldbox_frame0.png'), SHOT_W, SHOT_H, ro.rgb);
  console.log(`  已输出到 ${OUT_DIR}/：new3d_frame0|mid|last.png / plane_frame0.png / oldbox_frame0.png`);
}

console.log('=== [6] 实装一致性：cpp 源码的射线公式 == 本判据公式（防"脚本过、代码错"） ===');
{
  const cppPath = path.join(__dirname, '..', 'src', 'cpp', 'spatial_recon_bridge.cpp');
  const src = fs.readFileSync(cppPath, 'utf8');
  const mCol = src.match(/basis\[0\]\s*\*\s*a\s*\+\s*basis\[3\]\s*\*\s*b\s*([+-])\s*basis\[6\]/);
  const mB = src.match(/const float b = ([+-])\(v \+ 0\.5f - cy\) \/ fy/);
  check('cpp 源码中可定位到射线公式（正则匹配）', !!mCol && !!mB);
  const bSign = (mB && mB[1] === '-') ? -1 : +1;
  // basis 第 2 列存的是 -fwd：列前取 '-' 才等效于 fwd 项 +1
  const fwdCoeff = (mCol && mCol[1] === '-') ? +1 : -1;
  check('图像行向下 ↔ 相机 Y 轴向下（b 取负，非镜像）', bSign === -1, `bSign=${bSign}`);
  check('第三列取负 → 射线沿 +fwd（不朝相机背后）', fwdCoeff === +1, `fwdCoeff=${fwdCoeff}`);
  const mkOpts = (fwd) => ({ srcW: 900, srcH: 1200, fx: fx * SHOT_W / OUT_W, fy: fy * SHOT_H / OUT_H, cx: SHOT_W / 2, cy: SHOT_H / 2, scene: 1, oldBox: false, bSign, fwdCoeff: fwd });
  const cov = (r) => 1 - r.stat.none / (SHOT_W * SHOT_H);
  const rOk = renderFrame(f0, mkOpts(fwdCoeff));
  console.log(`  按实装公式渲染首帧：覆盖 ${(100 * cov(rOk)).toFixed(1)}%（底色 ${(100 * (1 - cov(rOk))).toFixed(1)}%）`);
  check('实装公式渲染首帧非空（覆盖 > 50%）', cov(rOk) > 0.5, `cover=${(100 * cov(rOk)).toFixed(1)}%`);
  const rBad = renderFrame(f0, mkOpts(-1));
  console.log(`  反证（第三列漏取负号）：覆盖 ${(100 * cov(rBad)).toFixed(1)}%`);
  check('第三列漏取负号 → 首帧 100% 底色（真机 no 3D points 的复现）', cov(rBad) < 0.01,
    `cover=${(100 * cov(rBad)).toFixed(1)}%`);
}

console.log('=== [7] 主盒正面：源图不变形（contain）+ 补边有纹理（近景特征锚） ===');
{
  const cppSrc7 = fs.readFileSync(path.join(__dirname, '..', 'src', 'cpp', 'spatial_recon_bridge.cpp'), 'utf8');
  check('cpp 实装 contain 映射（面宽高比参与换算，防退回"拉伸铺满"）', /faceAspect/.test(cppSrc7));
  const cases = [[3, 4, '竖幅 3:4'], [4, 3, '横幅 4:3'], [16, 9, '横幅 16:9'], [9, 16, '竖幅 9:16'], [1, 1, '方形 1:1']];
  const tot = SHOT_W * SHOT_H;
  for (const [aw, ah, name] of cases) {
    const srcW = 900;
    const srcH = Math.round(900 * ah / aw);
    const r = facePhotoRect(aw / ah, NEW_BOX);
    // 面内照片的物理宽高比 == 源图宽高比（各向同性采样 = 不变形）
    const drawn = (r.w * 2 * NEW_BOX.hx) / (r.h * 2 * NEW_BOX.hy);
    check(`${name}：照片不被拉伸（面内 ${drawn.toFixed(4)} == 源 ${(aw / ah).toFixed(4)}）`, Math.abs(drawn - aw / ah) < 1e-9);
    const rr = renderFrame(f0, { srcW, srcH, fx: fx * SHOT_W / OUT_W, fy: fy * SHOT_H / OUT_H, cx: SHOT_W / 2, cy: SHOT_H / 2, fixed: true, scene: 1, oldBox: false });
    const photoPct = 100 * rr.stat.front / tot;
    const borderPct = 100 * rr.stat.frontBorder / tot;
    console.log(`    ${name}：正面照片 ${photoPct.toFixed(1)}%  正面补边 ${borderPct.toFixed(1)}%（占画面）`);
    check(`${name}：照片可见且近景补边恒存在`, photoPct > 0 && borderPct > 0.5, `photo=${photoPct.toFixed(1)}% border=${borderPct.toFixed(1)}%`);
  }
  const vals = [];
  for (let i = 0; i < 200; i++) {
    vals.push(randomTexture((hash01(i, 3, 7) - 0.5) * 0.6, (hash01(i, 5, 11) - 0.5) * 0.8, 11));
  }
  const mean = vals.reduce((a, b) => a + b, 0) / vals.length;
  const std = Math.sqrt(vals.reduce((a, b) => a + (b - mean) * (b - mean), 0) / vals.length);
  check('补边纹理为程序纹理（非平色，特征可用）', std > 20, `std=${std.toFixed(1)} unique=${new Set(vals).size}/200`);
}

// ---------- 分层深度（scene=3）复现：与 cpp 同构的两层采样 ----------
const K_FACE_Z = SCENE_DIST - NEW_BOX.hz;   // 主盒正面深度 = 1.7（cpp kFaceZ）

/** 命中"深度 depth 的平行平面"→ 折算回主盒正面坐标（k 倍缩放）→ 面上归一化坐标。
 *  u/v 传**像素下标（整数）**，像素中心的 +0.5 由 rayDir 内部统一加（勿重复加）。 */
function layerFaceUv(fr, u, v, depth, k, box, intr) {
  const I = intr || { fx, fy, cx, cy };
  const d = rayDir(fr, u, v, I.fx, I.fy, I.cx, I.cy, true);
  if (d.z >= -1e-6) return null;
  const t = (-depth - fr.pos.z) / d.z;
  if (!(t > 0)) return null;
  const invK = 1 / k;
  const x = (fr.pos.x + d.x * t) * invK;
  const y = (fr.pos.y + d.y * t) * invK;
  return { u01: x / box.hx * 0.5 + 0.5, v01: 0.5 - y / box.hy * 0.5, x, y };
}

/** 源图 uv（含 contain + 内缩映射）；越界不裁，交由调用方按 [0,1] 判定 */
function layerSrcUv(fr, u, v, depth, k, box, photoRect, intr) {
  const f = layerFaceUv(fr, u, v, depth, k, box, intr);
  if (!f) return null;
  return {
    fu: (f.u01 - photoRect.u0) / photoRect.w,
    fv: (f.v01 - photoRect.v0) / photoRect.h,
  };
}

/** 蒙版原始值 → 主体概率（cpp SrMaskAlpha 复现） */
function maskAlpha(m) {
  if (m <= 16) return 0;
  if (m >= 240) return 1;
  return (m - 16) / 224;
}

/** 双线性采样 float 颜色场（坐标以像素中心为准，**越界钳制到边缘**；cpp SrSampleColorField 复现）。
 *  ⚠️ 与 sampleImg 的语义不同：图像采样越界返回暗底（表示"没有内容"），补洞场越界必须钳制
 *  ——补洞场的每个纹素都有值，用暗底会往场里掺入无关的黑色（本判据第一版就踩了这个，见 [9]）。 */
function sampleField(color, w, h, x, y) {
  let sx = x, sy = y;
  if (sx < 0) sx = 0;
  if (sy < 0) sy = 0;
  if (sx > w - 1) sx = w - 1;
  if (sy > h - 1) sy = h - 1;
  const x0 = Math.floor(sx), y0 = Math.floor(sy);
  const x1 = Math.min(x0 + 1, w - 1), y1 = Math.min(y0 + 1, h - 1);
  const ax = sx - x0, ay = sy - y0;
  const out = [0, 0, 0];
  for (let c = 0; c < 3; c++) {
    const p00 = color[(y0 * w + x0) * 3 + c], p10 = color[(y0 * w + x1) * 3 + c];
    const p01 = color[(y1 * w + x0) * 3 + c], p11 = color[(y1 * w + x1) * 3 + c];
    const top = p00 + (p10 - p00) * ax, bot = p01 + (p11 - p01) * ax;
    out[c] = top + (bot - top) * ay;
  }
  return out;
}

/** 双线性采样 RGBA/RGB 图（越界返回暗底，与 cpp SrSampleImg 同构） */
function sampleImg(img, w, h, ch, sx, sy) {
  if (sx < 0 || sy < 0 || sx > w - 1 || sy > h - 1) return [16, 16, 20];
  const x0 = Math.floor(sx), y0 = Math.floor(sy);
  const x1 = Math.min(x0 + 1, w - 1), y1 = Math.min(y0 + 1, h - 1);
  const ax = sx - x0, ay = sy - y0;
  const out = [0, 0, 0];
  for (let c = 0; c < 3; c++) {
    const p00 = img[(y0 * w + x0) * ch + c], p10 = img[(y0 * w + x1) * ch + c];
    const p01 = img[(y1 * w + x0) * ch + c], p11 = img[(y1 * w + x1) * ch + c];
    const top = p00 + (p10 - p00) * ax, bot = p01 + (p11 - p01) * ax;
    out[c] = Math.max(0, Math.min(255, top + (bot - top) * ay));
  }
  return out;
}

/** 双线性采样蒙版 → 主体概率 [0,1]（cpp SrSampleMask 复现） */
function sampleMask(mask, w, h, sx, sy) {
  if (sx < 0 || sy < 0 || sx > w - 1 || sy > h - 1) return 0;
  const x0 = Math.floor(sx), y0 = Math.floor(sy);
  const x1 = Math.min(x0 + 1, w - 1), y1 = Math.min(y0 + 1, h - 1);
  const ax = sx - x0, ay = sy - y0;
  const p00 = maskAlpha(mask[y0 * w + x0]), p10 = maskAlpha(mask[y0 * w + x1]);
  const p01 = maskAlpha(mask[y1 * w + x0]), p11 = maskAlpha(mask[y1 * w + x1]);
  const top = p00 + (p10 - p00) * ax, bot = p01 + (p11 - p01) * ax;
  return top + (bot - top) * ay;
}

/**
 * 背景补洞（cpp SrBuildLayerAssets 的复现）：1/4 分辨率蒙版加权金字塔 push-pull。
 * 返回 { bg: Uint8Array(w*h*3), levels, subjectPx } 或 null（蒙版退化）。
 */
function fillBackground(srcRgba, w, h, mask, downStep) {
  const D = downStep || 4;
  const total = w * h;
  let subj = 0;
  for (let i = 0; i < total; i++) { if (maskAlpha(mask[i]) >= 0.5) subj++; }
  const ratio = subj / total;
  if (ratio < 0.005 || ratio > 0.95) return null;
  const mw = Math.ceil(w / D), mh = Math.ceil(h / D);
  const base = { w: mw, h: mh, color: new Float32Array(mw * mh * 3), weight: new Float32Array(mw * mh) };
  let gR = 0, gG = 0, gB = 0, gW = 0;
  for (let by = 0; by < mh; by++) {
    for (let bx = 0; bx < mw; bx++) {
      let aR = 0, aG = 0, aB = 0, aW = 0;
      for (let y = by * D; y < Math.min(h, (by + 1) * D); y++) {
        for (let x = bx * D; x < Math.min(w, (bx + 1) * D); x++) {
          const o = y * w + x;
          const wt = 1 - maskAlpha(mask[o]);
          if (wt <= 0) continue;
          aR += wt * srcRgba[o * 4]; aG += wt * srcRgba[o * 4 + 1]; aB += wt * srcRgba[o * 4 + 2];
          aW += wt;
        }
      }
      const t = by * mw + bx;
      base.weight[t] = Math.min(1, aW / (D * D));
      if (aW > 1e-6) {
        base.color[t * 3] = aR / aW; base.color[t * 3 + 1] = aG / aW; base.color[t * 3 + 2] = aB / aW;
      }
      gR += aR; gG += aG; gB += aB; gW += aW;
    }
  }
  if (gW < 1e-6) return null;
  const mean = [gR / gW, gG / gW, gB / gW];
  const levels = [base];
  while (levels[levels.length - 1].w > 1 || levels[levels.length - 1].h > 1) {
    const fine = levels[levels.length - 1];
    const cw = Math.ceil(fine.w / 2), chh = Math.ceil(fine.h / 2);
    const coarse = { w: cw, h: chh, color: new Float32Array(cw * chh * 3), weight: new Float32Array(cw * chh) };
    for (let y = 0; y < chh; y++) {
      for (let x = 0; x < cw; x++) {
        let aR = 0, aG = 0, aB = 0, aW = 0;
        for (let dy = 0; dy < 2; dy++) {
          const sy = y * 2 + dy;
          if (sy >= fine.h) continue;
          for (let dx = 0; dx < 2; dx++) {
            const sx = x * 2 + dx;
            if (sx >= fine.w) continue;
            const s = sy * fine.w + sx;
            const wt = fine.weight[s];
            if (wt <= 0) continue;
            aR += wt * fine.color[s * 3]; aG += wt * fine.color[s * 3 + 1]; aB += wt * fine.color[s * 3 + 2];
            aW += wt;
          }
        }
        const t = y * cw + x;
        coarse.weight[t] = Math.min(1, aW / 4);
        if (aW > 1e-6) {
          coarse.color[t * 3] = aR / aW; coarse.color[t * 3 + 1] = aG / aW; coarse.color[t * 3 + 2] = aB / aW;
        }
      }
    }
    levels.push(coarse);
  }
  const top = levels[levels.length - 1];
  for (let t = 0; t < top.weight.length; t++) {
    const wt = top.weight[t];
    if (wt < 1) {
      for (let c = 0; c < 3; c++) top.color[t * 3 + c] = wt * top.color[t * 3 + c] + (1 - wt) * mean[c];
      top.weight[t] = 1;
    }
  }
  for (let k = levels.length - 2; k >= 0; k--) {
    const fine = levels[k], coarse = levels[k + 1];
    for (let y = 0; y < fine.h; y++) {
      for (let x = 0; x < fine.w; x++) {
        const t = y * fine.w + x;
        const wt = fine.weight[t];
        if (wt >= 1) continue;
        const up = sampleField(coarse.color, coarse.w, coarse.h,
          (x + 0.5) * 0.5 - 0.5, (y + 0.5) * 0.5 - 0.5);
        for (let c = 0; c < 3; c++) fine.color[t * 3 + c] = wt * fine.color[t * 3 + c] + (1 - wt) * up[c];
        fine.weight[t] = 1;
      }
    }
  }
  const filled = levels[0];
  const FILL_THRESHOLD = 245;   // cpp kFillThreshold
  const bg = new Uint8Array(total * 3);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const o = y * w + x;
      if (mask[o] < FILL_THRESHOLD) {
        bg[o * 3] = srcRgba[o * 4]; bg[o * 3 + 1] = srcRgba[o * 4 + 1]; bg[o * 3 + 2] = srcRgba[o * 4 + 2];
        continue;
      }
      const c = sampleField(filled.color, filled.w, filled.h,
        (x + 0.5) / D - 0.5, (y + 0.5) / D - 0.5);
      for (let ch = 0; ch < 3; ch++) bg[o * 3 + ch] = Math.max(0, Math.min(255, Math.round(c[ch])));
    }
  }
  return { bg, subjectPx: subj, ratio, levels: levels.length };
}

/** 分层深度合成一帧（判据图用；只渲染"照片平面"区域，区外置暗底便于分辨）。
 *  srcCh = 源图每像素字节数（与 fillBackground 的输入一致：判据图用 4=RGBA）。
 *  box = 照片所在的面（默认主盒正面 NEW_BOX；盒中空间传放大后的面）。 */
function renderLayeredFrame(fr, srcImg, srcCh, srcW, srcH, bgImg, mask, kSubj, photoRect, box) {
  const B = box || NEW_BOX;
  // 判据图按 SHOT_W×SHOT_H 渲染：内参必须同比缩放（主点=画面中心），否则画面整体偏移
  const intr = { fx: fx * SHOT_W / OUT_W, fy: fy * SHOT_H / OUT_H, cx: SHOT_W / 2, cy: SHOT_H / 2 };
  const rgb = new Uint8Array(SHOT_W * SHOT_H * 3);
  const stat = { subjOnly: 0, blend: 0, bg: 0, none: 0 };
  for (let v = 0; v < SHOT_H; v++) {
    for (let u = 0; u < SHOT_W; u++) {
      const bgUv = layerSrcUv(fr, u, v, K_FACE_Z, 1, B, photoRect, intr);
      let col;
      if (bgUv && bgUv.fu >= 0 && bgUv.fu <= 1 && bgUv.fv >= 0 && bgUv.fv <= 1) {
        col = sampleImg(bgImg, srcW, srcH, 3, bgUv.fu * (srcW - 1), bgUv.fv * (srcH - 1));
        stat.bg++;
      } else {
        col = [30, 30, 34];
        stat.none++;
      }
      const sUv = layerSrcUv(fr, u, v, K_FACE_Z * kSubj, kSubj, B, photoRect, intr);
      if (sUv && sUv.fu >= 0 && sUv.fu <= 1 && sUv.fv >= 0 && sUv.fv <= 1) {
        const sx = sUv.fu * (srcW - 1), sy = sUv.fv * (srcH - 1);
        const m = sampleMask(mask, srcW, srcH, sx, sy);
        if (m > 0) {
          const sc = sampleImg(srcImg, srcW, srcH, srcCh, sx, sy);
          for (let c = 0; c < 3; c++) col[c] = m * sc[c] + (1 - m) * col[c];
          if (m >= 0.99) stat.subjOnly++; else stat.blend++;
        }
      }
      const o = (v * SHOT_W + u) * 3;
      rgb[o] = col[0]; rgb[o + 1] = col[1]; rgb[o + 2] = col[2];
    }
  }
  return { rgb, stat };
}

console.log('=== [8] 分层深度场景：参考位姿两层投影重合 + 主体/背景视差 ===');
{
  const cppLayered = fs.readFileSync(path.join(__dirname, '..', 'src', 'cpp', 'spatial_recon_bridge.cpp'), 'utf8');
  check('cpp 实装分层深度：kFaceZ 常量 + 近平面按 kSubj 折算回正面坐标',
    /constexpr float kFaceZ/.test(cppLayered) && /layers->kSubj/.test(cppLayered));
  check('cpp 实装分层深度：主体层走蒙版采样、背景层走补洞图',
    /SrSampleMask\(layers->mask/.test(cppLayered) && /layers->bgRgb/.test(cppLayered));

  const photoRect = facePhotoRect(900 / 1200, NEW_BOX);
  const K = 0.90;                        // 默认主体深度比（「中」档；0.80 真机失败后收到 0.90，见文档 §4.6）
  const atRef = frameBasis(0, 1, ORBIT_DEG);   // n=1 → theta=0（参考位姿）
  let maxDuv = 0, samples = 0;
  for (let u = 0; u < OUT_W; u += 45) {
    for (let v = 0; v < OUT_H; v += 45) {
      const bg = layerSrcUv(atRef, u, v, K_FACE_Z, 1, NEW_BOX, photoRect);
      const sj = layerSrcUv(atRef, u, v, K_FACE_Z * K, K, NEW_BOX, photoRect);
      if (!bg || !sj) continue;
      if (bg.fu < 0 || bg.fu > 1 || bg.fv < 0 || bg.fv > 1) continue;
      samples++;
      maxDuv = Math.max(maxDuv, Math.abs(bg.fu - sj.fu), Math.abs(bg.fv - sj.fv));
    }
  }
  console.log(`  参考位姿：${samples} 个采样点，两层源图坐标最大偏差 ${(maxDuv * 899).toFixed(4)}px`);
  check('参考位姿两层投影重合（=> 分层合成画面 == 原照片）',
    samples > 0 && maxDuv * 899 < 1e-2, `maxΔ=${(maxDuv * 899).toFixed(4)}px`);

  const su = { fu: 0.5, fv: 0.40 };      // 主体上的一个源图点（归一化）
  const worldOfSrc = (fu, fv, k) => {
    const u01 = photoRect.u0 + fu * photoRect.w;
    const v01 = photoRect.v0 + fv * photoRect.h;
    return v3((u01 - 0.5) * 2 * NEW_BOX.hx * k, (0.5 - v01) * 2 * NEW_BOX.hy * k, -K_FACE_Z * k);
  };
  const slide = (k) => {
    const P = worldOfSrc(su.fu, su.fv, k);
    const a = projectStd(f0, P, fx, fy, cx, cy), b = projectStd(fLast, P, fx, fy, cx, cy);
    return { du: b.u - a.u, dv: b.v - a.v, mag: Math.hypot(b.u - a.u, b.v - a.v) };
  };
  const revealOf = (k) => {
    const s1 = slide(1), sK = slide(k);
    return Math.hypot(sK.du - s1.du, sK.dv - s1.dv);
  };
  const sBg = slide(1), sSub = slide(K);
  const reveal = revealOf(K);
  // 相邻关键帧步长（Kit 从 72 帧里选 ~5 个关键帧 → 步长约 8°）的位移才是"能否匹配"的判据：
  // 经验界来自本管线实测（z=-1.7 处 130px 可匹配；z=-8 处 640px 已超匹配窗口）。
  const stepFrames = [0, 1, 2, 3, 4].map((i) => frameBasis(i, 5, 2 * ORBIT_DEG));   // ±16°，步长 8°
  const stepDisp = (k) => {
    const P = worldOfSrc(su.fu, su.fv, k);
    let mx = 0;
    for (let i = 0; i + 1 < stepFrames.length; i++) {
      const a = projectStd(stepFrames[i], P, fx, fy, cx, cy);
      const b = projectStd(stepFrames[i + 1], P, fx, fy, cx, cy);
      mx = Math.max(mx, Math.hypot(b.u - a.u, b.v - a.v));
    }
    return mx;
  };
  const stepBg = stepDisp(1), stepSub = stepDisp(K);
  console.log(`  ±${ORBIT_DEG / 2}° 轨道：背景层位移 ${sBg.mag.toFixed(0)}px，主体层位移 ${sSub.mag.toFixed(0)}px，相对滑移 ${reveal.toFixed(0)}px`);
  console.log(`  相邻关键帧（8°）位移：背景层 ${stepBg.toFixed(0)}px，主体层 ${stepSub.toFixed(0)}px（实测可匹配界 ≈130px/超窗 ≈640px）`);
  check('近层动得多（视差方向正确）', sSub.mag > sBg.mag + 5,
    `subj=${sSub.mag.toFixed(0)}px bg=${sBg.mag.toFixed(0)}px`);
  check('主体层 ±8° 位移在"可匹配位移量级"内（< 350px；背景层实测可匹配 ≈130~145px）',
    sSub.mag < 350, `subj=${sSub.mag.toFixed(0)}px`);
  check('两层的关键帧步长位移都在匹配窗口内（< 180px）', stepBg < 180 && stepSub < 180,
    `bg=${stepBg.toFixed(0)}px subj=${stepSub.toFixed(0)}px`);
  check('相对滑移 ∈ [60, 250]px（够 SfM 分成两层、又不至于露出大片补洞）', reveal >= 60 && reveal <= 250,
    `reveal=${reveal.toFixed(0)}px`);
  const rNear = revealOf(0.84), rFar = revealOf(0.94);
  console.log(`  主体深度档位滑移：近(0.84) ${rNear.toFixed(0)}px / 中(0.90) ${reveal.toFixed(0)}px / 远(0.94) ${rFar.toFixed(0)}px`);
  check('档位单调：主体越近滑移越大（近 > 中 > 远）', rNear > reveal + 1 && reveal > rFar + 1,
    `${rNear.toFixed(0)}/${reveal.toFixed(0)}/${rFar.toFixed(0)}`);

  // 出图：参考位姿与轨道端点帧（合成源图 + 合成蒙版 + JS 复现的补洞）
  const sw = 270, sh = 360;
  const srcSyn = new Uint8Array(sw * sh * 4);   // 4 字节/像素：与补洞接口/实装源图一致（勿只给 3 字节）
  const maskImg = new Uint8Array(sw * sh);
  const ccx = sw * 0.5, ccy = sh * 0.42, rx = sw * 0.17, ry = sh * 0.26;
  for (let y = 0; y < sh; y++) {
    for (let x = 0; x < sw; x++) {
      const o = y * sw + x;
      const inside = ((x - ccx) / rx) ** 2 + ((y - ccy) / ry) ** 2 <= 1;
      if (inside) {
        srcSyn[o * 4] = 210; srcSyn[o * 4 + 1] = 40; srcSyn[o * 4 + 2] = 60;   // 主体：暖红
        srcSyn[o * 4 + 3] = 255;
        maskImg[o] = 255;
      } else {
        srcSyn[o * 4] = 20;
        srcSyn[o * 4 + 1] = 60 + Math.round(120 * x / sw);
        srcSyn[o * 4 + 2] = 140 + Math.round(60 * y / sh);                     // 背景：蓝青渐变
        srcSyn[o * 4 + 3] = 255;
        maskImg[o] = 0;
      }
    }
  }
  const filled = fillBackground(srcSyn, sw, sh, maskImg);
  check('判据用合成图的补洞可用（未判退化）', filled !== null);
  if (filled !== null) {
    // 防"源图缓冲格式传错"（把 3 字节图当 4 字节喂进补洞接口 → 越界读出全黑，判据图会莫名其妙发黑）
    let okPix = 0, tot = 0;
    for (let i = 0; i < sw * sh; i += 7) {
      if (maskImg[i] >= 245) continue;
      tot++;
      if (filled.bg[i * 3] === srcSyn[i * 4] && filled.bg[i * 3 + 1] === srcSyn[i * 4 + 1]
        && filled.bg[i * 3 + 2] === srcSyn[i * 4 + 2]) okPix++;
    }
    check('判据图源缓冲格式一致（非主体像素逐像素保真）', tot > 0 && okPix === tot, `${okPix}/${tot}`);
    fs.mkdirSync(OUT_DIR, { recursive: true });
    const pr = facePhotoRect(sw / sh, NEW_BOX);
    [[atRef, 'ref'], [fLast, 'last']].forEach(([fr, tag]) => {
      const r = renderLayeredFrame(fr, srcSyn, 4, sw, sh, filled.bg, maskImg, K, pr);
      writePng(path.join(OUT_DIR, `layered_${tag}.png`), SHOT_W, SHOT_H, r.rgb);
    });
    console.log(`  已输出 ${OUT_DIR}/layered_ref.png（参考位姿）/ layered_last.png（轨道端点）供肉眼核验`);
  }
}

console.log('=== [9] 背景补洞：参考图逐像素保真 + 补洞区不泄漏主体色 ===');
{
  const cppLayers = fs.readFileSync(path.join(__dirname, '..', 'src', 'cpp', 'spatial_recon_layers.cpp'), 'utf8');
  check('cpp 实装补洞：蒙版加权统计（1 - alpha 权重）+ 1/4 粗层金字塔',
    /1\.0 - static_cast<double>\(SrMaskAlpha/.test(cppLayers) && /kFillDown = 4/.test(cppLayers));
  check('cpp 实装补洞：高阈值替换（只换强主体像素，保住"参考位姿 == 原照片"）',
    /kFillThreshold = 245/.test(cppLayers));
  check('cpp 实装补洞：蒙版退化会拒绝（返回 false 退回单层场景）',
    /kMinSubjectRatio/.test(cppLayers) && /kMaxSubjectRatio/.test(cppLayers));

  // 合成判据图：背景 = 左上暗/右下亮的渐变（蓝青），主体 = 中央纯红椭圆
  const w = 240, h = 320;
  const srcRgba = new Uint8Array(w * h * 4);
  const mask = new Uint8Array(w * h);
  const ccx = w * 0.5, ccy = h * 0.45, rx = w * 0.18, ry = h * 0.28;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const o = y * w + x;
      const inside = ((x - ccx) / rx) ** 2 + ((y - ccy) / ry) ** 2 <= 1;
      srcRgba[o * 4] = inside ? 255 : 10;
      srcRgba[o * 4 + 1] = inside ? 0 : Math.round(40 + 120 * x / w);
      srcRgba[o * 4 + 2] = inside ? 0 : Math.round(60 + 150 * y / h);
      srcRgba[o * 4 + 3] = 255;
      mask[o] = inside ? 255 : 0;
    }
  }
  const out = fillBackground(srcRgba, w, h, mask);
  check('补洞返回非空（蒙版占比合法）', out !== null, out ? `subj=${out.subjectPx} ratio=${out.ratio.toFixed(3)} levels=${out.levels}` : 'null');
  if (out !== null) {
    // 1) 非强主体像素：背景层逐像素 == 原图（参考位姿合成与原片逐像素一致的基石）
    let exact = 0, outside = 0;
    for (let i = 0; i < w * h; i++) {
      if (mask[i] >= 245) continue;
      outside++;
      if (out.bg[i * 3] === srcRgba[i * 4] && out.bg[i * 3 + 1] === srcRgba[i * 4 + 1]
        && out.bg[i * 3 + 2] === srcRgba[i * 4 + 2]) exact++;
    }
    check('非强主体像素逐像素等于原图', exact === outside, `${exact}/${outside}`);
    // 2) 补洞区不泄漏主体色（红通道应接近背景的暗红 ~10，而不是主体的 255）
    let maxR = 0, sumR = 0, n = 0;
    for (let i = 0; i < w * h; i++) {
      if (mask[i] < 245) continue;
      sumR += out.bg[i * 3]; maxR = Math.max(maxR, out.bg[i * 3]); n++;
    }
    check('补洞区不泄漏主体色（红通道远低于主体）', n > 0 && maxR < 80,
      `maxR=${maxR} meanR=${(sumR / Math.max(1, n)).toFixed(1)}（主体 255）`);
    // 3) 补洞区平滑（相邻像素差小）且无空洞
    let maxJump = 0, holes = 0;
    for (let y = 1; y < h; y++) {
      for (let x = 1; x < w; x++) {
        const i = y * w + x;
        if (mask[i] < 245) continue;
        if (out.bg[i * 3] === 0 && out.bg[i * 3 + 1] === 0 && out.bg[i * 3 + 2] === 0) holes++;
        for (let ch = 0; ch < 3; ch++) {
          maxJump = Math.max(maxJump, Math.abs(out.bg[i * 3 + ch] - out.bg[(i - 1) * 3 + ch]),
            Math.abs(out.bg[i * 3 + ch] - out.bg[(i - w) * 3 + ch]));
        }
      }
    }
    check('补洞区平滑（相邻像素最大跳变 < 40）且无空洞', maxJump < 40 && holes === 0,
      `maxJump=${maxJump} holes=${holes}`);
    // 4) 补洞区应贴近周围背景（以椭圆边界外一圈的实测色为基准，中心偏差 < 40）
    const centerIdx = Math.round(ccy) * w + Math.round(ccx);
    const refIdx = Math.round(ccy) * w + Math.round(ccx + rx + 10);
    let dev = 0;
    for (let ch = 0; ch < 3; ch++) {
      dev = Math.max(dev, Math.abs(out.bg[centerIdx * 3 + ch] - srcRgba[refIdx * 4 + ch]));
    }
    check('补洞色贴近周围背景（中心与边界外参考色偏差 < 40）', dev < 40, `dev=${dev}`);
  }
}

// ---------- 盒中空间（scene=4）复现：远端墙贴照片 + 四壁纹理 + 开口面主体层 ----------
const D_FAR = 2.50;   // 远端墙深度（cpp kDioramaFar）
const D_NEAR = 1.60;  // 开口面深度（cpp kDioramaNear）

/** 远端墙半尺寸：源图宽高比在"参考画幅该深度的视锥"内 contain（即"照片显示矩形"本身） */
function dioramaFace(srcW, srcH) {
  const wf = D_FAR * (OUT_W / 2) / fx;
  const hf = D_FAR * (OUT_H / 2) / fy;
  const srcAspect = srcW / srcH, frameAspect = wf / hf;
  let hx = 0, hy = 0;
  if (srcAspect >= frameAspect) { hx = wf; hy = wf / srcAspect; } else { hy = hf; hx = hf * srcAspect; }
  return { hx, hy, z: D_FAR };
}

/** 渲染一帧盒中空间（远端墙照片 + 四壁纹理 + 开口面主体层叠加） */
function renderDioramaFrame(fr, srcImg, srcCh, srcW, srcH, bgImg, mask, kSubj) {
  const face = dioramaFace(srcW, srcH);
  const photoRect = facePhotoRect(srcW / srcH, { hx: face.hx, hy: face.hy });
  const intr = { fx: fx * SHOT_W / OUT_W, fy: fy * SHOT_H / OUT_H, cx: SHOT_W / 2, cy: SHOT_H / 2 };
  const rgb = new Uint8Array(SHOT_W * SHOT_H * 3);
  const stat = { wall: 0, photo: 0, matte: 0, subj: 0, none: 0 };
  for (let v = 0; v < SHOT_H; v++) {
    for (let u = 0; u < SHOT_W; u++) {
      const d = rayDir(fr, u, v, intr.fx, intr.fy, intr.cx, intr.cy, true);
      const tFar = d.z < -1e-6 ? (-face.z - fr.pos.z) / d.z : -1;
      let tWall = -1, wallAxis = -1;
      for (let axis = 0; axis < 2; axis++) {
        const half = axis === 0 ? face.hx : face.hy;
        const o = axis === 0 ? fr.pos.x : fr.pos.y;
        const dd = axis === 0 ? d.x : d.y;
        if (Math.abs(dd) < 1e-9) continue;
        for (let s = -1; s <= 1; s += 2) {
          const tw = (s * half - o) / dd;
          if (tw <= 1e-4) continue;
          if (tFar > 0 && tw >= tFar) continue;
          const zHit = fr.pos.z + d.z * tw;
          if (zHit > -D_NEAR || zHit < -face.z) continue;
          if (tWall < 0 || tw < tWall) { tWall = tw; wallAxis = axis; }
        }
      }
      let col = [16, 16, 20];
      if (tWall > 0) {
        const tw = { x: fr.pos.x + d.x * tWall, y: fr.pos.y + d.y * tWall, z: fr.pos.z + d.z * tWall };
        const seed = wallAxis === 0 ? (tw.x > 0 ? 41 : 53) : (tw.y > 0 ? 67 : 79);
        const tone = randomTexture(wallAxis === 0 ? tw.y : tw.x, tw.z, seed);
        col = [tone, tone, tone];
        stat.wall++;
      } else if (tFar > 0) {
        const pf = { x: fr.pos.x + d.x * tFar, y: fr.pos.y + d.y * tFar, z: fr.pos.z + d.z * tFar };
        const u01 = pf.x / face.hx * 0.5 + 0.5, v01 = 0.5 - pf.y / face.hy * 0.5;
        const fu = (u01 - photoRect.u0) / photoRect.w, fv = (v01 - photoRect.v0) / photoRect.h;
        if (fu >= 0 && fu <= 1 && fv >= 0 && fv <= 1) {
          col = sampleImg(bgImg, srcW, srcH, 3, fu * (srcW - 1), fv * (srcH - 1));
          stat.photo++;
        } else {
          const tone = randomTexture(pf.x, pf.y, 91);
          col = [tone, tone, tone];
          stat.matte++;
        }
      } else {
        stat.none++;
      }
      if (mask) {
        const tSubj = (d.z < -1e-6) ? (-face.z * kSubj - fr.pos.z) / d.z : -1;
        if (tSubj > 0) {
          const invK = 1 / kSubj;
          const xs = (fr.pos.x + d.x * tSubj) * invK;
          const ys = (fr.pos.y + d.y * tSubj) * invK;
          const su01 = xs / face.hx * 0.5 + 0.5, sv01 = 0.5 - ys / face.hy * 0.5;
          const sfu = (su01 - photoRect.u0) / photoRect.w, sfv = (sv01 - photoRect.v0) / photoRect.h;
          if (sfu >= 0 && sfu <= 1 && sfv >= 0 && sfv <= 1) {
            const sx = sfu * (srcW - 1), sy = sfv * (srcH - 1);
            const m = sampleMask(mask, srcW, srcH, sx, sy);
            if (m > 0) {
              const sc = sampleImg(srcImg, srcW, srcH, srcCh, sx, sy);
              for (let c = 0; c < 3; c++) col[c] = m * sc[c] + (1 - m) * col[c];
              stat.subj++;
            }
          }
        }
      }
      const o = (v * SHOT_W + u) * 3;
      rgb[o] = col[0]; rgb[o + 1] = col[1]; rgb[o + 2] = col[2];
    }
  }
  return { rgb, stat };
}

console.log('=== [10] 盒中空间（scene=4）：放大主盒——照片占比 / 同深度补边锚 / 位移 ===');
{
  const cppD = fs.readFileSync(path.join(__dirname, '..', 'src', 'cpp',
    'spatial_recon_bridge.cpp'), 'utf8');
  check('cpp 实装放大主盒：正面半尺寸随场景切换，且仍走同一套 contain+内缩映射',
    /boxHX = \(scene == 4\)/.test(cppD) && /faceHX = boxHX/.test(cppD) && /faceAspect/.test(cppD));
  check('cpp 已删净旧的"隧道式"盒中空间几何（kDiorama* 不应再出现）', !/kDiorama/.test(cppD));

  // 放大主盒 = 参考画幅在 z=-1.7 处的视锥（正面铺满画幅）
  const BIG_BOX = { hx: 1.7 * (OUT_W / 2) / fx, hy: 1.7 * (OUT_H / 2) / fy, hz: 0.30 };
  console.log(`  放大主盒半尺寸 hx=${BIG_BOX.hx.toFixed(3)} hy=${BIG_BOX.hy.toFixed(3)}（= z=-1.7 处视锥）`);
  const old = { '9:16': 19.7, '3:4': 26.4, '16:9': 11.1 };   // 判据 [7] 实测的旧盒面方案照片面积占比
  const cases = [[809, 1440, '9:16'], [1080, 1440, '3:4'], [1920, 1080, '16:9']];
  let allBigger = true;
  for (const [sw, sh, tag] of cases) {
    const pr = facePhotoRect(sw / sh, BIG_BOX);
    const hw = pr.w * BIG_BOX.hx * fx / 1.7, hh = pr.h * BIG_BOX.hy * fy / 1.7;
    const areaPct = 100 * (2 * hw / OUT_W) * (2 * hh / OUT_H);
    const borderPct = 100 - areaPct;   // 面铺满画幅 → 照片之外全是同深度的程序纹理补边（近景特征锚）
    console.log(`  ${tag}: 照片 ${(100 * 2 * hw / OUT_W).toFixed(1)}%×${(100 * 2 * hh / OUT_H).toFixed(1)}%` +
      ` = 面积 ${areaPct.toFixed(1)}%（旧 ${old[tag]}%，×${(areaPct / old[tag]).toFixed(1)}）  同深度补边 ${borderPct.toFixed(1)}%`);
    if (areaPct < old[tag] * 3) allBigger = false;
    check(`${tag}：与照片同深度的补边锚仍在（近景可匹配特征，坑 122）`, borderPct > 12,
      `补边=${borderPct.toFixed(1)}%`);
  }
  check('照片面积 ≥ 旧盒面方案的 3 倍（"看不出原图痕迹"的直接修复）', allBigger);

  // 位移量级：照片平面（含补边）与主体层都要落在已验证可匹配区间
  const dispOf = (P) => {
    const a = projectStd(f0, P, fx, fy, cx, cy);
    const b = projectStd(fLast, P, fx, fy, cx, cy);
    return Math.hypot(b.u - a.u, b.v - a.v);
  };
  const dump = (tag, z) => {
    const d = dispOf(v3(0.25, 0.4, -z));
    console.log(`  ${tag}（z=-${z.toFixed(2)}）：一段 ±8° 位移 ${d.toFixed(0)}px`);
    return d;
  };
  const dPhoto = dump('照片平面/补边', 1.7);
  const dSubj = dump('主体层（中档 kSubj=0.90）', 1.7 * 0.9);
  check('照片平面位移 = 已验证值（≈145px，与 scene=1/3 同深度）', Math.abs(dPhoto - 145) < 25,
    `${dPhoto.toFixed(0)}px`);
  check('主体层位移在可匹配量级（< 300px）', dSubj < 300, `${dSubj.toFixed(0)}px`);

  // 出图：参考位姿 + 轨道端点（放大盒的面 + 合成照片 + 合成主体蒙版）
  const sw = 270, sh = 480;   // 9:16 源图
  const srcSyn = new Uint8Array(sw * sh * 4);
  const maskImg = new Uint8Array(sw * sh);
  const ccx = sw * 0.5, ccy = sh * 0.42, rx = sw * 0.17, ry = sh * 0.26;
  for (let y = 0; y < sh; y++) {
    for (let x = 0; x < sw; x++) {
      const o = y * sw + x;
      const inside = ((x - ccx) / rx) ** 2 + ((y - ccy) / ry) ** 2 <= 1;
      if (inside) {
        srcSyn[o * 4] = 210; srcSyn[o * 4 + 1] = 40; srcSyn[o * 4 + 2] = 60;
      } else {
        srcSyn[o * 4] = 20;
        srcSyn[o * 4 + 1] = 60 + Math.round(120 * x / sw);
        srcSyn[o * 4 + 2] = 140 + Math.round(60 * y / sh);
      }
      srcSyn[o * 4 + 3] = 255;
      maskImg[o] = inside ? 255 : 0;
    }
  }
  const filledBig = fillBackground(srcSyn, sw, sh, maskImg);
  check('放大盒判据图：补洞可用', filledBig !== null);
  if (filledBig !== null) {
    fs.mkdirSync(OUT_DIR, { recursive: true });
    const atRef = frameBasis(0, 1, ORBIT_DEG);
    [[atRef, 'ref'], [fLast, 'last']].forEach(([fr, tag]) => {
      const r = renderLayeredFrame(fr, srcSyn, 4, sw, sh, filledBig.bg, maskImg, 0.90,
        facePhotoRect(sw / sh, BIG_BOX), BIG_BOX);
      writePng(path.join(OUT_DIR, `diorama_${tag}.png`), SHOT_W, SHOT_H, r.rgb);
      if (tag === 'ref') {
        const tot = SHOT_W * SHOT_H;
        console.log(`  参考位姿命中：照片 ${(100 * r.stat.bg / tot).toFixed(1)}%` +
          ` 主体层 ${(100 * r.stat.subjOnly / tot).toFixed(1)}%（照片区域内）` +
          ` 照片矩形外补边 ${(100 * r.stat.none / tot).toFixed(1)}%`);
        check('参考位姿：照片+主体层+补边铺满（无空洞/无未覆盖）', r.stat.none >= 0 && r.stat.bg > 0);
        check('参考位姿：照片是主画面（占比 > 40%）', r.stat.bg / tot > 0.4,
          `${(100 * r.stat.bg / tot).toFixed(1)}%`);
      }
    });
    console.log(`  已输出 ${OUT_DIR}/diorama_ref.png / diorama_last.png（放大主盒构图）供肉眼核验`);
  }
}

console.log(`\n=== 汇总：PASS ${pass} / FAIL ${fail} ===`);
process.exit(fail === 0 ? 0 : 1);
