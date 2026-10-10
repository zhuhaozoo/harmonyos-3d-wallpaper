/**
 * 地球预置 · 模型几何自检（PLY 侧车 → **日志读数**）
 *
 * 为什么要在端上算、只打日志：
 *  - 模型几何是"黑缺口/球偏大/纵横比"三类可见缺陷的**唯一**判据来源（渲染只是它的投影）；
 *  - 而模型侧唯一可解剖产物（Kit 写的 PLY 侧车）位于应用沙箱：`hdc file recv` 与
 *    `hdc shell ls` 对该目录一律 permission denied（真机实测），
 *    即"把 PLY 拷到电脑再看"这条路在 CLI 下不通（只能走 IDE 的设备文件浏览器）。
 *  - 因此把**关键统计量直接打进日志**：`hdc shell hilog -x | grep SR3D plydiag` 即可取回，
 *    零文件外带、零手工步骤、每轮自动产出。
 *
 * 读数设计（一次生成 ≈ 12+2 行日志）：
 *  - `R*`：半径直方图在 (0.3,2.5) 内的峰位 = 模型里"球壳"的半径（模型单位）。
 *    与日志里 `SR3D glmmeta initialCamera pos=` 的 |pos| 相除即 **camR/R***——
 *    声明值 = manifest.cameraRadius，偏小多少 = 球看起来大多少。
 *  - `shell`：壳层点（0.8~1.2·R*）的协方差三轴尺度（纵横比直接可读）+ 平均不透明度。
 *  - `cover`（24×12 经纬格，lon 列 × lat 行）：每格的点数分档（'.'=0 空 / 1-9 对数分档）
 *    ⇒ "黑洞"是**缺几何**（空档）还是**黑点挡在前**（有档但暗）一眼可判。
 *  - `lum`：同格的 SH 直流项亮度数字（0-9）⇒ 黑点挡前时该格数字极小。
 *
 * 口径：与 tools/depth_mesh/earth_scene.py 的等距柱状约定一致（u = lon/360+0.5，
 * v = 0.5−lat/π），便于与输入纹理逐格对照（离线脚本 tools/depth_mesh/earth_ply_probe.py 同口径）。
 *
 * 只读 + 只打日志：任何异常都不抛出、不阻断流程；采样上限 kMaxPoints 保证耗时可控。
 */
import { fileIo } from '@kit.CoreFileKit';
import { util } from '@kit.ArkTS';

const TAG: string = 'SR3D plydiag';
/** 采样上限（点数超过则等步长抽样；统计量对抽样稳健） */
const kMaxPoints: number = 240000;
/** 记录缓冲（按整记录读；每块 ≈ 512 条记录，随 stride 自适应） */
const kChunkRecords: number = 512;
/** 经纬格分辨率（lon 24 列 × lat 12 行 = 15°×15°） */
const kLonCells: number = 24;
const kLatCells: number = 12;
/** 壳层判定带宽（相对 R*） */
const kShellLo: number = 0.80;
const kShellHi: number = 1.20;
/** 半径直方图（0~4，步长 0.05） */
const kRadBinW: number = 0.05;
const kRadBins: number = 80;

class PlyLayout {
  count: number = 0;
  stride: number = 0;     // 每条记录字节数
  dataOff: number = 0;    // 顶点数据起点
  ox: number = -1;
  oy: number = -1;
  oz: number = -1;
  oOp: number = -1;       // opacity（logit 域，端侧只统计不解释）
  oDc0: number = -1;      // f_dc_0..2（SH 直流项 = 颜色）
  oDc1: number = -1;
  oDc2: number = -1;
  ok: boolean = false;
}

/** 属性名 → 偏移/尺寸（float=4 / double=8 / uchar,int8=1 / char=1 / short,ushort=2 / int,uint=4） */
function propSize(t: string): number {
  if (t === 'float' || t === 'int' || t === 'uint') {
    return 4;
  }
  if (t === 'double') {
    return 8;
  }
  if (t === 'uchar' || t === 'char' || t === 'int8' || t === 'uint8') {
    return 1;
  }
  if (t === 'short' || t === 'ushort' || t === 'int16' || t === 'uint16') {
    return 2;
  }
  return 4;
}

/** 解析 PLY 头（binary_little_endian 顶点记录）。头部 ≤ 16KB，一次读完。 */
function parseHeader(fd: number): PlyLayout {
  const lay: PlyLayout = new PlyLayout();
  const head: ArrayBuffer = new ArrayBuffer(16384);
  let hn: number = 0;
  try {
    hn = fileIo.readSync(fd, head, { offset: 0 });
  } catch (e) {
    return lay;
  }
  const u8: Uint8Array = new Uint8Array(head, 0, hn);
  let txt: string = '';
  for (let i = 0; i < hn; i++) {
    txt += String.fromCharCode(u8[i]);
  }
  const lines: string[] = txt.split('\n');
  // ⚠️ 两个偏移必须分开记：
  //   lineOff = 当前行在**文件**里的起始字节（数据区起点 = end_header 行之后）
  //   recOff  = 当前行已贡献的**记录内**字节（stride = 所有属性尺寸之和，≠ 行长度之和）
  // 二者混用会把 stride 记成"属性尺寸 + 行长度"（真机 PLY 实测 68B 会算成 503B）。
  let lineOff: number = 0;
  let recOff: number = 0;
  let inVertex: boolean = false;
  for (let i = 0; i < lines.length; i++) {
    const raw: string = lines[i];
    const ln: string = raw.trim();
    if (ln.length === 0) {
      lineOff += raw.length + 1;        // ⚠️ 用**原始**行长度（含 \r），否则数据偏移整体漂移
      continue;
    }
    if (ln.startsWith('format') && ln.indexOf('binary_little_endian') < 0) {
      return lay;                       // 只支持 binary_little_endian
    }
    if (ln.startsWith('element ')) {
      inVertex = ln.startsWith('element vertex ');
      if (inVertex) {
        lay.count = parseInt(ln.substring('element vertex '.length).trim(), 10);
      }
    } else if (inVertex && ln.startsWith('property ')) {
      // 逐词切（ArkTS 侧不用正则；属性行 "property <type> <name> [<n>]"）
      const p1: number = ln.indexOf(' ', 9);
      if (p1 > 0) {
        const type: string = ln.substring(9, p1);
        const rest: string = ln.substring(p1 + 1);
        const p2: number = rest.indexOf(' ');
        const name: string = p2 > 0 ? rest.substring(0, p2) : rest;   // 行尾属性无后随空格
        const sz: number = propSize(type);
        if (name === 'x') {
          lay.ox = recOff;
        } else if (name === 'y') {
          lay.oy = recOff;
        } else if (name === 'z') {
          lay.oz = recOff;
        } else if (name === 'opacity') {
          lay.oOp = recOff;
        } else if (name === 'f_dc_0') {
          lay.oDc0 = recOff;
        } else if (name === 'f_dc_1') {
          lay.oDc1 = recOff;
        } else if (name === 'f_dc_2') {
          lay.oDc2 = recOff;
        }
        recOff += sz;
      }
    }
    if (ln === 'end_header') {
      lay.dataOff = lineOff + raw.length + 1;
      break;
    }
    lineOff += raw.length + 1;
  }
  lay.stride = recOff;
  lay.ok = lay.count > 0 && lay.stride > 0 && lay.ox >= 0 && lay.oy >= 0 && lay.oz >= 0;
  return lay;
}

/** 读第 i 条记录的 x/y/z（+ 可选 opacity / f_dc）到 out[0..5]；越界返回 false */
function readVec(dv: DataView, base: number, lay: PlyLayout, out: number[]): boolean {
  if (base + lay.stride > dv.byteLength) {
    return false;
  }
  out[0] = dv.getFloat32(base + lay.ox, true);
  out[1] = dv.getFloat32(base + lay.oy, true);
  out[2] = dv.getFloat32(base + lay.oz, true);
  out[3] = lay.oOp >= 0 ? dv.getFloat32(base + lay.oOp, true) : 0;
  out[4] = lay.oDc0 >= 0 ? dv.getFloat32(base + lay.oDc0, true) : 0;
  out[5] = lay.oDc1 >= 0 ? dv.getFloat32(base + lay.oDc1, true) : 0;
  out[6] = lay.oDc2 >= 0 ? dv.getFloat32(base + lay.oDc2, true) : 0;
  return true;
}

/** 扫一遍（等步长抽样）→ 半径直方图；返回直方图与抽样点数 */
function radialPass(fd: number, lay: PlyLayout, stride: number): number[] {
  const hist: number[] = new Array<number>(kRadBins + 1).fill(0);
  const buf: ArrayBuffer = new ArrayBuffer(kChunkRecords * lay.stride);
  const dv: DataView = new DataView(buf);
  const recsPerChunk: number = kChunkRecords;
  const v: number[] = new Array<number>(7).fill(0);
  let idx: number = 0;
  while (idx < lay.count) {
    const want: number = Math.min(recsPerChunk, lay.count - idx);
    let got: number = 0;
    try {
      got = fileIo.readSync(fd, buf, { offset: lay.dataOff + idx * lay.stride, length: want * lay.stride });
    } catch (e) {
      break;
    }
    const have: number = Math.floor(got / lay.stride);
    if (have <= 0) {
      break;
    }
    for (let r = 0; r < have; r++) {
      if (idx % stride !== 0) {
        idx++;
        continue;
      }
      idx++;
      if (!readVec(dv, r * lay.stride, lay, v)) {
        continue;
      }
      const rr: number = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
      const b: number = Math.floor(rr / kRadBinW);
      if (b >= 0 && b < kRadBins) {
        hist[b] += 1;
      } else if (b >= kRadBins) {
        hist[kRadBins] += 1;           // ≥4 单独一档（远景板/异常点）
      }
    }
  }
  return hist;
}

/** 壳层统计（点数/协方差各轴标准差/平均不透明度/经纬格点数与亮度） */
class ShellStats {
  n: number = 0;
  nShell: number = 0;
  opSum: number = 0;
  sxx: number = 0;
  syy: number = 0;
  szz: number = 0;
  cnt: number[] = new Array<number>(kLonCells * kLatCells).fill(0);
  lumSum: number[] = new Array<number>(kLonCells * kLatCells).fill(0);
}

function shellPass(fd: number, lay: PlyLayout, stride: number, rStar: number): ShellStats {
  const st: ShellStats = new ShellStats();
  const buf: ArrayBuffer = new ArrayBuffer(kChunkRecords * lay.stride);
  const dv: DataView = new DataView(buf);
  const v: number[] = new Array<number>(7).fill(0);
  const lo: number = kShellLo * rStar;
  const hi: number = kShellHi * rStar;
  let idx: number = 0;
  while (idx < lay.count) {
    const want: number = Math.min(kChunkRecords, lay.count - idx);
    let got: number = 0;
    try {
      got = fileIo.readSync(fd, buf, { offset: lay.dataOff + idx * lay.stride, length: want * lay.stride });
    } catch (e) {
      break;
    }
    const have: number = Math.floor(got / lay.stride);
    if (have <= 0) {
      break;
    }
    for (let r = 0; r < have; r++) {
      if (idx % stride !== 0) {
        idx++;
        continue;
      }
      idx++;
      if (!readVec(dv, r * lay.stride, lay, v)) {
        continue;
      }
      const rr: number = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
      if (rr <= 1e-6) {
        continue;
      }
      // 经纬格（与生成器同口径）：lon = atan2(x,z)，lat = asin(y/r)
      const lon: number = Math.atan2(v[0], v[2]);
      const lat: number = Math.asin(Math.max(-1, Math.min(1, v[1] / rr)));
      const cLon: number = Math.min(kLonCells - 1,
        Math.floor((lon / (2 * Math.PI) + 0.5) * kLonCells));
      const cLat: number = Math.min(kLatCells - 1,
        Math.floor((0.5 - lat / Math.PI) * kLatCells));
      const ci: number = cLat * kLonCells + cLon;
      st.cnt[ci] += 1;
      // SH 直流 → 亮度（C0 = 0.2820948；只做相对分档）
      const lum: number = 255 * (0.5 + 0.2820948 * (v[4] + v[5] + v[6]) / 3);
      st.lumSum[ci] += Math.max(0, Math.min(255, lum));
      if (rr >= lo && rr <= hi) {
        st.nShell += 1;
        st.sxx += v[0] * v[0];
        st.syy += v[1] * v[1];
        st.szz += v[2] * v[2];
        st.opSum += v[3];
      }
      st.n += 1;
    }
  }
  return st;
}

/** 打印统计（日志行数固定：壳层 3 行 + 两张 12 行地图 + 汇总 1 行 ≈ 28 行） */
function report(path: string, lay: PlyLayout, hist: number[], rStar: number,
                st: ShellStats, sampled: number): void {
  let tot: number = 0;
  for (let i = 0; i <= kRadBins; i++) {
    tot += hist[i];
  }
  // 半径直方图峰位附近（±3 档）的占比 → 壳层集中度
  let peak: number = 0;
  for (let i = 1; i < kRadBins; i++) {
    if (hist[i] > hist[peak]) {
      peak = i;
    }
  }
  let peakBand: number = 0;
  for (let i = Math.max(0, peak - 3); i <= Math.min(kRadBins, peak + 3); i++) {
    peakBand += hist[i];
  }
  const rPeak: number = (peak + 0.5) * kRadBinW;
  console.info(`${TAG} file=${path.substring(path.lastIndexOf('/') + 1)} n=${lay.count}`
    + ` sampled=${sampled} stride=${strideOf(lay.count)} dataOff=${lay.dataOff}`
    + ` props(${lay.stride}B) x=${lay.ox} y=${lay.oy} z=${lay.oz} dc=${lay.oDc0}`);
  console.info(`${TAG} R*=histPeak=${rPeak.toFixed(3)} (peakBand±3=${(peakBand / Math.max(tot, 1) * 100).toFixed(1)}%`
    + ` of ${tot})  shellN=${st.nShell} of inside=${st.n}`
    + ` rStarFromMedian=${rStar.toFixed(3)}`);
  if (st.nShell > 16) {
    const sx: number = Math.sqrt(st.sxx / st.nShell);
    const sy: number = Math.sqrt(st.syy / st.nShell);
    const sz: number = Math.sqrt(st.szz / st.nShell);
    console.info(`${TAG} shell axisStd x=${sx.toFixed(3)} y=${sy.toFixed(3)} z=${sz.toFixed(3)}`
      + `  x/y=${(sx / Math.max(sy, 1e-9)).toFixed(3)} x/z=${(sx / Math.max(sz, 1e-9)).toFixed(3)}`
      + `  opacityMean(logit)=${(st.opSum / st.nShell).toFixed(3)}`);
  }
  // 经纬格：覆盖（点数对数分档）与亮度（0-9）两张地图，lon 从左到右 = -180°→+180°
  let emptyCells: number = 0;
  let darkCells: number = 0;
  const cover: string[] = [];
  const lum: string[] = [];
  for (let r = 0; r < kLatCells; r++) {
    let crow: string = '';
    let lrow: string = '';
    for (let c = 0; c < kLonCells; c++) {
      const i: number = r * kLonCells + c;
      const k: number = st.cnt[i];
      if (k === 0) {
        crow += '.';
        lrow += '.';
        emptyCells += 1;
        continue;
      }
      // 1-9：对数分档（1,2,4,8,...）
      let bucket: number = 1;
      let thr: number = 1;
      while (k > thr && bucket < 9) {
        thr *= 4;
        bucket += 1;
      }
      crow += String.fromCharCode(48 + bucket);
      const mLum: number = st.lumSum[i] / k;
      const dl: number = Math.max(0, Math.min(9, Math.round(mLum / 26)));
      lrow += String.fromCharCode(48 + dl);
      if (mLum < 20) {
        darkCells += 1;
      }
    }
    cover.push(crow);
    lum.push(lrow);
  }
  console.info(`${TAG} cover 24x12 (lon -180→180, lat +90→-90; '.'=0 pts, 1-9=log count):`);
  for (let r = 0; r < kLatCells; r++) {
    console.info(`${TAG}  |${cover[r]}|`);
  }
  console.info(`${TAG} lum 24x12 (0-9 = mean SH-DC luminance / 26; '.'=no pts):`);
  for (let r = 0; r < kLatCells; r++) {
    console.info(`${TAG}  |${lum[r]}|`);
  }
  console.info(`${TAG} verdict: emptyCells=${emptyCells}/${kLonCells * kLatCells}`
    + ` darkCells(lum<20)=${darkCells} —— 空档多=缺几何；有档但全暗=黑点挡前`);
}

function strideOf(count: number): number {
  return count > kMaxPoints ? Math.ceil(count / kMaxPoints) : 1;
}

/** 对外入口：给一个 PLY 路径，打一组 `SR3D plydiag` 日志。失败静默（只告警）。 */
export class EarthPlyReport {
  static run(plyPath: string): void {
    if (plyPath.length === 0) {
      return;
    }
    let fd: number = -1;
    try {
      if (!fileIo.accessSync(plyPath)) {
        console.info(`${TAG} missing (no sidecar) ${plyPath.substring(plyPath.lastIndexOf('/') + 1)}`);
        return;
      }
      fd = fileIo.openSync(plyPath, fileIo.OpenMode.READ_ONLY).fd;
      const lay: PlyLayout = parseHeader(fd);
      if (!lay.ok) {
        console.warn(`${TAG} header unsupported count=${lay.count} stride=${lay.stride} ox=${lay.ox}`);
        return;
      }
      const hist: number[] = radialPass(fd, lay, strideOf(lay.count));
      // R*：直方图在 (0.3,2.5) 内的峰位（球壳在半径处成峰）
      let peak: number = 0;
      for (let i = Math.floor(0.3 / kRadBinW); i < Math.min(kRadBins, Math.floor(2.5 / kRadBinW)); i++) {
        if (hist[i] > hist[peak]) {
          peak = i;
        }
      }
      const rStar: number = (peak + 0.5) * kRadBinW;
      if (!(rStar > 0.05)) {
        console.warn(`${TAG} no sphere peak (hist peak=${rStar})`);
        return;
      }
      const st: ShellStats = shellPass(fd, lay, strideOf(lay.count), rStar);
      let sampled: number = 0;
      for (let i = 0; i <= kRadBins; i++) {
        sampled += hist[i];
      }
      report(plyPath, lay, hist, rStar, st, sampled);
    } catch (err) {
      console.warn(`${TAG} skip ${(err as Error).message || String(err)}`);
    } finally {
      if (fd >= 0) {
        try {
          fileIo.closeSync(fd);
        } catch (e) {
          // 关闭失败不影响主流程
        }
      }
    }
  }
}
