// 地球预置建模管线判据（预置数据集机器判据 + 五层源码守卫）
//
// 背景：预置 3D 壁纸"地球"走 preset 管线——离线合成数据集（tools/depth_mesh/earth_scene.py
// 产出：帧序列 + manifest.json，帧与位姿声明**同源同参数**）→ ArkTS 逐帧解码灌入 native
// 预置帧队列（背压退让，不冻结 UI）→ bridge 按 manifest 声明的位姿域给出环形基位姿 →
// 既有推帧/交换/门控/销毁全链 → 端侧 3DGS 重建 → MP4。
//
// 本判据只做**源码/静态文件层**守卫（不跑真机、不构建）：帧尺寸用 JPEG SOF 头直接解析，
// 精确到像素；位姿域要求 manifest ↔ ArkTS 读取端 ↔ native 公式域三处配对（防"改了数据
// 没改代码"的静默错配——历史上出现过"代码回退但包内仍是新数据"的埋雷组合）。
//
// 用法：
//   node tools/earth_preset_check.js                     # 源码守卫 + 数据集判据（数据集缺席时跳过）
//   node tools/earth_preset_check.js --data=<数据集目录> # 指定数据集目录（含 manifest.json）
// 数据集目录默认取 tools/depth_mesh/out/earth_africa（earth_scene.py 的典型输出位置）。
'use strict';
const fs = require('fs');
const path = require('path');

const ROOT = path.resolve(__dirname, '..');
let pass = 0;
let fail = 0;
let skip = 0;
function check(name, ok, extra) {
  if (ok) {
    pass++;
    console.log(`PASS  ${name}`);
  } else {
    fail++;
    console.log(`FAIL  ${name}${extra ? '  -- ' + extra : ''}`);
  }
}
function skipped(name, why) {
  skip++;
  console.log(`SKIP  ${name}  -- ${why}`);
}
const read = (p) => fs.readFileSync(path.join(ROOT, p), 'utf8');

const args = {};
for (const a of process.argv.slice(2)) {
  const m = a.match(/^--([\w-]+)=(.*)$/);
  if (m) {
    args[m[1]] = m[2];
  }
}
const DATA_DIR = args.data
  ? path.resolve(ROOT, args.data)
  : path.join(ROOT, 'tools', 'depth_mesh', 'out', 'earth_africa');

// ---------- [1] 数据集（manifest + 帧尺寸 SOF 判据） ----------
console.log('=== [1] 数据集（manifest + 帧尺寸 SOF 判据） ===');
const DATA_PRESENT = fs.existsSync(path.join(DATA_DIR, 'manifest.json'));
if (!DATA_PRESENT) {
  skipped('数据集判据', `未找到 ${path.relative(ROOT, DATA_DIR)}/manifest.json`
    + '（本仓库不分发数据集；用 tools/depth_mesh/earth_scene.py 生成后 --data=<目录> 指过来）');
} else {
  let man = null;
  try {
    man = JSON.parse(fs.readFileSync(path.join(DATA_DIR, 'manifest.json'), 'utf8'));
  } catch (e) {
    man = null;
  }
  check('manifest.json 可解析且 count = frames.length',
    !!man && man.count === man.frames.length,
    man ? `count=${man.count} frames=${man.frames.length}` : 'unreadable');
  check('汇总字段自洽（cameraRadius > 1、focalPx > 0、frameW/H = 1080×1440）',
    !!man && man.cameraRadius > 1 && man.focalPx > 0 && man.frameW === 1080 && man.frameH === 1440,
    man ? `cameraRadius=${man.cameraRadius} focalPx=${man.focalPx} ${man.frameW}x${man.frameH}` : 'unreadable');
  // 近距域（jpeg SOF 头部尺寸逐帧核对——防"尺寸改了只改一处"）
  if (man && Array.isArray(man.frames) && man.frames.length > 0) {
    let bad = 0;
    let checked = 0;
    for (const fr of man.frames) {
      const fp = path.join(DATA_DIR, fr.file);
      if (!fs.existsSync(fp)) {
        continue;
      }
      checked++;
      const buf = fs.readFileSync(fp);
      // SOF0/SOF2 段：FF C0/C2, len, precision, H(2), W(2)
      let w = 0;
      let h = 0;
      let i = 2;
      while (i + 9 < buf.length) {
        if (buf[i] !== 0xFF) {
          i++;
          continue;
        }
        const marker = buf[i + 1];
        if (marker === 0xC0 || marker === 0xC2) {
          h = buf.readUInt16BE(i + 5);
          w = buf.readUInt16BE(i + 7);
          break;
        }
        if (marker === 0xD8 || (marker >= 0xD0 && marker <= 0xD9)) {
          i += 2;
          continue;
        }
        const len = buf.readUInt16BE(i + 2);
        i += 2 + len;
      }
      if (w !== man.frameW || h !== man.frameH) {
        bad++;
      }
    }
    if (checked > 0) {
      check('全部帧的 JPEG SOF 尺寸 = manifest 声明（1080×1440，逐帧机器核对）',
        bad === 0, `checked=${checked} bad=${bad}`);
    } else {
      skipped('帧尺寸 SOF 判据', '本目录只有 manifest、无帧文件（数据集可能只留档了 manifest）');
    }
  }
  check('数据集：位姿域为受支持域（diag-arc 推荐 / flat-pitch 历史域）',
    !!man && (man.poseDomain === 'diag-arc' || typeof man.camPitchDeg === 'number'
      || Array.isArray(man.camAxis)),
    man ? `poseDomain=${man.poseDomain}` : 'unreadable');
  check('数据集：内参声明诚实各向同性（focalYRatio = 1.0 或缺省；非 1.0 的"声明偏置"实测崩）',
    !!man && (man.focalYRatio == null || man.focalYRatio === 1.0),
    man ? `focalYRatio=${man.focalYRatio}` : 'unreadable');
  check('数据集：背景为球后远景板（bgMode=plate，|z| > 1；天空球包住相机会致空白帧）',
    !!man && !!man.source && man.source.bgMode === 'plate'
      && typeof man.source.bgPlaneZ === 'number' && man.source.bgPlaneZ < -1.0,
    man && man.source ? `bgMode=${man.source.bgMode} z=${man.source.bgPlaneZ}` : 'unreadable');
  check('数据集：背景含可匹配的非对称结构（星点或软斑 > 0；纯黑背景重建质量被证伪）',
    !!man && !!man.source && !!man.source.bgParams
      && ((man.source.bgParams.star_count || 0) > 0 || (man.source.bgParams.nebula_count || 0) > 0),
    man && man.source && man.source.bgParams
      ? `star=${man.source.bgParams.star_count} nebula=${man.source.bgParams.nebula_count}`
      : 'unreadable');
  // 帧 ↔ manifest 采样的自洽（θ 无重复；含"参考位姿帧"——等距抽样下离 0 最近的一帧）
  if (man && Array.isArray(man.frames) && man.frames.length > 1) {
    const th = man.frames.map((f) => f.thetaDeg);
    let mono = true;
    for (let i = 1; i < th.length; i++) {
      if (Math.abs(th[i] - th[i - 1]) < 1e-9) {
        mono = false;
      }
    }
    check('数据集：帧角度序列无重复值（等距抽样前提）', mono);
    const minAbs = Math.min(...th.map((v) => Math.abs(v)));
    const span = Math.abs(th[th.length - 1] - th[0]);
    const stepDeg = span / (th.length - 1);
    check('数据集：含参考位姿帧（离 θ=0 最近的帧 ≤ 步距/2——参考位姿帧恒等是照片保真的硬约束）',
      minAbs <= Math.max(stepDeg / 2 + 1e-6, 1e-9),
      `frames=${th.length} θ范围=${th[0]}..${th[th.length - 1]} 最近 |θ|=${minAbs.toFixed(4)} 步距=${stepDeg.toFixed(4)}`);
  }
}

// ---------- [2] ArkTS 读取端（Service）：preset 五层接线 ----------
console.log('=== [2] ArkTS 读取端：SpatialReconService（preset 接线） ===');
{
  const svc = read('src/ets/SpatialReconService.ets');
  check('Service：声明并导入预置帧队列 API（spatialReconPresetPush / spatialReconPresetFail）',
    /spatialReconPresetPush, spatialReconPresetFail,/.test(svc)
    || (/spatialReconPresetPush/.test(svc) && /spatialReconPresetFail/.test(svc)));
  check('Service：manifest 解析入口（parsePresetManifest；帧清单来自数据集目录）',
    /parsePresetManifest/.test(svc) && /manifest\.json/.test(svc));
  check('Service：等距抽样含首帧（i=0 取 frames[0]，帧数 = 目标档位）',
    /Math\.round\(i \* \(count - 1\) \/ \(want - 1\)\)/.test(svc));
  check('Service：把 manifest 字段透传为 native 参数（preset/presetTheta/presetRadius/focalPx/frameCount）',
    /preset: true,/.test(svc) && /presetTheta: theta,/.test(svc)
    && /presetRadius: man\.cameraRadius,/.test(svc) && /focalPx: man\.focalPx/.test(svc)
    && /frameCount: theta\.length,/.test(svc));
  check('Service：位姿域三选一透传（diag-arc 优先 / camPitchDeg / camAxis 默认 (0,1,0)）',
    /opts\.presetDiagArc = true;/.test(svc) && /opts\.presetPitchDeg = pitchDeg;/.test(svc)
    && /opts\.presetAxis = man\.camAxis \?\? \[0, 1, 0\];/.test(svc));
  check('Service：worldFlip180 由数据集声明（缺省 true = 历史口径）',
    /worldFlip180: man\.worldFlip180 !== false,/.test(svc));
  check('Service：灌帧循环带背压退让（队列满 rc===1 → delay 后重试，不忙等不冻结 UI）',
    /let rc: number = spatialReconPresetPush\(rgba\);/.test(svc)
    && /while \(rc === 1 && !genSettled\)/.test(svc)
    && /await SpatialReconService\.delay\(8\)/.test(svc));
  check('Service：生产端失败放行（spatialReconPresetFail 后回收结算 Promise，再上抛生产错误）',
    /spatialReconPresetFail\(\);/.test(svc) && /await genPromise;/.test(svc));
  check('Service：用户终止（-6）不换档重试（直接上抛）',
    /lastErrMsg\.includes\('-6'\)/.test(svc));
  check('Service：同会话双产物 PLY（alsoPly: true）+ 几何自检日志（EarthPlyReport.run）',
    /alsoPly: true,/.test(svc) && /EarthPlyReport\.run\(plyPath\)/.test(svc)
    && /import \{ EarthPlyReport \} from '\.\/EarthPlyReport';/.test(svc));
  check('Service：端侧不做"谎报内参"偏置（focalYratio 直接透传 manifest，仅钳制在 native 侧）',
    /focalYratio: \(man\.focalYRatio != null && man\.focalYRatio > 0\) \? man\.focalYRatio : 1,/.test(svc));
}

// ---------- [3] native 层（bridge）：预置队列 + 位姿域公式 + 双产物 ----------
console.log('=== [3] native 层：bridge（队列 / 位姿域 / 双产物） ===');
{
  const bH = read('src/cpp/spatial_recon_bridge.h');
  const bCpp = read('src/cpp/spatial_recon_bridge.cpp');
  check('params：preset 模式字段齐备（preset/presetTheta/presetRadius/presetAxis/presetPitchDeg/presetDiagArc）',
    /bool preset = false;/.test(bH) && /std::vector<float> presetTheta;/.test(bH)
    && /float presetRadius = 0\.0f;/.test(bH) && /std::vector<float> presetAxis;/.test(bH)
    && /float presetPitchDeg = kPresetPitchUnused;/.test(bH) && /bool presetDiagArc = false;/.test(bH));
  check('队列 API：Push / Fail / Reset 三件齐（Push 非阻塞，容量 3；Reset 在 napi 入口清队）',
    /int32_t SpatialReconPresetQueuePush\(/.test(bH) && /int32_t SpatialReconPresetQueueFail\(/.test(bH)
    && /void SpatialReconPresetQueueReset\(/.test(bH)
    && /int32_t SpatialReconPresetQueuePush\(/.test(bCpp) && /SpatialReconPresetQueueReset\(/.test(bCpp));
  check('运行时校验：preset 模式帧数上界（240）+ 半径 > 1（相机在球外）',
    /params\.presetTheta\.size\(\) > 240/.test(bCpp) && /!\(params\.presetRadius > 1\.0f\)/.test(bCpp));
  check('位姿域互斥防呆：diag-arc × (pitch/axis) 同传报错；pitch × axis 同传报错',
    /params\.presetDiagArc && \(presetPitchValid \|\| !params\.presetAxis\.empty\(\)\)/.test(bCpp)
    && /presetPitchValid && !params\.presetAxis\.empty\(\)/.test(bCpp));
  check('三套位姿基公式在位（SrPresetDiagArcBasis / SrPresetFlatPitchBasis / SrPresetRingBasis）',
    /static void SrPresetDiagArcBasis\(/.test(bCpp)
    && /static void SrPresetFlatPitchBasis\(/.test(bCpp)
    && /static void SrPresetRingBasis\(/.test(bCpp));
  check('帧循环按域择一取基（diag-arc / flat-pitch / ring）且逐帧角来自 presetTheta',
    /SrPresetDiagArcBasis\(params\.presetTheta\[i\], params\.presetRadius, pos, basis\);/.test(bCpp)
    && /SrPresetFlatPitchBasis\(params\.presetTheta\[i\], params\.presetRadius,/.test(bCpp));
  check('各向异性内参：仅预置路线生效（照片路线恒 1）+ 越界回 1.0 钳制',
    /if \(presetMode\) \{[\s\S]{0,200}?yRatio = params\.focalYratio/.test(bCpp)
    && /0\.3f && yRatio <= 3\.0f/.test(bCpp));
  check('双产物：SaveResultToFile 运行期绑定（缺符号只少 PLY 侧车，不影响 MP4 交付）',
    /FnSaveResultToFile saveResultToFile = nullptr;/.test(bCpp)
    && /SR_DLSYM_OPT\(FnSaveResultToFile, saveResultToFile, SaveResultToFile\);/.test(bCpp));
}

// ---------- [4] 声明层（d.ts）与几何自检模块 ----------
console.log('=== [4] d.ts 声明 + EarthPlyReport ===');
{
  const dts = read('src/ets/libglassrender-3d.d.ts');
  check('d.ts：preset 相关字段与队列 API 齐备',
    /preset\?: boolean;/.test(dts) && /presetTheta\?: number\[\];/.test(dts)
    && /presetRadius\?: number;/.test(dts) && /presetAxis\?: number\[\];/.test(dts)
    && /presetPitchDeg\?: number;/.test(dts) && /presetDiagArc\?: boolean;/.test(dts)
    && /export declare function spatialReconPresetPush\(rgbaBuffer: ArrayBuffer\): number;/.test(dts)
    && /export declare function spatialReconPresetFail\(\): number;/.test(dts));
  const ply = read('src/ets/EarthPlyReport.ets');
  check('EarthPlyReport：只读只打日志（run 静态入口 + 不外抛）',
    /export class EarthPlyReport/.test(ply) && /static run\(/.test(ply));
  check('EarthPlyReport：读数覆盖"缺几何 vs 黑点挡前"判据（cover 点号分档 + lum 亮度）',
    /cover/.test(ply) && /lum/.test(ply));
}

console.log(`\n=== 汇总：PASS ${pass} / FAIL ${fail}${skip > 0 ? ` / SKIP ${skip}` : ''} ===`);
process.exit(fail === 0 ? 0 : 1);
