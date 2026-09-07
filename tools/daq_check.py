#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
daq_check.py —— 手套采集数据【上位机验收工具】(纯标准库, Windows/Linux 都能跑)

用法:
  python daq_check.py <段目录>                 验收一段, 打印报告
  python daq_check.py <daq根目录>              逐段验收 + 跨段连续性
  python daq_check.py <段目录> --export         另存 glove_decoded.csv(关节/IMU/四元数, 可用 Excel 画图)
  python daq_check.py <段目录> --frame 100      打印第 100 帧手套数据的解码结果

一段数据 = seg_<段号>_<开机秒>/ 下 6 个文件, 全部按 cycle(周期号)对齐:
  cam0.h265 cam1.h265  两路 H.265 裸码流(ffplay -f hevc cam0.h265 可播), 每帧在文件里的位置见 pairs.csv
  pairs.csv            相机: 一行 = 一对配好的左右帧 → 属于哪个 cycle、两路各在码流文件的 off/len
  glove.bin            手套: 原始 2690 字节 SPI 数据帧顺序追加(协议 v2 布局, 大端)
  glove.csv            手套索引: cycle, PA1 沿内核时间戳, 该帧在 glove.bin 的偏移
  ext_joints.csv       外接转接板 21 路 ADC(仅 -U 启用时有内容)
"""
import sys, os, csv, struct, math, argparse

FRAME = 2690
PERIOD_US = 16667
HALF_US = PERIOD_US // 2

# ---- 协议 v2 字节偏移(与 RV 侧 glove_link.h 一致) ----
OFF_MAGIC, OFF_LEN, OFF_CYCLE, OFF_HB, OFF_JOINT = 0, 2, 4, 8, 12
OFF_TACSTAT, OFF_TAC, OFF_IMU, OFF_MAG, OFF_QUAT, OFF_CRC = 92, 102, 2662, 2674, 2680, 2688
MAGIC, LENGTH = 0x6A01, 0x053F

def crc16_arc(b):
    crc = 0
    for x in b:
        crc ^= x
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc

def decode_frame(b):
    f = {}
    f['magic'], f['length'] = struct.unpack_from('>HH', b, 0)
    f['cycle'], f['heartbeat'] = struct.unpack_from('>II', b, 4)
    j = struct.unpack_from('>20I', b, OFF_JOINT)
    f['joint_raw'] = j
    f['joint_missing'] = [v == 0xFFFFFFFF for v in j]
    f['joint_angle'] = [v & 0x7FFFFF for v in j]
    f['tac_stat'] = struct.unpack_from('>5H', b, OFF_TACSTAT)
    f['imu'] = struct.unpack_from('>6h', b, OFF_IMU)      # ax ay az gx gy gz
    f['mag'] = struct.unpack_from('>3h', b, OFF_MAG)
    f['quat'] = struct.unpack_from('>4h', b, OFF_QUAT)    # w x y z, Q14
    f['crc_recv'] = struct.unpack_from('>H', b, OFF_CRC)[0]
    f['crc_calc'] = crc16_arc(b[:OFF_CRC])
    q = [v / 16384.0 for v in f['quat']]
    f['quat_norm'] = math.sqrt(sum(v * v for v in q))
    return f

def hb_summary(hb):
    """心跳位图: bit=1 表示异常/缺席。0..19 关节, 20..24 触觉, 25 IMU, 26 磁, 27 PD"""
    return {'joint_missing': sum(1 for i in range(20) if hb >> i & 1),
            'tac_missing': sum(1 for i in range(20, 25) if hb >> i & 1),
            'imu_off': bool(hb >> 25 & 1), 'mag_off': bool(hb >> 26 & 1), 'pd_off': bool(hb >> 27 & 1)}

def stats(xs):
    if not xs: return (0, 0, 0, 0)
    n = len(xs); m = sum(xs) / n
    var = sum((x - m) ** 2 for x in xs) / n
    return (m, math.sqrt(var), min(xs), max(xs))

OK, BAD, WARN = '  ✓ ', '  ✗ ', '  ! '

def check_segment(seg, export=False, frame_no=None):
    print(f"\n════════ {seg} ════════")
    problems = []
    def bad(msg): problems.append(msg); print(BAD + msg)
    def warn(msg): print(WARN + msg)
    def ok(msg): print(OK + msg)

    # ---------- glove.bin + glove.csv ----------
    gbin = os.path.join(seg, 'glove.bin'); gcsv = os.path.join(seg, 'glove.csv')
    print("[手套 glove.bin / glove.csv]")
    if not (os.path.isfile(gbin) and os.path.isfile(gcsv)):
        bad("缺 glove.bin 或 glove.csv"); return problems
    size = os.path.getsize(gbin)
    nfr = size // FRAME
    (ok if size % FRAME == 0 else bad)(f"glove.bin {size} 字节 = {nfr} 帧 × 2690" + ("" if size % FRAME == 0 else f", 余 {size % FRAME} 字节(文件被截断?)"))
    rows = list(csv.DictReader(open(gcsv, newline='')))
    (ok if len(rows) == nfr else bad)(f"glove.csv {len(rows)} 条索引 vs bin {nfr} 帧" + ("" if len(rows) == nfr else " ★不一致★"))

    cycles, edges, frames = [], [], []
    n_magic = n_crc = n_cycle_mismatch = n_offset_bad = 0
    small_pkts = []
    with open(gbin, 'rb') as fb:
        for i, r in enumerate(rows):
            off = int(r['off'])
            if off != i * FRAME: n_offset_bad += 1
            fb.seek(off); b = fb.read(FRAME)
            if len(b) < FRAME: bad(f"第 {i} 帧读不满(off={off})"); break
            f = decode_frame(b)
            c = int(r['cycle'])
            if f['magic'] != MAGIC or f['length'] != LENGTH:
                n_magic += 1; small_pkts.append((i, f['magic'], c)); continue    # 不是数据帧, 不参与统计
            if f['crc_recv'] != f['crc_calc']: n_crc += 1
            if f['cycle'] != c: n_cycle_mismatch += 1
            cycles.append(c); edges.append(int(r['edge_ns'])); frames.append(f)
    (ok if n_offset_bad == 0 else bad)(f"索引偏移连续 (错 {n_offset_bad})")
    if n_magic:
        bad(f"{n_magic} 帧不是数据帧(magic≠0x6A01): " + ", ".join(f"第{i}帧 magic=0x{m:04X} 记的cycle={c}" for i, m, c in small_pkts[:3])
            + (" …" if len(small_pkts) > 3 else "") + "  ← 多半是采集收尾时 STM32 的自检小包被当数据帧记录(RV 已修)")
    else: ok("全部帧头 0x6A01 / 长度 0x053F")
    (ok if n_crc == 0 else bad)(f"CRC-16/ARC 复算: 错 {n_crc}/{len(frames)}")
    (ok if n_cycle_mismatch == 0 else bad)(f"bin 内 cycle 与 csv 一致 (不一致 {n_cycle_mismatch})")

    # 周期号连续性
    gaps = [(i, cycles[i-1], cycles[i]) for i in range(1, len(cycles)) if cycles[i] - cycles[i-1] != 1]
    lost = sum(c - p - 1 for _, p, c in gaps if c > p)
    if gaps:
        warn(f"cycle 不连续 {len(gaps)} 处, 共丢 {lost} 拍: " + "; ".join(f"第{i}条 {p}→{c}" for i, p, c in gaps[:5]) + (" …" if len(gaps) > 5 else ""))
    else: ok(f"cycle 连续 {cycles[0]}→{cycles[-1]}, 零丢拍")
    if cycles: print(f"    cycle 范围 {cycles[0]} ~ {cycles[-1]}, 时长 {(cycles[-1]-cycles[0]+1)/60:.1f} s @60Hz")

    # PA1 沿间隔
    d = [(edges[i] - edges[i-1]) / 1e6 for i in range(1, len(edges))]
    if d:
        m, s, lo, hi = stats(d)
        out = [(i+1, x) for i, x in enumerate(d) if x < 8 or x > 25]
        (ok if not out else warn)(f"PA1 沿间隔 均值 {m:.3f} ms σ {s:.3f} ms, 范围 {lo:.1f}~{hi:.1f} ms, 异常 {len(out)} 处"
                                  + ("" if not out else ": " + ", ".join(f"第{i}条 {x:.0f}ms" for i, x in out[:4])))

    # 心跳 / 数据健康
    if frames:
        hb = hb_summary(frames[-1]['heartbeat'])
        print(f"    末帧心跳: 关节缺 {hb['joint_missing']}/20, 触觉缺 {hb['tac_missing']}/5, IMU {'离线' if hb['imu_off'] else '在线'}, 磁 {'离线' if hb['mag_off'] else '在线'}, PD {'缺' if hb['pd_off'] else '在'}")
        qn = [f['quat_norm'] for f in frames]
        m, s, lo, hi = stats(qn)
        imu_on = not hb['imu_off']
        (ok if (abs(m - 1) < 0.02 and s < 0.02) or not imu_on else warn)(f"四元数模长 |q| 均值 {m:.4f} σ {s:.4f} (IMU 在线时应≈1.000; 偏离=Q14/字节序/字段错位)")
        miss = [sum(f['joint_missing']) for f in frames]
        print(f"    关节缺失数/帧: 均值 {sum(miss)/len(miss):.1f}, 全 20 缺的帧 {sum(1 for x in miss if x == 20)}/{len(miss)}")

    # ---------- pairs.csv + h265 ----------
    print("[相机 pairs.csv / cam0.h265 / cam1.h265]")
    pcsv = os.path.join(seg, 'pairs.csv')
    prow = list(csv.DictReader(open(pcsv, newline=''))) if os.path.isfile(pcsv) else []
    h0, h1 = (os.path.getsize(os.path.join(seg, f)) if os.path.isfile(os.path.join(seg, f)) else -1 for f in ('cam0.h265', 'cam1.h265'))
    if not prow:
        warn(f"pairs.csv 空 (cam0.h265 {h0} B, cam1.h265 {h1} B) —— 相机没出帧或 -X 模式")
    else:
        rel, unrel, resid, dpts, pts0 = [], 0, [], [], []
        off_ok = True; last_end0 = last_end1 = 0
        for r in prow:
            c = r['cycle']
            if c.startswith('-'): unrel += 1
            else:
                rel.append(int(c)); resid.append(int(r['residual_us']))
            dpts.append(int(r['dpts_us'])); pts0.append(int(r['pts0']))
            o0, l0, o1, l1 = int(r['off0']), int(r['len0']), int(r['off1']), int(r['len1'])
            if o0 != last_end0 or o1 != last_end1: off_ok = False
            last_end0, last_end1 = o0 + l0, o1 + l1
        dur = (pts0[-1] - pts0[0]) / 1e6 if len(pts0) > 1 else 0
        ok(f"{len(prow)} 对帧, 时长 {dur:.1f} s, 平均 {len(prow)/dur if dur else 0:.2f} fps")
        (ok if off_ok and last_end0 == h0 and last_end1 == h1 else bad)(
            f"码流偏移自洽: cam0 末尾 {last_end0} vs 文件 {h0}, cam1 {last_end1} vs {h1}" + ("" if off_ok else " ★off 不连续★"))
        m, s, lo, hi = stats(dpts)
        (ok if abs(m) < 100 and s < 50 else warn)(f"双摄 dpts 均值 {m:+.1f} µs σ {s:.1f} µs (硬同步应恒定在 −10~0 µs)")
        print(f"    对齐: 可靠归属 {len(rel)} 对, 不可靠('-'前缀=标定中/无临近锚点) {unrel} 对")
        if resid:
            m, s, lo, hi = stats(resid)
            oor = sum(1 for x in resid if abs(x) > HALF_US)
            (ok if oor == 0 else bad)(f"对齐残差 均值 {m:+.0f} µs σ {s:.0f} µs 极值 {lo}/{hi} µs; |残差|>半周期({HALF_US}µs) 的 {oor} 对"
                                      + ("" if oor == 0 else " ★这些帧的 cycle 不可信(多为手套流先停后的相机尾巴)★"))
            # 联结覆盖率
            gset = set(cycles)
            hit = sum(1 for c in rel if c in gset)
            (ok if hit == len(rel) else warn)(f"pairs↔glove 按 cycle 联结: {hit}/{len(rel)} 对能找到同拍手套数据 ({100*hit/max(1,len(rel)):.1f}%)")
            dup = len(rel) - len(set(rel))
            (ok if dup == 0 else warn)(f"同一 cycle 被多对相机帧占用: {dup} 处 (>0 = 相机比 XVS 快或对齐滑移)")

    # ---------- ext_joints.csv ----------
    ecsv = os.path.join(seg, 'ext_joints.csv')
    if os.path.isfile(ecsv):
        er = list(csv.DictReader(open(ecsv, newline='')))
        print("[外接 ext_joints.csv]")
        if not er: print("    (空: 未启用 -U)")
        else:
            v = [r for r in er if r['valid'] == '1']
            lat = [int(r['latency_us']) for r in v]
            m, s, lo, hi = stats(lat)
            (ok if len(v) > 0.98 * len(er) else warn)(f"{len(er)} 拍, 有效 {len(v)} ({100*len(v)/len(er):.1f}%), 延时 均值 {m/1000:.2f} ms max {hi/1000:.2f} ms")
            ff = sum(1 for r in v for k in range(21) if r[f'ch{k}'] == '65535')
            print(f"    通道 0xFFFF(未采到) 次数: {ff}")

    # ---------- 导出 / 单帧 ----------
    if export and frames:
        outp = os.path.join(seg, 'glove_decoded.csv')
        with open(outp, 'w', newline='') as fo:
            w = csv.writer(fo)
            w.writerow(['cycle', 'edge_ns'] + [f'j{i}' for i in range(20)] + ['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'mx', 'my', 'mz', 'qw', 'qx', 'qy', 'qz', 'quat_norm', 'heartbeat_hex'])
            for c, e, f in zip(cycles, edges, frames):
                w.writerow([c, e] + [('' if mflag else a) for a, mflag in zip(f['joint_angle'], f['joint_missing'])]
                           + list(f['imu']) + list(f['mag']) + [v / 16384.0 for v in f['quat']] + [f"{f['quat_norm']:.4f}", f"0x{f['heartbeat']:08X}"])
        print(f"  → 已导出 {outp} ({len(frames)} 行, Excel 直接开)")
    if frame_no is not None and 0 <= frame_no < len(frames):
        f = frames[frame_no]
        print(f"\n[第 {frame_no} 帧 cycle={f['cycle']} heartbeat=0x{f['heartbeat']:08X} crc {'✓' if f['crc_recv']==f['crc_calc'] else '✗'}]")
        print("  关节角(raw&0x7FFFFF, 缺=-):", [('-' if mflag else a) for a, mflag in zip(f['joint_angle'], f['joint_missing'])])
        print("  IMU ax ay az gx gy gz:", f['imu'], " 磁:", f['mag'])
        print("  四元数 wxyz(Q14):", [round(v/16384.0, 4) for v in f['quat']], f"|q|={f['quat_norm']:.4f}")
        print("  触觉 tac_stat:", f['tac_stat'])
    return problems

def main():
    ap = argparse.ArgumentParser(description='手套采集数据验收')
    ap.add_argument('path'); ap.add_argument('--export', action='store_true'); ap.add_argument('--frame', type=int)
    a = ap.parse_args()
    p = a.path
    segs = [p] if os.path.isfile(os.path.join(p, 'glove.csv')) else sorted(os.path.join(p, d) for d in os.listdir(p) if d.startswith('seg_'))
    if not segs: print("没找到段目录(seg_*)"); sys.exit(2)
    allp = {}
    for s in segs: allp[s] = check_segment(s, a.export, a.frame)
    print("\n════════ 总结 ════════")
    for s, pr in allp.items(): print(f"  {os.path.basename(s)}: " + ("通过 ✓" if not pr else f"{len(pr)} 项问题 ✗"))
    sys.exit(1 if any(allp.values()) else 0)

if __name__ == '__main__': main()
