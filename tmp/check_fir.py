"""
G_Export_V2 FIR 滤波器验证与信号链分析
=============================================
验证 Simulink 导出的 64 阶 FIR 低通滤波器（4 倍抽取）的频率响应，
以及含谐波信号下 3 参数正弦拟合的精度。

用法:
    python check_fir.py

依赖: numpy（仅用于多频联合拟合对比）
"""

import math
import io
import sys

# 强制 UTF-8 输出 (Windows GBK 兼容)
if sys.stdout.encoding != 'utf-8':
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
import cmath
import sys

# ============================================================
# 1. FIR 系数
# ============================================================
# 来源: G_Export_V2_data.c → ConstP.FIR4_FILT[64]
# 存储格式: 多相分解 (4 相 × 16 抽头)
#   FIR4_FILT[phase * 16 + tap] = h_natural[phase + tap * 4]
# ============================================================

FIR4_FILT_POLYPHASE = [
    -1.09383236E-5, -0.000171520427, 0.0013099619, -0.003265423,
    0.00242713792, 0.00806980859, -0.033233352, 0.0768118352, 0.184631377,
    0.0310403127, -0.0276700184, 0.0126849255, -0.00200400129,
    -0.00138964527, 0.000979136443, -0.000210121216, -6.29225469E-5,
    5.20178619E-5, 0.0011394436, -0.00457738154, 0.00778010441,
    -0.00199512416, -0.0267355982, 0.123946846, 0.162737817, -0.00509603694,
    -0.0153666306, 0.0119966483, -0.00437429268, 0.000227175,
    0.000474089233, -0.000145619051, -0.000145619051, 0.000474089233,
    0.000227175, -0.00437429268, 0.0119966483, -0.0153666306,
    -0.00509603694, 0.162737817, 0.123946846, -0.0267355982, -0.00199512416,
    0.00778010441, -0.00457738154, 0.0011394436, 5.20178619E-5,
    -6.29225469E-5, -0.000210121216, 0.000979136443, -0.00138964527,
    -0.00200400129, 0.0126849255, -0.0276700184, 0.0310403127, 0.184631377,
    0.0768118352, -0.033233352, 0.00806980859, 0.00242713792, -0.003265423,
    0.0013099619, -0.000171520427, -1.09383236E-5
]

FS_INPUT = 8_000_000     # ADC 采样率 (Hz)
FS_OUTPUT = 2_000_000    # 抽取后采样率 (Hz)
DECIMATION = 4
NFFT = 4096


def polyphase_to_natural(poly):
    """将多相排序系数还原为自然顺序冲激响应 h[n]"""
    M = DECIMATION
    taps = len(poly) // M
    h = [poly[(n % M) * taps + n // M] for n in range(len(poly))]
    return h


def freqz(h, f, fs):
    """计算频率 f (Hz) 处的复频率响应 H(f)"""
    omega = 2.0 * math.pi * f / fs
    H = complex(0, 0)
    for n, coeff in enumerate(h):
        H += coeff * cmath.exp(-1j * omega * n)
    return H


def magn_db(h, f, fs):
    """返回 |H(f)| 和 dB 值"""
    H = freqz(h, f, fs)
    mag = abs(H)
    db = 20.0 * math.log10(mag) if mag > 1e-30 else -999.0
    return mag, db


# ============================================================
# 2. 验证: 多相排序 ↔ 自然顺序 系数一致性
# ============================================================
H_NATURAL = polyphase_to_natural(FIR4_FILT_POLYPHASE)

# 验证对称性 (线性相位 FIR 的必要条件)
max_asymmetry = max(
    abs(H_NATURAL[i] - H_NATURAL[63 - i]) for i in range(32)
)

# ============================================================
# 3. FIR 频率响应分析
# ============================================================
TEST_FREQS = [
    0, 1_000, 10_000, 50_000, 100_000, 200_000, 300_000,
    400_000, 500_000, 600_000, 700_000, 800_000, 900_000,
    1_000_000, 1_500_000, 2_000_000
]


def find_cutoff(h, fs, target_db=-3.0):
    """查找 -3dB 截止频率"""
    for f in range(0, int(fs / 2), 1000):
        _, db = magn_db(h, f, fs)
        if db < target_db - 0.05 and db > target_db + 0.05:
            continue
        if abs(db - target_db) < 0.1:
            return f
    return None


def find_first_null(h, fs):
    """查找第一个传输零点"""
    for f in range(0, int(fs / 2), 1000):
        mag, _ = magn_db(h, f, fs)
        if mag < 0.015:
            return f
    return None


# ============================================================
# 4. 3 参数正弦拟合仿真 (含谐波)
# ============================================================
def sine_fit_3param(signal, freq, fs):
    """
    三参数最小二乘正弦拟合: y[n] = A·cos(ωn) + B·sin(ωn) + DC
    返回 (amplitude_Vpk, dc_offset)
    算法与 G_Export_V2.c / MATLAB Function 完全一致
    """
    N = len(signal)
    dc = sum(signal) / N
    omega = 2.0 * math.pi * freq / fs
    cw = math.cos(omega)
    sw = math.sin(omega)

    c, s = 1.0, 0.0
    cc = ss = cs = yc = ys = 0.0

    for n in range(N):
        y = signal[n] - dc
        cc += c * c
        ss += s * s
        cs += c * s
        yc += y * c
        ys += y * s

        # 递推生成 sin/cos，每 256 点归一化防止数值漂移
        cn = c * cw - s * sw
        sn = s * cw + c * sw
        c, s = cn, sn
        if (n + 1) & 255 == 0:
            r = math.sqrt(c * c + s * s)
            c /= r
            s /= r

    det = cc * ss - cs * cs
    if abs(det) < 1e-20:
        return 0.0, dc

    A_cos = (yc * ss - ys * cs) / det
    A_sin = (ys * cc - yc * cs) / det
    amplitude = math.sqrt(A_cos * A_cos + A_sin * A_sin)
    return amplitude, dc


def generate_test_signal(f0, harmonics, fs, N):
    """生成含谐波的测试信号"""
    import math
    t = [n / fs for n in range(N)]
    signal = [0.0] * N
    for order, (amp, phase) in harmonics.items():
        for n in range(N):
            signal[n] += amp * math.sin(2 * math.pi * order * f0 * t[n] + phase)
    return signal


# ============================================================
# 主程序
# ============================================================
if __name__ == "__main__":
    print("=" * 72)
    print("  G_Export_V2 FIR 滤波器与信号链验证")
    print("=" * 72)

    # --- 系数基本信息 ---
    print(f"\n  抽头数:       {len(H_NATURAL)}")
    print(f"  对称性:       {'[OK] 线性相位' if max_asymmetry < 1e-8 else '[FAIL] 不对称'}")
    print(f"  DC 增益:      {sum(H_NATURAL):.10f}")
    print(f"  抽取比:       {DECIMATION}:1")
    print(f"  输入 Fs:      {FS_INPUT / 1e6:.0f} MHz")
    print(f"  输出 Fs:      {FS_OUTPUT / 1e6:.0f} MHz")

    # --- 频率响应 ---
    cutoff_3db = find_cutoff(H_NATURAL, FS_INPUT)
    null_freq = find_first_null(H_NATURAL, FS_INPUT)

    print(f"\n  -3 dB 截止:   {cutoff_3db / 1000:.0f} kHz" if cutoff_3db else "")
    print(f"  第一零点:     {null_freq / 1000:.0f} kHz" if null_freq else "")

    print(f"\n  {'频率':>10s}  {'|H|':>10s}  {'|H| (dB)':>10s}  {'×0.25 后':>10s}")
    print(f"  {'-' * 45}")

    for f in TEST_FREQS:
        mag, db = magn_db(H_NATURAL, f, FS_INPUT)
        after_gain = mag * 0.25
        print(f"  {f:>8d} Hz  {mag:>8.4f}   {db:>8.2f}   {after_gain:>8.4f}")

    # --- 含谐波正弦拟合测试 ---
    print(f"\n{'=' * 72}")
    print("  3 参数正弦拟合 — 含谐波测试")
    print(f"{'=' * 72}")

    try:
        import numpy as np
        HAS_NUMPY = True
    except ImportError:
        HAS_NUMPY = False

    if HAS_NUMPY:
        f0 = 21000.0
        harmonics = {
            1: (1.0, 0.1),    # 基波 1Vpk
            2: (0.3, 0.5),    # 二次谐波 0.3Vpk
            3: (0.15, 1.2),   # 三次谐波 0.15Vpk
        }

        N = NFFT
        t = np.arange(N) / FS_OUTPUT
        signal = np.zeros(N)
        for order, (amp, phase) in harmonics.items():
            signal += amp * np.sin(2 * np.pi * order * f0 * t + phase)

        # Hann 窗 + FFT 峰值检测 (匹配 C 代码逻辑)
        hann = 0.5 * (1 - np.cos(2 * np.pi * np.arange(N) / (N - 1)))
        X = np.fft.fft(signal * hann)
        mag_fft = np.abs(X[:N // 2])

        bin_res = FS_OUTPUT / N
        k0 = max(2, int(math.ceil(10000 * N / FS_OUTPUT)) + 1)
        k1 = min(N // 2, int(math.ceil(500000 * N / FS_OUTPUT)) + 2)

        best_scores = np.zeros(3)
        best_bins = np.zeros(3)
        for k in range(k0, k1):
            if mag_fft[k] >= mag_fft[k - 1] and mag_fft[k] > mag_fft[k + 1]:
                y1 = math.log(max(float(mag_fft[k - 1]), 1e-30))
                y2 = math.log(max(float(mag_fft[k]), 1e-30))
                y3 = math.log(max(float(mag_fft[k + 1]), 1e-30))
                den = y1 - 2 * y2 + y3
                delta = 0.5 * (y1 - y3) / den if abs(den) > 1e-20 else 0.0
                delta = max(-0.5, min(0.5, delta))
                score = 2.0 * (mag_fft[k - 1] + mag_fft[k] + mag_fft[k + 1]) / (N - 1)
                if score >= 0.004:
                    for j in range(3):
                        if score > best_scores[j]:
                            for q in range(2, j, -1):
                                best_scores[q] = best_scores[q - 1]
                                best_bins[q] = best_bins[q - 1]
                            best_scores[j] = score
                            best_bins[j] = k + delta
                            break

        freqs = np.array([
            best_bins[j] * bin_res if best_scores[j] > 0 else 0
            for j in range(3)
        ])

        # 按频率排序
        for a in range(2):
            for b in range(a + 1, 3):
                if freqs[b] > 0 and (freqs[a] == 0 or freqs[b] < freqs[a]):
                    freqs[a], freqs[b] = freqs[b], freqs[a]

        print(f"\n  输入: 基波 {f0 / 1000:.0f} kHz, Vpk=1.0 V")
        print(f"        含 2 次 (0.3V) + 3 次 (0.15V) 谐波\n")
        print(f"  {'分量':>6s}  {'检测频率 (Hz)':>16s}  {'拟合幅值 (Vpk)':>16s}  "
              f"{'真实幅值':>10s}  {'误差':>8s}  {'×0.25 后':>10s}")

        true_amps = [1.0, 0.3, 0.15]
        for j in range(3):
            f = float(freqs[j])
            if f > 0:
                amp, dc = sine_fit_3param(signal, f, FS_OUTPUT)
                true = true_amps[j]
                err = 100 * (amp / true - 1)
                label = ["基波", "2次谐波", "3次谐波"][j]
                print(f"  {label:>6s}  {f:>16.3f}  {amp:>16.6f}  "
                      f"{true:>10.3f}  {err:>+7.2f}%  {amp * 0.25:>10.6f}")

        # 多频联合拟合对比
        print(f"\n  --- 多频联合最小二乘拟合 (对照) ---")
        valid = freqs > 0
        n_valid = sum(valid)
        D = np.zeros((N, 2 * n_valid + 1))
        D[:, 0] = 1.0
        for j in range(n_valid):
            omega = 2 * math.pi * freqs[j] / FS_OUTPUT
            D[:, 2 * j + 1] = np.cos(omega * np.arange(N))
            D[:, 2 * j + 2] = np.sin(omega * np.arange(N))
        coeffs, _, _, _ = np.linalg.lstsq(D, signal, rcond=None)
        for j in range(n_valid):
            amp_joint = math.sqrt(coeffs[2 * j + 1] ** 2 + coeffs[2 * j + 2] ** 2)
            true = true_amps[j]
            err = 100 * (amp_joint / true - 1)
            label = ["基波", "2次谐波", "3次谐波"][j]
            print(f"  {label:>6s}  联合拟合: {amp_joint:.6f} Vpk  "
                  f"(误差 {err:+.4f}%)")

        # --- 能量归一化信号源测试 ---
        print(f"\n  --- 能量归一化信号源 (匹配真实信号发生器) ---")
        print(f"  真实信号发生器采用 RMS 能量归一化，而非简单幅度叠加。")
        print(f"  添加谐波后，基波幅值会因总能量守恒而降低。")

        # 信号发生器参数
        amp0 = 1.0      # 设定的基波幅值
        ha = 0.5        # a次谐波相对幅值
        hb = 0.55       # b次谐波相对幅值

        # 能量归一化系数
        K = amp0 / math.sqrt(amp0**2 + ha**2 + hb**2)
        total_rms_before = math.sqrt(amp0**2/2 + ha**2/2 + hb**2/2)
        total_rms_after = math.sqrt((amp0*K)**2/2 + (ha*K)**2/2 + (hb*K)**2/2)

        print(f"\n  归一化系数 K = A0 / sqrt(A0^2 + ha^2 + hb^2)")
        print(f"                = {amp0} / sqrt({amp0}^2 + {ha}^2 + {hb}^2)")
        print(f"                = {K:.4f}")
        print(f"  归一化前 RMS = {total_rms_before:.4f} V")
        print(f"  归一化后 RMS = {total_rms_after:.4f} V")
        print(f"\n  各分量幅值变化:")
        print(f"  {'分量':>8s}  {'归一化前':>10s}  {'归一化后':>10s}  {'×0.25 后':>10s}")
        print(f"  {'-' * 42}")
        print(f"  {'基波':>8s}  {amp0:>10.4f}  {amp0*K:>10.4f}  {amp0*K*0.25:>10.4f}")
        print(f"  {'a次谐波':>8s}  {ha:>10.4f}  {ha*K:>10.4f}  {ha*K*0.25:>10.4f}")
        print(f"  {'b次谐波':>8s}  {hb:>10.4f}  {hb*K:>10.4f}  {hb*K*0.25:>10.4f}")

        # 与实测对比
        print(f"\n  实测基波 (×0.25后) = 0.199916 V")
        print(f"  预测基波 (×0.25后) = {amp0*K*0.25:.6f} V")
        print(f"  匹配度 = {amp0*K*0.25/0.199916*100:.2f}%")

    else:
        print("\n  (需要 numpy 进行谐波拟合仿真, pip install numpy)")

    # --- ADC 链缩放验证 ---
    print(f"\n{'=' * 72}")
    print("  ADC 信号链缩放验证")
    print(f"{'=' * 72}")

    ADC_SCALE = 7.62939453E-5    # 2.5V / 32768
    PREAMP_GAIN_COMP = 0.25      # 前级 4× 增益补偿

    # 示例: 1V DC 输入
    v_input = 1.0
    raw = int(2048 + 2048 * v_input / 2.5)
    adc_block = (raw - 2048) << 4
    q15_volts = adc_block * ADC_SCALE

    print(f"\n  输入 {v_input}V → ADC raw={raw} → adc_block={adc_block}")
    print(f"  → Q15ADC={q15_volts:.6f}V → FIR(|H|≈1)={q15_volts:.6f}V")
    print(f"  → ×{PREAMP_GAIN_COMP} 补偿 = {q15_volts * PREAMP_GAIN_COMP:.6f}V")
    print(f"\n  ADC 缩放因子: {ADC_SCALE:.10f} = 2.5V / 32768")
    print(f"  前级增益补偿: ×{PREAMP_GAIN_COMP} (对应 4× 前级放大)")

    print(f"\n{'=' * 72}")
    print("  验证完成")
    print(f"{'=' * 72}")
