import csv
import glob
import math
import os
import statistics

ROOT = r"D:\lwx\lwx-project\HookWeightModule\code\STM32G031C8T6\log"
FILES = glob.glob(os.path.join(ROOT, "新传感器测长时间稳定性", "*.csv"))
FILES += glob.glob(os.path.join(ROOT, "新传感器新PCB", "*.csv"))


def quantile(values, q):
    values = sorted(values)
    if not values:
        return 0.0
    p = (len(values) - 1) * q
    lo = int(p)
    hi = min(lo + 1, len(values) - 1)
    return values[lo] + (values[hi] - values[lo]) * (p - lo)


def median5(values):
    out = []
    window = []
    for value in values:
        window.append(value)
        if len(window) > 5:
            window.pop(0)
        out.append(statistics.median(window))
    return out


def kalman(values, q, r, threshold=3000.0, boost=100.0):
    out = []
    x = values[0]
    p = r
    for z in values:
        innovation = z - x
        q_now = q * boost if abs(innovation) > threshold else q
        p += q_now
        k = p / (p + r)
        x += k * innovation
        p *= 1.0 - k
        out.append(x)
    return out


def local_residuals(values, block=100):
    result = []
    for start in range(100, len(values), block):
        chunk = values[start:start + block]
        if len(chunk) < 20:
            continue
        # 在每个 10 秒数据块中去除线性趋势，防止长期热
        # 漂移被误认为采样噪声。
        n = len(chunk)
        xm = (n - 1) / 2.0
        ym = sum(chunk) / n
        den = sum((i - xm) ** 2 for i in range(n))
        slope = sum((i - xm) * (v - ym) for i, v in enumerate(chunk)) / den
        result.extend(v - (ym + slope * (i - xm)) for i, v in enumerate(chunk))
    return result


all_residuals = []
loaded = []
for path in sorted(FILES):
    with open(path, encoding="utf-8-sig", newline="") as stream:
        raw = [float(row["I0"]) for row in csv.DictReader(stream)]
    residuals = local_residuals(raw)
    all_residuals.extend(residuals)
    diffs = [abs(raw[i] - raw[i - 1]) for i in range(1, len(raw))]
    loaded.append((path, raw))
    print(os.path.basename(path), "n", len(raw),
          "res_sd", round(statistics.pstdev(residuals), 2),
          "res_p95", round(quantile([abs(x) for x in residuals], .95), 2),
          "diff_med", round(statistics.median(diffs), 2),
          "diff_p95", round(quantile(diffs, .95), 2),
          "max_diff", round(max(diffs), 1))

noise_sd = statistics.pstdev(all_residuals)
print("ALL noise_sd", noise_sd, "variance", noise_sd ** 2,
      "abs_p95", quantile([abs(x) for x in all_residuals], .95))

# 排除大幅跳变后比较候选稳态滤波器。
for q, r in [(4, 2500), (9, 2500), (16, 2500), (25, 2500),
             (16, 1600), (25, 1600), (36, 1600)]:
    residuals = []
    gains = []
    for _, raw in loaded:
        med = median5(raw)
        filt = kalman(med, q, r)
        residuals.extend(local_residuals(filt))
    k = (math.sqrt(q*q + 4*q*r) - q) / (2*r)
    print("candidate", q, r, "K", round(k, 4),
          "equiv_samples", round(1/k, 1),
          "filtered_local_sd", round(statistics.pstdev(residuals), 2))

# 利用原始数据的最大变化定位新 PCB 日志中的负载跳变。
for path, raw in loaded:
    if "1527" not in os.path.basename(path):
        continue
    changes = sorted(((abs(raw[i]-raw[i-1]), i, raw[i]-raw[i-1])
                      for i in range(1, len(raw))), reverse=True)[:20]
    print("TOP_CHANGES", os.path.basename(path))
    for magnitude, index, delta in changes:
        print(index, round(index/10,1), round(delta,1), round(magnitude,1))
