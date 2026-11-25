# 生成三张图的 SVG polyline 坐标点，供 report.html 内联使用
# 运行: python3 -I gen_chart_points.py
import math

T = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
C_obs = [12, 8, 5, 4, 3, 4, 6, 9]
C0, a, b = 0.0, 1.2190, 9.1559
T_star = (a / (2 * b)) ** (1 / 3)
o0, o2 = 0.6, 8.0
O = [o0 + o2 * t * t for t in T]

# 图框逻辑坐标 -> SVG 像素：x 域 [0.08,0.82]，各图自带 y 域
W, H = 640, 300          # 逻辑宽高
ML, MR, MT, MB = 56, 18, 16, 40
PW, PH = W - ML - MR, H - MT - MB

def Xpx(t):
    return ML + (t - 0.08) / (0.82 - 0.08) * PW

def Ypx(v, vmin, vmax):
    return MT + (vmax - v) / (vmax - vmin) * PH

def line(ts, fn, vmin, vmax):
    return " ".join(f"{Xpx(t):.1f},{Ypx(fn(t), vmin, vmax):.1f}" for t in ts)

fine = [0.1 + i * 0.004 for i in range(176)]   # 0.1..0.8 细网格

# ---- 图1：碰撞观测 + 拟合模型 + 两个最优点 ----
print("P1_OBS_DOTS =", " ".join(f"{Xpx(t):.1f},{Ypx(c, 2, 13):.1f}" for t, c in zip(T, C_obs)))
print("P1_FIT_LINE =", line(fine, lambda t: C0 + a / t + b * t * t, 2, 13))
print("P1_TSTAR =", f"{Xpx(T_star):.1f},{Ypx(C0 + a/T_star + b*T_star**2, 2, 13):.1f}", " T*=%.3f" % T_star)
print("P1_TEMP =", f"{Xpx(0.5):.1f},{Ypx(3, 2, 13):.1f}")
print("P1_XTICKS =", " ".join(f"{Xpx(t):.1f}|{t:.1f}" for t in T))

# ---- 图2：双谷反例 ----
def f_true(t):
    base = 7.0 + 30.0 * (t - 0.42) ** 2
    return base - 2.5 * math.exp(-((t - 0.39) / 0.025) ** 2) - 2.5 * math.exp(-((t - 0.45) / 0.025) ** 2)

vmin2, vmax2 = 4.0, 12.0
print("P2_TRUE_LINE =", line(fine, f_true, vmin2, vmax2))
print("P2_SAMPLE_DOTS =", " ".join(f"{Xpx(t):.1f},{Ypx(f_true(t), vmin2, vmax2):.1f}" for t in T))
print("P2_MIN1 =", f"{Xpx(0.3905):.1f},{Ypx(f_true(0.3905), vmin2, vmax2):.1f}")
print("P2_MIN2 =", f"{Xpx(0.4495):.1f},{Ypx(f_true(0.4495), vmin2, vmax2):.1f}")
print("P2_HUMP =", f"{Xpx(0.42):.1f},{Ypx(f_true(0.42), vmin2, vmax2):.1f}")

# ---- 图3：双曲线小多联（上：碰撞，下：过冲），共享 x，各自 y 域 ----
H3a, H3b = 150, 150
def Ypx_a(v, vmin, vmax):
    return 16 + (vmax - v) / (vmax - vmin) * (H3a - 16 - 30)
def Ypx_b(v, vmin, vmax):
    return H3a + 16 + (vmax - v) / (vmax - vmin) * (H3b - 16 - 30)

print("P3_C_DOTS =", " ".join(f"{Xpx(t):.1f},{Ypx_a(c, 2, 13):.1f}" for t, c in zip(T, C_obs)))
print("P3_O_DOTS =", " ".join(f"{Xpx(t):.1f},{Ypx_b(o, 0.5, 6):.1f}" for t, o in zip(T, O)))
print("P3_O_LINE =", " ".join(f"{Xpx(t):.1f},{Ypx_b(o0+o2*t*t, 0.5, 6):.1f}" for t in fine))

# 归一化联合代价 J
C_ref, O_ref = max(C_obs) - min(C_obs), max(O) - min(O)
J = [(C_obs[i] - min(C_obs)) / C_ref + (O[i] - min(O)) / O_ref for i in range(8)]
print("P3_J =", " ".join(f"{T[i]:.1f}:{J[i]:.3f}" for i in range(8)))
