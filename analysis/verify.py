# 纯标准库验证脚本：碰撞-预测时长模型、采样密度判据、双谷反例
# 运行: python3 -I verify.py
import math

# ----------------------------------------------------------------------
# 0. 八组扫描的可复现算例（与题给包络一致：0.1~0.8s、0.5~4 格、12->3->9）
#    固定速度 v 与栅格 dx 下，提前量 L(格) = (v/dx) * T，取 v/dx = 5 格/s
#    => T 与 L 在本次扫描中一一绑定，是同一条有效参数轴。
# ----------------------------------------------------------------------
T = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
L = [round(5 * t, 1) for t in T]                      # 0.5,1.0,...,4.0 格
C_obs = [12, 8, 5, 4, 3, 4, 6, 9]                     # 碰撞次数（整数计数）
assert C_obs[0] == 12 and min(C_obs) == 3 and C_obs[-1] == 9

# ----------------------------------------------------------------------
# 1. 模型 C(T) = C0 + a/T + b T^2
#    1/T 项 = 滞后碰撞（T 短追不上），T^2 项 = 过预测碰撞（预判到障碍后方）
#    对 (C0, a, b) 做最小二乘：线性设计矩阵 X = [1, 1/T, T^2]
# ----------------------------------------------------------------------
def solve3(A, y):
    """高斯消元解 3x3 线性方程组"""
    M = [A[i][:] + [y[i]] for i in range(3)]
    for col in range(3):
        p = max(range(col, 3), key=lambda r: abs(M[r][col]))
        M[col], M[p] = M[p], M[col]
        for r in range(3):
            if r == col:
                continue
            f = M[r][col] / M[col][col]
            for k in range(col, 4):
                M[r][k] -= f * M[col][k]
    return [M[i][3] / M[i][i] for i in range(3)]

X = [[1.0, 1.0 / t, t * t] for t in T]
XtX = [[sum(X[r][i] * X[r][j] for r in range(8)) for j in range(3)] for i in range(3)]
Xty = [sum(X[r][i] * C_obs[r] for r in range(8)) for i in range(3)]
C0u, au, bu = solve3(XtX, Xty)
print(f"[拟合-无约束OLS] C0={C0u:.4f}, a={au:.4f}, b={bu:.4f}（C0<0 无物理意义，仅作对照）")

# 非负约束最小二乘：C0>=0，网格搜索 C0 后对 a,b 解 2x2 正规方程
def solve2(A, y):
    d = A[0][0] * A[1][1] - A[0][1] * A[1][0]
    return ([(y[0] * A[1][1] - y[1] * A[0][1]) / d,
             (A[0][0] * y[1] - A[1][0] * y[0]) / d])

def sse(c0):
    y = [C_obs[r] - c0 for r in range(8)]
    f1 = [1.0 / t for t in T]
    f2 = [t * t for t in T]
    A22 = [[sum(f1[r] ** 2 for r in range(8)), sum(f1[r] * f2[r] for r in range(8))],
           [sum(f1[r] * f2[r] for r in range(8)), sum(f2[r] ** 2 for r in range(8))]]
    y2 = [sum(f1[r] * y[r] for r in range(8)), sum(f2[r] * y[r] for r in range(8))]
    aa, bb = solve2(A22, y2)
    e = sum((c0 + aa * f1[r] + bb * f2[r] - C_obs[r]) ** 2 for r in range(8))
    return e, aa, bb

best = min((sse(c) + (c,) for c in [k * 0.01 for k in range(201)]), key=lambda z: z[0])
ss_res, C0, a, b = best[0], best[3], best[1], best[2]
print(f"[拟合-非负约束] C(T) = {C0:.4f} + {a:.4f}/T + {b:.4f}*T^2")
pred = [C0 + a / T[i] + b * T[i] ** 2 for i in range(8)]
resid = [C_obs[i] - pred[i] for i in range(8)]
ss_tot = sum((c - sum(C_obs) / 8) ** 2 for c in C_obs)
print(f"[拟合] 预测值={['%.2f' % p for p in pred]}  R^2={1 - ss_res/ss_tot:.4f}")

# 解析最优：dC/dT = -a/T^2 + 2bT = 0  =>  T* = (a/(2b))^(1/3)
T_star = (a / (2 * b)) ** (1.0 / 3.0)
C_star = C0 + a / T_star + b * T_star ** 2
print(f"[解析最优] T* = {T_star:.4f} s, C(T*) = {C_star:.3f} 次")

# 经验最优（八个点中碰撞最少）
i_emp = min(range(8), key=lambda i: C_obs[i])
T_emp, C_emp = T[i_emp], C_obs[i_emp]
dev = abs(T_star - T_emp) / T_emp
print(f"[经验最优] T_emp = {T_emp} s (L={L[i_emp]} 格), C = {C_emp} 次")
print(f"[偏差] |T*-T_emp|/T_emp = {dev*100:.1f}%  (阈值 20%)")

# 谷宽：最小值处局部二次近似 C ≈ C* + (1/2) C'' h^2；C''(T*) = 6b
# 定义 w = 碰撞计数上升 1 次（整数计数的可分辨量级）对应的全宽
Cpp = 6 * b
w = 2 * math.sqrt(2.0 / Cpp)
Delta = 0.1
print(f"[谷宽] C''(T*)={Cpp:.3f}, 计数上升1次的全宽 w = {w:.4f} s")
print(f"[比值] 采样间隔/谷宽 = Δ/w = {Delta/w:.3f}  (判据要求 < 0.5)")

# ----------------------------------------------------------------------
# 2. 采样密度判据：八个点能否支撑“单峰(单谷)”结论
# ----------------------------------------------------------------------
N = 8
span = 0.7
Delta_uniform = span / (N - 1)
w_crit_half = 2 * Delta_uniform          # Δ < w/2  => w > 2Δ
print(f"[采样] Δ = {Delta_uniform:.4f}s；按 Δ<w/2，临界谷宽 w_crit = {w_crit_half:.3f}s")
w_crit_quarter = 4 * Delta_uniform       # 题给保守口径 Δ < w/4
print(f"[采样] 按保守口径 Δ<w/4，临界谷宽 w_crit = {w_crit_quarter:.3f}s")

# ----------------------------------------------------------------------
# 3. 双谷反例：真实曲线有两个极小值，八个网格点只看到单谷
#    宽单谷底 + 两个窄高斯凹陷（σ=0.025s），两谷中心 0.39 / 0.45
# ----------------------------------------------------------------------
def f_true(t):
    base = 7.0 + 30.0 * (t - 0.42) ** 2
    dip1 = -2.5 * math.exp(-((t - 0.39) / 0.025) ** 2)
    dip2 = -2.5 * math.exp(-((t - 0.45) / 0.025) ** 2)
    return base + dip1 + dip2

samples = [f_true(t) for t in T]
print("[反例] 8 网格点取值:", ["%.3f" % s for s in samples])
assert min(samples) > 0, "合成碰撞量必须保持正值"
# 检查采样序列是否单谷：差分符号只变号一次（负->正）
diffs = [samples[i + 1] - samples[i] for i in range(7)]
sign_changes = sum(1 for i in range(6) if diffs[i] < 0 < diffs[i + 1] or diffs[i] > 0 > diffs[i + 1])
print(f"[反例] 采样序列差分符号变号次数 = {sign_changes}（1 => 被误判为单谷）")

# 细网格求真实极值
fine = [0.1 + i * 0.0005 for i in range(int(0.7 / 0.0005) + 1)]
vals = [f_true(t) for t in fine]
mins = [fine[i] for i in range(1, len(fine) - 1) if vals[i] < vals[i - 1] and vals[i] < vals[i + 1]]
maxs = [fine[i] for i in range(1, len(fine) - 1) if vals[i] > vals[i - 1] and vals[i] > vals[i + 1]]
print(f"[反例] 真实极小值位置 ≈ {[round(m, 4) for m in mins]}")
print(f"[反例] 两谷间局部极大 ≈ {[round(m, 4) for m in maxs]}，f={[round(f_true(m),3) for m in maxs]}")

# 单个窄谷的特征全宽 w_d ≈ 4σ
w_d = 4 * 0.025
N_nyquist = math.ceil(span / (w_d / 2)) + 1     # Δ < w/2
N_three = math.ceil(span / (w_d / 3)) + 1       # 谷内至少 3 点
print(f"[反例] 单谷特征宽 w_d ≈ {w_d:.3f}s；Δ<w/2 需 N ≥ {N_nyquist} 点；谷内3点需 N ≥ {N_three} 点")
# 两谷间距 d=0.06，要分开两谷还需 Δ < d/2
d_sep = 0.06
N_sep = math.ceil(span / (d_sep / 2)) + 1
print(f"[反例] 分开间距 d={d_sep}s 的两谷需 Δ<d/2，即 N ≥ {N_sep} 点")

# ----------------------------------------------------------------------
# 4. 过冲距离曲线 O(T) 与双曲线权衡
#    过冲随预测时长单调上升，取 O(T) = o0 + o2 T^2（格），纯预测误差机制
# ----------------------------------------------------------------------
o0, o2 = 0.6, 8.0
O = [o0 + o2 * t * t for t in T]
print("[过冲] O(T) 格 =", ["%.2f" % o for o in O])

# 归一化联合代价，等权：J = C/C_ref + O/O_ref，ref 取各自区间最大极差
C_ref = max(C_obs) - min(C_obs)
O_ref = max(O) - min(O)
J = [(C_obs[i] - min(C_obs)) / C_ref + (O[i] - min(O)) / O_ref for i in range(8)]
print("[权衡] 归一化 J =", ["%.3f" % j for j in J])
i_j = min(range(8), key=lambda i: J[i])
print(f"[权衡] 网格上 J 最小点 T = {T[i_j]}s (L={L[i_j]}格)")

# 边际平衡：碰撞下降速率 = λ * 过冲上升速率（λ=1，归一化后）
for i in range(7):
    mc = (C_obs[i + 1] - C_obs[i]) / C_ref / 0.1      # dC/dt 归一化
    mo = (O[i + 1] - O[i]) / O_ref / 0.1              # dO/dt 归一化
    flag = "  <-- 边际交叉区间" if i > 0 and ((C_obs[i]-C_obs[i-1])/C_ref/0.1 + (O[i]-O[i-1])/O_ref/0.1 < 0) and (mc + mo > 0) else ""
    print(f"  区间 {T[i]:.1f}-{T[i+1]:.1f}: 边际碰撞={mc:+.2f}/s 边际过冲={mo:+.2f}/s 合计={mc+mo:+.2f}{flag}")
