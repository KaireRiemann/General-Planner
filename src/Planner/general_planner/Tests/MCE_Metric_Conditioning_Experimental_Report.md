# MCE / Intrinsic Metric 条件数验证：完整实验报告

- **日期**：2026-08-17
- **对应框架**：`MCE_Metric_Condition_Number_Validation_Framework.md`（§1–§33）
- **前置实验**：`Euclidean_Gradient_Basis_Invariance_Experimental_Report.md`（参数化不变性，49/49）
- **自测程序**：`Tests/minco_metric_conditioning_self_test.cpp`
- **图**：`Tests/plot_mce_metric_conditioning.py`
- **总检查**：43/43 通过

---

## 0. 一句话结论

固定时间下，MCE 度量就是 reduced snap energy 的 Hessian。用它对 Hessian 做 whitening 后，纯 MCE 问题的条件数从 \(7\)–\(1.6\times 10^7\) 降到 \(1.000\)；混入跟踪、动力学、走廊后仍能把 \(\kappa\) 降两个数量级，并同步减少 L-BFGS 迭代。\(L^2\) 度量虽能恢复坐标不变性，但对 snap 主导的 Hessian **并不**改善 conditioning（\(R_\kappa\approx 0.56\)）。

本阶段回答的是：

\[
\kappa(H_J)
\quad\text{vs}\quad
\kappa\!\left(G^{-1/2}H_JG^{-1/2}\right)
\quad\text{，以及}\quad
\kappa_G\downarrow
\Longrightarrow
N_{\mathrm{iter}},\,N_{\mathrm{LS}}\downarrow
\text{ 是否成立。}
\]

前一阶段已经证明 Euclidean \(G=I\) 坐标依赖。本阶段证明：在 MINCO reduced 坐标上，**选对 intrinsic metric 还会改善数值 conditioning**。

---

## 1. 与前一阶段的逻辑关系

前一阶段已经完成：

1. 同一物理轨迹、同一 objective 在不同坐标下保持一致；
2. Euclidean steepest descent 的 \(\delta p(t)\) 坐标依赖；
3. pullback / natural gradient 恢复方向不变性；
4. L-BFGS 路径、早停、非凸盆地也会坐标依赖。

因此 parameterization invariance 已经验证完。本阶段不再比较 Power vs Bernstein，而是固定 MINCO waypoint 坐标 \(P\)，比较不同 **metric**：

\[
G\in\{I,\,G_{L^2},\,G_{H^2},\,G_{\mathrm{MCE}},\,G_{\mathrm{mix}},\,G_{\mathrm{MCE}}+G_{\mathrm{GN}}\}.
\]

核心命题从

> “为什么不能在每种参数化里都设 \(G=I\)？”

换成

> “为什么某一个 intrinsic metric 在数值优化上更合适？”

---

## 2. 理论依据：为什么条件数是正确指标

局部二次模型

\[
J(x+\delta x)
\approx
J(x)
+
g^T\delta x
+
\frac12\delta x^T H_J\delta x,
\qquad
g=\nabla J,\quad H_J=\nabla^2 J.
\]

若 \(H_J\succ 0\)，

\[
\kappa(H_J)=\frac{\lambda_{\max}(H_J)}{\lambda_{\min}(H_J)}.
\]

\(\kappa\gg 1\) 时等高线是扁椭球：gradient zig-zag、线搜索步长变小、quasi-Newton 需要更多迭代学习曲率、参数空间 early-stop 更容易把 stagnation 误判为收敛。

Euclidean 优化默认 \(G=I\)，优化器直接面对 \(H_J\)：

\[
\kappa_E=\kappa(H_J).
\]

设 \(G\succ 0\)，Cholesky \(G=LL^T\)，whitened 坐标 \(z=L^Tx\)。二次项变成

\[
\delta z^T
\bigl(L^{-1}H_JL^{-T}\bigr)
\delta z,
\]

因此

\[
\widetilde H_J
=
G^{-1/2}H_JG^{-1/2},
\qquad
\kappa_G
=
\kappa(\widetilde H_J).
\]

Natural gradient \(d=-G^{-1}\nabla J\) 等价于在 \(z=G^{1/2}x\) 里做 Euclidean gradient。比较 \(\kappa_E\) 与 \(\kappa_G\) 就是在比较：参数欧式空间 vs metric whitening 后，局部曲率是否更 isotropic。

改善比定义为

\[
R_\kappa=\frac{\kappa_E}{\kappa_G}.
\]

| \(R_\kappa\) | 含义 |
|---|---|
| \(\approx 1\) | metric 基本没有改善 |
| \(>10\) | 明显改善 |
| \(>10^2\) | 非常明显 |
| \(\gg 10^3\) | 原始 Euclidean 参数化极度失衡 |

---

## 3. Pure-MCE 的解析结论

固定时间 MINCO reduced control energy 可写为

\[
E(P)=\frac12 P^T G_{\mathrm{MCE}} P.
\]

因此 \(\nabla E=G_{\mathrm{MCE}}P\)，并且

\[
H_E=G_{\mathrm{MCE}}.
\]

若取 \(G=G_{\mathrm{MCE}}\)，则

\[
\widetilde H_E
=
G_{\mathrm{MCE}}^{-1/2}H_EG_{\mathrm{MCE}}^{-1/2}
=I,
\qquad
\kappa(\widetilde H_E)=1.
\]

这是严格结论，不是经验现象：MCE metric 是 reduced Hessian 的精确 whitening。在 \(z=G_{\mathrm{MCE}}^{1/2}P\) 中，

\[
E(z)=\frac12 z^Tz.
\]

所有主方向曲率完全统一。

### \(G_{\mathrm{MCE}}\) 的来源

固定时间 \(A(T)C=b_0+BP\)，\(C=C_0+A^{-1}BP\)，\(J_P=A^{-1}B\)。系数空间 \(E(C)=\mathrm{tr}(C^TQC)\)（本仓库 `getEnergy()` 不含 \(1/2\)），因此 reduced Hessian 是

\[
G_{\mathrm{MCE}}
=
2J_P^TQJ_P
=
2B^TA^{-T}QA^{-1}B.
\]

代码里用 MINCO JVP 基 \(\{\partial C/\partial P_i\}\) 组装 \(G_{\mathrm{snap}}=J_P^TQJ_P\)，再设 \(G_{\mathrm{MCE}}=2G_{\mathrm{snap}}\)，使其与 `getEnergy()` 的 Hessian 一致。它同时是 pullback metric、reduced control Hessian、Schur complement、以及 principled preconditioner。

---

## 4. Mixed objective 的 Hessian whitening

真实 planner 一般为

\[
J=\rho_E E_{\mathrm{MCE}}+J_{\mathrm{other}}.
\]

局部 Hessian

\[
H_J=\rho_E G_{\mathrm{MCE}}+H_{\mathrm{other}}.
\]

MCE whitening 后

\[
\widetilde H_J
=
\rho_E I
+
G_{\mathrm{MCE}}^{-1/2}H_{\mathrm{other}}G_{\mathrm{MCE}}^{-1/2}.
\]

MCE metric **精确消除** minimum-control 部分的 anisotropy，剩下的 conditioning 只来自其他 cost。因此：

- 若 \(J_{\mathrm{other}}\) 相对 snap 很小，\(\kappa_G\approx 1\)；
- 若走廊 / 跟踪残差变大，\(\kappa_G\) 会从 1 往上走；
- 这时应把 Gauss-Newton 约束度量加进 \(G\)，而不是放弃 MCE。

---

## 5. 实验设计

### 5.1 隔离原则

只改 metric。固定：

- MINCO-S4，3D，固定时间，固定 head/tail PVAJ；
- waypoint \(P\) 为唯一决策变量；
- 同一套中心差分 Hessian；
- 同一套 frozen-metric L-BFGS（`mem_size=16`，`g_epsilon=1e-8`，`past=3`，`delta=1e-12`）。

**本阶段不做**：自由时间 \((P,T)\)、逐步更新的 dynamic metric、完整 ROS State2State 节点。Planner-like 是隔离 proxy，不是 onboard 规划器。

### 5.2 数值 Hessian

梯度中心差分

\[
H e_j
\approx
\frac{g(P+h e_j)-g(P-h e_j)}{2h},
\qquad
h=\eta\max(1,|P_j|),\quad\eta=10^{-6}.
\]

随后对称化 \(H\leftarrow\frac12(H+H^T)\)。不显式求逆：Cholesky \(G=LL^T\) 后用三角求解得到 \(L^{-1}HL^{-T}\)。

验收量

\[
e_H
=
\frac{\|H^{FD}-G_{\mathrm{MCE}}\|_F}{\max(1,\|H^{FD}\|_F)}.
\]

纯 MCE 期望 \(e_H\ll 1\) 且 \(\kappa_G\approx 1\)。

### 5.3 非凸情形

真实 Hessian 可能 indefinite。实现同时记录：

- \(\lambda_{\min},\lambda_{\max}\)
- 负特征值个数 \(n_{\mathrm{neg}}\)
- 最负特征值
- 正谱条件数

\[
\kappa_+
=
\frac{\lambda_{\max}^+}{\lambda_{\min}^+},
\qquad
|\lambda|<10^{-10}\lambda_{\max}\text{ 视为零}.
\]

若 \(H\succ 0\) 用 \(\kappa\)；否则用 \(\kappa_+\)。这样把各向异性与非凸性分开。本轮所有 mixed case 实际 \(n_{\mathrm{neg}}=0\)（snap 主导），\(\kappa_+\) 通路已接通但未触发 indefinite 分支。

### 5.4 Metric 候选

| 记号 | 定义 | 几何含义 |
|---|---|---|
| \(I\) | 单位阵 | 参数欧式 |
| \(L^2\) | \(\int\|\delta p\|^2\,dt\) | 位置 Sobolev |
| \(H^2\) | \(\int\|\delta\ddot p\|^2\,dt\) | 加速度 Sobolev |
| MCE | \(2J_P^TQJ_P\) | snap / minimum-control |
| Mix | \(G_{L^2}+G_{\mathrm{MCE}}\) | 位置 + 控制 |
| MCE+GN | \(G_{\mathrm{MCE}}+J_r^TWJ_r\) | 控制 + 走廊 Gauss-Newton |

前一阶段已证明 \(L^2\) 与 snap 都能恢复坐标不变性。本阶段比较的是：**谁对真实 Hessian whitening 更有效**。

### 5.5 Frozen-metric L-BFGS

一次 solve 开始时取 \(G_0=G(x_0)\)，分解 \(G_0=LL^T\)，全程优化 \(z=L^Tx\)。优点：metric 只构造一次、L-BFGS history 在固定坐标中一致、\(\kappa_G\) 可直接解释、适合作为第一版 production candidate。Dynamic metric 按框架 §23 放到最后。

---

## 6. 实验顺序与完成状态

框架 §31 的九步：

| # | 内容 | 状态 |
|---|---|---|
| 1 | Pure MCE：\(H\approx G_{\mathrm{MCE}}\)，whiten 后 \(\kappa\approx 1\) | **完成** |
| 1b | 不同 segment-time 比例 | **完成** |
| 2 | MCE + \(L^2\) tracking，对比 \(I/L^2/H^2/\mathrm{MCE}/\mathrm{Mix}\) | **完成** |
| 3 | MCE + vel/acc dynamics | **完成** |
| 4 | MCE + corridor penalty | **完成** |
| 5 | 约束 Gauss-Newton metric 恢复 conditioning | **完成** |
| 6 | Frozen-metric L-BFGS，关联 \(\kappa\) 与迭代/线搜索 | **完成** |
| 7 | General-Planner State2State 全节点 runtime | **未做**（隔离 Planner-like proxy 已做） |
| 8 | Free-time，把 metric 扩到 \((P,T)\) | **未做** |
| 9 | Dynamic metric | **未做** |

---

## 7. 第 1 组：Pure-MCE Hessian Validation

MINCO-S4，均匀时间 \(T_i=1\)，\(J=E_{\mathrm{snap}}\)。

| \(M\) | \(n=\dim P\) | \(e_H\) | \(\kappa_E\) | \(\kappa_{\mathrm{MCE}}\) | \(R_\kappa\) |
|---|---:|---:|---:|---:|---:|
| 3 | 6 | \(\sim 10^{-9}\) | \(7.14\) | \(1.0000\) | \(7.14\) |
| 5 | 12 | \(\sim 10^{-9}\) | \(3.20\times 10^{2}\) | \(1.0000\) | \(320\) |
| 10 | 27 | \(\sim 10^{-9}\) | \(6.46\times 10^{4}\) | \(1.0000\) | \(6.46\times 10^{4}\) |
| 20 | 57 | \(\sim 10^{-9}\) | \(1.58\times 10^{7}\) | \(1.0000\) | \(1.58\times 10^{7}\) |

数值 Hessian 与解析 \(G_{\mathrm{MCE}}\) 一致到 \(e_H\sim 10^{-9}\)。Euclidean 条件数随 piece 数爆炸；MCE whitening 后始终 \(\kappa_G=1.000\)，与解析结论一致。对纯 MCE，\(R_\kappa\approx\kappa_E\)。

\(M=5\) 的谱（图 1）：欧式特征值从 \(\sim 71\) 跨到 \(\sim 2.3\times 10^{4}\)；MCE whitening 后 12 个特征值全部落在 \(1\pm 1.4\times 10^{-8}\)。

### Segment-time 比例（\(M=5\)）

| 模式 | 时长 | \(\kappa_E\) | \(\kappa_{\mathrm{MCE}}\) |
|---|---|---:|---:|
| uniform | \([1,1,1,1,1]\) | \(319\) | \(1.0000\) |
| mild | \([0.7,1.0,1.3,0.9,1.1]\) | \(807\) | \(1.0000\) |
| strong | \([0.3,0.6,1.2,1.8,1.1]\) | \(9.88\times 10^{4}\) | \(1.0000\) |
| extreme | \([0.2,0.4,1.5,2.0,0.9]\) | \(1.36\times 10^{6}\) | \(1.0000\) |

不均匀时间把 Euclidean Hessian 再拧三个数量级；MCE 仍然精确 whitening。这解释了为什么自由时间规划里“短段 / 长段混在一起”时欧式 L-BFGS 特别难：问题不在 objective，而在 \(G=I\) 把不同时间尺度的控制能量塞进同一参数欧式几何。

---

## 8. 第 2–5 组：Mixed objective 与 metric 对比

全部在 \(M=5\)、均匀时间、同一初值 \(P_0\) 上。\(\kappa_E\) 是该 case 的 Euclidean Hessian 条件数（与 metric 无关）；\(\kappa_G\) 是对应 metric 的 whitened 条件数。

### 8.1 MCE + \(L^2\) tracking

\(J=E+\lambda\int\|p-p_{\mathrm{ref}}\|^2\,dt\)，\(\lambda=8\)。\(\kappa_E=288\)，\(n_{\mathrm{neg}}=0\)。

| Metric | \(\kappa_G\) | \(R_\kappa\) | Iter | Eval | \(J^\star\) |
|---|---:|---:|---:|---:|---:|
| \(I\) | \(288\) | \(1.00\) | 20 | 90 | \(40.816\) |
| \(L^2\) | \(514\) | **\(0.56\)** | 18 | 91 | \(40.816\) |
| \(H^2\) | \(12.7\) | \(22.7\) | 11 | 14 | \(40.816\) |
| MCE | \(1.11\) | \(259\) | **5** | **11** | \(40.816\) |
| Mix | \(1.10\) | \(262\) | **5** | **11** | \(40.816\) |

Whitening 后 MCE 谱从 \(\approx 1.000\) 微升到 \(\approx 1.112\)，正好对应 \(\rho_E I + G_{\mathrm{MCE}}^{-1/2}H_{L^2}G_{\mathrm{MCE}}^{-1/2}\) 的残差。

### 8.2 MCE + dynamics（vel/acc）

\(w_v=0.4\)，\(w_a=0.15\)。\(\kappa_E=317\)。

| Metric | \(\kappa_G\) | \(R_\kappa\) | Iter | Eval |
|---|---:|---:|---:|---:|
| \(I\) | \(317\) | \(1.00\) | 20 | 28 |
| \(L^2\) | \(567\) | \(0.56\) | 21 | 91 |
| \(H^2\) | \(13.9\) | \(22.9\) | 12 | 19 |
| MCE | \(1.008\) | \(315\) | **3** | **9** |
| Mix | \(1.008\) | \(315\) | **3** | **9** |

动力学二次项与 snap 同族（都是系数空间质量矩阵），MCE 几乎仍是精确 Newton。

### 8.3 MCE + corridor，以及 Gauss-Newton 恢复

走廊 \(y_{\max}=0.08\)，采样 8/piece，二次 penalty。\(\kappa_E=320\)。

| Metric | \(\kappa_G\) | \(R_\kappa\) | Iter | Eval |
|---|---:|---:|---:|---:|
| \(I\) | \(320\) | \(1.00\) | 21 | 25 |
| \(L^2\) | \(572\) | \(0.56\) | 22 | 30 |
| \(H^2\) | \(14.0\) | \(22.9\) | 14 | 17 |
| MCE | \(1.160\) | \(276\) | 4 | 10 |
| Mix | \(1.160\) | \(276\) | 4 | 10 |
| **MCE+GN** | **\(1.000\)** | **\(320\)** | 4 | 10 |

纯 MCE 在走廊残差下从 \(1.000\) 退化到 \(1.160\)。把走廊 Gauss-Newton \(G_c=J_r^TWJ_r\) 加进度量后，\(\kappa_G\) 回到 \(1.000\)。这正是框架 §14 的预测：penalty 更适合 GN metric，而不是继续只用 snap。

本实例中走廊残差相对 snap 仍小，所以 MCE 单独已经把迭代从 21 降到 4；GN 主要修谱，迭代不再继续下降。残差更大时应预期 GN 同时修 \(\kappa\) 和迭代。

### 8.4 Planner-like proxy

\(J=E+\lambda_{\mathrm{track}}\int\|p-p_{\mathrm{ref}}\|^2+w_v\|v\|^2+w_a\|a\|^2+J_{\mathrm{corridor}}\)。这是隔离的 State2State 形状，不是 ROS 节点。\(\kappa_E=294\)，\(J^\star=39.903\)。

| Metric | \(\kappa_G\) | \(R_\kappa\) | Iter | Eval | \(t_{\mathrm{opt}}\) (s) | \(t_{\mathrm{metric}}\) (s) |
|---|---:|---:|---:|---:|---:|---:|
| \(I\) | \(294\) | \(1.00\) | 21 | 25 | \(4.0\times 10^{-4}\) | \(3.8\times 10^{-4}\) |
| \(L^2\) | \(525\) | \(0.56\) | 20 | 91 | \(1.4\times 10^{-3}\) | \(3.8\times 10^{-4}\) |
| \(H^2\) | \(12.9\) | \(22.7\) | 15 | 18 | \(2.8\times 10^{-4}\) | \(3.8\times 10^{-4}\) |
| MCE | \(1.25\) | \(236\) | **5** | **11** | \(1.9\times 10^{-4}\) | \(3.8\times 10^{-4}\) |
| Mix | \(1.23\) | \(239\) | **5** | **11** | \(1.9\times 10^{-4}\) | \(3.8\times 10^{-4}\) |
| MCE+GN | \(1.09\) | \(270\) | **5** | **11** | \(1.9\times 10^{-4}\) | \(3.8\times 10^{-4}\) |

同一终点。MCE / Mix / MCE+GN 把迭代从 21 降到 5。\(L^2\) 迭代几乎不降，线搜索评估反而从 25 升到 91：错误的 intrinsic metric 可以比 \(G=I\) 更差。

### 8.5 非凸探针：MCE + cosine

\(J=E+\kappa\int\cos(\omega p_x)\,dt\)，\(\kappa=2.5\)，\(\omega=5\)。用来接通 \(\kappa_+\) 通路。

实测 \(n_{\mathrm{neg}}=0\)：snap Hessian 的最小特征值约 \(71\)，cosine 扰动不足以让谱穿越零。仍记录 \(\kappa_+\)；本实例 \(\kappa_+=\kappa_E=320\)。

| Metric | \(\kappa_G\) | Iter | Eval | \(J^\star\) |
|---|---:|---:|---:|---:|
| \(I\) | \(320\) | 29 | 35 | \(36.292\) |
| \(L^2\) | \(572\) | 35 | 47 | \(36.292\) |
| \(H^2\) | \(14.0\) | 17 | 22 | \(36.292\) |
| MCE | \(1.15\) | 5 | 11 | \(36.292\) |
| Mix | \(1.15\) | **4** | **10** | \(36.292\) |

同一盆地、同一 \(J^\star\)。非凸项增加了欧式迭代（20→29），MCE 仍 4–5 步。**不能**从本实例外推“障碍问题 Hessian 总是 SPD”；只能说当前权重下 snap 压过了 cosine，\(\kappa_+\) 工具已就位。

---

## 9. Sobolev / MCE / \(L^2\) 对比的结论

坐标不变性实验已经证明 \(G_{L^2}\) 和 \(G_{\mathrm{snap}}\) 都能恢复 \(\delta p(t)\) 不变性。Conditioning 实验给出另一条排序：

\[
R_\kappa(L^2)\approx 0.56
\;<\;
R_\kappa(I)=1
\;<\;
R_\kappa(H^2)\approx 23
\;<\;
R_\kappa(\mathrm{MCE})\approx 250\text{–}320
\;\approx\;
R_\kappa(\mathrm{Mix}).
\]

原因直接来自 Hessian 结构。本轮 mixed \(J\) 的主导二次项仍是 snap energy，所以

- \(G_{L^2}\) 度量的是位置变形，和 \(H_J\) 的主方向不对齐，whitening **恶化**谱；
- \(G_{H^2}\) 介于位置与 snap 之间，有中等预条件作用；
- \(G_{\mathrm{MCE}}\) 对齐主导 Hessian，接近 Newton；
- \(G_{\mathrm{mix}}=G_{L^2}+G_{\mathrm{MCE}}\) 在本轮几乎等于 MCE（\(L^2\) 相对权重小），Planner-like 上 \(\kappa_G\) 仅从 \(1.25\) 降到 \(1.23\)。

框架 §16 猜想“复合 Sobolev 可能同时兼顾位置障碍和控制 effort”。本轮走廊残差还不够大，Mix 相对纯 MCE 没有实质优势。该猜想要等到障碍 / 走廊主导 Hessian 时才能判决；目前只能说：**能恢复不变性的 metric 不必是好的预条件。**

---

## 10. L-BFGS 迭代关联

最重要的相关性是 \(\kappa_G\downarrow\Longrightarrow N_{\mathrm{iter}},N_{\mathrm{LS}}\downarrow\)。

### 10.1 纯 MCE，\(M=5\)

| Metric | \(\kappa_G\) | Iter | Eval | \(J^\star\) |
|---|---:|---:|---:|---:|
| Euclidean \(I\) | \(320\) | 19 | 28 | \(32.256\) |
| MCE | \(1.000\) | **2** | **8** | \(32.256\) |

解析上这是精确 Newton：whitened 空间里 \(E=\frac12\|z\|^2\)，L-BFGS 两步收敛（含有限差分 / 线搜索开销）。同一 \(J^\star\)。

### 10.2 Mixed cases 汇总

所有 mixed case 最终 \(J^\star\) 与 metric 无关（同一凸/同一盆地）。迭代随 \(\kappa_G\) 单调下降：

- \(\kappa_G\sim 300\)（\(I\)）：19–29 步；
- \(\kappa_G\sim 13\)（\(H^2\)）：11–17 步；
- \(\kappa_G\sim 1.0\)–\(1.25\)（MCE / Mix / MCE+GN）：3–5 步；
- \(\kappa_G\sim 520\)（\(L^2\)）：18–35 步，线搜索评估往往更多。

图 2（\(\kappa_G\) vs \(N_{\mathrm{iter}}\)）呈正相关。这不是“更小 condition number 保证更低最终 objective”——本轮 \(J^\star\) 全部相同——而是 **同一终点上的局部二次效率**。

### 10.3 Metric 构造成本

Planner-like 上 \(t_{\mathrm{metric}}\approx 0.38\,\mathrm{ms}\)（组装全部 JVP 基与 \(L^2/H^2/\mathrm{snap}\) 矩阵），欧式 \(t_{\mathrm{opt}}\approx 0.40\,\mathrm{ms}\)，MCE \(t_{\mathrm{opt}}\approx 0.19\,\mathrm{ms}\)。\(n=12\) 时 metric 成本与一次短 solve 同量级；\(M=20\)（\(n=57\)）时 \(t_{\mathrm{metric}}\approx 31\,\mathrm{ms}\)，会开始压过迭代节省。

框架 §21 的工程判据

\[
T_{\mathrm{total}}
=
T_{\mathrm{objective}}+T_{\mathrm{line-search}}+T_{\mathrm{metric}}
\]

在本隔离测试里 **尚未**对真实 State2State 成立或失败下结论。当前只能说：frozen metric 把构造摊到一次 solve 开头是合理的第一版；是否实时划算取决于 onboard 的 \(n\)、采样和 objective 成本。

---

## 11. General-Planner 实际 benchmark：当前做到哪一步

框架 §20 要求在 isolated test 之后进入真实 planner：固定走廊、goal、初始化、Bezier/MINVO、penalty、采样、停机规则，只改 metric，比较

1. Euclidean \(G=I\)
2. Frozen \(G_{\mathrm{MCE}}\)
3. Frozen \(G_{L^2}\)
4. Frozen composite Sobolev
5. Constraint-aware GN

并记录 \(\kappa_G\)、迭代、线搜索、\(t_{\mathrm{metric}}\)、\(t_{\mathrm{total}}\)、\(J_{\mathrm{final}}\)、\(\max\) violation。

**本轮没有接 ROS State2State 节点。** Planner-like 只是把 energy + tracking + vel/acc + corridor 放进同一隔离 MINCO 问题，用来定位 residual 来源：

- 跟踪把 \(\kappa_{\mathrm{MCE}}\) 从 1.00 抬到 1.11；
- 动力学几乎不抬（1.008）；
- 走廊抬到 1.16，GN 拉回 1.00；
- 四者叠加抬到 1.25，GN 收到 1.09。

这完成了框架 §11 “逐级增加、定位哪类 residual 恶化 conditioning”，但 **不等于** onboard benchmark。缺的是：真实走廊几何、时间正则、guide cost、Bezier 表示、以及包含 metric 构造的 wall-clock。这些对应顺序 7–9，需要单独接线，不能从本测试外推“规划器一定少 4 倍迭代”。

---

## 12. 图

由 `Tests/plot_mce_metric_conditioning.py` 生成。

- `mce_conditioning_overview.png`
  - 图 1：Pure-MCE \(M=5\) 特征值谱（\(I\) vs MCE）
  - 图 2：\(\kappa_G\) vs L-BFGS 迭代
  - 图 3：\(\kappa\) vs piece 数 \(M=3,5,10,20\)
  - 图 4：segment-time 比例（uniform / mild / strong / extreme）
- `mce_conditioning_cost_curves.png`
  - 图 5：Planner-like frozen-metric \(J_k\) 曲线

原始数字：

- `mce_conditioning_table.csv`
- `mce_conditioning_spectrum.csv`
- `mce_conditioning_cost_traces.csv`
- `mce_conditioning_scaling.csv`

---

## 13. 实现要点（对应框架 §25–§26）

测试文件：`Tests/minco_metric_conditioning_self_test.cpp`。

1. MINCO JVP 基构造 \(G_{L^2},G_{H^2},G_{\mathrm{snap}}\)；\(G_{\mathrm{MCE}}=2G_{\mathrm{snap}}\) 对齐 `getEnergy()`。
2. 中心差分 Hessian，\(\eta=10^{-6}\)，对称化。
3. Whitening：`LLT` + 下三角 solve，不显式 `inverse()`。
4. Metric 近奇异时 \(G_\epsilon=G+\epsilon I\)，\(\epsilon\) 写入日志；本轮 SPD metric 实际 \(\epsilon=0\)。
5. Frozen L-BFGS：物理梯度 \(g_P\) 变成 \(g_z=L^{-1}g_P\)，变量 \(z=L^TP\)。
6. 走廊 GN：\(G_c=\sum w\,(\partial r/\partial P)^T(\partial r/\partial P)\)，不含 \(\sum r_i\nabla^2 r_i\)。

CMakeLists.txt 在本仓库中不可写，测试按既有方式独立编译：

```text
g++ -O2 -std=c++17 -DROOT_DIR='"<general_planner>/"' \
  -I<general_planner>/include -I/usr/include/eigen3 \
  Tests/minco_metric_conditioning_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_metric_conditioning_self_test
```

---

## 14. 当前可以声称的结论

### 理论

固定时间 minimum-control：

\[
G_{\mathrm{MCE}}=H_{E,\mathrm{reduced}},
\qquad
\kappa\!\bigl(G_{\mathrm{MCE}}^{-1/2}H_EG_{\mathrm{MCE}}^{-1/2}\bigr)=1.
\]

数值 Hessian 以 \(e_H\sim 10^{-9}\) 确认这一点，并在 \(M=3\ldots 20\) 与极端时间比例下保持 \(\kappa_G=1.000\)。

### 数值

对 mixed objective，MCE / Mix / MCE+GN 满足 \(\kappa_G\ll\kappa_E\)（\(R_\kappa\sim 10^2\)）。\(H^2\) 中等改善。\(L^2\) 在 snap 主导时恶化 conditioning。L-BFGS 迭代与线搜索随 \(\kappa_G\) 下降而下降，且到达同一 \(J^\star\)。

走廊残差使纯 MCE 从 \(\kappa_G=1\) 退化到 \(1.16\)；加上约束 GN 后回到 \(1.00\)。

### 与不变性实验合在一起

\[
\text{geometric consistency}
+
\text{numerical conditioning advantage}
\]

对 **MCE（以及接近它的 Mix / MCE+GN）** 成立；对 **\(L^2\)** 只成立前半句。

---

## 15. 当前不能提前宣称的内容（框架 §30）

即便本轮 43/43 通过，也不能宣称：

1. MCE 是唯一最佳 metric；
2. MCE 在所有 obstacle problem 中都优于 \(L^2\)（本轮 Hessian 仍是 snap 主导）；
3. dynamic metric 一定优于 frozen metric（未测）；
4. 更小 condition number 保证更低最终 objective（本轮 \(J^\star\) 相同）；
5. natural gradient 保证全局最优；
6. metric-aware optimizer 在所有场景都比任意 Euclidean parameterization 更快；
7. General-Planner onboard runtime 一定更好（顺序 7 未做）；
8. 自由时间 \((P,T)\) 上同样 \(\kappa_G\approx 1\)（顺序 8 未做）。

更准确的结论：

\[
\text{MCE metric 提供了一个从轨迹控制结构推导出的、
坐标一致且在固定时间 MINCO 上显著改善 conditioning 的候选几何。}
\]

\(L^2\) 是好的不变性度量，但不是本问题的好预条件。Mix 是否在障碍主导时超过纯 MCE，仍待顺序 7。

---

## 16. 下一步（顺序 7–9）

1. **State2State 全节点**：在 `planner_runtime` / `interface_state2state_corridor` 上只替换 metric，记录 \(\kappa_G\)、violation、wall-clock（含 \(T_{\mathrm{metric}}\)）。
2. **Free-time**：把 \(G\) 扩到 \((P,T)\)，检查时间尺度是否再次把 \(\kappa_E\) 拧坏、MCE 是否仍能 whitening 空间部分。
3. **Dynamic metric**：仅在 frozen baseline 稳定之后；需要 tangent transport / Riemannian secant，不适合作为第一版 production。
4. **更强非凸**：加大走廊权重或接入真实障碍，直到 \(n_{\mathrm{neg}}>0\)，用 \(\kappa_+\) 而不是假装 SPD。
5. **Mix 权重扫描**：当 \(H_{\mathrm{other}}\) 真正主导时，再判断 \(\lambda_0 G_{L^2}+\lambda_4 G_{H^4}\) 是否优于纯 snap。

---

## 17. 研究主线（到目前为止）

```text
轨迹是坐标无关对象
        ↓
不同 basis / reduced coordinates 只是表示
        ↓
Euclidean G=I 在不同坐标中不是同一几何
        ↓
Euclidean gradient 的物理轨迹方向坐标依赖     ← 阶段 1，已验证
        ↓
pullback metric 恢复方向不变性                 ← 阶段 1，已验证
        ↓
选择 MCE / Sobolev trajectory metric
        ↓
研究 whitened Hessian
        ↓
验证 condition number 下降                     ← 本阶段，已验证
        ↓
验证 iteration / line-search 同步下降           ← 本阶段，隔离 MINCO 已验证
        ↓
真实 State2State runtime（含 metric 成本）     ← 未做
        ↓
形成 metric-aware trajectory optimization
```
