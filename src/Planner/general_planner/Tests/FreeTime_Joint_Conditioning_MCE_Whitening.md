# 自由时间联合优化中 Frozen MCE 白化失效：现象、条件数与处理

- **日期**：2026-08-20
- **问题**：固定时间隔离实验里 Frozen MCE 有 5–24× 迭代加速；接到自由时间 planning/replan 后加速消失，条件数反而更差。
- **策略文档**：`General_Planner_MCE_Metric_Optimization_Strategy.md`（Production V1，§15 规定时间不白化）
- **前置报告**：
  - `Tests/MCE_Metric_Conditioning_Experimental_Report.md`（固定时间条件数）
  - `Tests/Frozen_MCE_Whitening_Experimental_Report.md`（固定时间纯能量 / 联合目标 L-BFGS）
- **本报告自测**：
  - `Tests/minco_state2state_rolling_replan_self_test.cpp`（自由时间滚动 replan A/B）
  - `Tests/minco_freetime_joint_conditioning_self_test.cpp`（联合 Hessian 条件数）
- **实现**：`minco_metric.hpp`、`minco_whitening.hpp`、`minco_optimizer.hpp`、`traj_manager.cpp`
- **生产开关**：`minco_metric_mode: 1` 为 Frozen MCE；默认保持 `0`（欧式）
- **后续**：`Tests/Full_SpaceTime_MINCO_Joint_Optimization_Experimental_Report.md`（完整时空度量 C0–C4 / D/E/F）

---

## 0. 一句话结论

固定时间下，MCE 白化把 waypoint Hessian 变成 \(\kappa\approx 1\)，所以快。自由时间下决策是耦合的 \((\tau,P)\)，V1 **只白化 \(P\)、时间仍走 `QuadInvTimeMap`**。\(P\) 块确实变好，但联合 Hessian 的 \(\lvert\lambda_{\min}\rvert\) 塌缩，整体 \(\kappa\) 变差约 **\(80\sim 10^3\) 倍**（几何平均 **356**）。白化作为坐标变换仍然正确（全收敛时 \(J\) 对齐），因此这不是 \(L\) 算错，而是部分预条件在强耦合块上的必然结果。生产 Fast L-BFGS 再把 `rel_step`/`delta` 作用在 \(z\) 上，会在更差点停机——那是第二层设置问题，不要和 \(\kappa\) 变差混为一谈。

**现在不要把 `minco_metric_mode: 1` 设成默认。**

---

## 1. 现象综述

### 1.1 两套实验不是同一个问题

| | 隔离实验 | 滚动 replan / 生产求解图 |
|---|---|---|
| 时间 | **固定 \(T\)** | **自由 \(\tau\)**（`QuadInvTimeMap`） |
| 决策 | 只有 \(P\) | \(x=(\tau,P)\) 或 \((\tau,z_P)\) |
| 求解器 | 经典 L-BFGS，\(g_\epsilon\sim 10^{-7}\) | 生产 Fast L-BFGS，约 10 步后相对停机 |
| MCE 做什么 | 白化整个决策空间 | **只白化 waypoint**，\(\tau\) 保持欧式 |
| 欧式要付的代价 | 走到紧平稳点，\(\kappa(H_P)\sim 10^2\sim 10^5\) | Fast L-BFGS 在 ~55 步就停，从不付这笔税 |
| 观测 | MCE 5–24× 更少迭代，\(J\) 相同 | \(J\) 可对齐但更慢；Fast 停机时质量更差 |

隔离加速来自「固定 \(T\) 下能量 Hessian 的精确白化」。生产路径同时换了两根轴：是否自由时间、是否紧收敛。把隔离的 5–24× 当成 click_demo 的预期，本身就不成立。

### 1.2 三层现象必须拆开

```text
层 A  几何：只白化 P 之后，联合 κ(H_{τ,z}) 比 κ(H_{τ,P}) 差 10²～10³
层 B  求解：全收敛时 J 相同，但迭代 264 → 326，墙钟慢 32%
层 C  停机：生产 Fast L-BFGS 在 z 尺度上判稳，J 225 → 632，走廊 viol 更差
```

- A 解释 B：条件数变差，紧收敛不会更快。
- C 是额外的尺度错误，不改变 A。打开 phase-0 物理 guard 能减轻 C，但墙钟变成欧式的 2.4 倍，因为这时求解器被迫在更差的 \(\tilde H\) 上把物理 \(P,T\) 走完。

### 1.3 这不是实现错误

下列事实同时成立：

1. \(G_{\mathrm{MCE}}=2J_P^\top Q J_P\) 与 `getEnergy()` 的 reduced Hessian 对齐（\(e_H=2.2\times 10^{-9}\)）。
2. 固定时间纯 snap：白化后 \(\kappa=1\)，2 次迭代，终值与单步 Newton 相同。
3. 自由时间全收敛：欧式 \(J=259.36\)，MCE \(J=259.74\)（比值 1.001）。坐标变换没有改最优点。
4. 种子梯度有限差分：欧式 \(7.1\times 10^{-7}\)，MCE \(1.7\times 10^{-5}\)。
5. 时间 Schur \(S_\tau=H_{\tau\tau}-H_{\tau P}H_{PP}^{-1}H_{P\tau}\) 在白化前后**逐元素不变**。

所以 \(L\)、encode \(z=L^\top(P-P_0)\)、covector \(g_z=L^{-1}g_P\) 都是对的。坏的是「只预处理一块」这件事本身。

---

## 2. 策略 V1 实际做了什么

文档 §15 写明：条件数实验只严格支持 \(T=\mathrm{fixed}\)，因此 V1 的求解坐标是

\[
x_{\mathrm{solver}}
=
\begin{bmatrix}
\tau \\
z_P
\end{bmatrix},
\qquad
g_{\mathrm{solver}}
=
\begin{bmatrix}
g_\tau \\
L^{-1}g_P
\end{bmatrix}.
\]

时间保持 `QuadInvTimeMap` 和原有 \(\partial J/\partial\tau\)，**不用 snap waypoint Hessian 去白化 \(T\)**。冻结度量是

\[
G_0=\rho_E G_{\mathrm{MCE}}(T_0),\qquad
G_{\mathrm{MCE}}=2J_P^\top Q J_P=H_{E,P}(T_0).
\]

这是有意的分阶段：先把已经验证的固定时间结果接到线上，时空度量留到 V2/V3。隐含假设是「\(P\) 块变好之后，联合问题至少不会变差」。自由时间 Hessian 测量把这个假设证伪了。

生产代码与文档一致：`traj_manager.cpp` 在 `minco_metric_optimize_time: true`（yaml 默认）时保留 \(\tau\) 块，只对空间块做 Frozen whitening。

---

## 3. 对照实验设定

滚动 replan 与条件数实验共用同一组 remaining-horizon 快照：

- MINCO-S4，\(M=8\) 段，内点 \(21\) 维，加上 \(8\) 维 \(\tau\)
- 目标：能量 \(\rho_E=1\) + 线性时间权 \(20\) + vel \(0.25\) + acc \(0.08\) + 走廊 \(25\)
- 起点沿 \(x\) 推进，终点固定在 \((70,0,1.5)\)，内点带横向偏移使走廊在 \(t=0\) 激活
- 时间自由，`QuadInvTimeMap`
- 只白化 waypoint：\(z=L^\top(P-P_0)\)，\(LL^\top=G_{\mathrm{MCE}}(T_0)\)

这比线上 `penna_t=2\times 10^5`、`penna_pos=5\times 10^7` **更有利于**能量白化。即便如此联合 \(\kappa\) 已经变差；线上能量更不占主导时，V1 只会更吃亏。

---

## 4. 固定时间基线（隔离，作为对照）

目标 \(J=\rho_E E_{\mathrm{snap}}+J_{\mathrm{track}}+J_{\mathrm{vel}}+J_{\mathrm{acc}}+J_{\mathrm{corridor}}\)，**\(T\) 固定**。

| 场景 | \(\kappa_I\) | \(\kappa_{\mathrm{MCE}}\) | 欧式 | Frozen MCE |
|---|---:|---:|---:|---:|
| M=5 均匀 | \(2.94\times 10^2\) | **1.25** | 28 it，线搜失败 | **5 it** |
| M=10 均匀 | \(2.85\times 10^3\) | 66.7 | 188 it | **21 it** |
| M=5 时间很不均匀 | \(9.10\times 10^4\) | **1.43** | 120 it，线搜失败 | **5 it** |

纯 snap 更极端：欧式 21 / 91 步，MCE **2 步**，\(\kappa=1\)。终值 \(J\) 与欧式相同。

这里快，是因为决策空间就是 \(P\)，而 \(G_{\mathrm{MCE}}\) 就是这块的能量 Hessian。

---

## 5. 自由时间滚动 replan 结果

### 5.1 全收敛（经典 L-BFGS，\(g_\epsilon=0\)，\(\delta=10^{-12}\)，最多 400 步）

| | 欧式 | Frozen MCE | 比 |
|---|---:|---:|---:|
| 终值 \(J\) | 259.36 | 259.74 | **1.001×** |
| 走廊 viol | 0.014 | 0.027 | 同量级 |
| 迭代 / 评估 | 264 / 301 | 326 / 383 | 1.23× / 1.27× |
| L-BFGS ms/call | 3.47 | 4.57 | **1.32× 更慢** |
| 度量 ms/call | 0 | 0.039 | 可忽略 |

\(J\) 对齐 ⇒ 最优点没变。迭代变多 ⇒ 搜索图变差。度量构造不是瓶颈。

### 5.2 生产 Fast L-BFGS（yaml：`min_iterations=10`，`rel_cost=1e-3`，`phase0_guards=false`）

| | 欧式 | Frozen MCE | 比 |
|---|---:|---:|---:|
| L-BFGS ms/call | 0.70 | 0.88 | **1.26× 更慢** |
| 迭代 / 评估 | 55 / 58 | 63 / 70 | 1.16× |
| 终值 \(J\) | 225 | 632 | **2.81× 更差** |
| 走廊 viol | 0.022 | 1.03 | 明显更差 |
| fast-stop | 100% | 100% | 在更差点停下 |

欧式 Fast L-BFGS 已经在 ~55 步、0.70 ms 停住，质量可接受。隔离里欧式 188 步那种病态税，线上根本没在交，所以「从 188 砍到 21」没有墙钟空间。

MCE 的 `rel_step`/`delta` 作用在 \(x=(\tau,z)\) 上。\(z\) 从 0 出发，单位是能量曲率不是米。能量两步掉下去之后相对 \(\Delta J/J\) 很快小于 `delta`，走廊还没解开就停。打开 phase-0 物理 \(P,T\) guard 后 \(J\) 从 632 回到约 335，但迭代变成欧式的 **2.2 倍**，墙钟 **2.4 倍**。

`lbfgs_fast_phase0_guards_en: false` 对欧式合理（大走廊权重下原始梯度尺度敏感）。对白化图，这个开关关掉了唯一还连着物理 \(P\) 的停机量。

---

## 6. 联合 Hessian：条件数如何变差

在滚动实验同一组快照的种子处，对求解坐标做中心差分 Hessian。决策维数：\(\tau\in\mathbb{R}^8\)，\(P\in\mathbb{R}^{21}\)。\(\kappa\) 用 \(\lvert\lambda\rvert_{\max}/\lvert\lambda\rvert_{\min}\)（种子处 Hessian 不定，约 7 个负特征值）。

### 6.1 总表

| 快照 | \(\kappa(H_{\tau,P})\) | \(\kappa(H_{\tau,z})\) | 比 | \(\kappa(H_{PP})\) | \(\kappa(H_{zz})\) | \(\kappa(H_{\tau\tau})\) |
|---|---:|---:|---:|---:|---:|---:|
| 0 | \(4.38\times 10^6\) | \(3.53\times 10^8\) | **81×** | \(1.04\times 10^4\) | **6.82** | \(6.29\times 10^2\) |
| 4 | \(3.20\times 10^6\) | \(1.80\times 10^9\) | **563×** | \(1.10\times 10^4\) | **5.81** | \(6.84\times 10^2\) |
| 9 | \(1.71\times 10^6\) | \(1.70\times 10^9\) | **993×** | \(1.13\times 10^4\) | **1.35** | \(6.93\times 10^2\) |

几何平均：\(\kappa(H_{\tau,z})/\kappa(H_{\tau,P})\approx 356\)。

### 6.2 分块上同时成立的三件事

**Waypoint 块被修好了（隔离实验）。**

\[
\kappa(H_{PP})\sim 10^4
\quad\longrightarrow\quad
\kappa(H_{zz})=\kappa(L^{-1}H_{PP}L^{-T})\sim 1\sim 7.
\]

快照 9 上 \(H_{zz}\) 的 \(\kappa=1.35\)，已经接近单位二次型。

**时间块完全没动。**

\(\kappa(H_{\tau\tau})\) 两套坐标相同（\(6.3\times 10^2\sim 6.9\times 10^2\)）。消元后

\[
S_\tau
=
H_{\tau\tau}-H_{\tau P}H_{PP}^{-1}H_{P\tau}
\]

也逐元素不变（\(\kappa(S_\tau)\)、\(\lambda_{\min}\)、\(\lambda_{\max}\) 一致）。只白化 \(P\)，不改变「把 \(P\) 消掉之后时间有多难」。

**整体 \(\kappa\) 变差来自 \(\lvert\lambda_{\min}\rvert\) 塌缩，不是 \(\lambda_{\max}\) 变大。**

| 快照 | 欧式 \(\lvert\lambda_{\min}\rvert\) | 白化 \(\lvert\lambda_{\min}\rvert\) | 欧式 \(\lambda_{\max}\) | 白化 \(\lambda_{\max}\) |
|---|---:|---:|---:|---:|
| 0 | \(0.975\)（\(\approx\lambda_{\min}(H_{PP})\)） | \(1.21\times 10^{-2}\) | \(4.27\times 10^6\) | \(4.26\times 10^6\) |
| 4 | \(3.12\) | \(5.52\times 10^{-3}\) | \(9.99\times 10^6\) | \(9.95\times 10^6\) |
| 9 | \(20.4\) | \(2.04\times 10^{-2}\) | \(3.49\times 10^7\) | \(3.46\times 10^7\) |

\(\lambda_{\max}\) 始终由 \(H_{\tau\tau}\) 占着。欧式 \(\lvert\lambda_{\min}\rvert\) 几乎就是软 waypoint 模态；白化把它们抬到 \(O(1)\) 之后，新的近零特征值是 \((\tau,z)\) 的混合模态。

### 6.3 交叉项有多强

归一化耦合

\[
\eta
=
\frac{\|H_{\tau P}\|_F}{\sqrt{\|H_{\tau\tau}\|_F\,\|H_{PP}\|_F}}.
\]

| 快照 | \(\eta\) 欧式 | \(\eta\) 白化后 | \(\|H_{\tau P}\|_F\) | \(\|H_{\tau z}\|_F\) |
|---|---:|---:|---:|---:|
| 0 | 0.742 | 0.772 | \(2.32\times 10^5\) | \(4.57\times 10^3\) |
| 4 | 0.751 | 0.823 | \(6.60\times 10^5\) | \(7.07\times 10^3\) |
| 9 | 0.762 | **1.048** | \(3.23\times 10^6\) | \(1.34\times 10^4\) |

\(\eta\approx 0.75\) 已经是强耦合。白化后 Frobenius 范数 \(\|H_{\tau z}\|_F=\|H_{\tau P}L^{-T}\|_F\) 变小（\(L\) 的大特征值把刚的 \(P\) 方向缩掉），但**归一化** \(\eta\) 不降，快照 9 甚至升到 1。MINCO 的 \(C=C(P,T)\)、\(Q=Q(T)\)，交叉项来自几何，不是数值噪声。

---

## 7. 为什么部分白化会让联合 \(\kappa\) 更差

### 7.1 合同变换不保持特征值

\[
z=L^\top(P-P_0),
\qquad
\tilde H=A^\top H A,
\qquad
A=\operatorname{blkdiag}\bigl(I_\tau,\,L^{-T}\bigr).
\]

\(A\) 不正交。相似变换保持谱，合同变换只保持惯性（负惯性指数不变，实验里两边都是约 7 个负特征值）。条件数没有单调性。

分块写出

\[
H
=
\begin{bmatrix}
H_{\tau\tau} & H_{\tau P} \\
H_{P\tau} & H_{PP}
\end{bmatrix}
\quad\longrightarrow\quad
\tilde H
=
\begin{bmatrix}
H_{\tau\tau} & H_{\tau P}L^{-T} \\
L^{-1}H_{P\tau} & L^{-1}H_{PP}L^{-T}
\end{bmatrix}.
\]

能量主导时右下块 \(\approx I\)。左上块原样留下。交叉项变成 \(H_{\tau P}L^{-T}\)：刚的 \(P\) 方向被缩小，软的 \(P\) 方向被放大。软方向往往正是「整条路沿切向滑动、和 \(T\) 互换」的那些模态，所以交叉项在 \(z\) 图上相对更刺眼。

### 7.2 Schur 不变，整体 \(\kappa\) 仍可变

对 \(P\) 消元：

\[
S_\tau
=
H_{\tau\tau}-H_{\tau P}H_{PP}^{-1}H_{P\tau}.
\]

白化后

\[
\tilde S_\tau
=
H_{\tau\tau}
-
(H_{\tau P}L^{-T})
\,
(L^{-1}H_{PP}L^{-T})^{-1}
\,
(L^{-1}H_{P\tau})
=
S_\tau.
\]

实验逐元素验证了这一点。因此「时间有多难」没变。整体 \(\kappa(\tilde H)\) 却可以变差，因为它不是 \(\kappa(S_\tau)\cdot\kappa(H_{PP})\) 这种简单乘积，还取决于对角块之间的相对尺度。

### 7.3 二维模型

白化后对角尺度差大约 \(10^6\)：\(H_{\tau\tau}\) 的谱在 \(10^3\sim 10^7\)，\(H_{zz}\sim I\)。取模型

\[
\begin{bmatrix}
a & c \\
c & 1
\end{bmatrix},
\qquad
a\gg 1,
\qquad
c^2\sim \eta^2 a.
\]

小特征值大约是 \(1-c^2/a\sim 1-\eta^2\)。\(\eta\to 1\) 时这个值被压向 0。快照 9 的 \(\eta=1.05\) 已经超过这个粗模型的稳定区，对应实验里 \(\lvert\lambda_{\min}\rvert\) 从 20 掉到 \(2\times 10^{-2}\)。

欧式里 \(H_{PP}\) 的软模态（\(\lambda\sim 1\)）就是全矩阵的 \(\lambda_{\min}\)，时间块更刚，所以最小模态几乎是纯 \(P\)。白化把纯 \(P\) 软模态抬走之后，最小模态只能是 \(\tau\) 与 \(z\) 的混合近零方向——正是强耦合 + 对角尺度失衡制造出来的。

### 7.4 不定性

种子处 \(S_\tau\) 有约 7 个负特征值。联合问题在初值附近**不是局部凸的**。L-BFGS 仍能跑，但部分预条件在不定块上更容易制造近零方向。这解释了全收敛时 MCE 更容易打到 400 步上限（status `-1008`）。

### 7.5 和迭代的对应

```text
固定 T：L-BFGS 只看见 H_zz ~ I          →  2～21 步
自由 T：L-BFGS 看见 κ(H_{τ,z}) ~ 10^8–10^9
        比欧式 κ(H_{τ,P}) ~ 10^6 更差   →  264 → 326 步，墙钟慢 32%
```

隔离加速全部发生在「决策=P」的切片上。把同一把刀切到 \((\tau,P)\) 上，切到的是交叉项，不是病态的 \(P\) 块。

---

## 8. 哪些设置不合理

| 项 | 判断 |
|---|---|
| \(G_{\mathrm{MCE}}=2J^\top QJ\)、encode/decode、\(g_z=L^{-1}g_P\) | 合理。全收敛 \(J\) 对齐，FD 梯度通过 |
| 时间不白化 | 符合 V1 文档，但因此**不能预期**隔离加速出现在 replan 里 |
| Fast L-BFGS 的 `rel_step`/`delta` 直接作用在 \(z\) 上 | **不合理**。搜索方向可以在 \(z\) 里，停机必须用物理 \(P,T,J\) |
| `phase0_guards_en: false` 原样接到白化图 | **不合理**。欧式下为了避开梯度尺度；白化后它关掉了物理 waypoint 停机 |
| 用隔离 5–24× 当生产预期 | **不合理**。同时换了「是否自由时间」和「是否紧收敛」 |
| 滚动测试 `time=20, energy=1` | 比线上更偏向能量；线上 `penna_t=2\times 10^5` 会更差，不是测试过于苛刻 |
| 度量构造 ~0.04 ms | 不是瓶颈 |

---

## 9. 不应当推出的结论

1. **不是** Frozen MCE 实现错了。固定时间结果和自由时间 \(J\) 对齐都否定这一点。
2. **不是** 度量构造把时间吃掉了。
3. **不是** 任意混合目标固定时间都会 \(\kappa=1\)。走廊/动力学一旦和 snap 相当，仍需 MCE+GN。
4. **不是** 打开 `minco_metric_mode: 1` 就会在 click_demo 里降延迟。
5. **不是** 把 Fast L-BFGS 停机改成物理量之后，联合 \(\kappa\) 就会变好。那只修层 C；层 A 还在。

---

## 10. 建议的处理顺序

一次只动一根轴。

### 10.1 消融（确认诊断）

| 实验 | 设定 | 若成立则说明 |
|---|---|---|
| A | 同一快照，`setOptimizeTime(false)`，紧 \(g_\epsilon\) | 应找回隔离加速。否则实现坏了 |
| B | 自由时间，紧收敛（已做）+ 本报告的 \(\kappa\) 测量（已做） | 联合 \(\kappa\) 变差是根因 |
| C | 自由时间 + Fast L-BFGS，但 `rel_step`/`delta` 用物理 snapshot | 质量应回到欧式；若仍更慢，剩下的就是 \(\tilde H\) |
| D | 换成 hull yaml 的 `penna_t / penna_pos / penna_vel` | 能量不主导时 V1 更不应指望加速 |

click_demo 闭环放在 A–C 之后。否则 `TOTAL_REPLAN` 降不下来，分不清是 SFC、hull 还是求解图。

脚本已就绪，只改 `minco_metric_mode`：

```bash
bash scripts/run_mce_vs_euclidean_state2state_ab.sh
```

森林、目标 `(69.032, 1.901, 1.500)`、hull、Fast L-BFGS 保持不变。CSV 含 `EXP_METRIC_MS`、`EXP_METRIC_CACHE_HIT`。在 ROS1 Noetic 容器里跑。

### 10.2 修复

1. **不要**默认 `minco_metric_mode: 1`。
2. 若只想保住固定时间收益：replan 里先对 \(P\) 做固定 \(T\) 的白化子问题，再在欧式 \((\tau,P)\) 上做短的时间修正。V1 用在它真正 \(\kappa=1\) 的块上。
3. 若要在自由时间上也快：按策略文档 §16 做冻结时空度量。至少

   \[
   G=\operatorname{blkdiag}\bigl(\operatorname{diag}(T_i^{-2}),\,G_{\mathrm{MCE}}\bigr);
   \]

   \(\eta\sim 1\) 时必须把交叉项放进 \(G\)，例如控制残差的 \(J_r^\top J_r\)，而不是指望只白化 \(P\)。
4. 无论哪种几何，Fast L-BFGS 的相对步长在物理 \(P,T\) 上算。\(g_z=L^{-1}g_P\) 只用于搜索方向，不用于停机。

---

## 11. 复现

```bash
cd src/Planner/general_planner

# 固定时间：纯能量 / 联合目标（隔离加速）
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_frozen_whitening_comparison_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_whitening_cmp
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_joint_objective_comparison_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_joint_cmp

# 自由时间：滚动 replan A/B
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_state2state_rolling_replan_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_s2s_replan

# 自由时间：联合 Hessian 条件数
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_freetime_joint_conditioning_self_test.cpp \
  -o /tmp/minco_freetime_kappa
```

条件数自测应打印三组快照，并以

```text
geometric mean κ(τ,z)/κ(τ,P) = 3.560e+02
[minco_freetime_joint_conditioning_self_test] OK
```

结束。

---

## 12. 总结

```text
固定 T、决策 = P
    G_MCE = H_{E,P}
    白化后 κ = 1～7
    L-BFGS 2～21 步，J 与欧式相同
    ← 隔离实验，V1 做对了

自由 T、决策 = (τ, P)
    只白化 P：H_zz 变好，H_ττ 不动，S_τ 不变
    交叉项 η ≈ 0.75，对角尺度差 10⁶
    合同变换制造混合近零模态，|λ_min| 塌缩
    κ(H_{τ,z}) / κ(H_{τ,P}) ≈ 80～10³（几何平均 356）
    全收敛 J 相同，迭代和墙钟变差
    Fast L-BFGS 再在 z 上停机 → 质量更差
    ← 生产求解图，V1 的假设不成立
```

V1 是正确的**固定时间能量预条件**，不是正确的**自由时间生产开关**。要把联合问题的 \(\kappa\) 拉回来，必须动时间块或 \(H_{\tau P}\)，不能再只白化 waypoint。
