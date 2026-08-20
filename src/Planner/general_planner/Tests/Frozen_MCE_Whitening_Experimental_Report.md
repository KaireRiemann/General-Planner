# Frozen MCE Whitening 实验报告：纯能量与联合目标

- **日期**：2026-08-19
- **策略文档**：`General_Planner_MCE_Metric_Optimization_Strategy.md`（Production V1）
- **前置报告**：`Tests/MCE_Metric_Conditioning_Experimental_Report.md`（条件数与隔离 L-BFGS，43/43）
- **后续报告**：`Tests/FreeTime_Joint_Conditioning_MCE_Whitening.md`（自由时间联合 \(\kappa\) 变差的完整现象与处理）
- **实现**：`minco_metric.hpp`、`minco_whitening.hpp`、`minco_optimizer.hpp`、`traj_manager.cpp`
- **自测**：
  - `Tests/minco_metric_self_test.cpp`
  - `Tests/minco_frozen_whitening_comparison_self_test.cpp`
  - `Tests/minco_joint_objective_comparison_self_test.cpp`
- **设定**：固定时间 MINCO-S4，Fast L-BFGS，`mem=16`，`g_epsilon=1e-7`～`1e-8`

---

## 0. 一句话结论

固定时间下，MCE 度量就是当前 `getEnergy()` 的 reduced Hessian。冻结 Cholesky 白化之后，纯 snap 问题变成 \(\kappa=1\) 的二次型，L-BFGS **2 次迭代**到达最优点；把跟踪、速度、加速度、走廊同时加进去后，不再是精确 Newton，但仍把 \(\kappa\) 从 \(10^2\sim10^5\) 压到接近 \(1\)，迭代大约是欧式的 **5–24 倍**，终点 \(J\) 与各项分拆与欧式相同。

---

## 1. 这次实验回答什么

前置条件数实验已经说明：\(G_{\mathrm{MCE}}\) 与纯能量 Hessian 对齐，白化后 \(\kappa=1\)。本次在 **生产 V1 实现** 上验证两件事：

1. 度量是否严格等于当前能量定义（`getEnergy()` 不含 \(1/2\)）；
2. 按文档做 **frozen whitening**（而不是把 \(G^{-1}g\) 当梯度塞进 L-BFGS）之后，纯能量和联合目标实际怎么收敛。

对比对象固定为：

| 代号 | 几何 | 说明 |
|---|---|---|
| A Euclidean | \(G=I\) | 旧生产路径 |
| B H0 old | \(H_0=(H_E/2)^{-1}\) | 实现前差因子 2 的 H0 |
| C H0 consistent | \(H_0=H_E^{-1}\) | Hessian 对齐，但仍是动态 H0 |
| D Frozen MCE | \(z=L^\top(P-P_0)\) | 文档 V1 |
| E Frozen MCE+GN | \(G=\rho_EG_{\mathrm{MCE}}+G_c^{\mathrm{GN}}\) | 走廊激活时的推荐扩展 |

时间块始终走 `QuadInvTimeMap`，不白化 \(T\)。

---

## 2. 生产实现与能量一致性

当前 `getEnergy()` 为 \(\operatorname{tr}(C^\top QC)\)，不含 \(1/2\)。文档要求

\[
G_{\mathrm{MCE}}=2J_P^\top QJ_P=H_{E,\mathrm{reduced}},
\qquad
G_0=\rho_EG_{\mathrm{MCE}}(T_0).
\]

实现后有限差分 Hessian 对齐：

\[
e_H=\frac{\|H_E-G_{\mathrm{MCE}}\|}{\|H_E\|}=2.20\times 10^{-9},
\qquad
\kappa\!\left(G^{-1/2}H_EG^{-1/2}\right)=1.000.
\]

一次冻结白化 Newton 步（\(dP=-L^{-T}L^{-1}g_P\)）：

| | \(J\) | \(\|g_P\|\) |
|---|---|---|
| 初值 | \(1.202\times 10^4\) | \(2.128\times 10^4\) |
| 一步之后 | \(13.59\) | \(1.274\times 10^{-10}\) |

这就是“几次迭代到最优”的解析原因：固定时间纯 snap 是 \(P\) 的严格二次型，MCE 白化后 L-BFGS 看到的是单位二次型。

\(\rho_E\) 缩放也已对齐：\(\|G(4)\|/\|G(1)\|=4\)。

位置 MINCO 使用 Kronecker 结构 \(G_{\mathrm{MCE}}^{3D}=G_{\mathrm{scalar}}\otimes I_3\)，Cholesky 只在 \(M-1\) 维标量块上分解。

---

## 3. 纯 Minimum-Control 优化

目标：\(J=\rho_EE_{\mathrm{snap}}\)。初值同一组内点。

| 场景 | A 欧式 | B 旧 H0（\(G=H/2\)） | C 对齐 H0 | **D Frozen 白化** |
|---|---|---|---|---|
| M=5 均匀 | 21 it / 30 eval | 2 it | 2 it | **2 it** |
| M=10 均匀 | 91 it / 98 eval | 2 it | 2 it | **2 it** |
| M=5 时间很不均匀 | 39 it / 64 eval | 4 it | 2 it，线搜失败 | **2 it，稳定** |
| MCE+waypoint L2 | 20 it | 10 it | 7 it，线搜失败 | **6 it，稳定** |
| \(\rho_E=0.25\) 纯 MCE | 21 it | 2 it | 2 it | **2 it** |

四种方法终值 \(J\) 相同。M=5 均匀时 \(J^\star=13.59\)，与上一节单步 Newton 终值一致。

要点：

- 纯能量上 Frozen MCE 与精确 Newton 等价，所以是 2 步（第一步下降，第二步确认梯度为 0）。
- Hessian 对齐的 H0 迭代次数可以很少，但 history 仍写在欧式 \(x\) 里，时间不均匀或混入 L2 时会出现 `MAXIMUMLINESEARCH`。
- Frozen whitening 把几何冻成固定坐标 \(z\)，线搜与曲率对一致，因此稳定。这正是策略文档 §7 不推荐 Dynamic \(H_0\) 的原因。

欧式条件数随段数恶化（与前置报告一致）：

| \(M\) | \(\kappa(G_P)=\kappa(H_E)\) |
|---|---|
| 3 | \(7.14\) |
| 5 | \(3.20\times 10^2\) |
| 10 | \(6.46\times 10^4\) |
| 20 | \(1.58\times 10^7\) |

白化后全部 \(\kappa=1\)。迭代从 21 / 91 变成 2，对应的就是这块几何尺度，而不是 MINCO 生成本身变难。

---

## 4. 联合目标：能量 + 跟踪 + 速度 + 加速度 + 走廊

目标（固定时间）：

\[
J
=
\rho_EE_{\mathrm{snap}}
+
J_{\mathrm{track}}
+
J_{\mathrm{vel}}
+
J_{\mathrm{acc}}
+
J_{\mathrm{corridor}}.
\]

权重：\(\rho_E=1\)，\(w_{\mathrm{track}}=6\)，\(w_{\mathrm{vel}}=0.25\)，\(w_{\mathrm{acc}}=0.08\)，走廊 \(y_{\max}=0.08\)、\(w_{\mathrm{corr}}=25\)（窄走廊一组改为 \(y_{\max}=0.05\)、\(w_{\mathrm{corr}}=200\)）。初值内点带正弦横向偏移，走廊在起点是激活的。

### 4.1 条件数与迭代

| 场景 | \(\kappa_I\) | \(\kappa_{\mathrm{MCE}}\) | \(\kappa_{\mathrm{MCE+GN}}\) | 欧式 | Frozen MCE | MCE+GN |
|---|---|---|---|---|---|---|
| Planner-like M=5 | \(2.94\times 10^2\) | **1.25** | 1.09 | 28 it，线搜失败 | **5 it** | 5 it |
| Planner-like M=10 | \(2.85\times 10^3\) | 66.7 | 22.7 | 188 it | **21 it** | 20 it |
| M=5 时间很不均匀 | \(9.10\times 10^4\) | **1.43** | 1.09 | 120 it，线搜失败 | **5 it** | 6 it |
| 更窄走廊 M=5 | \(2.95\times 10^2\) | 2.76 | 1.09 | 23 it | **5 it** | 6 it |

三种几何终点 \(J\) 相同，走廊 violation 均为 0。

### 4.2 代价分拆（M=5 均匀，三种方法终点相同）

| | \(J\) | \(E\) | track | vel | acc | corr | viol |
|---|---|---|---|---|---|---|---|
| 起点 | \(1.81\times 10^3\) | \(1.81\times 10^3\) | 0.27 | 0.70 | 0.24 | 0.34 | 0.14 |
| 终点 | 10.67 | 8.10 | 1.79 | 0.73 | 0.051 | 0 | 0 |

起点几乎是 snap 主导；终点能量与跟踪已经同量级，所以这不是“假装混合、实际还是纯 MCE”。尽管如此，冻结 MCE 仍把 \(\kappa\) 从 294 降到 1.25，5 步走完欧式 28 步（且欧式线搜失败）的路径。

M=10 更明显：欧式 188 步，MCE 21 步，加速约 9 倍。时间不均匀时加速约 24 倍。

### 4.3 为什么不再是 2 步

纯 MCE 满足 \(H_J=G_{\mathrm{MCE}}\)。联合目标的 Hessian 是

\[
H_J
=
\rho_EH_E
+
H_{\mathrm{track}}
+
H_{\mathrm{vel}}
+
H_{\mathrm{acc}}
+
H_{\mathrm{corr}}.
\]

白化后

\[
G_{\mathrm{MCE}}^{-1/2}H_JG_{\mathrm{MCE}}^{-1/2}
=
I
+
G_{\mathrm{MCE}}^{-1/2}H_{\mathrm{rest}}G_{\mathrm{MCE}}^{-1/2}
\neq I.
\]

M=5 时这块扰动很小（\(\kappa_{\mathrm{MCE}}=1.25\)），所以仍接近 Newton，5 步结束。M=10 时跟踪/动力学相对曲率变大（\(\kappa_{\mathrm{MCE}}=66.7\)），需要 21 步，但仍然远好于欧式的 188 步。

### 4.4 为什么 MCE+GN 几乎没再加速

种子处走廊 Gauss-Newton 占 MCE 的相对迹

\[
\eta_{\mathrm{trace}}
=
\frac{\operatorname{tr}(G_c)}{\operatorname{tr}(G_{\mathrm{MCE}})}
\]

只有 \(10^{-4}\sim 6\times 10^{-3}\)。按策略文档，这表示 **MCE 仍主导曲率**，active GN 改善 \(\kappa\)（例如窄走廊 \(2.76\to 1.09\)），但第一步搜索方向几乎不变，迭代数持平。终点走廊已经 inactive（viol=0），冻结在起点的 \(G_c\) 对后半段求解也没有额外信息。

GN 真正开始主导的条件是 \(\eta_{\mathrm{trace}}\sim 1\) 或 \(\eta_c=\lambda_{\max}(G_{\mathrm{MCE}}^{-1/2}G_cG_{\mathrm{MCE}}^{-1/2})\sim 1\)，对应更窄走廊或更大 \(w_{\mathrm{corr}}\)。本次权重还没打到那一档。

---

## 5. 和欧式相比，到底快在哪里

| 机制 | 纯 MCE | 联合目标 |
|---|---|---|
| MCE 与 Hessian | 精确相等 | 能量块相等，其余是扰动 |
| 白化后 \(\kappa\) | \(1\) | \(1.25\)～\(67\) |
| 第一步方向 | Newton | 能量主导的预条件 Newton |
| 典型迭代 | 2 | 5～21 |
| 相对欧式 | 10～45× | 5～24× |
| 终值 \(J\) | 相同 | 相同 |
| 线搜 | 稳定 | 稳定；欧式在不均匀时间上失败 |

时间不均匀是欧式最痛的地方：\(\kappa_I=9.1\times 10^4\)，120 步仍线搜失败；MCE 把 \(\kappa\) 拉到 1.43，5 步结束。这与策略文档 §2.4 的判断一致：段数变多、各段时间差变大时，很大一部分 L-BFGS 变慢来自 waypoint 空间的几何尺度，而不是 MINCO 生成。

---

## 6. 不应当从本次结果推出的结论

1. **不是** State2State 全节点已经 5 步收敛。本次是固定时间、隔离 L-BFGS，没有 Bezier/MINVO 走廊硬约束、没有自由时间、没有 ROS 重规划。
2. **不是** 任意混合目标都 \(\kappa=1\)。走廊/动力学一旦和 snap 曲率相当，需要冻结 MCE+GN，而不是只靠 \(G_{\mathrm{MCE}}\)。
3. **不是** Dynamic \(H_0\) 可以代替 whitening。对齐 Hessian 的 H0 在纯能量上也能 2 步，但在不均匀时间和混合目标上线搜会坏。
4. 总成本仍应计 \(T_{\mathrm{metric}}+T_{\mathrm{objective}}+T_{\mathrm{line-search}}\)。本次度量构造相对目标评估可忽略（亚毫秒），高维或频繁重建时仍需缓存 / Kronecker（V1 已实现标量 Cholesky 与按 \(T\) 缓存）。
5. **不是** 打开 `minco_metric_mode: 1` 就会在真实 replan 里自动变快。固定时间隔离结果不能直接外推到自由时间 + Fast L-BFGS（见 §9）。

---

## 7. 复现

```bash
cd src/Planner/general_planner
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_metric_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_metric_self_test
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_frozen_whitening_comparison_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_whitening_cmp
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_joint_objective_comparison_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_joint_cmp
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_state2state_rolling_replan_self_test.cpp src/utils/lbfgs.cpp \
  -o /tmp/minco_s2s_replan
g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_freetime_joint_conditioning_self_test.cpp \
  -o /tmp/minco_freetime_kappa
```

真实 click_demo 闭环（ROS1 Noetic，需已编译带 V1 的 `fsm_node`）：

```bash
bash scripts/run_mce_vs_euclidean_state2state_ab.sh
```

只改 `minco_metric_mode`（0 vs 1）。森林、目标 `(69.032, 1.901, 1.500)`、hull、Fast L-BFGS、目标函数保持不变。CSV 新增 `EXP_METRIC_MS`、`EXP_METRIC_CACHE_HIT`。默认 yaml 仍是 mode 0。

---

## 9. 滚动 replan / 自由时间（生产求解图）

完整现象、分块 Hessian、合同变换推导与处理顺序见独立文档：

**`Tests/FreeTime_Joint_Conditioning_MCE_Whitening.md`**

本节只保留本报告用到的对照数字。隔离实验是**固定时间、经典 L-BFGS、\(g_\epsilon\sim10^{-7}\)**。生产 State2State 是：

- 时间自由，`QuadInvTimeMap` 不白化；
- 只冻 waypoint 的 \(G_{\mathrm{MCE}}\)；
- Fast L-BFGS（`min_iterations=10`，`rel_cost=1e-3`，`delta=1e-5`，`g_epsilon=0`）。

用同一组 remaining-horizon 快照（M=8，能量+时间+vel/acc+走廊）做 A/B。种子处有限差分：欧式 \(7.1\times10^{-7}\)，MCE \(1.7\times10^{-5}\)。

### 9.1 对齐终点质量（经典 L-BFGS，\(g_\epsilon=0\)，\(\delta=10^{-12}\)，最多 400 步）

| | 欧式 | Frozen MCE | 比 |
|---|---:|---:|---:|
| 终值 \(J\) | 259.36 | 259.74 | **1.001×** |
| 走廊 viol | 0.014 | 0.027 | 同量级 |
| 迭代 / 评估 | 264 / 301 | 326 / 383 | 1.23× / 1.27× |
| L-BFGS ms/call | 3.47 | 4.57 | **1.32× 更慢** |
| 度量 ms/call | 0 | 0.039 | 可忽略 |

白化是坐标变换：真正收敛时 \(J\) 相同。但**自由时间问题里，只白化 \(P\) 并不能减少迭代**，时间块仍是欧式，线搜更难，墙钟变慢。联合 Hessian 的条件数在白化后变差约 \(10^2\sim10^3\) 倍（§9.4）。

### 9.2 生产 Fast L-BFGS（`click_real_highspeed.yaml` 的停机规则）

| | 欧式 | Frozen MCE | 比 |
|---|---:|---:|---:|
| L-BFGS ms/call | 0.70 | 0.88 | **1.26× 更慢** |
| 迭代 / 评估 | 55 / 58 | 63 / 70 | 1.16× |
| 终值 \(J\) | 225 | 632 | **2.81× 更差** |
| 走廊 viol | 0.022 | 1.03 | 明显更差 |
| fast-stop | 100% | 100% | 在更差点停下 |

Fast L-BFGS 的 `rel_step` / `delta` 作用在 \(x=(\tau,z)\) 上。\(z=L^\top(P-P_0)\) 的尺度和欧式 \(P\) 不同，同样的相对停机在白化图上会过早判稳。打开 phase-0 物理 waypoint/time guard 能把 \(J\) 从 632 拉回 335，但迭代变成欧式的 **2.2 倍**，墙钟 **2.4 倍**。

### 9.3 对真实 click_demo 的含义

在 Noetic 机器上跑 `run_mce_vs_euclidean_state2state_ab.sh` 之前，**不要把 mode 1 设成默认**。固定时间隔离的 5–24× 加速，在生产「自由时间 + Fast L-BFGS」路径上没有出现。要让 V1 在 replan 里真正降延迟，需要至少一件事：

1. Fast L-BFGS 的停机量改成物理 \(P,T,J\)（而不是 \(z\) 上的相对步）；或
2. 按策略文档做冻结时空度量，而不是只白化 waypoint。

### 9.4 联合 Hessian 条件数如何变差

同一组自由时间快照上，对求解坐标做有限差分 Hessian（`Tests/minco_freetime_joint_conditioning_self_test.cpp`）。决策是 \(x=(\tau,P)\) 或 \((\tau,z)\)，\(z=L^\top(P-P_0)\)，\(LL^\top=G_{\mathrm{MCE}}(T_0)\)。

| 快照 | \(\kappa(H_{\tau P})\) | \(\kappa(H_{\tau z})\) | 比 | \(\kappa(H_{PP})\) | \(\kappa(H_{zz})\) | \(\kappa(H_{\tau\tau})\) |
|---|---:|---:|---:|---:|---:|---:|
| 0 | \(4.4\times10^6\) | \(3.5\times10^8\) | **81×** | \(1.0\times10^4\) | **6.8** | \(6.3\times10^2\) |
| 4 | \(3.2\times10^6\) | \(1.8\times10^9\) | **563×** | \(1.1\times10^4\) | **5.8** | \(6.8\times10^2\) |
| 9 | \(1.7\times10^6\) | \(1.7\times10^9\) | **993×** | \(1.1\times10^4\) | **1.35** | \(6.9\times10^2\) |

几何平均：\(\kappa(H_{\tau z})/\kappa(H_{\tau P})\approx 356\)。

分块上发生了三件同时成立的事：

1. **Waypoint 块被修好了**，这就是隔离实验：\(\kappa(H_{PP})\sim10^4\to\kappa(H_{zz})\sim1\sim7\)。
2. **时间块完全没动**：\(\kappa(H_{\tau\tau})\) 两套坐标相同；消元后的 Schur \(S_\tau=H_{\tau\tau}-H_{\tau P}H_{PP}^{-1}H_{P\tau}\) 也逐元素不变。只白化 \(P\) 不改变「把 \(P\) 消掉之后时间有多难」。
3. **整体 \(\kappa\) 变差来自 \(\lvert\lambda_{\min}\rvert\) 塌缩，不是 \(\lambda_{\max}\) 变大**。\(\lambda_{\max}\) 始终由 \(H_{\tau\tau}\) 占着（\(\sim10^6\sim10^7\)）。欧式 \(\lvert\lambda_{\min}\rvert\) 几乎等于 \(\lambda_{\min}(H_{PP})\)（软的 waypoint 模态）；白化后这些模态被抬到 \(O(1)\)，新的 \(\lvert\lambda_{\min}\rvert\) 掉到 \(10^{-2}\) 量级，是 \((\tau,z)\) 的混合近零模态。

机制是非正交合同。

\[
\tilde H = A^\top H A,\qquad
A=\operatorname{blkdiag}(I_\tau,L^{-T}).
\]

\(A\) 不正交，特征值不保持。归一化耦合

\[
\eta=\frac{\|H_{\tau P}\|_F}{\sqrt{\|H_{\tau\tau}\|_F\|H_{PP}\|_F}}\approx 0.74\sim 0.76
\]

已经很强；白化后 \(\eta\) 不降（快照 9 升到 \(1.05\)）。直观上 \(H_{\tau\tau}\) 比白化后的 \(H_{zz}\sim I\) 大 \(10^6\)，交叉项又去不掉，二维模型 \(\begin{bmatrix}a&c\\c&1\end{bmatrix}\)（\(a\gg1\)，\(c^2\sim \eta^2 a\)）的小特征值会被压到远小于 1。种子处 Hessian 还不定（约 7 个负特征值，在 \(S_\tau\) 里），L-BFGS 面对的不是 SPD 二次型。

所以：V1 精确改善了固定时间的 \(P\) 块，同时把自由时间联合问题的 \(\kappa\) 恶化了两到三个数量级。这不是度量算错，是部分预条件在强耦合块上的必然结果。

---

## 10. 总结

固定时间 MINCO 上，MCE 不是经验缩放，而是当前能量二次型的预条件。生产 V1 把它做成冻结坐标白化后：

- 纯 snap：\(\kappa=1\)，**2 次迭代**到最优；
- 能量+跟踪+速度+加速度+走廊（固定时间）：终值不变，迭代从数十到一百多降到 **5～21**；
- 时间不均匀时对比最强：欧式 120 步失败，MCE 5 步；
- **自由时间联合优化**：只白化 \(P\) 后 \(\kappa(H_{\tau z})/\kappa(H_{\tau P})\approx 80\sim 10^3\)（几何平均 356）；终点 \(J\) 仍可对齐，但迭代和墙钟没有变好。直接套生产 Fast L-BFGS 还会在更差点停机。

真实 click_demo A/B 脚本已就绪；在把 mode 1 接到线上之前，先改 Fast L-BFGS 停机尺度或补时空度量。
