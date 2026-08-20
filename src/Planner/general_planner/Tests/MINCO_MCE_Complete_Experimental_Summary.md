# MINCO MCE 度量优化：完整实验结果

- **日期**：2026-08-19 ～ 2026-08-20
- **分支**：`feature/minco-mce-metric`
- **默认开关**：`minco_metric_mode: 0`（欧式）。本轮**不**把 mode 1 / 3 / 4 设成生产默认。
- **未跑**：真实 State2State / click_demo（本机只有 ROS 2 Jazzy，无 Noetic 容器）

分报告：

| 阶段 | 报告 |
|---|---|
| 固定时间条件数 | `Tests/MCE_Metric_Conditioning_Experimental_Report.md` |
| 固定时间 Frozen MCE（V1） | `Tests/Frozen_MCE_Whitening_Experimental_Report.md` |
| 自由时间只白化 \(P\) 失败 | `Tests/FreeTime_Joint_Conditioning_MCE_Whitening.md` |
| 完整时空度量 + 生产接线 | `Tests/Full_SpaceTime_MINCO_Joint_Optimization_Experimental_Report.md` |

---

## 0. 一句话结论

固定时间下，waypoint MCE 就是当前 `getEnergy()` 的 reduced Hessian。冻结 Cholesky 白化之后，纯 snap 变成 \(\kappa=1\) 的二次型，L-BFGS **2 次迭代**到达最优点；联合目标不再是精确 Newton，但仍把 \(\kappa\) 从 \(10^2\sim10^5\) 压到接近 \(1\)，迭代大约是欧式的 **5–24 倍**。

自由时间下决策是耦合的 \((\tau,P)\)。V1 **只白化 \(P\)** 会把联合 \(\kappa\) 弄差 **\(80\sim10^3\)**。正确几何是

\[
G_\theta=\begin{bmatrix}G_{TT}&G_{TP}\\G_{PT}&G_{PP}\end{bmatrix}
=2\rho_E J_r^\top J_r+\lambda_T\operatorname{diag}(T_i^{-2}),
\]

再经 `QuadInvTimeMap` pullback 到 \((\tau,\xi)\)，做 **Frozen Full Joint / Block-Schur** 白化。均匀时长 \(\kappa\) 从 \(10^6\sim10^7\) 降到 \(10^3\)（约 \(300\sim1400\times\)）；走廊激活时紧 L-BFGS **104 vs 500** 步。极端 \(T_{\max}/T_{\min}=8\) 时单次冻结不够，**一次 metric refresh 把 \(J\) 从 468 拉回 289**。

Fast L-BFGS 墙钟 P50 仍是欧式的 **1.58×**。质量对齐 ≠ 默认加速。**不要打开 mode 1 或 3/4 当生产默认。**

---

## 1. Mode 语义（当前生产接线）

| mode | 含义 | 状态 |
|---|---|---|
| 0 | 欧式 L-BFGS | **默认** |
| 1 | Frozen waypoint MCE（V1） | 固定时间成立；自由时间是失败消融 |
| 2 | 动态 waypoint H0 | 实验；仅此模式把 \(H_0\) 交给 Fast L-BFGS |
| 3 | Frozen block space-time joint whitening | 已接线，不默认 |
| 4 | Frozen full space-time GN joint whitening（Block-Schur，含 \(Y d\tau\)） | 已接线，不默认 |

mode 3/4：整段 \(x=(\tau,\xi)\) 白化；Fast 用物理 \(T/P\) 停机（`rel_step=0`）；step bound 作用在物理 \(\tau\)；\(T\) 或 waypoint 相对漂移 \(>0.25\) 时最多 refresh 1 次。CSV 含 `EXP_METRIC_MS`、`EXP_METRIC_CACHE_HIT`、`EXP_METRIC_REFRESH`。

---

## 2. 阶段 1：固定时间 Frozen MCE（V1）

设定：固定时间 MINCO-S4，经典 L-BFGS，`mem=16`。

### 2.1 能量一致性

当前 `getEnergy()` 为 \(\operatorname{tr}(C^\top QC)\)，不含 \(1/2\)。文档要求

\[
G_{\mathrm{MCE}}=2J_P^\top QJ_P=H_{E,\mathrm{reduced}},
\qquad
G_0=\rho_E G_{\mathrm{MCE}}(T_0).
\]

| 检查 | 结果 |
|---|---|
| \(\|H_E-G_{\mathrm{MCE}}\|/\|H_E\|\) | \(2.20\times10^{-9}\) |
| \(\kappa(G^{-1/2}H_E G^{-1/2})\) | \(1.000\) |
| \(\|G(\rho_E=4)\|/\|G(\rho_E=1)\|\) | \(4\) |

一次冻结白化 Newton 步（\(dP=-L^{-T}L^{-1}g_P\)）：

| | \(J\) | \(\|g_P\|\) |
|---|---|---|
| 初值 | \(1.202\times10^4\) | \(2.128\times10^4\) |
| 一步之后 | \(13.59\) | \(1.274\times10^{-10}\) |

位置 MINCO 使用 Kronecker 结构 \(G_{\mathrm{MCE}}^{3D}=G_{\mathrm{scalar}}\otimes I_3\)。

### 2.2 纯 Minimum-Control

目标 \(J=\rho_E E_{\mathrm{snap}}\)。四种方法终值 \(J\) 相同。M=5 均匀时 \(J^\star=13.59\)，与单步 Newton 终值一致。

| 场景 | 欧式 | 旧 H0（\(G=H/2\)） | 对齐 H0 | **Frozen 白化** |
|---|---|---|---|---|
| M=5 均匀 | 21 it / 30 eval | 2 it | 2 it | **2 it** |
| M=10 均匀 | 91 it / 98 eval | 2 it | 2 it | **2 it** |
| M=5 时长很不均匀 | 39 it / 64 eval | 4 it | 2 it，线搜失败 | **2 it，稳定** |
| MCE + waypoint L2 | 20 it | 10 it | 7 it，线搜失败 | **6 it，稳定** |
| \(\rho_E=0.25\) | 21 it | 2 it | 2 it | **2 it** |

欧式条件数随段数恶化；白化后全部 \(\kappa=1\)。

| \(M\) | \(\kappa(G_P)=\kappa(H_E)\) |
|---|---|
| 3 | \(7.14\) |
| 5 | \(3.20\times10^2\) |
| 10 | \(6.46\times10^4\) |
| 20 | \(1.58\times10^7\) |

要点：纯能量上 Frozen MCE 与精确 Newton 等价（2 步：一步下降，一步确认梯度为 0）。对齐 H0 迭代可以很少，但 history 仍写在欧式 \(x\) 里，时间不均匀或混入 L2 时会出现 `MAXIMUMLINESEARCH`。Frozen whitening 把几何冻成固定坐标 \(z\)，因此稳定。

### 2.3 联合目标（固定时间）

\[
J=\rho_E E_{\mathrm{snap}}+J_{\mathrm{track}}+J_{\mathrm{vel}}+J_{\mathrm{acc}}+J_{\mathrm{corridor}}.
\]

权重：\(\rho_E=1\)，\(w_{\mathrm{track}}=6\)，\(w_{\mathrm{vel}}=0.25\)，\(w_{\mathrm{acc}}=0.08\)，走廊 \(y_{\max}=0.08\)、\(w_{\mathrm{corr}}=25\)（窄走廊一组改为 \(y_{\max}=0.05\)、\(w_{\mathrm{corr}}=200\)）。

| 场景 | \(\kappa_I\) | \(\kappa_{\mathrm{MCE}}\) | \(\kappa_{\mathrm{MCE+GN}}\) | 欧式 it | Frozen MCE it | MCE+GN it |
|---|---:|---:|---:|---:|---:|---:|
| Planner-like M=5 | \(2.94\times10^2\) | **1.25** | 1.09 | 28，线搜失败 | **5** | 5 |
| Planner-like M=10 | \(2.85\times10^3\) | 66.7 | 22.7 | 188 | **21** | 20 |
| M=5 时长很不均匀 | \(9.10\times10^4\) | **1.43** | 1.09 | 120，线搜失败 | **5** | 6 |
| 更窄走廊 M=5 | \(2.95\times10^2\) | 2.76 | 1.09 | 23 | **5** | 6 |

三种几何终点 \(J\) 相同，走廊 violation 均为 0。

M=5 均匀代价分拆（三种方法终点相同）：

| | \(J\) | \(E\) | track | vel | acc | corr | viol |
|---|---|---|---|---|---|---|---|
| 起点 | \(1.81\times10^3\) | \(1.81\times10^3\) | 0.27 | 0.70 | 0.24 | 0.34 | 0.14 |
| 终点 | 10.67 | 8.10 | 1.79 | 0.73 | 0.051 | 0 | 0 |

终点能量与跟踪已经同量级，不是“假装混合、实际还是纯 MCE”。加速：M=5 约 5×；M=10 约 9×；时间不均匀约 24×。

---

## 3. 阶段 2：自由时间只白化 \(P\)（V1 失败）

隔离加速来自「固定 \(T\) 下能量 Hessian 的精确白化」。生产路径同时换了两根轴：是否自由时间、是否紧收敛。把隔离的 5–24× 当成 click_demo 的预期，本身就不成立。

### 3.1 三层现象必须拆开

```text
层 A  几何：只白化 P 之后，联合 κ(H_{τ,z}) 比 κ(H_{τ,P}) 差 10²～10³
层 B  求解：全收敛时 J 相同，但迭代更多、墙钟更慢
层 C  停机：生产 Fast L-BFGS 在 z 尺度上判稳，J 225 → 632，走廊 viol 更差
```

- A 解释 B：条件数变差，紧收敛不会更快。
- C 是额外的尺度错误，不改变 A。打开 phase-0 物理 guard 能减轻 C，但墙钟变成欧式的 2.4 倍。

### 3.2 这不是实现错误

下列事实同时成立：

1. \(G_{\mathrm{MCE}}=2J_P^\top Q J_P\) 与 `getEnergy()` 的 reduced Hessian 对齐（\(e_H=2.2\times10^{-9}\)）。
2. 固定时间纯 snap：白化后 \(\kappa=1\)，2 次迭代。
3. 自由时间全收敛：欧式 \(J=259.36\)，MCE \(J=259.74\)（比值 1.001）。坐标变换没有改最优点。
4. 时间 Schur \(S_\tau=H_{\tau\tau}-H_{\tau P}H_{PP}^{-1}H_{P\tau}\) 在白化前后**逐元素不变**。

所以 \(L\)、encode \(z=L^\top(P-P_0)\)、covector \(g_z=L^{-1}g_P\) 都是对的。坏的是「只预处理一块」这件事本身。

### 3.3 联合 \(\kappa\)（M=8 rolling）

决策维数：\(\tau\in\mathbb{R}^8\)，\(P\in\mathbb{R}^{21}\)。\(\kappa=\lvert\lambda\rvert_{\max}/\lvert\lambda\rvert_{\min}\)（种子处 Hessian 不定，约 7 个负特征值）。

| 快照 | \(\kappa(H_{\tau,P})\) | \(\kappa(H_{\tau,z})\) | 比 | \(\kappa(H_{PP})\) | \(\kappa(H_{zz})\) | \(\lvert\lambda\rvert_{\min}\) 欧式 → 白化 |
|---|---:|---:|---:|---:|---:|---|
| 0 | \(4.38\times10^6\) | \(3.53\times10^8\) | **81×** | \(1.04\times10^4\) | 6.82 | 0.975 → **0.012** |
| 4 | \(3.20\times10^6\) | \(1.80\times10^9\) | **563×** | \(1.10\times10^4\) | 5.81 | 3.12 → **0.0055** |
| 9 | \(1.71\times10^6\) | \(1.70\times10^9\) | **993×** | \(1.13\times10^4\) | 1.35 | 20.4 → **0.020** |

几何平均变差 **356×**。

分块上同时成立三件事：

1. **Waypoint 块被修好了**：\(\kappa(H_{PP})\sim10^4\to\kappa(H_{zz})\sim1\sim7\)。
2. **时间块完全没动**：\(\kappa(H_{\tau\tau})\) 与 \(S_\tau\) 不变。
3. **整体 \(\kappa\) 变差来自 \(\lvert\lambda_{\min}\rvert\) 塌缩**，不是 \(\lambda_{\max}\) 变大。新的近零特征值是 \((\tau,z)\) 的混合模态。

### 3.4 生产 Fast L-BFGS（yaml：`min_iterations=10`，`rel_cost=1e-3`，`phase0_guards=false`）

| | 欧式 | Frozen MCE V1 | 比 |
|---|---:|---:|---|
| L-BFGS ms/call | 0.70 | 0.88 | **1.26× 更慢** |
| 迭代 / 评估 | 55 / 58 | 63 / 70 | 1.16× |
| 终值 \(J\) | **225** | **632** | **2.81× 更差** |
| 走廊 viol | 0.022 | 1.03 | 明显更差 |
| fast-stop | 100% | 100% | 在更差点停下 |

欧式 Fast 已经在 ~55 步、0.70 ms 停住。隔离里欧式 188 步那种病态税，线上根本没在交。`rel_step`/`delta` 作用在 \(x=(\tau,z)\) 上：\(z\) 从 0 出发，单位是能量曲率不是米，走廊还没解开就停。打开 phase-0 物理 \(P,T\) guard 后 \(J\) 从 632 回到约 335，但迭代变成欧式的 **2.2 倍**，墙钟 **2.4 倍**。

**不要把 `minco_metric_mode: 1` 设成默认。**

---

## 4. 阶段 3：完整时空度量

### 4.1 Gate 总表

| Gate | 要求 | 结果 |
|---|---|---|
| 1 Full metric | \(G_{PP}^{\mathrm{full}}\approx G_{PP}^{\mathrm{MCE}}\) | **PASS** \(e_{PP}=9.77\times10^{-15}\) |
| 2 Joint whitening | \(dx^\top G_x dx=\|z\|^2\) | **PASS** \(6.5\times10^{-16}\) |
| 3 Covector | \(g_x^\top dx=g_z^\top dz\) | **PASS** \(2.1\times10^{-16}\) |
| 4 Conditioning | \(\kappa_{\mathrm{C3}}<\kappa_{\mathrm{C0}}\) 且 \(\lvert\lambda_{\min}\rvert\) 不塌缩 | **均匀时长 PASS**（\(300\sim1400\times\)）；**\(T_{\max}/T_{\min}=8\) 时 C3 失败、C4 成功** |
| 5 Tight solver | \(J^\star\) 对齐且迭代下降 | **对齐 PASS**；snap4 **104 vs 500**；snap0 双方打到 500 上限 |
| 6 Production wall-clock | \(t_{\mathrm{metric}}+t_{\mathrm{solver}}<t_{\mathrm{Euclidean}}\) | **尚未作为默认依据** |
| G 生产图恒等 | encode/decode、配对、物理 step bound、refresh | **PASS**（见 §5.2） |

### 4.2 Level A：残差 Jacobian 与 \(G_{PP}\) 对齐

控制残差 \(r_i(u)=\sqrt{\rho_E T_i}\,p_i^{(4)}(uT_i)\) 的 Gauss-Legendre 4 点积分，以及 \(G_{\mathrm{ctrl}}=2\rho_E J_r^\top J_r\)。此前实现少了因子 \(2\rho_E\)，现已乘上。

| 检查 | 误差 / 值 |
|---|---|
| A1 系数 JVP vs FD | \(1.95\times10^{-7}\) |
| A2 控制残差 JVP vs FD | \(9.47\times10^{-9}\) |
| A3 \(\|G-G^\top\|\) | \(0\) |
| A4 \(\lambda_{\min}(G)\) | \(30.9>0\)（SPD） |
| A4 \(\kappa(G)\)、\(\eta_F(G_\theta)\) | \(1.74\times10^4\)，\(\eta=0.729\) |
| A5 \(e_{PP}\) | **\(9.77\times10^{-15}\)** |
| \(\|G_{TP}\|\) | \(1.65\times10^5\)（交叉项真实存在） |

### 4.3 Level B：Pullback 与 Frozen Joint / Schur

\(T=\phi(\tau)\) 为 `QuadInvTimeMap`，空间图为恒等。

| 检查 | 结果 |
|---|---|
| B1 \(d\theta^\top G_\theta d\theta=dx^\top G_x dx\) | \(5.2\times10^{-16}\) |
| B2 恒等空间图 \(G_{PP}\) 不变 | PASS |
| B3 \(dx^\top G_x dx=\|z\|^2\) | \(6.5\times10^{-16}\) |
| B4 \(g_x^\top dx=g_z^\top dz\) | \(2.1\times10^{-16}\) |
| B5 \(dx=L^{-T}dz\) | \(4.1\times10^{-16}\) |
| B6 Dense vs Block-Schur 二次型 | \(\Delta=4.4\times10^{-11}\) |
| Schur \(Y=C^{-1}B^\top\) | \(\|Y\|=7.65\neq0\) |

\(Y\) 就是 V1 缺失的时空去相关项。Dense 与 Block-Schur 在 SPD 度量上等价。

### 4.4 Level C：自由时间联合 Hessian 消融

同一组 remaining-horizon 快照。目标 = 能量 + 时间权 20 + vel/acc + 走廊。Hessian 在 \((\tau,P)\) 上中心差分一次，再用各 \(G\) 做合同 \(\widetilde H=L^{-1}HL^{-T}\)。种子处 Hessian **不定**（约 4–12 个负特征值），所以不要求 \(\kappa=1\)。

消融代号：

| | 几何 |
|---|---|
| C0 | 欧式 \(G=I\) |
| C1 | 只白化 \(P\)（Production V1 失败模式） |
| C2 | 对角 \(G_\tau^{rel}\oplus G_{PP}\) |
| C3 | full space-time GN + \(G_T^{rel}\) |
| C4 | C3 + active corridor Gauss-Newton |

#### 均匀时长 rolling（M=8，snap 0 / 4 / 9）

| | \(\kappa\) snap0 | \(\kappa\) snap4 | \(\kappa\) snap9 | \(\lvert\lambda\rvert_{\min}\) snap0 | \(\eta\) |
|---|---:|---:|---:|---:|---:|
| C0 Euclidean | \(4.38\times10^6\) | \(3.20\times10^6\) | \(1.71\times10^6\) | \(0.975\) | \(0.74\) |
| C1 waypoint-only V1 | \(3.53\times10^8\) | \(1.80\times10^9\) | \(1.70\times10^9\) | **\(1.21\times10^{-2}\)** | \(0.77\) |
| C2 block \(G_\tau^{rel}\oplus G_{PP}\) | \(3.56\times10^8\) | \(1.81\times10^9\) | \(1.87\times10^9\) | **\(1.21\times10^{-2}\)** | \(0.77\) |
| **C3 full GN + \(G_T^{rel}\)** | **\(3.18\times10^3\)** | **\(3.04\times10^3\)** | **\(5.22\times10^3\)** | **\(0.806\)** | **\(0.23\)** |
| C4 C3 + corridor GN | \(1.67\times10^4\) | \(1.93\times10^4\) | \(7.63\times10^3\) | \(0.153\) | \(0.23\) |

C3 / C0：\(7.3\times10^{-4}\)、\(9.5\times10^{-4}\)、\(3.1\times10^{-3}\)。

要点：

1. C1 重现 V1：\(P\) 块变好，联合 \(\kappa\) 差 \(80\sim10^3\) 倍，\(\lvert\lambda_{\min}\rvert\) 塌到 \(10^{-2}\)。
2. **C2 ≈ C1**。对角相对时间度量不改变混合近零模态。「给时间加对角缩放」被证伪。
3. **C3 把 \(\lambda_{\max}\) 从 \(10^6\sim10^7\) 压到 \(\sim10^3\)**，\(\lvert\lambda_{\min}\rvert\) 与欧式同量级，\(\eta\) 从 0.74 降到 0.23。这就是 \(G_{TP}\)。
4. 当前走廊权重下 C4 略差于 C3：种子处 active GN 把 \(H_{PP}\) 再拧了一点，还没到约束主导。

| \(M\) | C0 \(\kappa\) | C3 \(\kappa\) | C3/C0 |
|---|---:|---:|---:|
| 5 | \(2.26\times10^5\) | **\(1.02\times10^3\)** | \(4.5\times10^{-3}\) |
| 8 snap0 | \(4.38\times10^6\) | **\(3.18\times10^3\)** | \(7.3\times10^{-4}\) |
| 12 | \(4.46\times10^7\) | **\(6.08\times10^4\)** | \(1.36\times10^{-3}\) |

高 piece 时相对收益更大。

#### 不均匀时长 \(T_{\max}/T_{\min}=8\)

| | \(\kappa\) | \(\lvert\lambda\rvert_{\min}\) | \(\kappa(H_{PP})\) |
|---|---:|---:|---:|
| C0 | \(4.67\times10^6\) | \(7.2\times10^{-3}\) | \(4.21\times10^6\) |
| C1 / C2 / C3 | \(\sim4\times10^8\) | \(\sim1\) | \(\sim4\times10^8\) |
| **C4 + corridor GN** | **\(2.20\times10^5\)** | \(8.4\times10^{-2}\) | \(5.24\times10^4\) |

极端时间比下，控制残差 GN 的 \(G_{PP}\) 不再近似混合目标的 \(H_{PP}\)。**C3 单独不够，C4 或 metric refresh 才救。** 全体几何平均 \(\kappa_{\mathrm{C3}}/\kappa_{\mathrm{C0}}=1.01\times10^{-2}\)（含不均匀点）；去掉该点后约 \(1.5\times10^{-3}\)。C1/C0 几何平均仍是 **230**。

#### \(\lambda_T\) 扫描（snap0 M=8，C3）

| \(\lambda_T\) | \(\kappa(\widetilde H)\) | \(\lvert\lambda\rvert_{\min}\) |
|---|---:|---:|
| \(10^{-3}\) | 3198 | 0.806 |
| \(10^{-1}\) | 3196 | 0.806 |
| \(1\) | 3177 | 0.806 |
| \(10\) | 3009 | 0.806 |
| \(10^2\) | 2035 | 0.803 |

相对时间权几乎不改联合谱。默认 \(\lambda_T=1\) 合理；不要靠它替代 \(G_{TP}\)。

#### 正则 \(\alpha\) 扫描（\(G\leftarrow G+\alpha\,\mathrm{tr}(G)/n\,I\)）

| \(\alpha\) | Cholesky | \(\kappa(G)\) | \(\lambda_{\min}(G)\) |
|---|---|---:|---:|
| \(10^{-12}\) | ok | \(2.16\times10^7\) | 0.104 |
| \(10^{-8}\) | ok | \(2.14\times10^7\) | 0.105 |
| \(10^{-6}\) | ok | \(1.11\times10^7\) | 0.202 |
| \(10^{-4}\) | ok | \(2.28\times10^5\) | 9.88 |

求解器用 \(\alpha\sim10^{-8}\) 足够稳定。\(\alpha=10^{-4}\) 会明显改几何，只适合病态 \(T\) 比。

#### 走廊权扫描（snap0 M=8）

| \(w_c\) | \(\kappa_{\mathrm{C0}}\) | \(\kappa_{\mathrm{C3}}\) | \(\kappa_{\mathrm{C4}}\) | \(\eta_{\mathrm{C0}}\) |
|---|---:|---:|---:|---:|
| \(10\) | \(4.38\times10^6\) | 3209 | \(1.25\times10^4\) | 0.74 |
| \(10^2\) | \(4.38\times10^6\) | 3143 | \(2.04\times10^4\) | 0.74 |
| \(10^3\) | \(4.38\times10^6\) | 3056 | \(2.20\times10^4\) | 0.74 |
| \(10^4\) | \(4.38\times10^6\) | 2989 | \(2.24\times10^4\) | 0.71 |
| \(10^5\) | \(4.38\times10^6\) | \(2.59\times10^4\) | **\(2.05\times10^4\)** | 0.40 |

能量/中等走廊：C3 足够。\(w_c\sim10^5\) 约束开始主导，C4 才反超。

### 4.5 Level D：紧收敛 L-BFGS

经典 L-BFGS：`mem=16`，\(g_\epsilon=10^{-6}\)，\(\delta=10^{-12}\)，最多 500 步。Chart：D0 欧式 / D1 V1 / D2 block / D3 full joint Schur / D4 + corridor GN。

#### 均匀 M=8 snap0（走廊在种子处几乎 inactive）

| | \(J^\star\) | viol | it / ev | L-BFGS ms | metric ms | status |
|---|---:|---:|---:|---:|---:|---|
| D0 | 289.26 | 0 | 500 / 592 | 7.47 | 0.01 | -1008 上限 |
| D1 | 289.33 | 0.010 | 500 / 557 | 7.26 | 0.05 | 上限 |
| D2 | 289.32 | 0.009 | 500 / 589 | 7.96 | 0.07 | 上限 |
| D3 | **289.20** | 0 | 500 / 712 | 9.48 | 0.11 | 上限 |
| D4 | 289.21 | 0 | 500 / 700 | 9.45 | 0.18 | 上限 |

\(\Delta J_{\mathrm{D3/D0}}=1.9\times10^{-4}\)。种子 Hessian 有 7 个负特征值，\(g_\epsilon=10^{-6}\) 对所有 chart 都偏紧，大家都打满 500。白化没有改最优点。

#### 均匀 M=8 snap4（走廊激活）

| | \(J^\star\) | it | L-BFGS ms | 相对欧式墙钟 |
|---|---:|---:|---:|---:|
| D0 | 255.18 | 500 | 7.45 | 1 |
| D1 | 255.22 | 500 | 7.73 | 1.04 |
| D2 | 255.22 | 500 | 8.24 | 1.11 |
| **D3** | **255.14** | **104** | **1.84** | **0.26** |
| D4 | 255.14 | 390 | 6.61 | 0.91 |

Gate 5 在这里成立：\(J\) 对齐，迭代 \(0.21\times\)，墙钟 \(0.26\times\)。D4 正确但比纯 C3 慢——当前 \(w_c=25\) 不需要 active GN。

#### 不均匀 \(T_{\max}/T_{\min}=8\)

| | \(J^\star\) | viol | 备注 |
|---|---:|---:|---|
| D0 | **289.30** | 0 | 欧式仍能摸到好盆地 |
| D1 | 439 | 0.045 | V1 漂了 |
| D2 | 442 | 0 | block 同样漂 |
| D3 无 refresh | 468 | 0.65 | 冻结 \(G(T_0)\) 在大时间漂移下失效 |
| D4 无 refresh | 350 | 0.36 | 好于 D3，仍差于欧式 |

与条件数一致：极端时间比下单次冻结 full GN 不够。

#### 均匀 M=12

| | \(J^\star\) | it | 备注 |
|---|---:|---:|---|
| D0 | 289.28 | 500 上限 | |
| D1 / D2 | 354 | 500 | 质量掉了 |
| **D3** | **289.22** | **360 收敛** | 高维时联合度量开始拉开 |
| D4 | 289.22 | 500 | \(J\) 对齐，迭代仍满 |

### 4.6 Level E：Metric Refresh

在 D3 chart 上：E0 不刷新；E1 最多 1 次；E2 最多 2 次。每 outer 重置 L-BFGS history。

| 快照 | E0 \(J\) | E1 \(J\) | E2 \(J\) | 结论 |
|---|---:|---:|---:|---|
| 均匀 snap0 | 289.20 | 289.13 | 289.01 | 刷新多余，只多花度量时间 |
| 均匀 snap4 | 255.14（104 步已收敛） | 255.13（354 步又打满） | 255.09 | 已收敛后再 refresh 会浪费 |
| **不均匀 ratio=8** | **468** | **289.21** | **289.05** | **1 次 refresh 足够，且必要** |
| M=12 | 289.22（已收敛） | 289.21 | 289.20 | 同 snap4 |

计划问题「1 次 refresh 是否足够」：**极端 duration-ratio 下是，而且必须有；均匀时长下 0 次更好。** 生产条件：

\[
\max_i\frac{\lvert T_i-T_i^{\mathrm{seed}}\rvert}{T_i^{\mathrm{seed}}}>\epsilon_T
\quad\text{或}\quad
\max_i\frac{\|P_i-P_i^{\mathrm{seed}}\|}{s_{\mathrm{corr}}}>\epsilon_P
\]

时才 rebuild（当前阈值 \(0.25\)），不要无条件 outer loop。已收敛后再 freeze 会把 snap4 的 104 步变成 354 步。

### 4.7 Level F-proxy：Physical Fast L-BFGS

`rel_step=0`（不再用求解坐标相对步长），打开 `phase0_guards`（物理 \(T,P\)），`rel_cost=10^{-3}`，最少 10 步。

| 快照 | F0 欧式 \(J\) / it / ms | F1 V1 | F2 full joint | F3 +GN |
|---|---|---|---|---|
| M=8 snap0 | 289.32 / 59 / 1.01 | 289.62 / 105 / 1.65，viol 0.024 | **289.22 / 55 / 0.97** | 289.23 / 52 / 0.94 |
| M=8 snap4 | 255.29 / 49 / 0.69 | **304.9 / 81 / 1.20**，viol 0.28 | **255.15 / 60 / 1.02** | 255.18 / 80 / 1.41 |
| ratio=8 | **289.31 / 129 / 2.15** | 442.5 / 161 | 450.6 / 169 | 385.7 / 55 |
| M=12 | 289.54 / 123 / 2.55 | 331.2 / 218 | **289.22 / 108 / 2.90** | 289.24 / 92 / 2.47 |

观察：

- 均匀时长：full joint 与欧式 **质量对齐**，V1 会停在更差走廊。
- 墙钟：度量 0.10–0.26 ms。Fast 路径欧式已经 ~50–120 步停，full joint 迭代略少但总时间经常持平或略慢。Gate 6 **还不能**作为默认后端的理由。
- 极端时间比：Fast 路径没有 refresh，F2 质量明显差于欧式。必须先接条件刷新，才能谈 onboard A/B。
- F3 在均匀问题上几乎不比 F2 好，在高 \(w_c\) / 高 duration-ratio 上才值得。

---

## 5. 阶段 4：生产接线与 follow-up

全部改动在 `feature/minco-mce-metric`。默认 yaml 未改。

### 5.1 接线行为

| 项 | 行为 |
|---|---|
| 编码 | `FrozenJointWhitening` 作用在整段 \(x=(\tau,\xi)\) |
| Fast stop | `rel_step=0`，`phase0_guards_en=true`，快照与停机用物理 \(T,P\) |
| step bound | `toChart` + `transformDirectionToChart`，界在物理 \(\tau\) |
| refresh | `minco_metric_refresh_max=1`，仅当 \(T\) 相对漂移 \(>0.25\) 或 waypoint 相对漂移 \(>0.25\) |
| H0 | 仅 mode 2；Phase-2 packed 不再对 mode 3/4 注入 H0 |
| CSV | `EXP_METRIC_REFRESH` |

### 5.2 Gate G：生产图恒等

`Tests/minco_production_joint_path_self_test.cpp`（能量 + `LinearTimeCost` + `QuadInvTimeMap` + Frozen Joint）。

| Gate | 结果 |
|---|---|
| G1 encode/decode | **PASS** round-trip \(=0\) |
| G2 \(J(z)=J(x)\) | **PASS** |
| G2 \(g_x^\top dx=g_z^\top dz\) | **PASS** \(1.27\times10^{-14}\) |
| G3 物理 \(\tau\) step bound | **PASS**；\(\|Y\|=5.06\neq0\)，用 \(z\) 当 \(\tau\) 会得到不同界 |
| G4 drift 后 refresh | **PASS** \(z'=0\) 且 \(J\) 不变 |

### 5.3 C3 vs C4 solver switching（紧 L-BFGS max=200）

推荐规则：\(J\) 相对下降 \(>0.5\%\) 才换图。

| 工况 | C3 \(J\) | C4 \(J\) | E1 \(J\) | 推荐 |
|---|---:|---:|---:|---|
| 均匀 M=8，\(w_c=25\) | 289.21 | 289.22 | 289.13 | **C3** |
| 均匀 M=8，\(w_c=10^5\) | 289.22 | 289.46 | 289.16 | **C3** |
| \(T_{\max}/T_{\min}=8\)，\(w_c=25\) | 510.63 | 349.68 | **289.21** | **E1 refresh** |
| \(T_{\max}/T_{\min}=8\)，\(w_c=10^5\) | 1482 | 958 | **376** | **E1 refresh** |

\(\kappa\) 扫描在均匀时长上 \(w_c=10^5\) 时 C4 才略优于 C3（\(2.05\times10^4\) vs \(2.59\times10^4\)）。**求解器上均匀问题仍选 C3**；极端 duration-ratio 用 **一次 refresh**，比切到 C4 更稳地把 \(J\) 拉回 \(\sim289\)。

### 5.4 Fast L-BFGS buckets（物理停机，`rel_step=0`）

\(w_c=25\)。墙钟含度量构建。

| \(M\) | \(T_{\max}/T_{\min}\) | F0 it / ms / \(J\) | F2 it / ms / \(J\) | F2/F0 wall |
|---:|---:|---|---|---:|
| 5 | 1 | 39 / 0.35 / 289 | 34 / 0.45 / 289 | 1.28 |
| 5 | 8 | 46 / 0.46 / 289 | 93 / 1.03 / 289 | 2.22 |
| 8 | 1 | 59 / 0.79 / 289 | 55 / 1.06 / 289 | 1.33 |
| 8 | 8 | 129 / 2.12 / 289 | 169 / 3.90 / **451** | 1.84 |
| 12 | 1 | 123 / 2.52 / 290 | 108 / 3.16 / 289 | 1.25 |
| 12 | 8 | 107 / 2.40 / 450 | 190 / 5.31 / **680** | 2.22 |

\(n=6\)：wall F2/F0 **P50=1.58，P95=2.22**。均匀时长质量对齐；不均匀时长 Fast 早停会把 F2 停在坏的冻结图上，必须靠 refresh（E1）而不是 Fast 默认加速。

### 5.5 State2State A/B

脚本已支持 `MODE_B=4 LABEL_B=frozen_joint`。本机 `/opt/ros` 只有 **Jazzy**，`docker` 不可用，真实 click_demo 未跑。默认 yaml 保持 mode 0。

```bash
MODE_B=4 LABEL_B=frozen_joint \
  bash scripts/run_mce_vs_euclidean_state2state_ab.sh
```

---

## 6. 和 V1 失败现象的对照

| | 固定时间 | 自由时间只白化 \(P\)（V1） | 自由时间 full joint |
|---|---|---|---|
| 决策 | \(P\) | \((\tau,z_P)\) | \(z=L^\top(x-x_0)\)，含 \(Yd\tau\) |
| \(\kappa(H_{PP})\) | \(\to1\sim7\) | \(\to1\sim7\) | 不再单独追求 |
| 联合 \(\kappa\) | 就是 \(\kappa(H_{PP})\) | **差 \(80\sim10^3\)** | **均匀时长好 \(10^3\)** |
| \(\lvert\lambda_{\min}\rvert\) | \(O(1)\) | 塌到 \(10^{-2}\) | 与欧式同量级 |
| \(S_\tau\) | — | 不变 | 被 \(G_{TT},G_{TP}\) 真正改写（\(\kappa(S)\) 从 \(10^3\sim10^4\) 到 \(10^1\sim10^2\)） |
| 紧收敛 \(J\) | 对齐 | 对齐但更慢 | 对齐；走廊激活时更快 |
| Fast 停机 | — | 在 \(z_P\) 上早停，质量差 | 物理 \(T,P\) 停机，均匀时长质量对齐 |

C2 实验把「给时间加对角缩放」证伪了：没有 \(G_{TP}\) 就没有联合谱的改善。

---

## 7. 不应当推出的结论

1. **不是** \(\kappa=1\)。自由时间 \(H_E=2J_r^\top J_r+2\sum r\nabla^2 r\)，GN 只吃第一项。
2. **不是** 任意 duration-ratio 上 C3 都赢。\(T_{\max}/T_{\min}=8\) 需要 C4 或 1 次 refresh。
3. **不是** 无条件 metric refresh 会更快。已收敛后再 freeze 一次会把 104 步变成 354 步。
4. **不是** Fast L-BFGS 墙钟已经低于欧式。质量对齐 ≠ 默认加速。P50 仍是欧式的 \(1.58\times\)。
5. **不是** 可以把 `minco_metric_mode` 改成 1、3 或 4 当生产默认。
6. **不是** V1 waypoint whitening 被推翻。固定时间结果全部仍成立。
7. **不是** 均匀问题上 C4 比 C3 好。高 \(w_c\) 的 \(\kappa\) 优势没有变成更低的 \(J^\star\)。
8. **不是** Fast 物理停机可以替代 refresh。\(T_{\max}/T_{\min}=8\) 上 F2 的 \(J\) 明显差于欧式。
9. **不是** 本轮已经在真实森林 replan 上证明延迟下降。那一步仍要 Noetic。

---

## 8. 建议的生产形态（仍为候选，不是默认）

```text
Full Space-Time Control GN
+ Relative-Time (λ_T = 1)
+ Optional Active GN          （高 w_c 或高 T 比；求解器上更优先 refresh）
+ Frozen Block-Schur Whitening
+ Exact MINCO adjoint
+ Physical Fast Stop          （rel_step 不作为白化图停机）
+ At-most-1 refresh           （仅当 T/P drift 超阈）
```

---

## 9. 复现

```bash
cd src/Planner/general_planner

g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_metric_self_test.cpp -o /tmp/minco_metric

g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_joint_metric_self_test.cpp -o /tmp/minco_joint_metric

g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_joint_whitening_self_test.cpp -o /tmp/minco_joint_whitening

g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_freetime_joint_conditioning_self_test.cpp \
  -o /tmp/minco_freetime_kappa_st

g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_freetime_joint_whitening_comparison_self_test.cpp \
  src/utils/lbfgs.cpp -o /tmp/minco_freetime_joint_cmp

g++ -O2 -std=c++17 -Iinclude -I/usr/include/eigen3 \
  Tests/minco_production_joint_path_self_test.cpp \
  -o /tmp/minco_production_joint_path
```

期望：

```text
e_H ~ 2e-9，κ(G^{-1/2} H_E G^{-1/2}) = 1
A5 e_PP ~ 1e-14
B3/B4/B6 machine precision
geometric mean κ_C3/κ_C0 ~ 1e-2（含不均匀点）
D3 J 与 D0 在均匀时长上相对误差 < 1e-3
E1 在 Tmax/Tmin=8 上把 J 从 ~470 拉回 ~289
G1–G4 production chart identity machine precision
```

---

## 10. 总表

```text
G_PP^{full} = G_PP^{MCE}                 Gate 1，e = 10^{-14}
G_TP ≠ 0，η_F(G) ≈ 0.5–0.73
只白化 P 或再加对角 G_Trel              联合 κ 更差 10²～10³
Full GN pullback + frozen Schur          均匀时长 κ 降 10³，|λmin| 不塌
走廊激活的紧 L-BFGS                      104 vs 500 步，J 对齐
Tmax/Tmin=8                              需要 1 次 refresh 或 C4
Fast L-BFGS                              质量可对齐；墙钟 P50 = 1.58× 欧式
生产接线                                 mode 3/4 Frozen Joint 已落地，默认仍是 0
真实 replan A/B                          未跑，需要 ROS1 Noetic
```
