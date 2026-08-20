# Full Space-Time MINCO Joint Optimization 实验报告

- **日期**：2026-08-20
- **计划文档**：`Full_SpaceTime_MINCO_Joint_Optimization_Experimental_Plan.md`
- **前置**：
  - `Tests/Frozen_MCE_Whitening_Experimental_Report.md`（固定时间 V1）
  - `Tests/FreeTime_Joint_Conditioning_MCE_Whitening.md`（只白化 \(P\) 后联合 \(\kappa\) 变差）
- **本阶段实现**：
  - `include/traj_opt/minco/minco_metric.hpp`：\(G_{\mathrm{ctrl}}=2\rho_E J_r^\top J_r\)，并暴露 \(G_{TT},G_{TP},G_{PP}\)
  - `include/traj_opt/minco/minco_joint_whitening.hpp`：TimeMap pullback、Dense / Block-Schur Frozen Joint Whitening
- **自测**：
  - Level A：`Tests/minco_joint_metric_self_test.cpp`
  - Level B：`Tests/minco_joint_whitening_self_test.cpp`
  - Level C：`Tests/minco_freetime_joint_conditioning_self_test.cpp`
  - Level D/E/F-proxy：`Tests/minco_freetime_joint_whitening_comparison_self_test.cpp`
  - Gate G / 生产图：`Tests/minco_production_joint_path_self_test.cpp`
- **生产开关**：默认保持 `minco_metric_mode: 0`。本轮**不**把 mode 3/4 设成默认。 mode 3/4 现为 Frozen Joint（不再是动态 H0）。

---

## 0. 一句话结论

固定时间 waypoint MCE 是 \(T=T_0\) 切片上的精确 Hessian。自由时间必须用完整

\[
G_\theta=\begin{bmatrix}G_{TT}&G_{TP}\\G_{PT}&G_{PP}\end{bmatrix}
=2\rho_E J_r^\top J_r+\lambda_T\operatorname{diag}(T_i^{-2}),
\]

再经 `QuadInvTimeMap` pullback 到 \((\tau,P)\)，做 **Frozen Full Joint / Block-Schur** 白化。

在均匀时长的 rolling 快照上，联合 \(\kappa\) 从 \(10^6\sim10^7\) 降到 \(10^3\sim10^4\)（约 **\(300\sim1400\times\)**），\(|\lambda|_{\min}\) 不再塌缩。只加对角 \(G_T^{rel}\)（C2）**几乎等于 V1**，交叉项 \(G_{TP}\) 才是修复。紧收敛时 \(J^\star\) 与欧式对齐；在 M=8 snap4 上 D3 用 104 步走完欧式 500 步上限。极端 \(T_{\max}/T_{\min}=8\) 时单次冻结不够，**一次 metric refresh 把 \(J\) 从 468 拉回 289**。Waypoint-only V1 在自由时间里仍然是失败消融，不要打开。

---

## 1. 实现与 Gate 总表

| Gate | 要求 | 结果 |
|---|---|---|
| 1 Full metric 数学 | \(G_{PP}^{\mathrm{full}}\approx G_{PP}^{\mathrm{MCE}}\) | **PASS** \(e_{PP}=9.8\times10^{-15}\) |
| 2 Joint whitening | \(dx^\top G_x dx=\|z\|^2\) | **PASS** \(6.5\times10^{-16}\) |
| 3 Covector | \(g_x^\top dx=g_z^\top dz\) | **PASS** \(2.1\times10^{-16}\) |
| 4 Conditioning | \(\kappa_{\mathrm{C3}}<\kappa_{\mathrm{C0}}\) 且 \(\lvert\lambda_{\min}\rvert\) 不塌缩 | **均匀时长 PASS**（\(300\sim1400\times\)）；**\(T_{\max}/T_{\min}=8\) 时 C3 失败、C4 成功** |
| 5 Tight solver | \(J^\star\) 对齐且迭代下降 | **对齐 PASS**；snap4 **104 vs 500**；snap0 双方打到 500 上限（种子 Hessian 不定） |
| 6 Production wall-clock | \(t_{\mathrm{metric}}+t_{\mathrm{solver}}<t_{\mathrm{Euclidean}}\) | **尚未作为默认依据**。Fast L-BFGS 质量对齐，墙钟持平或略慢（度量 0.1–0.3 ms） |

旧 `minco_metric_self_test` 在 \(2\rho_E\) 对齐后仍 全部 OK。

---

## 2. Level A：残差 Jacobian 与 \(G_{PP}\) 对齐

目标：控制残差

\[
r_i(u)=\sqrt{\rho_E T_i}\,p_i^{(4)}(uT_i)
\]

的 Gauss-Legendre 4 点积分，以及

\[
G_{\mathrm{ctrl}}=2\rho_E J_r^\top J_r.
\]

此前实现少了因子 \(2\rho_E\)，\(G_{PP}^{\mathrm{full}}\) 会比 \(G_{\mathrm{MCE}}\) 小一半。现已乘上。

| 检查 | 误差 / 值 |
|---|---|
| A1 系数 JVP vs FD | \(1.95\times10^{-7}\) |
| A2 控制残差 JVP vs FD | \(9.47\times10^{-9}\) |
| A3 \(\|G-G^\top\|\) | \(0\) |
| A4 \(\lambda_{\min}(G)\) | \(30.9>0\)（SPD） |
| A4 \(\kappa(G)\)、\(\eta_F(G_\theta)\) | \(1.74\times10^4\)，\(\eta=0.729\) |
| A5 \(e_{PP}\) | **\(9.77\times10^{-15}\)** |
| \(\|G_{TP}\|\) | \(1.65\times10^5\)（交叉项真实存在） |

Gate 1 以机器精度通过：full space-time 的 \(P\) 块就是固定时间 MCE。

---

## 3. Level B：Pullback 与 Frozen Joint / Schur

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

---

## 4. Level C：自由时间联合 Hessian 消融

同一组 remaining-horizon 快照，目标 = 能量 + 时间权 20 + vel/acc + 走廊。Hessian 在 \((\tau,P)\) 上中心差分一次，再用各 \(G\) 做合同

\[
\widetilde H=L^{-1}HL^{-T},\qquad G=LL^\top.
\]

\(\kappa=\lvert\lambda\rvert_{\max}/\lvert\lambda\rvert_{\min}\)。种子处 Hessian **不定**（约 4–12 个负特征值），所以不要求 \(\kappa=1\)。

### 4.1 均匀时长 rolling（与 V1 失败报告同一组）

**M=8，snap 0 / 4 / 9**

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
2. **C2 ≈ C1**。对角相对时间度量不改变混合近零模态。计划里「C2 至少好于 C1」在本目标上不成立。
3. **C3 把 \(\lambda_{\max}\) 从 \(10^6\sim10^7\) 压到 \(\sim10^3\)**，\(\lvert\lambda_{\min}\rvert\) 与欧式同量级，\(\eta\) 从 0.74 降到 0.23。这就是 \(G_{TP}\)。
4. 当前走廊权重下 C4 略差于 C3：种子处 active GN 把 \(H_{PP}\) 再拧了一点，还没到约束主导。

**M=5**：C0 \(2.26\times10^5\) → C3 **\(1.02\times10^3\)**（\(4.5\times10^{-3}\)）。

**M=12**：C0 \(4.46\times10^7\) → C3 **\(6.08\times10^4\)**（\(1.36\times10^{-3}\)）。高 piece 时相对收益更大。

### 4.2 不均匀时长 \(T_{\max}/T_{\min}=8\)

| | \(\kappa\) | \(\lvert\lambda\rvert_{\min}\) | \(\kappa(H_{PP})\) |
|---|---:|---:|---:|
| C0 | \(4.67\times10^6\) | \(7.2\times10^{-3}\) | \(4.21\times10^6\) |
| C1 / C2 / C3 | \(\sim4\times10^8\) | \(\sim1\) | \(\sim4\times10^8\) |
| **C4 + corridor GN** | **\(2.20\times10^5\)** | \(8.4\times10^{-2}\) | \(5.24\times10^4\) |

极端时间比下，控制残差 GN 的 \(G_{PP}\) 不再近似混合目标的 \(H_{PP}\)（短段 snap 曲率与走廊/时间权错位）。**C3 单独不够，C4 或 metric refresh 才救。** 这使全体几何平均 \(\kappa_{\mathrm{C3}}/\kappa_{\mathrm{C0}}=1.01\times10^{-2}\)，去掉该点后约 \(1.5\times10^{-3}\)。C1/C0 几何平均仍是 **230**（V1 全面变差）。

### 4.3 \(\lambda_T\) 扫描（snap0 M=8，C3）

| \(\lambda_T\) | \(\kappa(\widetilde H)\) | \(\lvert\lambda\rvert_{\min}\) |
|---|---:|---:|
| \(10^{-3}\) | 3198 | 0.806 |
| \(10^{-1}\) | 3196 | 0.806 |
| \(1\) | 3177 | 0.806 |
| \(10\) | 3009 | 0.806 |
| \(10^2\) | 2035 | 0.803 |

相对时间权几乎不改联合谱。默认 \(\lambda_T=1\) 合理；不要靠它替代 \(G_{TP}\)。

### 4.4 正则 \(\alpha\) 扫描（\(G\leftarrow G+\alpha\,\mathrm{tr}(G)/n\,I\)）

| \(\alpha\) | Cholesky | \(\kappa(G)\) | \(\lambda_{\min}(G)\) |
|---|---|---:|---:|
| \(10^{-12}\) | ok | \(2.16\times10^7\) | 0.104 |
| \(10^{-8}\) | ok | \(2.14\times10^7\) | 0.105 |
| \(10^{-6}\) | ok | \(1.11\times10^7\) | 0.202 |
| \(10^{-4}\) | ok | \(2.28\times10^5\) | 9.88 |

求解器用 \(\alpha\sim10^{-8}\) 足够稳定。\(\alpha=10^{-4}\) 会明显改几何，只适合病态 \(T\) 比。

### 4.5 走廊权扫描（snap0 M=8）

| \(w_c\) | \(\kappa_{\mathrm{C0}}\) | \(\kappa_{\mathrm{C3}}\) | \(\kappa_{\mathrm{C4}}\) | \(\eta_{\mathrm{C0}}\) |
|---|---:|---:|---:|---:|
| \(10\) | \(4.38\times10^6\) | 3209 | \(1.25\times10^4\) | 0.74 |
| \(10^2\) | \(4.38\times10^6\) | 3143 | \(2.04\times10^4\) | 0.74 |
| \(10^3\) | \(4.38\times10^6\) | 3056 | \(2.20\times10^4\) | 0.74 |
| \(10^4\) | \(4.38\times10^6\) | 2989 | \(2.24\times10^4\) | 0.71 |
| \(10^5\) | \(4.38\times10^6\) | \(2.59\times10^4\) | **\(2.05\times10^4\)** | 0.40 |

能量/中等走廊：C3 足够。\(w_c\sim10^5\) 约束开始主导，C4 才反超。与计划 §31 一致。

---

## 5. Level D：紧收敛 L-BFGS

经典 L-BFGS：`mem=16`，\(g_\epsilon=10^{-6}\)，\(\delta=10^{-12}\)，最多 500 步。Chart：D0 欧式 / D1 V1 / D2 block / D3 full joint Schur / D4 + corridor GN。

### 5.1 均匀 M=8 snap0（走廊在种子处几乎 inactive）

| | \(J^\star\) | viol | it / ev | L-BFGS ms | metric ms | status |
|---|---:|---:|---:|---:|---:|---|
| D0 | 289.26 | 0 | 500 / 592 | 7.47 | 0.01 | -1008 上限 |
| D1 | 289.33 | 0.010 | 500 / 557 | 7.26 | 0.05 | 上限 |
| D2 | 289.32 | 0.009 | 500 / 589 | 7.96 | 0.07 | 上限 |
| D3 | **289.20** | 0 | 500 / 712 | 9.48 | 0.11 | 上限 |
| D4 | 289.21 | 0 | 500 / 700 | 9.45 | 0.18 | 上限 |

\(\Delta J_{\mathrm{D3/D0}}=1.9\times10^{-4}\)。种子 Hessian 有 7 个负特征值，\(g_\epsilon=10^{-6}\) 对所有 chart 都偏紧，大家都打满 500。白化没有改最优点。

### 5.2 均匀 M=8 snap4（走廊激活）

| | \(J^\star\) | it | L-BFGS ms | 相对欧式墙钟 |
|---|---:|---:|---:|---:|
| D0 | 255.18 | 500 | 7.45 | 1 |
| D1 | 255.22 | 500 | 7.73 | 1.04 |
| D2 | 255.22 | 500 | 8.24 | 1.11 |
| **D3** | **255.14** | **104** | **1.84** | **0.26** |
| D4 | 255.14 | 390 | 6.61 | 0.91 |

Gate 5 在这里成立：\(J\) 对齐，迭代 \(0.21\times\)，墙钟 \(0.26\times\)。D4 正确但比纯 C3 慢——当前 \(w_c=25\) 不需要 active GN。

### 5.3 不均匀 \(T_{\max}/T_{\min}=8\)

| | \(J^\star\) | viol | 备注 |
|---|---:|---:|---|
| D0 | **289.30** | 0 | 欧式仍能摸到好盆地 |
| D1 | 439 | 0.045 | V1 漂了 |
| D2 | 442 | 0 | block 同样漂 |
| D3 无 refresh | 468 | 0.65 | 冻结 \(G(T_0)\) 在大时间漂移下失效 |
| D4 无 refresh | 350 | 0.36 | 好于 D3，仍差于欧式 |

与 §4.2 一致：极端时间比下单次冻结 full GN 不够。

### 5.4 均匀 M=12

| | \(J^\star\) | it | 备注 |
|---|---:|---:|---|
| D0 | 289.28 | 500 上限 | |
| D1 / D2 | 354 | 500 | 质量掉了 |
| **D3** | **289.22** | **360 收敛** | 高维时联合度量开始拉开 |
| D4 | 289.22 | 500 | \(J\) 对齐，迭代仍满 |

---

## 6. Level E：Metric Refresh

在 D3 chart 上：E0 不刷新；E1 最多 1 次；E2 最多 2 次。每 outer 重置 L-BFGS history。

| 快照 | E0 \(J\) | E1 \(J\) | E2 \(J\) | 结论 |
|---|---:|---:|---:|---|
| 均匀 snap0 | 289.20 | 289.13 | 289.01 | 刷新多余，只多花度量时间 |
| 均匀 snap4 | 255.14（104 步已收敛） | 255.13（354 步又打满） | 255.09 | 已收敛后再 refresh 会浪费 |
| **不均匀 ratio=8** | **468** | **289.21** | **289.05** | **1 次 refresh 足够，且必要** |
| M=12 | 289.22（已收敛） | 289.21 | 289.20 | 同 snap4 |

计划问题「1 次 refresh 是否足够」：**极端 duration-ratio 下是，而且必须有；均匀时长下 0 次更好。** 建议生产条件仍用文档公式：

\[
\max_i\frac{\lvert T_i-T_i^{\mathrm{seed}}\rvert}{T_i^{\mathrm{seed}}}>\epsilon_T
\quad\text{或}\quad
\max_i\frac{\|P_i-P_i^{\mathrm{seed}}\|}{s_{\mathrm{corr}}}>\epsilon_P
\]

时才 rebuild，不要无条件 outer loop。

---

## 7. Level F-proxy：Physical Fast L-BFGS

`rel_step=0`（不再用求解坐标相对步长），打开 `phase0_guards`（物理 \(T,P\)），`rel_cost=10^{-3}`，最少 10 步。这是计划 §13 / §25，不是 yaml 里欧式用的 `rel_step` 停机。

| 快照 | F0 欧式 \(J\) / it / ms | F1 V1 | F2 full joint | F3 +GN |
|---|---|---|---|---|
| M=8 snap0 | 289.32 / 59 / 1.01 | 289.62 / 105 / 1.65，viol 0.024 | **289.22 / 55 / 0.97** | 289.23 / 52 / 0.94 |
| M=8 snap4 | 255.29 / 49 / 0.69 | **304.9 / 81 / 1.20**，viol 0.28 | **255.15 / 60 / 1.02** | 255.18 / 80 / 1.41 |
| ratio=8 | **289.31 / 129 / 2.15** | 442.5 / 161 | 450.6 / 169 | 385.7 / 55 |
| M=12 | 289.54 / 123 / 2.55 | 331.2 / 218 | **289.22 / 108 / 2.90** | 289.24 / 92 / 2.47 |

观察：

- 均匀时长：full joint 与欧式 **质量对齐**，V1 会停在更差走廊。
- 墙钟：度量 0.10–0.26 ms。Fast 路径欧式已经 ~50–120 步停，full joint 迭代略少但总时间经常持平或略慢。Gate 6 **还不能**作为默认后端的理由。
- 极端时间比：Fast 路径没有 refresh，F2 质量明显差于欧式。必须先接 E 的条件刷新，才能谈 onboard A/B。
- F3 在均匀问题上几乎不比 F2 好，在高 \(w_c\) / 高 duration-ratio 上才值得。

真实 click_demo / ROS1 State2State 闭环本机没有 Noetic，未跑。脚本仍是 `scripts/run_mce_vs_euclidean_state2state_ab.sh`；接 full joint 前需要新的 mode 语义（计划 §36）和 refresh，而不是把现有 mode 1 打开。

---

## 8. 和 V1 失败现象的对照

| | 固定时间 | 自由时间只白化 \(P\)（V1） | 自由时间 full joint（本轮） |
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

## 9. 不应当推出的结论

1. **不是** \(\kappa=1\)。自由时间 \(H_E=2J_r^\top J_r+2\sum r\nabla^2 r\)，GN 只吃第一项。
2. **不是** 任意 duration-ratio 上 C3 都赢。\(T_{\max}/T_{\min}=8\) 需要 C4 或 1 次 refresh。
3. **不是** 无条件 metric refresh 会更快。已收敛后再 freeze 一次会把 104 步变成 354 步。
4. **不是** Fast L-BFGS 墙钟已经低于欧式。质量对齐 ≠ 默认加速。
5. **不是** 可以把 `minco_metric_mode` 改成 3 或 4 当生产默认。
6. **不是** V1 waypoint whitening 被推翻。固定时间结果全部仍成立。

---

## 10. 建议的生产形态（仍为候选，不是默认）

与计划最终形式一致：

```text
Full Space-Time Control GN
+ Relative-Time (λ_T = 1)
+ Optional Active GN   （高 w_c 或高 T 比）
+ Frozen Block-Schur Whitening
+ Exact MINCO adjoint
+ Physical Fast Stop（rel_step 不作为白化图停机）
+ At-most-1 refresh，仅当 T/P drift 超阈
```

内部 mode 语义按计划 §36：1=waypoint V1，2=dynamic H0，3=frozen block ST，4=frozen full ST GN。`traj_manager` 现已把 mode 3/4 接到 `FrozenJointWhitening`（不再把 3/4 当动态 H0）。默认 yaml 仍是 mode 0。

---

## 11. 复现

```bash
cd src/Planner/general_planner

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
A5 e_PP ~ 1e-14
B3/B4/B6 machine precision
geometric mean κ_C3/κ_C0 ~ 1e-2（含不均匀点）
D3 J 与 D0 在均匀时长上相对误差 < 1e-3
E1 在 Tmax/Tmin=8 上把 J 从 ~470 拉回 ~289
G1–G4 production chart identity machine precision
```

Noetic A/B（本机未跑）：

```bash
MODE_B=4 LABEL_B=frozen_joint \
  bash scripts/run_mce_vs_euclidean_state2state_ab.sh
```

---

## 12. 总结

```text
G_PP^{full} = G_PP^{MCE}                 Gate 1，e = 10^{-14}
G_TP ≠ 0，η_F(G) ≈ 0.5–0.73
只白化 P 或再加对角 G_Trel              联合 κ 更差 10²～10³
Full GN pullback + frozen Schur          均匀时长 κ 降 10³，|λmin| 不塌
走廊激活的紧 L-BFGS                      104 vs 500 步，J 对齐
Tmax/Tmin=8                              需要 1 次 refresh 或 C4
Fast L-BFGS                              质量可对齐；墙钟尚未稳定优于欧式
```

生产接线（mode 3/4 Frozen Joint + 至多 1 次 refresh + 物理 Fast stop）已在本分支落地，隔离 Gate G1–G4 / C3–C4 switching / Fast buckets 已跑完。默认 yaml 仍是 `minco_metric_mode: 0`。真实 State2State A/B 需要 ROS1 Noetic 容器，本机只有 Jazzy。

---

## 13. Follow-up：生产接线与 C3/C4 / Fast buckets

日期：2026-08-20。全部改动在 `feature/minco-mce-metric`。默认 `minco_metric_mode: 0` 未改。

### 13.1 生产接线

| 项 | 行为 |
|---|---|
| mode 1 | Frozen waypoint MCE（V1，自由时间消融） |
| mode 2 | 动态 waypoint H0，仅此模式把 \(H_0\) 交给 Fast L-BFGS |
| mode 3 | Frozen block space-time joint whitening |
| mode 4 | Frozen full space-time GN joint whitening（Block-Schur，含 \(Y d\tau\)） |
| 编码 | `FrozenJointWhitening` 作用在整段 \(x=(\tau,\xi)\) |
| Fast stop | `rel_step=0`，`phase0_guards_en=true`，快照与停机用物理 \(T,P\) |
| step bound | `toChart` + `transformDirectionToChart`，界在物理 \(\tau\) |
| refresh | `minco_metric_refresh_max=1`，仅当 \(T\) 相对漂移 \(>0.25\) 或 waypoint 相对漂移 \(>0.25\) |
| CSV | `EXP_METRIC_REFRESH` |

`MINCOOptimizer` 在 `encode` / `decode` / `writeDecisionGradient` 走联合图。Phase-2 packed 不再对 mode 3/4 注入 H0。

### 13.2 Gate G：生产图恒等

`Tests/minco_production_joint_path_self_test.cpp`（能量 + `LinearTimeCost` + `QuadInvTimeMap` + Frozen Joint）。

| Gate | 结果 |
|---|---|
| G1 encode/decode | **PASS** round-trip \(=0\) |
| G2 \(J(z)=J(x)\) | **PASS** |
| G2 \(g_x^\top dx=g_z^\top dz\) | **PASS** \(1.27\times10^{-14}\) |
| G3 物理 \(\tau\) step bound | **PASS**；\(\|Y\|=5.06\neq0\)，用 \(z\) 当 \(\tau\) 会得到不同界 |
| G4 drift 后 refresh | **PASS** \(z'=0\) 且 \(J\) 不变 |

### 13.3 C3 vs C4 solver switching（紧 L-BFGS max=200）

同一套 rolling / uneven 快照。推荐规则：\(J\) 相对下降 \(>0.5\%\) 才换图。

| 工况 | C3 \(J\) | C4 \(J\) | E1 \(J\) | 推荐 |
|---|---:|---:|---:|---|
| 均匀 M=8，\(w_c=25\) | 289.21 | 289.22 | 289.13 | **C3** |
| 均匀 M=8，\(w_c=10^5\) | 289.22 | 289.46 | 289.16 | **C3** |
| \(T_{\max}/T_{\min}=8\)，\(w_c=25\) | 510.63 | 349.68 | **289.21** | **E1 refresh** |
| \(T_{\max}/T_{\min}=8\)，\(w_c=10^5\) | 1482 | 958 | **376** | **E1 refresh** |

\(\kappa\) 扫描（Level C）在均匀时长上 \(w_c=10^5\) 时 C4 才略优于 C3（\(2.05\times10^4\) vs \(2.59\times10^4\)）。**求解器上均匀问题仍选 C3**；极端 duration-ratio 用 **一次 refresh**，比切到 C4 更稳地把 \(J\) 拉回 \(\sim289\)。

### 13.4 Fast L-BFGS buckets（物理停机，`rel_step=0`）

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

### 13.5 State2State A/B

脚本已支持 `MODE_B=4 LABEL_B=frozen_joint`。本机 `/opt/ros` 只有 **Jazzy**，`docker` 不可用，真实 click_demo 未跑。默认 yaml 保持 mode 0。

### 13.6 不应当推出的结论

1. **不是** 可以把 mode 4 设成默认。Fast 墙钟 P50 仍是欧式的 \(1.6\times\)。
2. **不是** 均匀问题上 C4 比 C3 好。高 \(w_c\) 的 \(\kappa\) 优势没有变成更低的 \(J^\star\)。
3. **不是** Fast 物理停机可以替代 refresh。\(T_{\max}/T_{\min}=8\) 上 F2 的 \(J\) 明显差于欧式。
4. **不是** 本轮已经在真实森林 replan 上证明延迟下降。那一步仍要 Noetic。

