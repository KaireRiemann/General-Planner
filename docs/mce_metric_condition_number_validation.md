# 轨迹度量、坐标不变性与 MCE 条件数验证

- **日期**：2026-08-17
- **范围**：MINCO-S4 + Fast L-BFGS；固定时间隔离实验，不含 onboard ROS 全节点
- **对应框架**：
  - `Euclidean_Gradient_Basis_Invariance_Validation_Framework.md`
  - `MCE_Metric_Condition_Number_Validation_Framework.md`
- **本地自测**（不入库，见仓库 `.gitignore`）：
  `src/Planner/general_planner/Tests/`
- **检查**：梯度层 35/35，优化层 14/14，条件数 43/43；合计 92/92 通过

---

## 0. 一句话结论

规划器优化的是物理轨迹 \(p(t)\)，不是某一组系数。各坐标系若都设 \(G=I\)，欧式最陡下降映回轨迹空间后一般不是同一个 \(\delta p(t)\)；L-BFGS 路径、早停和非凸盆地也会跟着分叉。度量按坐标正确 pullback 后，physical 更新方向恢复一致。

在已经固定的 MINCO waypoint 坐标上，MCE 度量还额外对齐了 snap energy 的 Hessian：纯 MCE 的条件数从 \(7\)–\(1.6\times 10^7\) 降到 \(1.000\)，混合目标仍能降约两个数量级，并同步减少 L-BFGS 迭代。\(L^2\) 能恢复不变性，但对 snap 主导的 Hessian **不是**好预条件（\(R_\kappa\approx 0.56\)）。

工程用法不是换求解器，而是：冻结 \(G_{\mathrm{MCE}}\)（走廊激活时再加 Gauss-Newton），只白化路点块 \(P\)，在 \(z=L^TP\) 上继续跑现有 Fast L-BFGS；早停看物理量，不要看拼接决策向量的 `rel_step`。

---

## 1. 研究主线

```text
轨迹是坐标无关对象
        ↓
不同 basis / reduced coordinates 只是表示
        ↓
Euclidean G=I 在不同坐标中不是同一几何
        ↓
Euclidean gradient 的物理轨迹方向坐标依赖     ← 阶段 1
        ↓
pullback metric 恢复方向不变性                 ← 阶段 1
        ↓
选择 MCE / Sobolev trajectory metric
        ↓
whitened Hessian 的 condition number 下降     ← 阶段 2
        ↓
L-BFGS 迭代 / 线搜索同步下降                   ← 阶段 2（隔离 MINCO）
        ↓
真实 State2State runtime（含 metric 成本）     ← 未做
```

两阶段回答的问题不同：

| 阶段 | 问题 | 核心比较 |
|---|---|---|
| 1 不变性 | 为什么不能在每种参数化里都设 \(G=I\)？ | \(\Phi(t)(-\nabla_a J)\) vs \(B(t)(-\nabla_c J)\) |
| 2 条件数 | 为什么某一个 intrinsic metric 数值上更合适？ | \(\kappa(H_J)\) vs \(\kappa(G^{-1/2}H_JG^{-1/2})\) |

错误对照是比较 \(\nabla_a J\) 与 \(\nabla_c J\) 的分量。梯度作为 covector 换基后本来就该变。

---

## 2. 阶段 1：欧式梯度的参数化依赖

### 2.1 命题

同一物理轨迹、同一 \(J\)、不同坐标、各自 \(G=I\) \(\Longrightarrow\) \(\delta p_1(t)\neq\delta p_2(t)\)。

度量随坐标正确变换后，natural gradient 满足 \(\delta p_a^N(t)=\delta p_c^N(t)\)。

### 2.2 设计

只改坐标 / 度量。轨迹、objective、初值、次数、时间、梯度求值完全一致。梯度层不引入 L-BFGS、障碍、自由时间。优化层只额外打开 L-BFGS 与早停。

程序：

- `Tests/basis_gradient_invariance_self_test.cpp`
- `Tests/coordinate_opt_invariance_self_test.cpp`

### 2.3 梯度层结果（35/35）

| 实验 | 轨迹误差 | 欧式方向失配 \(e_E\) | Natural 失配 \(e_N\) |
|---|---:|---:|---:|
| Level 1 线性代数 | — | \(6.40\) | \(3.08\times 10^{-15}\) |
| Level 2A Power↔Bernstein 二次 | \(\sim 10^{-16}\) | \(0.584\) | \(\sim 10^{-16}\) |
| Level 2B 七次 | \(\sim 10^{-16}\) | \(0.164\) | \(\sim 10^{-12}\) |
| Level 2C 三维七次 | \(\sim 10^{-16}\) | \(0.920\) | \(\sim 10^{-11}\) |
| Level 3 MINCO \(P\) vs \(y=RP\) | \(\sim 10^{-13}\) | \(4.43\) | \(10^{-17}\)–\(10^{-14}\) |

### 2.4 优化层结果（14/14）

| 实验 | 欧式路径 | Natural 路径 |
|---|---|---|
| 非凸 Power/Bernstein L-BFGS | 第 1 步 RMS \(0.33\)；不同局部极小 \(J^\star=-0.034\) vs \(-0.127\) | RMS \(\sim 10^{-15}\)；同一盆地 |
| 参数空间 `rel_cost`/`rel_step` 早停 | 冻在不同 \(p(t)\)（RMS \(0.63\)） | 物理终点重合 |
| MINCO \(P\) vs \(y=RP\)（强凸收满） | 路径不同，终点 \(J^\star\) 相同 | 每步重合 |

同一终点时的迭代（不能拿“进了另一个盆地”的欧式 Bernstein 来比快慢）：

| 设定 | 欧式 | Natural |
|---|---|---|
| 纯二次 \(J=\frac12\int p^2\) | 16 / 12 | **2 / 2** |
| 非凸多项式，同一盆地 | Power 24 | **11 / 10** |
| MINCO \(P\) | 11 | 11 |
| MINCO \(y=RP\)（故意拧过的坐标） | **34** | **11** |

Natural 保证的是同一几何里优化同一条轨迹。速度收益主要来自消掉坏尺度，以及 \(G\approx H_J\) 时的 Newton 效应。它不是“随便换坐标都少很多步”。

---

## 3. 阶段 2：条件数与 Hessian whitening

### 3.1 理论

局部二次模型 \(J(x+\delta x)\approx J+g^T\delta x+\frac12\delta x^TH_J\delta x\)。若 \(H_J\succ 0\)，

\[
\kappa_E=\kappa(H_J)=\frac{\lambda_{\max}}{\lambda_{\min}}.
\]

\(\kappa\gg 1\)：等高线扁、gradient zig-zag、线搜索变短、quasi-Newton 要更多步学曲率、参数空间早停更容易误判收敛。

设 \(G=LL^T\succ 0\)，\(z=L^Tx\)，则

\[
\widetilde H_J=G^{-1/2}H_JG^{-1/2},
\qquad
\kappa_G=\kappa(\widetilde H_J),
\qquad
R_\kappa=\frac{\kappa_E}{\kappa_G}.
\]

Natural gradient \(d=-G^{-1}\nabla J\) 等价于在 \(z\) 里做欧式梯度。比较 \(\kappa_E\) 与 \(\kappa_G\) 就是在问：metric 是否具有有效预条件作用。

固定时间 MINCO 控制能量（本仓库 `getEnergy()` 不含 \(1/2\)）：

\[
G_{\mathrm{MCE}}=2J_P^TQJ_P=H_{E,\mathrm{reduced}}.
\]

因此 \(\kappa(G_{\mathrm{MCE}}^{-1/2}H_EG_{\mathrm{MCE}}^{-1/2})=1\)。这是解析结论。混合目标

\[
H_J=\rho_EG_{\mathrm{MCE}}+H_{\mathrm{other}}
\quad\Longrightarrow\quad
\widetilde H_J=\rho_EI+G_{\mathrm{MCE}}^{-1/2}H_{\mathrm{other}}G_{\mathrm{MCE}}^{-1/2}.
\]

MCE 精确消除 energy 的各向异性，残差只来自 other cost。

### 3.2 数值做法

- MINCO-S4，3D，固定时间，只优化 waypoint \(P\)
- 中心差分 Hessian：\(h=\eta\max(1,|P_j|)\)，\(\eta=10^{-6}\)，再 \(H\leftarrow\frac12(H+H^T)\)
- Whitening：Cholesky 三角求解，不显式求逆
- 非凸：同时记 \(n_{\mathrm{neg}}\) 与 \(\kappa_+=\lambda_{\max}^+/\lambda_{\min}^+\)（阈值 \(10^{-10}\lambda_{\max}\)）
- Frozen-metric L-BFGS：solve 开头取 \(G_0=G(x_0)\)，全程 \(z=L^Tx\)
- 度量：\(I\)，\(L^2\)，\(H^2\)，MCE，Mix \(=L^2+\mathrm{MCE}\)，MCE+走廊 GN

程序：`Tests/minco_metric_conditioning_self_test.cpp`

### 3.3 Pure-MCE（解析结论的数值确认）

\(e_H=\|H^{FD}-G_{\mathrm{MCE}}\|_F/\max(1,\|H^{FD}\|_F)\sim 10^{-9}\)。

| \(M\) | \(n\) | \(\kappa_E\) | \(\kappa_{\mathrm{MCE}}\) | \(R_\kappa\) |
|---|---:|---:|---:|---:|
| 3 | 6 | \(7.14\) | \(1.000\) | \(7.14\) |
| 5 | 12 | \(320\) | \(1.000\) | \(320\) |
| 10 | 27 | \(6.46\times 10^{4}\) | \(1.000\) | \(6.46\times 10^{4}\) |
| 20 | 57 | \(1.58\times 10^{7}\) | \(1.000\) | \(1.58\times 10^{7}\) |

\(M=5\) 时间比例：

| 模式 | \(\kappa_E\) | \(\kappa_{\mathrm{MCE}}\) |
|---|---:|---:|
| uniform \([1,1,1,1,1]\) | \(319\) | \(1.000\) |
| mild | \(807\) | \(1.000\) |
| strong | \(9.88\times 10^{4}\) | \(1.000\) |
| extreme \([0.2,0.4,1.5,2.0,0.9]\) | \(1.36\times 10^{6}\) | \(1.000\) |

piece 数和时间不均匀都在**同一个物理问题**里把 \(G=I\) 拧坏。MCE 两边都回到 1。规划器“段一多 / 时间一不均，L-BFGS 就磨”，有一部分原因在这里。

纯 MCE、\(M=5\)：欧式 19 步 / 28 次求值，MCE **2 步 / 8 次求值**，同一 \(J^\star=32.256\)。L-BFGS 在 \(G=I\) 下花的步，很大一部分是在学 \(G_{\mathrm{MCE}}\) 本身。

### 3.4 Mixed objective（\(M=5\)，同一 \(J^\star\)，\(n_{\mathrm{neg}}=0\)）

| Case | \(\kappa_E\) | \(\kappa_{L^2}\) | \(\kappa_{H^2}\) | \(\kappa_{\mathrm{MCE}}\) | \(\kappa_{\mathrm{Mix}}\) | \(\kappa_{\mathrm{MCE+GN}}\) | 欧式迭代 | MCE 迭代 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| MCE+\(L^2\) 跟踪 | 288 | 514 | 12.7 | 1.11 | 1.10 | — | 20 | **5** |
| MCE+vel/acc | 317 | 567 | 13.9 | 1.008 | 1.008 | — | 20 | **3** |
| MCE+走廊 | 320 | 572 | 14.0 | 1.16 | 1.16 | **1.000** | 21 | **4** |
| Planner-like | 294 | 525 | 12.9 | 1.25 | 1.23 | 1.09 | 21 | **5** |
| MCE+cosine | 320 | 572 | 14.0 | 1.15 | 1.15 | — | 29 | **4–5** |

排序：

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

走廊残差把纯 MCE 从 \(1.000\) 抬到 \(1.160\)；加上 \(G_c=J_r^TWJ_r\) 后回到 \(1.000\)。Planner-like 是隔离 proxy（energy + tracking + vel/acc + corridor），**不是** ROS State2State。

Cosine 探针未使 Hessian 变 indefinite（snap 最小特征值约 71）。\(\kappa_+\) 通路已接通，但不能外推障碍问题总是 SPD。

---

## 4. 两阶段合在一起意味着什么

测试把两件常被混在一起的事拆开了。

**几何一致性**：\(L^2\)、snap、Mix 只要按坐标 pullback，\(\delta p(t)\) 都不变。\(G=I\) 在 Power / Bernstein / \(P\) / \(y=RP\) 里是不同几何。

**数值 conditioning**：在已经固定的坐标里，还要问 \(G\) 是否对齐 \(H_J\)。

| 度量 | 不变性 | 对本轮 Hessian 的预条件 |
|---|---|---|
| \(G=I\) | 错 | 错，且随 \(M\)、时间比爆炸 |
| \(G_{L^2}\) | 对 | 更错（谱更扁，线搜索评估 25→91） |
| \(G_{H^2}\) | 对 | 中等（\(\sim 23\times\)） |
| \(G_{\mathrm{MCE}}\) | 对 | 接近 Newton |
| Mix / MCE+GN | 对 | 本轮几乎等于 MCE；走廊时 GN 把 \(\kappa\) 拉回 1 |

能恢复不变性的 metric 不必是好的预条件。不要因为 \(L^2\) “更像轨迹空间内积”就拿它当规划器预条件。

所有 mixed case 的 \(J^\star\) 与度量无关。本轮证明的是**同一盆地里的局部二次效率**，不是更好的终点，也不是全局最优。阶段 1 里欧式 Bernstein 曾更快掉进另一个局部极小——那种“更快”不能算进条件数表。

---

## 5. 对 General-Planner 的实际价值

当前 `ExpTrajOpt`：决策量是拼接的 \((T,P)\)（经 `time_map` / `spatial_map`），默认 \(G=I\)，Fast L-BFGS 早停看 `rel_cost` + `rel_step`。`click_real_highspeed.yaml` 里 `penna_pos` 到 \(5\times 10^7\)，`lbfgs_fast_phase0_guards_en` 被关掉，注释写明梯度守卫在大走廊权重下尺度敏感。

### 5.1 现在就能用、不必改求解器

1. 比较快慢时比 \(J\)、violation、物理路点/时间变化、wall-clock，不要比 Euclidean 迭代数。
2. 早停优先用已有物理量：`relative_waypoint_step`、`relative_physical_time_change`、penalty。拼接向量的 `rel_step` 把短段、长段、时间和路点混在同一相对步长里。
3. 段数多、时间很不均匀、replan 变慢时，先怀疑 \(P\) 空间尺度，而不是先加 `mem_size` 或把 `rel_step` 从 `2e-2` 拧到 `1e-3`。
4. 换多项式基或凸包表示，若两边都是 \(G=I\)，路径分叉是预期现象，不是 bug。
5. **不要**用 \(G_{L^2}\) 做默认预条件。

### 5.2 第一版代码形态：frozen MCE，只作用在 \(P\)

Natural gradient 不是新求解器。现有 Fast L-BFGS 继续当欧式拟牛顿，变量换成 \(z=L^Tx\)。

一次 `optimize()` 开始、`generate()` 之后：

1. 用冻结的 \(T_0\) 组装 \(G_{\mathrm{MCE}}=2J_P^TQJ_P\)（对齐 `getEnergy()` Hessian）。
2. \(G=LL^T\)，只变换路点块 \(z_P=L^TP\)。时间块保持现有 `time_map`，不要塞进 \(G_{\mathrm{MCE}}\)。
3. 回调：\(P=L^{-T}z_P\)，\(g_z=L^{-1}g_P\)。
4. 开关默认关。记录迭代、线搜索、\(t_{\mathrm{metric}}\)、\(J\)、max violation、是否 fast-stop。失败回退 \(G=I\)。

验收：同一初值、同一停机，violation 不差；迭代或线搜索下降；\(t_{\mathrm{metric}}+t_{\mathrm{opt}}\) 仍小于现在的 \(t_{\mathrm{opt}}\)。

测试里 \(n=12\) 时 metric 组装约 \(0.4\,\mathrm{ms}\)（与一次短 solve 同量级），\(n=57\) 约 \(31\,\mathrm{ms}\)。replan 5 Hz、piece 很多时，这是唯一可能“理论更快、墙上更慢”的地方。wall-clock **必须含 metric 成本**。

### 5.3 走廊激活后再加 GN，仍然 freeze

\[
G_{\mathrm{frozen}}=G_{\mathrm{MCE}}+J_{\mathrm{corr}}^TWJ_{\mathrm{corr}}.
\]

仍只在 solve 开头构造一次。不要每步更新 \(G_k\)。Mix 在 energy 主导时几乎没有额外好处，不必作为第一版。

隔离测试里 Hessian 仍被 snap 主导，所以纯 MCE 极强。真机 `penna_pos` / `penna_vel` 很大。若惩罚已经压过 energy，只用 \(G_{\mathrm{MCE}}\) 可能重演 \(L^2\) 那种“intrinsic 但更慢”。经验规则：

- energy 梯度明显更大 → frozen MCE
- 两者同量级 → MCE+GN
- 惩罚大一个数量级以上 → 不要只开纯 MCE；至少加 GN，或先保持 \(G=I\)
- \(n\) 很大且 \(t_{\mathrm{metric}}\) 已接近 objective 时间 → 对 \(P\) 做块对角近似，不要追求稠密 \(G\)

自由时间放到更后：先证明 \(P\) 块有收益，再单独给 \(T\) 做尺度。用 snap 的 \(P\)-Hessian 去白化 \(T\) 是误用。

### 5.4 使用原则

> 给 Fast L-BFGS 换一套和 MINCO energy 对齐的冻结坐标，而不是换求解器。

是否装进 onboard，只由“含 metric 成本的 wall-clock + violation”决定，不能从隔离测试的 19→2 步直接外推。

---

## 6. 实验顺序与完成状态

| # | 内容 | 状态 |
|---|---|---|
| 1 | Pure MCE：\(H\approx G_{\mathrm{MCE}}\)，whiten 后 \(\kappa\approx 1\) | 完成 |
| 1b | 不同 segment-time 比例 | 完成 |
| 2 | MCE + \(L^2\) tracking | 完成 |
| 3 | MCE + vel/acc | 完成 |
| 4 | MCE + corridor | 完成 |
| 5 | 约束 Gauss-Newton 恢复 conditioning | 完成 |
| 6 | Frozen-metric L-BFGS，关联 \(\kappa\) 与迭代 | 完成 |
| 7 | General-Planner State2State 全节点 runtime | 未做 |
| 8 | Free-time，metric 扩到 \((P,T)\) | 未做 |
| 9 | Dynamic metric | 未做 |

---

## 7. 当前不能宣称的内容

1. MCE 是唯一最佳 metric。
2. MCE 在所有 obstacle 问题中都优于 \(L^2\)（本轮 Hessian 仍是 snap 主导）。
3. dynamic metric 一定优于 frozen metric。
4. 更小 condition number 保证更低最终 objective。
5. natural gradient 保证全局最优。
6. metric-aware optimizer 在所有场景都比任意 Euclidean 参数化更快。
7. onboard runtime 一定更好（顺序 7 未做）。
8. 自由时间 \((P,T)\) 上同样 \(\kappa_G\approx 1\)。
9. MINCO 消元本身错误。MINCO 是对的；缺的是消元后 \(P\) 上的几何。

更准确的结论：MCE 提供了一个从轨迹控制结构推导出的、坐标一致且在固定时间 MINCO 上显著改善 conditioning 的候选几何。

---

## 8. 本地复现（自测不入库）

`src/Planner/general_planner/Tests/` 已被 `.gitignore` 忽略，仅在本地运行。

```bash
GP=src/Planner/general_planner
INC="-I${GP}/include -I/usr/include/eigen3"
DEF="-DROOT_DIR='\"${GP}/\"'"

g++ -O2 -std=c++17 $DEF $INC \
  ${GP}/Tests/basis_gradient_invariance_self_test.cpp \
  -o /tmp/basis_gradient_invariance_self_test

g++ -O2 -std=c++17 $DEF $INC \
  ${GP}/Tests/coordinate_opt_invariance_self_test.cpp \
  ${GP}/src/utils/lbfgs.cpp \
  -o /tmp/coordinate_opt_invariance_self_test

g++ -O2 -std=c++17 $DEF $INC \
  ${GP}/Tests/minco_metric_conditioning_self_test.cpp \
  ${GP}/src/utils/lbfgs.cpp \
  -o /tmp/minco_metric_conditioning_self_test
```

详细表、谱、代价曲线与图在 `Tests/` 下由对应 `plot_*.py` 生成，同样不入库。

---

## 9. 下一步

1. State2State 全节点：只替换 metric，记录 \(\kappa_G\)、violation、含 \(T_{\mathrm{metric}}\) 的 wall-clock。
2. 看真实 Hessian 是否仍由 snap 主导；若走廊主导，默认 MCE+GN。
3. Free-time：\(G\) 扩到 \((P,T)\)。
4. Frozen baseline 稳定后再考虑 dynamic metric。
5. 加大走廊权重直到 \(n_{\mathrm{neg}}>0\)，用 \(\kappa_+\) 分析非凸。
