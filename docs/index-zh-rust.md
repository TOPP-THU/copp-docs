# COPP文档

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](https://github.com/TOPP-THU/copp/blob/main/LICENSE) [![Website](https://img.shields.io/badge/website-copp.pro-2ff0d8)](https://copp.pro/) [![Docs](https://img.shields.io/badge/docs-docs.copp.pro-1f6feb)](https://docs.copp.pro/) [![Crates.io](https://img.shields.io/crates/v/copp.svg)](https://crates.io/crates/copp) [![PyPI](https://img.shields.io/pypi/v/copp-py.svg)](https://pypi.org/project/copp-py/)

[![Rust](https://img.shields.io/badge/Rust-native-b7410e)](https://docs.rs/copp/latest/copp/) [![C](https://img.shields.io/badge/C-ABI-00599c)](https://github.com/TOPP-THU/copp/tree/main/bindings/c) [![Python](https://img.shields.io/badge/Python-bindings-3776ab)](https://github.com/TOPP-THU/copp/tree/main/bindings/python)

## 核心问题

COPP专注于解决给定几何路径(path)生成时间参数化的轨迹 (trajectory)，并满足用户指定的约束、优化给定的目标。特别地，COPP规划的轨迹一般是二阶光滑（加速度有界）或三阶光滑（加加速度有界）。

### 路径参数化 (Path Parameterization)

给定一个$n$维机器人系统和一条足够光滑的几何路径：
$$
\boldsymbol{q}=\boldsymbol{q}(s)\in\mathbb{R}^n,\qquad s\in[0,s_\text{f}],
$$
其中$s$是一般的路径参数，且$s_\text{f}$已知。对于$m$阶问题，$m\in\{2,3\}$，要求路径$\boldsymbol{q}=\boldsymbol{q}(s)$在$s$域上$\mathcal{C}^m$连续。$m$阶路径参数化 (Path Parameterization) 的目标是构建一个严格递增的时间参数化
$$
s=s(t),\qquad t\in[0,t_\text{f}],
$$
且$\frac{\mathrm{d}^ms}{\mathrm{d}t^m}(t)$在有限个时间点之外存在且有界，其中终端时间$t_\text{f}$未知。从而构建出严格遵循给定几何路径的机器人轨迹
$$
\boldsymbol{q}=\boldsymbol{q}(s(t))\in\mathbb{R}^n,\qquad t\in[0,t_\text{f}].
$$
上述轨迹经过插补 (Interpolation) 可以作为参考轨迹 (Reference) 发送给底层伺服系统，设其控制周期为$T_\text{s}$（例如$T_\text{s}=\text{1ms}$），那么发送给底层伺服系统的信息一般包括插补轨迹
$$
\{\boldsymbol{q}(s(iT_\text{s}))\}_{i\in\mathbb{N}}
$$
及相应前馈所需信息，例如PID控制所需的参考速度项$\{\dot{\boldsymbol{q}}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$及前馈所需的参考加速度项$\{\ddot{\boldsymbol{q}}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$和力矩项$\{\boldsymbol{\tau}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$等。

记$\dot{\bullet}$为对时间$t$的导数，$\bullet'$为对参数$s$的导数。定义
$$
a(s)=\dot{s}^2,\qquad b(s)=\ddot{s}=\frac12a'(s),\qquad c(s)=\frac{\dddot{s}}{\dot{s}}=b'(s).
$$
在二阶问题中，状态量为$a(s)$，控制量为$b(s)$；在三阶问题中，状态量为$(a(s),b(s))$，控制量为$c(s)$。

### 最优路径参数化 (Optimal Path Parameterization)

相比一般的路径参数化，最优路径参数化进一步引入了约束条件 (constraint) 和优化目标 (objective)。

约束条件用于提高轨迹光滑性、确保参考轨迹在驱动性能范围内，在实际效果中有利于减少振动、提高轨迹跟踪精度等。一阶约束可以表达合成速度约束$\|\boldsymbol{J(s)}\dot{\boldsymbol{q}}(s)\|\leq V(s)$、各轴速度约束$\dot{\boldsymbol{q}}_\text{min}(s)\leq\dot{\boldsymbol{q}}(s)\leq\dot{\boldsymbol{q}}_\text{max}(s)$等，一般形式为
$$
a(s)\leq a^+(s).
$$
二阶约束可以表达各轴加速度约束$\ddot{\boldsymbol{q}}_\text{min}(s)\leq\ddot{\boldsymbol{q}}(s)\leq\ddot{\boldsymbol{q}}_\text{max}(s)$、各轴力矩约束$\boldsymbol{\tau}_\text{min}(s)\leq\boldsymbol{\tau}(s)\leq\boldsymbol{\tau}_\text{max}(s)$等，一般形式为
$$
\boldsymbol{n}(s)a(s)+\boldsymbol{m}(s)b(s)\leq\boldsymbol{g}(s).
$$
三阶约束可以表达各轴加加速度约束$\dddot{\boldsymbol{q}}_\text{min}(s)\leq\dddot{\boldsymbol{q}}(s)\leq\dddot{\boldsymbol{q}}_\text{max}(s)$、各轴力矩导数约束$\dot{\boldsymbol{\tau}}_\text{min}(s)\leq\dot{\boldsymbol{\tau}}(s)\leq\dot{\boldsymbol{\tau}}_\text{max}(s)$等，一般形式为
$$
\sqrt{a(s)}\left(\boldsymbol{r}(s)a(s)+\boldsymbol{v}(s)b(s)+\boldsymbol{w}(s)c(s)+\boldsymbol{h}(s)\right)\leq\boldsymbol{f}(s).
$$
上述除了$a(s), b(s), c(s)$为决策变量外，其他量均为根据路径、模型、物理约束计算得到的已知量。注意到上述约束支持分段约束、关于参数变化的约束、非对称约束。

常见的优化目标包括终端时间
$$
t_\text{f}=\int_0^{t_\text{f}}\mathrm{d}t,
$$
热能耗散
$$
J_\text{th}=\int_0^{t_\text{f}}\left\|\frac{\boldsymbol{\tau}(t)}{\boldsymbol{\tau}_\text{normal}(t)}\right\|_2^2\mathrm{d}t,
$$
力矩全变分
$$
J_\text{tv}=\int_0^{t_\text{f}}\left\|\frac{\mathrm{d}\boldsymbol{\tau}(t)}{\boldsymbol{\tau}_\text{normal}(t)}\right\|_1,
$$
和线性目标
$$
J_\text{lin}=\begin{cases}
\int_0^{s_\text{f}}(\alpha(s)a(s)+\beta(s)b(s))\mathrm{d}s,&\text{二阶问题},\\
\int_0^{s_\text{f}}(\alpha(s)a(s)+\beta(s)b(s)+\gamma(s)c(s))\mathrm{d}s,&\text{三阶问题}.\\
\end{cases}
$$
最常用的目标是终端时间$t_\text{f}$，即时间最优路径参数化 (Time-Optimal Path Parameterization, TOPP)；在更一般的目标下，我们可以考虑上述目标或其他用户自定义凸目标的线性组合，即凸目标路径参数化 (Convex-Objective Path Parameterization)。[实验表明](https://arxiv.org/abs/2605.19089)，相比纯TOPP，终端时间-热能耗散的混合目标$J=t_\text{f}+\lambda J_\text{th}$的COPP所得轨迹能够在极小的终端时间代价下显著提升轨迹光滑性。

总结而言，`copp`库能够求解的问题分类如下：

| 问题类别   | 二阶（速度、加速度、力矩约束） | 三阶约束（额外加加速度、力矩导数约束等） |
| ---------- | ------------------------------ | ---------------------------------------- |
| 时间目标   | TOPP2                          | TOPP3                                    |
| 一般凸目标 | COPP2                          | COPP3                                    |

## 安装

Rust的安装非常简单，我们已经把`copp`库发布到[crates.io](https://crates.io/crates/copp)了。只需在根目录`Cargo.toml`加入

```toml
[dependencies]
copp = "0.2.1"
```

`copp` v0.2.1 需要 Rust 1.88 或更新版本。此外，我们强烈建议开启Release模式，以显著提高计算效率。

## 算法选择

`copp`中算法的选择主要取决于三个因素：问题是二阶还是三阶、目标是时间最优还是一般凸目标、以及是否需要PRO版本提供的更高性能。

| 问题类别 | 算法 | 可用版本 | 说明 |
| -------- | ---- | -------- | ---- |
| TOPP2 | TOPP2-RA | 开源 | 基于可达性分析的高速算法。在常见benchmark中相对全局优化基准的误差通常低于$10^{-4}$，适合作为二阶时间最优问题的默认入口。 |
| COPP2 | COPP2-SOCP | 开源 | 将问题建模为SOCP并由`clarabel`求解。在凸建模假设下具备全局最优性，但计算开销通常明显高于RA类方法。 |
| COPP2 | COPP2-RDDP | PRO | 原创的高速算法，保持全局最优解质量，并显著快于COPP2-SOCP。 |
| TOPP3 | TOPP3-SOCP | 开源 | 基于`clarabel`的锥优化形式，通常能得到高质量的KKT解；在特定数据集上计算成本可能较高。 |
| TOPP3 | TOPP3-LP | 开源 | TOPP3-SOCP的线性目标近似形式，通常更快；但在jerk约束较紧时可能次优，因此主要推荐在jerk边界较宽松时使用。 |
| TOPP3 | TOPP3-RA | PRO | 基于可达性分析的高速三阶算法；在jerk约束较紧时可能次优，主要推荐在jerk边界较宽松时使用。 |
| COPP3 | COPP3-SOCP | 开源 | 基于`clarabel`的锥优化形式，通常具有较强的实际最优性，但计算成本较高。 |
| COPP3 | COPP3-RDDP | PRO | 高速原创算法，能够得到接近TOPP3-SOCP质量的KKT解，同时显著快于TOPP3-SOCP、TOPP3-LP和COPP3-SOCP。COPP3-RDDP也可以作为高质量TOPP3求解器使用；在长路径问题上，它可能比COPP3-SOCP具有更好的实际最优性和数值稳定性。 |

更具体地说，可以按如下方式选择：

| 场景 | 推荐算法 | 可用版本 | 主要原因 | 注意事项 | 备选方案 |
| ---- | -------- | -------- | -------- | -------- | -------- |
| 二阶时间最优，且要求极低计算时间 | TOPP2-RA | 开源 | 速度和性能折中极好，在典型benchmark中接近全局最优。 | 目标固定为最短时间。 |  |
| 二阶凸目标，且更重视全局解质量 | COPP2-SOCP | 开源 | 凸锥优化形式，在模型假设下具有全局最优性。 | 计算时间高于RA/RDDP类方法。 | 若需要大幅提速，可使用COPP2-RDDP。 |
| 二阶凸目标，且要求最高计算效率 | COPP2-RDDP | PRO | 保持全局最优解质量，同时显著提高计算速度。 | 需要PRO授权。 | COPP2-SOCP。 |
| 三阶问题，且希望使用开源版本中最强的最优性质量 | TOPP3-SOCP / COPP3-SOCP | 开源 | 具有较强的KKT解质量和较广泛适用性。 | 在特定数据集上计算成本可能较高。 | 若需要大幅提速，可使用COPP3-RDDP。 |
| 三阶时间最优，且希望使用更快的开源近似 | TOPP3-LP | 开源 | 当用户自己的路径数据集显示它具有更好的速度/性能表现时可以使用。 | jerk约束较紧时可能次优。 | TOPP3-SOCP或COPP3-RDDP。 |
| 三阶时间最优，jerk边界较宽松且要求极低计算时间 | TOPP3-RA | PRO | 计算开销很低。 | jerk约束较紧时可能次优。 | COPP3-RDDP或TOPP3-SOCP。 |
| 三阶高质量、高稳定性，特别是困难长路径规划 | COPP3-RDDP | PRO | 实际最优性强、计算速度高，并且通常在长时域问题上更稳定。 | 需要PRO授权。 | COPP3-SOCP。 |

## Step-by-Step工作流

我们以二维路径的时间最优参数化为例给出一个简单的例程，高级接口详见[文档章节](#docs-architecture)。

```rust
const DIM: usize = 2;
```

### Step 1. 构造几何路径

严格地说，几何路径$\boldsymbol{q}=\boldsymbol{q}(s)$是用户端给`copp`的输入，而`copp`库提供了路径相关模块作为辅助。

#### Option A.  解析式自动微分

最简单的方式是通过解析式构造路径，例如：

```rust
use copp::path::autodiff::Jet3;
use copp::path::{cos, sin, Path};

let path = Path::from_parametric(|s: Jet3| vec![sin(s), cos(s)], 0.0, 1.0)?;
```

#### Option B. 路径点生成样条

如果提供路径点，可以通过如下方式构建样条路径，例如：

```rust
use copp::path::{Path, SplineConfig};
use nalgebra::DMatrix;

let waypoints = DMatrix::from_row_slice(
    2, // dim
    5, // number of waypoints
    &[
        0.0, 0.25, 0.5, 0.75, 1.0,
        0.0, 0.1, -0.1, 0.2, 0.0,
    ],
);
let path = Path::from_waypoints(&waypoints, SplineConfig::default())?;
```

### Option C. 用户手动微分

最一般的情况下，用户可以自行求导，在TOPP2/COPP2应在给定$s$下提供$\boldsymbol{q}(s),\boldsymbol{q}'(s),\boldsymbol{q}''(s)$，例如：

```rust
use copp::diag::PathError;
use copp::path::{Path, PathEvaluator2nd};

struct NormalizedEvaluator2nd;

impl PathEvaluator2nd for NormalizedEvaluator2nd {
    fn dim(&self) -> usize {
        2
    }

    fn evaluate_up_to_2nd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
    ) -> Result<(), PathError> {
        for (col, &sj) in s.iter().enumerate() {
            let row0 = 2 * col;
            q[row0] = 0.5 * sj * sj;
            q[row0 + 1] = sj;
            dq[row0] = sj;
            dq[row0 + 1] = 1.0;
            ddq[row0] = 1.0;
            ddq[row0 + 1] = 0.0;
        }
        Ok(())
    }
}

impl PathEvaluator3rd for NormalizedEvaluator3rd {
    fn evaluate_up_to_3rd(
        &self,
        s: &[f64],
        q: &mut [f64],
        dq: &mut [f64],
        ddq: &mut [f64],
        dddq: &mut [f64],
    ) -> Result<(), PathError> {
        self.evaluate_up_to_2nd(s, q, dq, ddq)?;
        dddq.fill(0.0);
        Ok(())
    }
}

let path = Path::from_evaluator_3rd(NormalizedEvaluator3rd, 0.0, 1.0)?;
// If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then `PathEvaluator3rd` can be removed and the path can be constructed by `from_evaluator_2nd`.
```

### Step 2. 离散化路径信息

路径参数化问题需要在给定的$s$离散网格上进行，例如：

```rust
// `n` is the number of path samples (s_i) to build robot constraints on.
let n = 1001;
let s: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();
```

创建机器人模型：

```rust
use copp::robot::Robot;

let mut robot = Robot::with_capacity(DIM, n);
```

输入$s$网格与路径信息：

```rust
robot
    .with_s(s.as_slice())?
    .with_q_from_path_3rd(&path, 0, n)?;
```

在更灵活的情况下，路径可以在线加入、删除等，机器人可以包含逆运动学信息，这些高级接口详见[文档章节](#docs-architecture)。

### Step 3. 约束构造

通常情况下，我们推荐用户使用具备物理含义的高级接口，例如：

```rust
// The axial velocity is -1 <= vel <= 1 for each axis in this example
let vel_max = vec![1.0; DIM];
let vel_min = vec![-1.0; DIM];
// The axial acceleration is -1 <= acc <= 1 for each axis in this example.
let acc_max = vec![1.0; DIM];
let acc_min = vec![-1.0; DIM];

robot
    .with_axial_velocity((vel_max.as_slice(), n), (vel_min.as_slice(), n), 0)?
    .with_axial_acceleration((acc_max.as_slice(), n), (acc_min.as_slice(), n), 0)?;
```

如果是求解三阶轨迹，则需要额外引入三阶约束，例如：

```rust
// The axial jerk is -1 <= jerk <= 1 for each axis in this example.
let jerk_max = vec![1.0; DIM];
let jerk_min = vec![-1.0; DIM];
robot.with_axial_jerk((jerk_max.as_slice(), n), (jerk_min.as_slice(), n), 0)?;
```

更高级的调用接口详见[文档章节](#docs-architecture)。

### Step 4. 调用求解器

我们以求解TOPP2问题、调用`topp2_ra`为例。首先定义问题，例如：

```rust
use copp::solver::topp2_ra::Topp2ProblemBuilder;

let idx_s_interval = (0, n - 1); // 0 <= k <= n-1
let a_boundary = (0.0, 0.0); // a(0) = 0, a(1) = 0
let problem = Topp2ProblemBuilder::new(&robot, idx_s_interval, a_boundary).build()?;
```

然后构造求解设置并调用求解器，例如：

```rust
use copp::solver::topp2_ra::{ReachSet2OptionsBuilder, topp2_ra};

let options = ReachSet2OptionsBuilder::new().build()?;
let a_ra = topp2_ra(&problem, &options)?;
```

据此，我们得到了$a=a(s)$。

### Step 5. 二阶轨迹后处理（仅二阶需要）

我们接下来希望得到真实的轨迹$\boldsymbol{q}=\boldsymbol{q}(t)$，特别地，应该得到插补轨迹以便于底层伺服驱动器跟踪。首先应求解$t=t(s)$，例如：

```rust
use copp::solver::topp2_ra::s_to_t_topp2;

// t_final is the traversal time of the path.
// t_s[i] is the time at which the path parameter s[i] is reached.
let (t_final, t_s) = s_to_t_topp2(&s, &a_ra, 0.0)?;
```

接下来求逆解$s=s(t)$并插补，例如：

```rust
use copp::solver::topp2_ra::t_to_s_topp2;
use copp::InterpolationMode;

// s_t is a uniform time grid of s(t) with dt = 1e-3s. This is useful for plotting and downstream control.
let dt = 1e-3;
let s_t = t_to_s_topp2(
    &s,
    &a_ra,
    &t_s,
    InterpolationMode::UniformTimeGrid(0.0, dt, true),
)?;
```

上述插补也支持非均匀时间采样方式，详见[文档章节](#docs-architecture)。最后可以求解插补轨迹$\boldsymbol{q}=\boldsymbol{q}(t)$$，一种简单的做法是：

```rust
let out = path.evaluate_q(s_t)?;
let q_t = out.q;
```

由此完成了二阶轨迹的完整求解。如果是求解三阶轨迹，那么二阶轨迹后处理步骤可以跳过，并继续如下流程。

### Step 6. 构造并求解三阶问题（仅三阶需要）

构造三阶问题如下，其中非凸的三阶约束用前面求解的二阶轨迹`a_ra`进行线性化：

```rust
use copp::solver::topp3_socp::Topp3ProblemBuilder;

// Note that in TOPP3Problem, the non-convex jerk constraints should be linearized into a convex one.
// More details can be found in the documentation of `Topp3ProblemBuilder::build_with_linearization`.
let topp3_problem =
    Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_ra0, (0.0, 0.0), (0.0, 0.0))
    .build_with_linearization()?;
```

我们以`topp3_socp`为例，调用求解器如下：

```rust
use copp::solver::topp3_socp::{ClarabelOptionsBuilder, topp3_socp};

let options_socp = ClarabelOptionsBuilder::new()
    .allow_almost_solved(true)
    .build()?;
let profile = topp3_socp(&topp3_problem, &options_socp)?;
```

理论上这已经生成了一条可行、近优的三阶轨迹$a_1(s),b_1(s)$了，可以直接进行下一步。如果希望通过更多的计算资源进一步地求解更优的轨迹，可以以$a_1(s)$为线性化点再次求解新的线性化三阶问题：

```rust
let topp3_problem =
    Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &profile.a, (0.0, 0.0), (0.0, 0.0))
    .build_with_linearization()?;
let profile = topp3_socp(&topp3_problem, &options_socp)?;
```

在计算资源允许的情况下，可以重复进行上述过程，也就是用$a_k(s)$线性化三阶非凸问题并求解得到$a_{k+1}(s),b_{k+1}(s)$，在非退化情况下最终能够收敛到KKT解。从在线进行的实用角度，我们推荐完成1到2次线性化即足够。

### Step 7. 三阶轨迹后处理（仅三阶需要）

我们接下来希望得到真实的轨迹$\boldsymbol{q}=\boldsymbol{q}(t)$，特别地，应该得到插补轨迹以便于底层伺服驱动器跟踪。首先应求解$t=t(s)$，例如：

```rust
use copp::solver::topp3_socp::s_to_t_topp3;

// t_final is the traversal time of the path.
// t_s[i] is the time at which the path parameter s_i is reached.
let (t_final, t_s) = s_to_t_topp3(&s, profile.as_parts(), 0.0)?;
```

接下来求逆解$s=s(t)$并插补，例如：

```rust
use copp::solver::topp3_socp::t_to_s_topp3;
use copp::InterpolationMode;

// s_t is a uniform time grid of s(t) with dt = 1e-3s. This is useful for plotting and downstream control.
let dt = 1e-3;
let s_t = t_to_s_topp3(
    &s,
    profile.as_parts(),
    &t_s,
    InterpolationMode::UniformTimeGrid(0.0, dt, true),
)?;
```

上述插补也支持非均匀时间采样方式，详见[文档章节](#docs-architecture)。最后可以求解插补轨迹$\boldsymbol{q}=\boldsymbol{q}(t)$$，一种简单的做法是：

```rust
let out = path.evaluate_q(s_t)?;
let q_t = out.q;
```

由此完成了三阶轨迹的完整求解。

### Step-by-Step小结

总的来说，一个最小闭环包括：构造路径$\boldsymbol{q}(s)$，在离散网格上构造`Robot`和约束，选择对应的problem builder和solver，得到$a(s)$或$(a(s),b(s))$，再通过`s_to_t_*`和`t_to_s_*`转回时间域，最终在$s(t)$上重新采样原始路径。仓库中也提供了TOPP2、COPP2、TOPP3、COPP3等可运行例程。

## Benchmark性能测试

以下测试来自仓库中的`tests/test_random_spline.rs`。测试条件为：

- `release, --include-ignored`
- CPU: Intel(R) Core(TM) Ultra 9 285K.
- 数据集：100条随机7-DOF样条路径，每条路径离散为1000个区间。

所有指标均以`mean ± std`的形式列出。

#### 时间最优 (Time-Optimal)

| 方法 | 计算时间 (ms) | 终端时间 (s) |
| ------ | --------------------: | -----------------: |
| TOPP2-RA | 0.615425 ± 0.244409 | 40.903420 ± 1.378671 |
| COPP2-SOCP | 149.969964 ± 9.364334 | 40.900039 ± 1.378613 |
| COPP2-RDDP | 5.436142 ± 0.465495 | 40.900135 ± 1.378613 |
| TOPP3-LP | 327.074029 ± 28.893341 | 41.422945 ± 1.381874 |
| TOPP3-SOCP | 289.654071 ± 12.862133 | 41.418608 ± 1.381202 |
| COPP3-SOCP | 285.004302 ± 13.471264 | 41.418608 ± 1.381202 |
| TOPP3-RA (Iteration 1) | 10.571045 ± 0.857653 | 41.499200 ± 1.385735 |
| TOPP3-RA (Iteration 2) | 20.300932 ± 1.237908 | 41.399867 ± 1.386791 |

#### 凸目标 (Convex-Objective)

在该测试中，TOPP方法仍以终端时间为优化目标。

| 方法 | 计算时间 (ms) | 目标函数值 |
| ------ | --------------------: | --------------: |
| TOPP2-RA | 0.534700 ± 0.069296 | 217.444861 ± 12.462360 |
| COPP2-SOCP | 270.059250 ± 52.073677 | 96.517354 ± 3.641154 |
| COPP2-RDDP | 12.667700 ± 0.429214 | 96.525785 ± 3.639733 |
| TOPP3-LP | 348.000000 ± 9.326314 | 211.611085 ± 12.367224 |
| TOPP3-SOCP | 301.227000 ± 12.938498 | 211.974066 ± 12.323865 |
| COPP3-SOCP | 301.227000 ± 12.938498 | 96.634962 ± 3.613264 |
| COPP3-RDDP | 65.823050 ± 0.087893 | 98.708998 ± 3.354004 |

## 文档与架构 { #docs-architecture }

### 文档

我们推荐使用[docs.rs 最新文档](https://docs.rs/copp/latest/copp/)。也支持本地文档：

- [v0.2.1 (Latest)](/rust/v0.2.1/copp/)
- [v0.2.0](/rust/v0.2.0/copp/)
- [v0.1.0](/rust/v0.1.0/copp/)

如果需要查看main branch上尚未发布的更新，我们推荐在`copp`仓库根目录本地生成文档：

```shell
cargo doc --no-deps --open
```

生成的文档包含数学基础、路径/约束构造方法、日志和输出约定、错误定义以及求解器接口。

### 项目架构

| 模块                            | 负责内容                                  |
| ----------------------------------------- | ----------------------------------------- |
| `path`                                    | 路径构造、参数范围、二阶/三阶求导。       |
| `robot`                                   | 机器人维度、逆动力学、物理约束入口。      |
| `constraints`                                   | 更底层的约束接口。      |
| `solver`                        | 各个求解器。 |
| `diag`                                    | 错误类型、日志 verbosity、诊断信息。      |

## 引用

如果你的工作使用了开源TOPP3/COPP3功能，建议引用：（即便是最基本的离散区间内profile模板也用到了该文章的贡献）

```tex
@article{wang2026online,
  title={Online time-optimal trajectory planning along parametric toolpaths with strict constraint satisfaction and certifiable feasibility guarantee},
  author={Wang, Yunan and Hu, Chuxiong and Li, Yuanshenglong and Yu, Jichuan and Yan, Jizhou and Liang, Yixuan and Jin, Zhao},
  journal={International Journal of Machine Tools and Manufacture},
  volume={215},
  pages={104355},
  year={2026}
}
```

如果你的工作使用了PRO版本中的TOPP3-RA, COPP2-RDDP, COPP3-RDDP方法，建议引用：（其中TOPP3-RA基于该论文进行改进）

```tex
@article{wang2026reachability,
  title={Reachability-augmented dual dynamic programming for optimal path parameterization},
  author={Yunan Wang and Jizhou Yan and Chuxiong Hu and Zeyang Li},
  journal={arXiv preprint arXiv:2605.19089},
  year={2026}
}
```

其他情况下可引用COPP项目本身，或对应论文：

```tex
@misc{thu2026copp,
  title = {COPP: Convex-Objective Path Parameterization},
  author = {Wang, Yunan and He, Suqin and Lin, Shize and Hu, Chuxiong},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/TOPP-THU/copp}}
}
```

## 联系

如果需要COPP PRO授权、商业合作、技术咨询或一般问题，可以联系[hello@copp.pro](mailto:hello@copp.pro)。
