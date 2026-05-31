# COPP

<style>
.copp-hero {
    border: 1px solid #dfe2e5;
    border-radius: 8px;
    padding: 1.15rem 1.25rem;
    margin: 1rem 0 1.25rem;
    background: #fbfcfd;
}
.copp-hero p {
    margin: 0.45rem 0;
}
.copp-link-row {
    display: flex;
    flex-wrap: wrap;
    gap: 0.5rem;
    margin-top: 0.8rem;
}
.copp-link-row a {
    display: inline-block;
    padding: 0.35rem 0.7rem;
    border: 1px solid #d0d7de;
    border-radius: 5px;
    text-decoration: none;
    color: #1f5f99;
    background: #fff;
}
.copp-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(15rem, 1fr));
    gap: 0.8rem;
    margin: 1rem 0;
}
.copp-card {
    border: 1px solid #dfe2e5;
    border-radius: 8px;
    padding: 0.85rem 0.95rem;
    background: #fff;
}
.copp-card h3 {
    margin-top: 0;
    margin-bottom: 0.45rem;
}
.copp-card p {
    margin: 0.35rem 0;
}
</style>

<div class="copp-hero">
    <p><strong>Convex-Objective Path Parameterization for robotic trajectory planning.</strong></p>
    <p>COPP 用于沿给定几何路径生成满足约束的时间参数化轨迹。它把“路径怎么走”与“沿路径何时走到哪里”解耦，在路径参数域中处理速度、加速度、力矩、jerk 和任务相关目标。</p>
    <div class="copp-link-row">
        <a href="https://copp.pro/">Homepage</a>
        <a href="https://docs.rs/copp/latest/copp/">Rust docs.rs</a>
        <a href="rust/v0.2.0/copp/index.html">Rust v0.2.0</a>
        <a href="c/v0.2.0/index.html">C v0.2.0</a>
    </div>
</div>

## 先读哪里

<div class="copp-grid">
    <div class="copp-card">
        <h3>理解问题</h3>
        <p>从 <a href="#核心问题">核心问题</a> 开始，先看 OPP 怎样从时间域转到路径参数域。</p>
    </div>
    <div class="copp-card">
        <h3>马上试用</h3>
        <p>看 <a href="#安装">安装</a> 和 <a href="#step-by-step-工作流">Step-by-step 工作流</a>，每一步都可以在 Rust/C 之间切换。</p>
    </div>
    <div class="copp-card">
        <h3>选择算法</h3>
        <p>如果只需要二阶时间最优，优先 TOPP2-RA；如果有凸目标或 jerk 约束，再进入 COPP2、TOPP3 或 COPP3。</p>
    </div>
</div>

## 核心问题

给定一个 $n$ 维机器人系统和一条足够光滑的几何路径：

$$
\boldsymbol{q}=\boldsymbol{q}(s),\qquad s\in[0,s_f].
$$

这里 $s$ 是路径参数，不一定是弧长；$s_f$ 已知。OPP 要做的是寻找一个严格递增的时间参数化：

$$
s=s(t),\qquad t\in[0,t_f],
$$

使机器人沿原几何路径运动时满足状态约束和状态-控制混合约束，并最小化用户指定的目标函数。终端时间 $t_f$ 是未知量，因此直接在时间域求解会把自由终端时间、单调性和非线性约束缠在一起。

标准做法是把问题转到路径参数域。令对 $s$ 的导数记为 $\bullet'=\mathrm{d}\bullet/\mathrm{d}s$，并定义：

$$
x_1(s)=\frac12\dot{s}^2,\qquad x_{i+1}(s)=x_i'(s).
$$

于是 $m$ 阶 OPP 在路径参数域中的状态和控制可以写成：

$$
\boldsymbol{x}(s)=(x_1,\ldots,x_{m-1}),\qquad u(s)=x_m.
$$

最常用的是二阶和三阶问题：

$$
\text{OPP2:}\quad \boldsymbol{x}=x_1=\frac12\dot{s}^2,\quad u=x_2=\ddot{s},
$$

$$
\text{OPP3:}\quad \boldsymbol{x}=(x_1,x_2)=\left(\frac12\dot{s}^2,\ddot{s}\right),\quad u=x_3=\frac{\dddot{s}}{\dot{s}}.
$$

COPP 的 Rust/C API 里通常把

$$
a(s)=\dot{s}^2=2x_1(s),\qquad b(s)=\ddot{s}=x_2(s)
$$

作为主要 profile 变量。二阶 solver 返回 $a(s)$；三阶 solver 返回 $(a(s), b(s))$ 以及 stationary boundary 信息。

将 $[0,s_f]$ 离散为 station grid：

$$
0=s_0<s_1<\cdots<s_N=s_f,\qquad \Delta_k=s_{k+1}-s_k.
$$

OPP 就变成在每个 station 上求 $\boldsymbol{x}_k$ 和每个区间上求 $u_k$ 的离散优化问题：

$$
\min_{\boldsymbol{x}_k,u_k}\quad
J=\sum_{k=0}^{N-1}L_k(\boldsymbol{x}_k,u_k)+\Phi(\boldsymbol{x}_N),
$$

$$
\boldsymbol{x}_{k+1}=A_k\boldsymbol{x}_k+B_k u_k,\qquad
f_k(\boldsymbol{x}_k)\le 0,\qquad h_k(\boldsymbol{x}_k,u_k)\le 0.
$$

这个形式解释了 COPP 文档里的几个设计选择：

- `Path` 负责提供 $\boldsymbol{q}(s)$ 及其导数。
- `Robot` 负责把速度、加速度、力矩和 jerk 等物理约束转成 $a,b,u$ 上的约束。
- problem builder 负责固定 station 区间、边界条件和目标函数。
- solver 负责在离散的路径参数域中求 profile。
- interpolation 负责把 $a(s)$ 或 $(a(s),b(s))$ 转回 $t(s)$ 和 $s(t)$。

| 问题类别 | 状态/控制 | 目标 | 典型约束 |
| --- | --- | --- | --- |
| TOPP2 | $a$；区间加速度由 $a$ 差分得到 | 最短时间 | 速度、加速度、力矩等二阶约束 |
| COPP2 | $a$；区间 $b$ | 时间、热能、力矩变化、线性项等凸目标 | 二阶约束与 torque 相关目标 |
| TOPP3 | $(a,b)$；控制对应 jerk 结构 | 时间目标 | jerk、stationary boundary、三阶平滑性 |
| COPP3 | $(a,b)$；目标引入辅助变量 | 一般凸目标 | 三阶约束与多目标权衡 |

时间最优 TOPP 往往会把速度、加速度、力矩或 jerk 推到约束边界，效率高，但可能带来振动、跟踪误差或控制信号抖动。引入 COPP/GOPP 式的一般目标后，可以在少量时间代价下换取更平滑、更节能或更适合下游控制器的 profile。

## 安装

语言只是入口，安装目标是一样的：把 COPP 放进你的工程，并能打开对应 API 文档。

<span data-copp-tabs="start:rust,c"></span>

#### Rust

Rust 是主实现语言，适合直接组合路径、机器人模型、约束、目标和 solver。

```toml
[dependencies]
copp = "0.2"
```

```shell
cargo add copp
cargo doc --no-deps --open
```

`copp` v0.2.0 需要 Rust 1.88 或更新版本。

#### C

C ABI 适合 C/C++ 工程、下游语言绑定和已有机器人软件栈。

```shell
cargo build --release
```

```cmake
find_package(copp CONFIG REQUIRED)

add_executable(app main.c)
target_link_libraries(app PRIVATE copp::copp)
```

动态库、静态库、安装前缀和 smoke test 的细节见 [C v0.2.0 文档](c/v0.2.0/index.html)。

<span data-copp-tabs="end"></span>

## Step-by-step 工作流

下面按“建模思想”组织，每一步都可以切换 Rust/C。代码是最小形状，不是完整程序；完整签名、错误处理和生命周期约束请进入对应 API 文档。

### Step 1. 构造几何路径

先得到 q(s) 及其导数。路径可以来自 waypoint spline，也可以来自你自己的 evaluator。

<span data-copp-tabs="start:rust,c"></span>

#### Rust

```rust
use copp::path::{Path, SplineConfig};

let path = Path::from_waypoints(
    &waypoints,
    SplineConfig::default(),
)?;
```

#### C

```c
struct CoppMatrixViewF64 waypoints =
    COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(data, dim, n_points);

struct CoppPathOptions options;
copp_path_default_options(0.0, 1.0, &options);

struct CoppPath *path = NULL;
copp_path_from_waypoints(waypoints, options, &path);
```

<span data-copp-tabs="end"></span>

### Step 2. 建 station grid 和 robot

station grid 是离散 OPP 的骨架。`s` 必须严格递增；robot 持有 station-indexed constraint buffer。

<span data-copp-tabs="start:rust,c"></span>

#### Rust

```rust
use copp::robot::Robot;

let mut robot = Robot::with_capacity(dim, s.len());
robot
    .with_s(s.as_slice())?
    .with_q_from_path_3rd(&path, 0, s.len())?;
```

#### C

```c
struct CoppRobot *robot = NULL;
copp_robot_create(dim, n, &robot);

copp_robot_append_s(robot, (struct CoppSliceF64){s, n});
copp_robot_sample_path_3rd(robot, path, 0, n);
```

<span data-copp-tabs="end"></span>

### Step 3. 添加约束

常见物理约束包括轴向速度、加速度、力矩和 jerk。二阶问题通常只需要速度/加速度/力矩；三阶问题需要 jerk 数据。

<span data-copp-tabs="start:rust,c"></span>

#### Rust

```rust
robot
    .with_axial_velocity((vmax.as_slice(), n), (vmin.as_slice(), n), 0)?
    .with_axial_acceleration((amax.as_slice(), n), (amin.as_slice(), n), 0)?
    .with_axial_jerk((jmax.as_slice(), n), (jmin.as_slice(), n), 0)?;
```

#### C

```c
copp_add_axial_velocity_limits(robot, 0, n, vmax, vmin);
copp_add_axial_acceleration_limits(robot, 0, n, amax, amin);
copp_add_axial_jerk_limits(robot, 0, n, jmax, jmin);
```

<span data-copp-tabs="end"></span>

### Step 4. 选择问题和目标

TOPP 是时间最优；COPP 用 objective list 表达时间、热能、力矩总变差或线性偏好。三阶问题通常先用一个可行的 `a` profile 作为线性化参考。

<span data-copp-tabs="start:rust,c"></span>

#### Rust

```rust
let objectives = [
    CoppObjective::Time(1.0),
    CoppObjective::ThermalEnergy(0.1, normalize.as_slice()),
];

let problem = Copp2ProblemBuilder::new(
    &robot, (0, n - 1), (0.0, 0.0), &objectives,
).build()?;
```

#### C

```c
struct CoppObjective objectives[] = {
    {.kind = COPP_OBJECTIVE_KIND_TIME,
     .weight = 1.0},
    {.kind = COPP_OBJECTIVE_KIND_THERMAL_ENERGY,
     .weight = 0.1,
     .normalize = normalize},
};

struct Copp2Problem problem = {
    robot, 0, n - 1, 0.0, 0.0, objectives, 2
};
```

<span data-copp-tabs="end"></span>

### Step 5. 调用 solver

先按问题类别选 solver。TOPP2-RA 很适合做二阶时间最优和三阶线性化的起点；COPP2/COPP3 通过 Clarabel 后端求解凸目标问题。

<span data-copp-tabs="start:rust,c"></span>

#### Rust

```rust
let options = ClarabelOptionsBuilder::new()
    .allow_almost_solved(true)
    .build()?;

let a_profile = copp2_socp(&problem, &options)?;
```

#### C

```c
struct CoppClarabelOptions options;
copp_clarabel_default_options(&options);

struct CoppVecF64 a_profile = {0};
copp2_socp(problem, options, &a_profile);
```

<span data-copp-tabs="end"></span>

### Step 6. 转回时间域并释放资源

求解结果仍然在路径参数域中。二阶结果是 a(s)；三阶结果是 (a(s), b(s))。最后要生成 t(s) 或 s(t)，再交给下游控制器。

<span data-copp-tabs="start:rust,c"></span>

#### Rust

```rust
let (t_final, t_s) = s_to_t_topp2(&s, &a_profile, 0.0)?;
let s_t = t_to_s_topp2(
    &s, &a_profile, &t_s,
    InterpolationMode::UniformTimeGrid(0.0, 1e-3, true),
)?;
```

#### C

```c
struct CoppSliceF64 a_view = {a_profile.data, a_profile.len};

double t_final = 0.0;
struct CoppVecF64 t_s = {0};
copp_s_to_t_2nd(s_view, a_view, 0.0, &t_final, &t_s);

copp_vec_f64_free(t_s);
copp_vec_f64_free(a_profile);
copp_robot_free(robot);
copp_path_free(path);
```

<span data-copp-tabs="end"></span>

## 算法选择

| 场景 | 推荐入口 | 输出 | 主要优点 | 注意事项 |
| --- | --- | --- | --- | --- |
| 二阶时间最优，优先追求速度 | `topp2_ra` | `a(s)` | 极快，适合作为默认起点和三阶线性化参考。 | 目标固定为时间最优。 |
| 二阶凸目标，关注目标函数质量 | `copp2_socp` | `a(s)` | 把时间、热能、力矩变化、线性项放入凸优化。 | 比 RA 慢，依赖 Clarabel 数值求解。 |
| 诊断二阶可行区间 | `reach_set2_backward` / `copp_reach_set2_backward` | reachable intervals | 适合排查约束冲突和分析 feasible set。 | 通常不是最终轨迹接口。 |
| 三阶时间目标，带 jerk 约束 | `topp3_lp`、`topp3_socp` | `(a(s), b(s))` | 支持 jerk 约束和 stationary boundary。 | 需要参考 `a_linearization`。 |
| 三阶凸目标 | `copp3_socp` | `(a(s), b(s))` | 三阶约束下优化多目标。 | 建模和求解成本更高。 |

实践中，一个稳妥的路线是：先用 TOPP2-RA 得到可行参考 $a$，再把它作为 `a_linearization` 进入 TOPP3 或 COPP3。必要时可以先用参考 profile 收紧一阶上界，例如 Rust 中的 `robot.constraints.amax_substitute(...)` 或 C 中的 `copp_robot_amax_substitute(...)`。

## 目标函数

COPP 的目标函数由 `CoppObjective` 表示。常用项包括：

| 目标项 | 含义 | 常见用途 |
| --- | --- | --- |
| `Time(weight)` | 运动时间 | 保留时间效率，或作为多目标中的主项。 |
| `ThermalEnergy(weight, normalize)` | 热能相关项 | 减少高力矩、高速度带来的热负担。 |
| `TotalVariationTorque(weight, normalize)` | 力矩总变差 | 让 torque profile 更平滑。 |
| `Linear(weight, alpha, beta)` | 对 $a$、$b$ 的线性偏好 | 表达额外工程偏好或自定义代价。 |

所有 COPP 目标都应使用非负且有限的权重。`normalize` 的长度需要匹配机器人维度；`Linear` 的 `alpha` 和 `beta` 长度需要匹配对应二阶或三阶问题的离散变量约定。

## API 入口

<span data-copp-tabs="start:rust,c"></span>

#### Rust

- [docs.rs 最新文档](https://docs.rs/copp/latest/copp/)
- [本地 Rust 文档：v0.2.0](rust/v0.2.0/copp/index.html)
- [本地 Rust 文档：v0.1.0](rust/v0.1.0/copp/index.html)

| 模块或命名空间 | 负责内容 |
| --- | --- |
| `path` | 路径构造、参数范围、二阶/三阶求导。 |
| `robot` | 机器人维度、逆动力学、物理约束入口。 |
| `solver::topp2_ra` | TOPP2-RA、ReachSet2、二阶问题构造和插值。 |
| `solver::copp2_socp` | COPP2-SOCP 和 Clarabel expert result。 |
| `solver::topp3_lp` / `solver::topp3_socp` | 三阶时间目标。 |
| `solver::copp3_socp` | 三阶凸目标。 |
| `diag` | 错误类型、日志 verbosity、诊断信息。 |

#### C

- [本地 C 文档：v0.2.0](c/v0.2.0/index.html)

| 头文件 | 内容 |
| --- | --- |
| `copp/copp.h` | umbrella header，包含完整 C API。 |
| `copp/core.h` | status、last error、matrix view、owned vector/matrix、Clarabel options。 |
| `copp/path.h` | path handle、waypoint path、evaluator path、路径求导。 |
| `copp/robot.h` | robot handle、station grid、path sampling、raw constraints、inverse dynamics callback。 |
| `copp/formulation.h` | problem descriptor、objective、三阶 profile。 |
| `copp/interpolation.h` | 二阶/三阶插值和 profile 到时间域的转换。 |
| `copp/topp2.h`、`copp/copp2.h` | TOPP2-RA、ReachSet2、COPP2-SOCP。 |
| `copp/topp3.h`、`copp/copp3.h` | TOPP3-LP/SOCP、COPP3-SOCP。 |

<span data-copp-tabs="end"></span>

## 开源与 PRO

开源版本和 PRO 版本是互补关系。开源版本覆盖 TOPP2-RA、COPP2-SOCP、TOPP3-LP、TOPP3-SOCP 和 COPP3-SOCP，适合研究、验证和一般工程集成。对于需要更高解质量、更强数值稳定性或更低在线延迟的复杂轨迹规划任务，PRO 版本提供 RDDP/RA 系列高性能 solver，例如 COPP2-RDDP、TOPP3-RA 和 COPP3-RDDP。COPP2-RDDP 面向二阶凸目标，COPP3-RDDP 面向三阶目标/约束场景；后者也可作为高质量 TOPP3 solver 使用。

许可、商业合作、技术咨询或一般问题，可以从 [COPP Homepage](https://copp.pro/) 进入，或联系 [hello@copp.pro](mailto:hello@copp.pro)。

## 引用

如果你的工作使用了开源 TOPP3/COPP3 功能，建议引用：

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

如果你的工作使用了 PRO 版本中的 RDDP 方法或TOPP3-RA，请同时引用：

```tex
@article{wang2026reachability,
  title={Reachability-augmented dual dynamic programming for optimal path parameterization},
  author={Yunan Wang and Jizhou Yan and Chuxiong Hu and Zeyang Li},
  journal={arXiv preprint arXiv:2605.19089},
  year={2026}
}
```

其他情况下可引用 COPP 项目本身：

```tex
@misc{thu2026copp,
  title = {COPP: Convex-Objective Path Parameterization},
  author = {Wang, Yunan and He, Suqin and Lin, Shize and Hu, Chuxiong},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/TOPP-THU/copp}}
}
```
