# COPP 文档

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](https://github.com/TOPP-THU/copp/blob/main/LICENSE) [![Website](https://img.shields.io/badge/website-copp.pro-2ff0d8)](https://copp.pro/) [![Docs](https://img.shields.io/badge/docs-docs.copp.pro-1f6feb)](https://docs.copp.pro/) [![Crates.io](https://img.shields.io/crates/v/copp.svg?color=b7410e)](https://crates.io/crates/copp) [![PyPI](https://img.shields.io/pypi/v/copp-py.svg)](https://pypi.org/project/copp-py/)

[![Rust](https://img.shields.io/badge/Rust-native-b7410e)](https://docs.rs/copp/latest/copp/) [![C](https://img.shields.io/badge/C-ABI-a8b9cc)](https://github.com/TOPP-THU/copp/tree/main/bindings/c) [![Python](https://img.shields.io/badge/Python-bindings-ffd43b)](https://github.com/TOPP-THU/copp/tree/main/bindings/python) [![C++](https://img.shields.io/badge/C%2B%2B-bindings-00599c)](https://github.com/TOPP-THU/copp/tree/main/bindings/cpp) [![MATLAB](https://img.shields.io/badge/MATLAB-bindings-e16737)](https://github.com/TOPP-THU/copp/tree/main/bindings/matlab)

## 核心问题

COPP 专注于解决给定几何路径 (path) 生成时间参数化的轨迹 (trajectory)，并满足用户指定的约束、优化给定的目标。特别地，COPP 规划的轨迹一般是二阶光滑（加速度有界）或三阶光滑（加加速度有界）。

### 路径参数化 (Path Parameterization)

给定一个 $n$ 维机器人系统和一条足够光滑的几何路径：

$$
\boldsymbol{q}=\boldsymbol{q}(s)\in\mathbb{R}^n,\qquad s\in[0,s_\text{f}],
$$

其中 $s$ 是一般的路径参数，且 $s_\text{f}$ 已知。对于 $m$ 阶问题，$m\in\{2,3\}$，要求路径 $\boldsymbol{q}=\boldsymbol{q}(s)$ 在 $s$ 域上 $\mathcal{C}^m$ 连续。$m$ 阶路径参数化的目标是构建一个严格递增的时间参数化

$$
s=s(t),\qquad t\in[0,t_\text{f}],
$$

且 $\frac{\mathrm{d}^ms}{\mathrm{d}t^m}(t)$ 在有限个时间点之外存在且有界，其中终端时间 $t_\text{f}$ 待求解。从而构建出严格遵循给定几何路径的机器人轨迹

$$
\boldsymbol{q}=\boldsymbol{q}(s(t))\in\mathbb{R}^n,\qquad t\in[0,t_\text{f}].
$$

上述轨迹经过插补 (Interpolation) 可以作为参考轨迹 (Reference) 发送给底层伺服系统，设其控制周期为 $T_\text{s}$（例如 $T_\text{s}=\text{1ms}$），那么发送给底层伺服系统的信息一般包括插补轨迹

$$
\{\boldsymbol{q}(s(iT_\text{s}))\}_{i\in\mathbb{N}}
$$

及相应前馈所需信息，例如 PID 控制所需的参考速度项 $\{\dot{\boldsymbol{q}}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$ 及前馈所需的参考加速度项 $\{\ddot{\boldsymbol{q}}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$ 和力矩项 $\{\boldsymbol{\tau}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$ 等。

记 $\dot{\bullet}$ 为对时间 $t$ 的导数，$\bullet'$ 为对参数 $s$ 的导数。定义

$$
a(s)=\dot{s}^2,\qquad b(s)=\ddot{s}=\frac12a'(s),\qquad c(s)=\frac{\dddot{s}}{\dot{s}}=b'(s).
$$

在二阶问题中，状态量为 $a(s)$，控制量为 $b(s)$；在三阶问题中，状态量为 $(a(s),b(s))$，控制量为 $c(s)$。

### 最优路径参数化 (Optimal Path Parameterization)

相比一般的路径参数化，最优路径参数化进一步引入了约束条件 (constraint) 和优化目标 (objective)。

约束条件用于提高轨迹光滑性、确保参考轨迹在驱动性能范围内，在实际效果中有利于减少振动、提高轨迹跟踪精度等。一阶约束可以表达合成速度约束 $\|\boldsymbol{J(s)}\dot{\boldsymbol{q}}(s)\|\leq V(s)$、各轴速度约束 $\dot{\boldsymbol{q}}_\text{min}(s)\leq\dot{\boldsymbol{q}}(s)\leq\dot{\boldsymbol{q}}_\text{max}(s)$ 等，一般形式为

$$
a(s)\leq a^+(s).
$$

二阶约束可以表达各轴加速度约束 $\ddot{\boldsymbol{q}}_\text{min}(s)\leq\ddot{\boldsymbol{q}}(s)\leq\ddot{\boldsymbol{q}}_\text{max}(s)$、各轴力矩约束 $\boldsymbol{\tau}_\text{min}(s)\leq\boldsymbol{\tau}(s)\leq\boldsymbol{\tau}_\text{max}(s)$ 等，一般形式为

$$
\boldsymbol{n}(s)a(s)+\boldsymbol{m}(s)b(s)\leq\boldsymbol{g}(s).
$$

三阶约束可以表达各轴加加速度约束 $\dddot{\boldsymbol{q}}_\text{min}(s)\leq\dddot{\boldsymbol{q}}(s)\leq\dddot{\boldsymbol{q}}_\text{max}(s)$、各轴力矩导数约束 $\dot{\boldsymbol{\tau}}_\text{min}(s)\leq\dot{\boldsymbol{\tau}}(s)\leq\dot{\boldsymbol{\tau}}_\text{max}(s)$ 等，一般形式为

$$
\sqrt{a(s)}\left(\boldsymbol{r}(s)a(s)+\boldsymbol{v}(s)b(s)+\boldsymbol{w}(s)c(s)+\boldsymbol{h}(s)\right)\leq\boldsymbol{f}(s).
$$

上述除了 $a(s), b(s), c(s)$ 为决策变量外，其他量均为根据路径、模型、物理约束计算得到的已知量。注意到上述约束支持分段约束、关于参数变化的约束、非对称约束。

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

最常用的目标是终端时间 $t_\text{f}$，即时间最优路径参数化 (Time-Optimal Path Parameterization, TOPP)；在更一般的目标下，我们可以考虑上述目标或其他用户自定义凸目标的线性组合，即凸目标路径参数化 (Convex-Objective Path Parameterization)。[实验表明](https://arxiv.org/abs/2605.19089)，相比纯 TOPP，终端时间-热能耗散的混合目标 $J=t_\text{f}+\lambda J_\text{th}$ 的 COPP 所得轨迹能够在极小的终端时间代价下显著提升轨迹光滑性。

总结而言，`copp` 库能够求解的问题分类如下：

| 问题类别   | 二阶（速度、加速度、力矩约束） | 三阶约束（额外加加速度、力矩导数约束等） |
| ---------- | ------------------------------ | ---------------------------------------- |
| 时间目标   | TOPP2                          | TOPP3                                    |
| 一般凸目标 | COPP2                          | COPP3                                    |

## 安装

=== "Rust"

    `copp` 库已经发布到 [crates.io](https://crates.io/crates/copp)，只需在根目录 `Cargo.toml` 加入

    ```toml
    [dependencies]
    copp = "0.2.1"
    ```

    `copp` v0.2.1 需要 Rust 1.88 或更新版本。此外，我们强烈建议开启 Release 模式，以显著提高计算效率。

=== "Python"

    Python 包发布在 [PyPI: copp-py](https://pypi.org/project/copp-py/)，安装命令是：

    ```sh
    python -m pip install copp-py
    ```

    导入包名是 `copp_py`，通常写成：

    ```python
    import copp_py as copp
    ```

    如果需要从本地源码构建，需要 Python 3.9 或更新版本、Rust/Cargo、`maturin`，以及平台 C/C++ 编译器工具链：Windows 上建议安装带 C++ workload 的 Visual Studio Build Tools，Linux 上使用 GCC/Clang，macOS 上使用 Xcode Command Line Tools。可以从本地库开始：

    ```sh
    git clone https://github.com/TOPP-THU/topp.git
    cd topp
    python -m pip install -U pip
    python -m pip install -U maturin numpy jax
    maturin develop --release --features python
    ```

    `maturin develop` 会把 Rust extension module 直接安装到当前 Python 环境中。构建完成后验证：

    ```sh
    python -c "import copp_py as copp; print(copp.version())"
    ```

=== "Matlab"

    TODO 这里后面再确定安装方式

=== "C++"

    C++ SDK 适合希望使用 RAII、`namespace copp`、`std::vector` / `copp::Matrix` / 可选 Eigen adapter 的工程。发布版本的预编译 C++ SDK 可以从 [GitHub Releases](https://github.com/TOPP-THU/copp/releases) 获取；目前 C++ 没有像 Rust 的 crates.io 或 Python 的 PyPI 那样唯一、事实标准的官方平台，常见选择是 GitHub Releases + CMake package，或后续再提供 vcpkg / Conan recipe。

    如果需要从源码构建，可以从本地库开始：

    ```sh
    git clone https://github.com/TOPP-THU/topp.git
    cd topp
    ```

    需要准备 Cargo、CMake 和支持 C++17 的编译器工具链：Windows 上建议安装带 C++ workload 的 Visual Studio Build Tools，Linux 上使用 GCC/Clang，macOS 上使用 Xcode Command Line Tools。若使用 `copp/eigen.hpp`，还需要安装 Eigen；若只使用核心 C++ API，可以在 CMake 中关闭 Eigen adapter。

    C++ facade 位于 Cargo 的 `cpp` feature 下，先在仓库根目录构建 native library：

    ```sh
    cargo build --release --lib --features cpp
    ```

    然后配置并构建 CMake 工程：

    === "Linux"

        ```sh
        cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_CPP_WITH_EIGEN=OFF -DCMAKE_BUILD_TYPE=Release
        cmake --build bindings/cpp/build
        ```

    === "Windows"

        ```powershell
        cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_CPP_WITH_EIGEN=OFF
        cmake --build bindings/cpp/build --config Release
        ```

    === "macOS"

        ```sh
        cmake -S bindings/cpp -B bindings/cpp/build -DCOPP_CPP_WITH_EIGEN=OFF -DCMAKE_BUILD_TYPE=Release
        cmake --build bindings/cpp/build
        ```

    如果要安装成下游 CMake 工程可查找的 package：

    === "Linux"

        ```sh
        cmake --install bindings/cpp/build --prefix <install-prefix>
        ```

    === "Windows"

        ```powershell
        cmake --install bindings/cpp/build --config Release --prefix <install-prefix>
        ```

    === "macOS"

        ```sh
        cmake --install bindings/cpp/build --prefix <install-prefix>
        ```

    将 `<install-prefix>` 替换为你自己的安装路径，例如 `C:/copp-install` 或 `$PWD/copp-install`。

    下游工程中使用：

    ```cmake
    find_package(copp CONFIG REQUIRED)

    add_executable(app main.cpp)
    target_link_libraries(app PRIVATE copp::copp)
    ```

    配置下游工程时把安装前缀传给 CMake：

    ```sh
    cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
    cmake --build app/build --config Release
    ```

    如果希望静态链接，在上面对应平台的 COPP CMake configure 命令中加入 `-DCOPP_LINK_STATIC=ON`。如果希望使用共享库，则加入 `-DCOPP_LINK_STATIC=OFF`，并确保运行时可以找到 `copp.dll`、`libcopp.so` 或 `libcopp.dylib`。

=== "C"

    C ABI 适合 C/C++ 工程、下游语言绑定和已有机器人软件栈。发布版本的预编译 C ABI SDK 可以从 [GitHub Releases](https://github.com/TOPP-THU/copp/releases) 获取；如果需要从源码构建，可以从本地库开始：

    ```sh
    git clone https://github.com/TOPP-THU/topp.git
    cd topp
    ```

    需要准备 Cargo、CMake 和 C 编译器工具链：Windows 上建议安装带 C++ workload 的 Visual Studio Build Tools，Linux 上使用 GCC/Clang，macOS 上使用 Xcode Command Line Tools。C ABI 位于 Cargo 的 `c` feature 下，先在仓库根目录构建 native library：

    ```sh
    cargo build --release --lib --features c
    ```

    然后配置并构建 CMake 工程：

    === "Linux"

        ```sh
        cmake -S bindings/c -B bindings/c/build -DCMAKE_BUILD_TYPE=Release
        cmake --build bindings/c/build
        ```

    === "Windows"

        ```powershell
        cmake -S bindings/c -B bindings/c/build
        cmake --build bindings/c/build --config Release
        ```

    === "macOS"

        ```sh
        cmake -S bindings/c -B bindings/c/build -DCMAKE_BUILD_TYPE=Release
        cmake --build bindings/c/build
        ```

    如果要安装成下游 CMake 工程可查找的 package：

    === "Linux"

        ```sh
        cmake --install bindings/c/build --prefix <install-prefix>
        ```

    === "Windows"

        ```powershell
        cmake --install bindings/c/build --config Release --prefix <install-prefix>
        ```

    === "macOS"

        ```sh
        cmake --install bindings/c/build --prefix <install-prefix>
        ```

    将 `<install-prefix>` 替换为你自己的安装路径，例如 `C:/copp-install` 或 `$PWD/copp-install`。

    下游工程中使用：

    ```cmake
    find_package(copp CONFIG REQUIRED)

    add_executable(app main.c)
    target_link_libraries(app PRIVATE copp::copp)
    ```

    配置下游工程时把安装前缀传给 CMake：

    ```sh
    cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
    cmake --build app/build --config Release
    ```

    如果希望静态链接，在上面对应平台的 COPP CMake configure 命令中加入 `-DCOPP_LINK_STATIC=ON`。

    若需要重新生成 C 头文件，先安装 `cbindgen`，再运行 PowerShell 头文件生成脚本；普通用户可以直接使用仓库中已生成的 `bindings/c/include/copp/` 头文件。

    === "Linux"

        ```sh
        pwsh -NoProfile -File bindings/c/scripts/generate_headers.ps1
        ```

    === "Windows"

        ```powershell
        powershell -ExecutionPolicy Bypass -File bindings/c/scripts/generate_headers.ps1
        ```

    === "macOS"

        ```sh
        pwsh -NoProfile -File bindings/c/scripts/generate_headers.ps1
        ```

## 算法选择

`copp` 中算法的选择主要取决于三个因素：问题是二阶还是三阶、目标是时间最优还是一般凸目标、以及是否需要 PRO 版本提供的更高性能。

| 问题类别 | 算法       | 可用版本 | 说明                                                                                                                                                                                                                       |
| -------- | ---------- | -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| TOPP2    | TOPP2-RA   | 开源     | 基于可达性分析的高速算法。在常见 benchmark 中相对全局优化基准的误差通常低于 $10^{-4}$，适合作为二阶时间最优问题的默认入口。                                                                                                |
| COPP2    | COPP2-SOCP | 开源     | 将问题建模为 SOCP 并由 `clarabel` 求解。在凸建模假设下具备全局最优性，但计算开销通常明显高于 RA 类方法。                                                                                                                   |
| COPP2    | COPP2-RDDP | PRO      | 原创的高速算法，保持全局最优解质量，并显著快于 COPP2-SOCP。                                                                                                                                                                |
| TOPP3    | TOPP3-SOCP | 开源     | 基于 `clarabel` 的锥优化形式，通常能得到高质量的 KKT 解；在特定数据集上计算成本可能较高。                                                                                                                                  |
| TOPP3    | TOPP3-LP   | 开源     | TOPP3-SOCP 的线性目标近似形式，通常更快；但在 jerk 约束较紧时可能次优，因此主要推荐在 jerk 边界较宽松时使用。                                                                                                              |
| TOPP3    | TOPP3-RA   | PRO      | 基于可达性分析的高速三阶算法；在 jerk 约束较紧时可能次优，主要推荐在 jerk 边界较宽松时使用。                                                                                                                               |
| COPP3    | COPP3-SOCP | 开源     | 基于 `clarabel` 的锥优化形式，通常具有较强的实际最优性，但计算成本较高。                                                                                                                                                   |
| COPP3    | COPP3-RDDP | PRO      | 高速原创算法，能够得到接近 TOPP3-SOCP 质量的 KKT 解，同时显著快于 TOPP3-SOCP、TOPP3-LP 和 COPP3-SOCP。COPP3-RDDP 也可以作为高质量 TOPP3 求解器使用；在长路径问题上，它可能比 COPP3-SOCP 具有更好的实际最优性和数值稳定性。 |

更具体地说，可以按如下方式选择：

| 场景                                            | 推荐算法                | 可用版本 | 主要原因                                                        | 注意事项                         | 备选方案                            |
| ----------------------------------------------- | ----------------------- | -------- | --------------------------------------------------------------- | -------------------------------- | ----------------------------------- |
| 二阶时间最优，且要求极低计算时间                | TOPP2-RA                | 开源     | 速度和性能折中极好，在典型 benchmark 中接近全局最优。           | 目标固定为最短时间。             |                                     |
| 二阶凸目标，且更重视全局解质量                  | COPP2-SOCP              | 开源     | 凸锥优化形式，在模型假设下具有全局最优性。                      | 计算时间高于 RA/RDDP 类方法。    | 若需要大幅提速，可使用 COPP2-RDDP。 |
| 二阶凸目标，且要求最高计算效率                  | COPP2-RDDP              | PRO      | 保持全局最优解质量，同时显著提高计算速度。                      | 需要 PRO 授权。                  | COPP2-SOCP。                        |
| 三阶问题，且希望使用开源版本中最强的最优性质量  | TOPP3-SOCP / COPP3-SOCP | 开源     | 具有较强的 KKT 解质量和较广泛适用性。                           | 在特定数据集上计算成本可能较高。 | 若需要大幅提速，可使用 COPP3-RDDP。 |
| 三阶时间最优，且希望使用更快的开源近似          | TOPP3-LP                | 开源     | 当用户自己的路径数据集显示它具有更好的速度/性能表现时可以使用。 | jerk 约束较紧时可能次优。        | TOPP3-SOCP 或 COPP3-RDDP。          |
| 三阶时间最优，jerk 边界较宽松且要求极低计算时间 | TOPP3-RA                | PRO      | 计算开销很低。                                                  | jerk 约束较紧时可能次优。        | COPP3-RDDP 或 TOPP3-SOCP。          |
| 三阶高质量、高稳定性，特别是困难长路径规划      | COPP3-RDDP              | PRO      | 实际最优性强、计算速度高，并且通常在长时域问题上更稳定。        | 需要 PRO 授权。                  | COPP3-SOCP。                        |

## Step-by-Step 工作流

我们以二维路径的时间最优参数化为例给出一个简单的例程，完整例程可以在每种语言的 `examples` 目录下看到，高级接口详见[文档章节](#docs-architecture)。

=== "Rust"

    ```rust
    const DIM: usize = 2;
    ```

=== "Python"

    ```python
    import copp_py as copp
    import numpy as np

    DIM = 2
    ```

=== "Matlab"

    ```matlab
    DIM = 2;
    ```

=== "C++"

    ```cpp
    #include <cmath>
    #include <cstddef>
    #include <vector>

    #include <copp/copp.hpp>

    constexpr std::size_t DIM = 2;
    ```

=== "C"

    ```c
    #include <math.h>
    #include <stddef.h>
    #include <stdio.h>

    #include "copp/copp.h"

    enum { DIM = 2 };

    static int check(enum CoppStatus status, const char *call)
    {
        if (status == COPP_STATUS_OK) {
            return 0;
        }
        fprintf(stderr, "%s failed: %s\n", call, copp_status_message(status));
        fprintf(stderr, "%s\n", copp_last_error_message());
        return 1;
    }
    ```

### Step 1. 构造几何路径

严格地说，几何路径 $\boldsymbol{q}=\boldsymbol{q}(s)$ 是用户端给 `copp` 的输入，而 `copp` 库提供了路径相关模块作为辅助。

#### Option A. 解析式自动微分

最简单的方式是通过解析式构造路径，例如用如下方式可以构造 $\boldsymbol{q}(s)=[sin(2\pi s), cos(2\pi s)]\in\mathbb{R}^2$的解析路径：

=== "Rust"

    ```rust
    use copp::path::autodiff::Jet3;
    use copp::path::{cos, sin, Path};
    use std::f64::consts::PI;

    let path = Path::from_parametric(|s: Jet3| vec![sin(2.0 * PI * s), cos(2.0 * PI * s)], 0.0, 1.0)?;
    ```

=== "Python"

    ```python
    import jax
    import jax.numpy as jnp

    jax.config.update("jax_enable_x64", True)

    def q_fn(s):
        freq = jnp.array([2.0 * jnp.pi, 2.0 * jnp.pi], dtype=jnp.float64)
        phase = jnp.array([0.0, 0.0], dtype=jnp.float64)
        return jnp.sin(freq * s + phase)

    path = copp.Path.from_jax(q_fn, 0.0, 1.0)
    ```

    Python 的自动微分支持 `jax`, `autograd`, `casadi`, `sympy` 四种工具，可由作者自行选择并安装对应依赖包。


=== "Matlab"

    ```matlab
    path = copp.Path.from_parametric( ...
        @(s) [sin(2*pi*s); cos(2*pi*s)], ...
        s_range=[0, 1]);
    ```

    `from_parametric` 使用 MATLAB 侧 `Jet3` 自动微分；公式返回向量长度就是路径维度 `dim`。

=== "C++"

    ```cpp
    auto path = copp::Path::from_parametric(
        [](copp::Jet3 s) {
            constexpr double pi = 3.14159265358979323846;
            return std::vector<copp::Jet3>{
                copp::sin(2.0 * pi * s),
                copp::cos(2.0 * pi * s),
            };
        },
        0.0,
        1.0);
    ```

=== "C"

    ```c
    // C ABI uses a callback-backed path for analytic formulas.
    // The callback below provides q, dq/ds, d2q/ds2, and d3q/ds3 for
    // q(s) = [sin(2*pi*s), cos(2*pi*s)].
    static enum CoppStatus evaluate_parametric_path_3rd(
        void *user_data,
        size_t dim,
        size_t n,
        const double *s,
        double *q,
        double *dq,
        double *ddq,
        double *dddq)
    {
        (void)user_data;
        if (dim != DIM) {
            return COPP_STATUS_INVALID_ARGUMENT;
        }
        if (n > 0 && (s == NULL || q == NULL || dq == NULL || ddq == NULL || dddq == NULL)) {
            return COPP_STATUS_NULL_POINTER;
        }

        const double pi2 = 6.28318530717958647692;
        const double pi2_sq = pi2 * pi2;
        const double pi2_cu = pi2_sq * pi2;

        for (size_t col = 0; col < n; ++col) {
            const double sin_v = sin(pi2 * s[col]);
            const double cos_v = cos(pi2 * s[col]);
            const size_t row0 = col * dim;

            q[row0] = sin_v;
            q[row0 + 1] = cos_v;
            dq[row0] = pi2 * cos_v;
            dq[row0 + 1] = -pi2 * sin_v;
            ddq[row0] = -pi2_sq * sin_v;
            ddq[row0 + 1] = -pi2_sq * cos_v;
            dddq[row0] = -pi2_cu * cos_v;
            dddq[row0 + 1] = pi2_cu * sin_v;
        }
        return COPP_STATUS_OK;
    }

    struct CoppPath *path = NULL;
    // On success, release `path` later with `copp_path_free(path)`.
    if (check(
            copp_path_from_evaluator_3rd(
                DIM,
                0.0,
                1.0,
                NULL,
                evaluate_parametric_path_3rd,
                NULL,
                &path),
            "copp_path_from_evaluator_3rd")) {
        return 1;
    }
    ```

#### Option B. 路径点生成样条

如果提供路径点，可以通过如下方式构建样条路径，例如：

=== "Rust"

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

=== "Python"

    ```python
    waypoints = np.array(
        [
            [0.0, 0.0],
            [0.25, 0.1],
            [0.5, -0.1],
            [0.75, 0.2],
            [1.0, 0.0],
        ],
        dtype=np.float64,
    )
    path = copp.Path.from_waypoints(waypoints)
    ```


=== "Matlab"

    ```matlab
    waypoints = [ ...
        0.0, 0.25, 0.5, 0.75, 1.0; ...
        0.0, 0.10, -0.10, 0.20, 0.0];

    path = copp.Path.from_waypoints( ...
        waypoints, ...          % dim x n_waypoints
        s_range=[0, 1], ...
        order=3);
    ```

=== "C++"

    ```cpp
    auto path = copp::Path::from_waypoints({
        {0.0, 0.0},
        {0.25, 0.1},
        {0.5, -0.1},
        {0.75, 0.2},
        {1.0, 0.0},
    });
    ```

=== "C"

    ```c
    enum { NUM_WAYPOINTS = 5 };

    // Column-major matrix with shape DIM x NUM_WAYPOINTS.
    // Each column is one waypoint.
    double waypoints[DIM * NUM_WAYPOINTS] = {
        0.0, 0.0,
        0.25, 0.1,
        0.5, -0.1,
        0.75, 0.2,
        1.0, 0.0,
    };

    struct CoppPathOptions path_options;
    struct CoppPath *path = NULL;
    // On success, release `path` later with `copp_path_free(path)`.
    if (check(copp_path_default_options(0.0, 1.0, &path_options), "copp_path_default_options")) {
        return 1;
    }
    if (check(
            copp_path_from_waypoints(
                COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(waypoints, DIM, NUM_WAYPOINTS),
                path_options,
                &path),
            "copp_path_from_waypoints")) {
        return 1;
    }
    ```

#### Option C. 用户手动微分

最一般的情况下，用户可以自行求导，在 TOPP2/COPP2 应在给定 $s$ 下提供 $\boldsymbol{q}(s),\boldsymbol{q}'(s),\boldsymbol{q}''(s)$，例如：

=== "Rust"

    ```rust
    use copp::diag::PathError;
    use copp::path::{Path, PathEvaluator2nd, PathEvaluator3rd};

    struct NormalizedEvaluator;

    impl PathEvaluator2nd for NormalizedEvaluator {
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
            const PI2 : f64 = 2.0 * PI;
            for (col, &sj) in s.iter().enumerate() {
                let row0 = 2 * col; // dim = 2
                let sin = (PI2 * sj).sin();
                let cos = (PI2 * sj).cos();
                q[row0] = sin;
                q[row0 + 1] = cos;
                dq[row0] = PI2 * cos;
                dq[row0 + 1] = -PI2 * sin;
                ddq[row0] = -PI2 * PI2 * sin;
                ddq[row0 + 1] = -PI2 * PI2 * cos;
            }
            Ok(())
        }
    }

    // If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then `PathEvaluator3rd` can be removed.
    impl PathEvaluator3rd for NormalizedEvaluator {
        fn evaluate_up_to_3rd(
            &self,
            s: &[f64],
            q: &mut [f64],
            dq: &mut [f64],
            ddq: &mut [f64],
            dddq: &mut [f64],
        ) -> Result<(), PathError> {
            const PI2 : f64 = 2.0 * PI;
            for (col, &sj) in s.iter().enumerate() {
                let row0 = 2 * col; // dim = 2
                let sin = (PI2 * sj).sin();
                let cos = (PI2 * sj).cos();
                q[row0] = sin;
                q[row0 + 1] = cos;
                dq[row0] = PI2 * cos;
                dq[row0 + 1] = -PI2 * sin;
                ddq[row0] = -PI2 * PI2 * sin;
                ddq[row0 + 1] = -PI2 * PI2 * cos;
                dddq[row0] = -PI2 * PI2 * PI2 * cos;
                dddq[row0 + 1] = PI2 * PI2 * PI2 * sin;
            }
            Ok(())
        }
    }

    // If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then the path can be constructed by `from_evaluator_2nd` without dependence on `PathEvaluator3rd`.
    let path = Path::from_evaluator_3rd(NormalizedEvaluator3rd, 0.0, 1.0)?;
    ```

=== "Python"

    ```python
    class Evaluator:
        dim = 2

        def evaluate_q(self, s):
            q = np.empty((s.size, self.dim), dtype=np.float64)
            pi2 = 2.0 * np.pi
            q[:, 0] = np.sin(pi2 * s)
            q[:, 1] = np.cos(pi2 * s)
            return q

        def evaluate_up_to_2nd(self, s):
            q = self.evaluate_q(s)
            pi2 = 2.0 * np.pi
            sin = np.sin(pi2 * s)
            cos = np.cos(pi2 * s)

            dq = np.empty_like(q)
            dq[:, 0] = pi2 * cos
            dq[:, 1] = -pi2 * sin

            ddq = np.empty_like(q)
            ddq[:, 0] = -(pi2**2) * sin
            ddq[:, 1] = -(pi2**2) * cos

            return q, dq, ddq

        # If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then `evaluate_up_to_3rd` can be removed.
        def evaluate_up_to_3rd(self, s):
            q, dq, ddq = self.evaluate_up_to_2nd(s)
            pi2 = 2.0 * np.pi
            sin = np.sin(pi2 * s)
            cos = np.cos(pi2 * s)

            dddq = np.empty_like(q)
            dddq[:, 0] = -(pi2**3) * cos
            dddq[:, 1] = (pi2**3) * sin

            return q, dq, ddq, dddq


    # If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then the path can be constructed by `from_evaluator_3rd` without dependence on `evaluate_up_to_3rd`.
    path = copp.Path.from_evaluator_3rd(Evaluator(), 0.0, 1.0)
    ```

=== "Matlab"

    ```matlab
    eval3 = @(s) deal( ...
        [sin(2*pi*s); cos(2*pi*s)], ...
        [2*pi*cos(2*pi*s); -2*pi*sin(2*pi*s)], ...
        [-(2*pi)^2*sin(2*pi*s); -(2*pi)^2*cos(2*pi*s)], ...
        [-(2*pi)^3*cos(2*pi*s); (2*pi)^3*sin(2*pi*s)]);

    % If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then
    % the path can be constructed by Path.from_evaluator_2nd without dddq.
    path = copp.Path.from_evaluator_3rd( ...
        eval3, dim=DIM, s_range=[0, 1]);
    ```

    MATLAB evaluator 是 batch callback：输入 `s` 固定为 `1 x N`，输出 `q,dq,ddq,dddq` 固定为 `dim x N`。

=== "C++"

    ```cpp
    auto path = copp::Path::from_evaluator_3rd(
        DIM,
        0.0,
        1.0,
        [](copp::Span<const double> s,
           copp::MatrixRef q,
           copp::MatrixRef dq,
           copp::MatrixRef ddq,
           copp::MatrixRef dddq) {
            const double pi2 = 2.0 * 3.14159265358979323846;
            const double pi2_sq = pi2 * pi2;
            const double pi2_cu = pi2_sq * pi2;

            for (std::size_t col = 0; col < s.size(); ++col) {
                const double sj = s[col];
                const double sin_v = std::sin(pi2 * sj);
                const double cos_v = std::cos(pi2 * sj);

                q(0, col) = sin_v;
                q(1, col) = cos_v;
                dq(0, col) = pi2 * cos_v;
                dq(1, col) = -pi2 * sin_v;
                ddq(0, col) = -pi2_sq * sin_v;
                ddq(1, col) = -pi2_sq * cos_v;
                dddq(0, col) = -pi2_cu * cos_v;
                dddq(1, col) = pi2_cu * sin_v;
            }
        });

    // 如果只需要 TOPP2/COPP2，可以改用 `Path::from_evaluator_2nd`，
    // 并省略 `dddq` 输出。
    ```

=== "C"

    ```c
    static enum CoppStatus evaluate_path_2nd(
        void *user_data,
        size_t dim,
        size_t n,
        const double *s,
        double *q,
        double *dq,
        double *ddq)
    {
        (void)user_data;
        if (dim != DIM) {
            return COPP_STATUS_INVALID_ARGUMENT;
        }
        if (n > 0 && (s == NULL || q == NULL || dq == NULL || ddq == NULL)) {
            return COPP_STATUS_NULL_POINTER;
        }

        const double pi2 = 6.28318530717958647692;
        for (size_t col = 0; col < n; ++col) {
            const double sin_v = sin(pi2 * s[col]);
            const double cos_v = cos(pi2 * s[col]);
            const size_t row0 = col * dim;

            q[row0] = sin_v;
            q[row0 + 1] = cos_v;
            dq[row0] = pi2 * cos_v;
            dq[row0 + 1] = -pi2 * sin_v;
            ddq[row0] = -(pi2 * pi2) * sin_v;
            ddq[row0 + 1] = -(pi2 * pi2) * cos_v;
        }
        return COPP_STATUS_OK;
    }

    static enum CoppStatus evaluate_path_3rd(
        void *user_data,
        size_t dim,
        size_t n,
        const double *s,
        double *q,
        double *dq,
        double *ddq,
        double *dddq)
    {
        (void)user_data;
        if (dim != DIM) {
            return COPP_STATUS_INVALID_ARGUMENT;
        }
        if (n > 0 && (s == NULL || q == NULL || dq == NULL || ddq == NULL || dddq == NULL)) {
            return COPP_STATUS_NULL_POINTER;
        }

        const double pi2 = 6.28318530717958647692;
        const double pi2_sq = pi2 * pi2;
        const double pi2_cu = pi2_sq * pi2;
        for (size_t col = 0; col < n; ++col) {
            const double sin_v = sin(pi2 * s[col]);
            const double cos_v = cos(pi2 * s[col]);
            const size_t row0 = col * dim;

            q[row0] = sin_v;
            q[row0 + 1] = cos_v;
            dq[row0] = pi2 * cos_v;
            dq[row0 + 1] = -pi2 * sin_v;
            ddq[row0] = -pi2_sq * sin_v;
            ddq[row0 + 1] = -pi2_sq * cos_v;
            dddq[row0] = -pi2_cu * cos_v;
            dddq[row0 + 1] = pi2_cu * sin_v;
        }
        return COPP_STATUS_OK;
    }

    // If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then
    // `copp_path_from_evaluator_2nd` can be used without `evaluate_path_3rd`.
    struct CoppPath *path = NULL;
    // On success, release `path` later with `copp_path_free(path)`.
    if (check(
            copp_path_from_evaluator_3rd(
                DIM,
                0.0,
                1.0,
                evaluate_path_2nd,
                evaluate_path_3rd,
                NULL,
                &path),
            "copp_path_from_evaluator_3rd")) {
        return 1;
    }
    ```

### Step 2. 离散化路径信息

路径参数化问题需要在给定的 $s$ 离散网格上进行，例如：

=== "Rust"

    ```rust
    // `n` is the number of path samples (s_i) to build robot constraints on.
    let n = 1001;
    let s: Vec<f64> = (0..n).map(|j| j as f64 / (n - 1) as f64).collect();
    ```

=== "Python"

    ```python
    n = 1001
    s = np.linspace(0.0, 1.0, n, dtype=np.float64)
    ```

=== "Matlab"

    ```matlab
    n = 1001;
    s = linspace(0.0, 1.0, n).';
    ```

=== "C++"

    ```cpp
    // `n` is the number of path samples (s_i) to build robot constraints on.
    const std::size_t n = 1001;
    std::vector<double> s(n);
    for (std::size_t j = 0; j < n; ++j) {
        s[j] = static_cast<double>(j) / static_cast<double>(n - 1);
    }
    ```

=== "C"

    ```c
    // `n` is the number of path samples (s_i) to build robot constraints on.
    enum { n = 1001 };
    double s[n];
    for (size_t j = 0; j < n; ++j) {
        s[j] = (double)j / (double)(n - 1);
    }
    ```

创建机器人模型：

=== "Rust"

    ```rust
    use copp::robot::Robot;

    let mut robot = Robot::with_capacity(DIM, n);
    ```

=== "Python"

    ```python
    robot = copp.Robot(DIM, capacity=n)
    ```

=== "Matlab"

    ```matlab
    robot = copp.Robot(DIM, Capacity=n);
    ```

=== "C++"

    ```cpp
    copp::Robot robot(DIM, n);
    ```

=== "C"

    ```c
    struct CoppRobot *robot = NULL;
    // On success, release `robot` later with `copp_robot_free(robot)`.
    if (check(copp_robot_create(DIM, n, &robot), "copp_robot_create")) {
        return 1;
    }
    ```

输入 $s$ 网格与路径信息：

=== "Rust"

    ```rust
    // If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then `with_q_from_path_3rd` should be replaced by `with_q_from_path_2nd` without dependence on `evaluate_up_to_3rd`.
    robot
        .with_s(s.as_slice())?
        .with_q_from_path_3rd(&path, 0, n)?;
    ```

=== "Python"

    ```python
    robot.append_s(s)
    robot.set_q_from_path_3rd(path, 0, n)
    ```

=== "Matlab"

    ```matlab
    robot.append_s(s);
    robot.set_q_from_path_3rd(path);
    ```

    如果只求解 TOPP2/COPP2，也可以使用 `set_q_from_path_2nd`。

=== "C++"

    ```cpp
    // 如果只需要 TOPP2/COPP2，可以把 `set_q_from_path_3rd` 换成
    // `set_q_from_path_2nd`，不依赖三阶路径求导。
    robot.append_s(s)
         .set_q_from_path_3rd(path, 0, n);
    ```

=== "C"

    ```c
    // If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then
    // `copp_robot_sample_path_3rd` should be replaced by `copp_robot_sample_path_2nd`.
    if (check(copp_robot_append_s(robot, (struct CoppSliceF64){s, n}), "copp_robot_append_s")) {
        return 1;
    }
    if (check(copp_robot_sample_path_3rd(robot, path, 0, n), "copp_robot_sample_path_3rd")) {
        return 1;
    }
    ```

在更灵活的情况下，路径可以在线加入、删除等，机器人可以包含逆运动学信息，这些高级接口详见[文档章节](#docs-architecture)。

### Step 3. 约束构造

通常情况下，我们推荐用户使用具备物理含义的高级接口，例如：

=== "Rust"

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

=== "Python"

    ```python
    vel_max = np.ones(DIM, dtype=np.float64)
    vel_min = -vel_max
    acc_max = np.ones(DIM, dtype=np.float64)
    acc_min = -acc_max

    robot.add_velocity_limits(vel_max, vel_min, start_idx_s=0, length=n)
    robot.add_acceleration_limits(acc_max, acc_min, start_idx_s=0, length=n)
    ```

=== "Matlab"

    ```matlab
    vel_max = ones(DIM, 1);
    vel_min = -vel_max;
    acc_max = ones(DIM, 1);
    acc_min = -acc_max;

    robot.add_velocity_limits(vel_max, vel_min);
    robot.add_acceleration_limits(acc_max, acc_min);
    ```

=== "C++"

    ```cpp
    // The axial velocity is -1 <= vel <= 1 for each axis.
    std::vector<double> vel_max(DIM, 1.0);
    std::vector<double> vel_min(DIM, -1.0);
    // The axial acceleration is -1 <= acc <= 1 for each axis.
    std::vector<double> acc_max(DIM, 1.0);
    std::vector<double> acc_min(DIM, -1.0);

    robot.add_velocity_limits(vel_max, vel_min, 0, n)
         .add_acceleration_limits(acc_max, acc_min, 0, n);
    ```

=== "C"

    ```c
    // The axial velocity is -1 <= vel <= 1 for each axis.
    double vel_max[DIM] = {1.0, 1.0};
    double vel_min[DIM] = {-1.0, -1.0};
    // The axial acceleration is -1 <= acc <= 1 for each axis.
    double acc_max[DIM] = {1.0, 1.0};
    double acc_min[DIM] = {-1.0, -1.0};

    if (check(
            copp_add_axial_velocity_limits(
                robot,
                0,
                n,
                (struct CoppSliceF64){vel_max, DIM},
                (struct CoppSliceF64){vel_min, DIM}),
            "copp_add_axial_velocity_limits")) {
        return 1;
    }
    if (check(
            copp_add_axial_acceleration_limits(
                robot,
                0,
                n,
                (struct CoppSliceF64){acc_max, DIM},
                (struct CoppSliceF64){acc_min, DIM}),
            "copp_add_axial_acceleration_limits")) {
        return 1;
    }
    ```

如果是求解三阶轨迹，则需要额外引入三阶约束，例如：

=== "Rust"

    ```rust
    // The axial jerk is -1 <= jerk <= 1 for each axis in this example.
    let jerk_max = vec![1.0; DIM];
    let jerk_min = vec![-1.0; DIM];
    robot.with_axial_jerk((jerk_max.as_slice(), n), (jerk_min.as_slice(), n), 0)?;
    ```

=== "Python"

    ```python
    jerk_max = np.ones(DIM, dtype=np.float64)
    jerk_min = -jerk_max
    robot.add_jerk_limits(jerk_max, jerk_min, start_idx_s=0, length=n)
    ```

=== "Matlab"

    ```matlab
    jerk_max = ones(DIM, 1);
    jerk_min = -jerk_max;
    robot.add_jerk_limits(jerk_max, jerk_min);
    ```

=== "C++"

    ```cpp
    // The axial jerk is -1 <= jerk <= 1 for each axis in this example.
    std::vector<double> jerk_max(DIM, 1.0);
    std::vector<double> jerk_min(DIM, -1.0);
    robot.add_jerk_limits(jerk_max, jerk_min, 0, n);
    ```

=== "C"

    ```c
    // The axial jerk is -1 <= jerk <= 1 for each axis in this example.
    double jerk_max[DIM] = {1.0, 1.0};
    double jerk_min[DIM] = {-1.0, -1.0};
    if (check(
            copp_add_axial_jerk_limits(
                robot,
                0,
                n,
                (struct CoppSliceF64){jerk_max, DIM},
                (struct CoppSliceF64){jerk_min, DIM}),
            "copp_add_axial_jerk_limits")) {
        return 1;
    }
    ```

更底层、灵活的约束构造接口详见[文档章节](#docs-architecture)。

### Step 4. 调用求解器

我们以求解 TOPP2 问题、调用 `topp2_ra` 为例。首先定义问题，例如：

=== "Rust"

    ```rust
    use copp::solver::topp2_ra::Topp2ProblemBuilder;

    let idx_s_interval = (0, n - 1); // 0 <= k <= n-1
    let a_boundary = (0.0, 0.0); // a(0) = 0, a(1) = 0
    let problem = Topp2ProblemBuilder::new(&robot, idx_s_interval, a_boundary).build()?;
    ```

=== "Python"

    ```python
    problem = copp.solver.topp2_ra.Problem(
        robot.constraints,
        idx_s_interval=(0, n - 1),
        a_boundary=(0.0, 0.0),
    )
    ```

=== "Matlab"

    ```matlab
    problem = copp.solver.topp2_ra.Problem( ...
        robot, ...
        idx_s_interval=[1, n], ...
        a_boundary=[0, 0]);
    ```

    MATLAB 用户侧索引是 1-based；这里 `[1,n]` 覆盖全部站点。

=== "C++"

    ```cpp
    namespace topp2 = copp::solver::topp2_ra;

    auto idx_s_interval = copp::IndexInterval{0, n - 1}; // 0 <= k <= n-1
    auto a_boundary = copp::Boundary2{0.0, 0.0}; // a(0) = 0, a(1) = 0
    topp2::Problem problem{
        robot.constraints(),
        idx_s_interval,
        a_boundary,
    };
    ```

=== "C"

    ```c
    struct Topp2Problem problem = {
        robot,
        0,
        n - 1,
        0.0,
        0.0,
    };
    ```

然后构造求解设置并调用求解器，例如：

=== "Rust"

    ```rust
    use copp::solver::topp2_ra::{ReachSet2OptionsBuilder, topp2_ra};

    let options = ReachSet2OptionsBuilder::new().build()?;
    let a_ra = topp2_ra(&problem, &options)?;
    ```

=== "Python"

    ```python
    a_ra = copp.solver.topp2_ra.solve(
        problem,
        copp.solver.topp2_ra.Options(),
    )
    ```

=== "Matlab"

    ```matlab
    options = copp.solver.topp2_ra.Options();
    a_ra = copp.solver.topp2_ra.solve(problem, options);
    ```

=== "C++"

    ```cpp
    auto options = topp2::Options{};
    auto a_ra = topp2::solve(problem, options);
    ```

=== "C"

    ```c
    struct Topp2RaOptions options;
    struct CoppVecF64 a_ra = {0};
    // On success, release `a_ra` later with `copp_vec_f64_free(a_ra)`.

    if (check(topp2_ra_default_options(&options), "topp2_ra_default_options")) {
        return 1;
    }
    if (check(topp2_ra(problem, options, &a_ra), "topp2_ra")) {
        return 1;
    }
    ```

据此，我们得到了 $a=a(s)$。

### Step 5. 二阶轨迹后处理（仅二阶需要）

我们接下来希望得到真实的轨迹 $\boldsymbol{q}=\boldsymbol{q}(t)$，特别地，应该得到插补轨迹以便于底层伺服驱动器跟踪。首先应求解 $t=t(s)$，例如：

=== "Rust"

    ```rust
    use copp::solver::topp2_ra::s_to_t_topp2;

    // t_final is the traversal time of the path.
    // t_s[i] is the time at which the path parameter s[i] is reached.
    let (t_final, t_s) = s_to_t_topp2(&s, &a_ra, 0.0)?;
    ```

=== "Python"

    ```python
    t_final, t_s = copp.interpolation.s_to_t_topp2(s, a_ra, 0.0)
    ```

=== "Matlab"

    ```matlab
    [t_final, t_s] = copp.solver.topp2_ra.s_to_t( ...
        s, a_ra, t0=0.0);
    ```

=== "C++"

    ```cpp
    // t_final is the traversal time of the path.
    // t_s[i] is the time at which the path parameter s[i] is reached.
    auto time = copp::interpolation::s_to_t_topp2(s, a_ra, 0.0);
    double t_final = time.t_final;
    const auto &t_s = time.t_s;
    ```

=== "C"

    ```c
    double t_final = 0.0;
    struct CoppVecF64 t_s = {0};
    // On success, release `t_s` later with `copp_vec_f64_free(t_s)`.

    if (check(
            copp_s_to_t_2nd(
                (struct CoppSliceF64){s, n},
                (struct CoppSliceF64){a_ra.data, a_ra.len},
                0.0,
                &t_final,
                &t_s),
            "copp_s_to_t_2nd")) {
        return 1;
    }
    ```

接下来求逆解 $s=s(t)$ 并插补，例如：

=== "Rust"

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

=== "Python"

    ```python
    s_t = copp.interpolation.t_to_s_topp2_uniform(
        s,
        a_ra,
        t_s,
        dt=1.0e-3,
        t0=0.0,
        include_final=True,
    )
    ```

=== "Matlab"

    ```matlab
    dt = 1.0e-3;
    s_t = copp.solver.topp2_ra.t_to_s( ...
        s, a_ra, t_s, dt=dt, ...
        t0=0.0, include_final=true);
    ```

=== "C++"

    ```cpp
    // s_t is a uniform time grid of s(t) with dt = 1e-3s. This is useful for plotting and downstream control.
    const double dt = 1.0e-3;
    auto s_t = copp::interpolation::t_to_s_topp2_uniform(s, a_ra, t_s, dt);
    ```

=== "C"

    ```c
    // s_t is a uniform time grid of s(t) with dt = 1e-3s. This is useful for plotting and downstream control.
    const double dt = 1e-3;
    struct CoppVecF64 s_t = {0};
    // On success, release `s_t` later with `copp_vec_f64_free(s_t)`.

    if (check(
            copp_t_to_s_uniform_2nd(
                (struct CoppSliceF64){s, n},
                (struct CoppSliceF64){a_ra.data, a_ra.len},
                (struct CoppSliceF64){t_s.data, t_s.len},
                0.0,
                dt,
                true,
                &s_t),
            "copp_t_to_s_uniform_2nd")) {
        return 1;
    }
    ```

上述插补也支持非均匀时间采样方式，详见[文档章节](#docs-architecture)。最后可以求解插补轨迹 $\boldsymbol{q}=\boldsymbol{q}(t)$，一种简单的做法是：

=== "Rust"

    ```rust
    let out = path.evaluate_q(&s_t)?;
    let q_t = out.q;
    ```

=== "Python"

    ```python
    q_t = path.evaluate_q(s_t).q
    ```

=== "Matlab"

    ```matlab
    q_t = path.evaluate_q(s_t);  % dim x numel(s_t)
    ```

=== "C++"

    ```cpp
    auto out = path.evaluate_up_to_2nd(s_t);
    const auto &q_t = out.q;
    const auto &dq_t = out.dq.value();
    const auto &ddq_t = out.ddq.value();
    ```

=== "C"

    ```c
    struct CoppMatrixF64 q_t = {0};
    struct CoppMatrixF64 dq_t = {0};
    struct CoppMatrixF64 ddq_t = {0};
    // On success, release returned matrices with `copp_matrix_f64_free`.

    if (check(
            copp_path_evaluate_up_to_2nd(
                path,
                (struct CoppSliceF64){s_t.data, s_t.len},
                &q_t,
                &dq_t,
                &ddq_t),
            "copp_path_evaluate_up_to_2nd")) {
        return 1;
    }

    // q_t is a column-major DIM x s_t.len matrix: q_t.data[row + col * q_t.rows].
    copp_matrix_f64_free(ddq_t);
    copp_matrix_f64_free(dq_t);
    copp_matrix_f64_free(q_t);
    ```

由此完成了二阶轨迹的完整求解。如果是求解三阶轨迹，那么二阶轨迹后处理步骤可以跳过，并继续如下流程。

### Step 6. 构造并求解三阶问题（仅三阶需要）

构造三阶问题如下，其中非凸的三阶约束用前面求解的二阶轨迹 `a_ra` 进行线性化：

=== "Rust"

    ```rust
    use copp::solver::topp3_socp::Topp3ProblemBuilder;

    // Note that in TOPP3Problem, the non-convex jerk constraints should be linearized into a convex one.
    // More details can be found in the documentation of `Topp3ProblemBuilder::build_with_linearization`.
    let topp3_problem =
        Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &a_ra, (0.0, 0.0), (0.0, 0.0))
        .build_with_linearization()?;
    ```

=== "Python"

    ```python
    robot.constraints.amax_substitute(a_ra, 0)
    problem = copp.solver.topp3_socp.Problem(
        robot.constraints,
        a_ra,
        idx_s_start=0,
        a_boundary=(0.0, 0.0),
        b_boundary=(0.0, 0.0),
    )
    ```

=== "Matlab"

    ```matlab
    problem = copp.solver.topp3_socp.Problem( ...
        robot, ...
        a_ra, ...
        idx_s_start=1, ...
        a_boundary=[0, 0], ...
        b_boundary=[0, 0]);
    ```

    三阶 MATLAB `Problem` 在 `solve` 时按 `a_ra` 进行 lazy linearize。

=== "C++"

    ```cpp
    robot.constraints().amax_substitute(a_ra, 0);

    auto boundary = copp::Boundary3{0.0, 0.0, 0.0, 0.0};
    copp::solver::topp3::Problem topp3_problem{
        robot.constraints(),
        a_ra,
        0,
        boundary,
    };
    ```

=== "C"

    ```c
    if (check(
            copp_robot_amax_substitute(
                robot,
                (struct CoppSliceF64){a_ra.data, a_ra.len},
                0),
            "copp_robot_amax_substitute")) {
        return 1;
    }

    struct Topp3Problem topp3_problem = {
        robot,
        0,
        (struct CoppSliceF64){a_ra.data, a_ra.len},
        0.0,
        0.0,
        0.0,
        0.0,
        1,
        1,
        1e-10,
    };
    ```

我们以 `topp3_socp` 为例，调用求解器如下：

=== "Rust"

    ```rust
    use copp::solver::topp3_socp::{ClarabelOptionsBuilder, topp3_socp};

    let options_socp = ClarabelOptionsBuilder::new()
        .allow_almost_solved(true)
        .build()?;
    let profile = topp3_socp(&topp3_problem, &options_socp)?;
    ```

=== "Python"

    ```python
    options = copp.solver.topp3_socp.Options(allow_almost_solved=True)
    profile = copp.solver.topp3_socp.solve(problem, options)
    ```

=== "Matlab"

    ```matlab
    options = copp.solver.topp3_socp.Options( ...
        allow_almost_solved=true);
    profile = copp.solver.topp3_socp.solve(problem, options);
    ```

=== "C++"

    ```cpp
    namespace topp3_socp = copp::solver::topp3_socp;

    copp::clarabel::Options options_socp;
    options_socp.allow_almost_solved = true;
    auto profile = topp3_socp::solve(topp3_problem, options_socp);
    ```

=== "C"

    ```c
    struct CoppClarabelOptions options_socp;
    struct CoppProfile3rd profile = {0};
    // On success, release `profile` later with `copp_profile_3rd_free(profile)`.

    if (check(copp_clarabel_default_options(&options_socp), "copp_clarabel_default_options")) {
        return 1;
    }
    options_socp.allow_almost_solved = true;

    if (check(topp3_socp(topp3_problem, options_socp, &profile), "topp3_socp")) {
        return 1;
    }
    ```

理论上这已经生成了一条可行、近优的三阶轨迹 $a_1(s),b_1(s)$ 了，可以直接进行下一步。如果希望通过更多的计算资源进一步地求解更优的轨迹，可以以 $a_1(s)$ 为线性化点再次求解新的线性化三阶问题：

=== "Rust"

    ```rust
    let topp3_problem =
        Topp3ProblemBuilder::new(&mut robot, idx_s_interval.0, &profile.a, (0.0, 0.0), (0.0, 0.0))
        .build_with_linearization()?;
    let profile = topp3_socp(&topp3_problem, &options_socp)?;
    ```

=== "Python"

    ```python
    problem = copp.solver.topp3_socp.Problem(
        robot.constraints,
        profile.a,
        idx_s_start=0,
        a_boundary=(0.0, 0.0),
        b_boundary=(0.0, 0.0),
    )
    profile = copp.solver.topp3_socp.solve(problem, options)
    ```

=== "Matlab"

    ```matlab
    problem = copp.solver.topp3_socp.Problem( ...
        robot, profile.a, ...
        idx_s_start=1, ...
        a_boundary=[0, 0], ...
        b_boundary=[0, 0]);
    profile = copp.solver.topp3_socp.solve(problem, options);
    ```

=== "C++"

    ```cpp
    copp::solver::topp3::Problem topp3_problem_next{
        robot.constraints(),
        profile.a,
        0,
        boundary,
    };
    profile = topp3_socp::solve(topp3_problem_next, options_socp);
    ```

=== "C"

    ```c
    struct Topp3Problem topp3_problem_next = {
        robot,
        0,
        (struct CoppSliceF64){profile.a.data, profile.a.len},
        0.0,
        0.0,
        0.0,
        0.0,
        1,
        1,
        1e-10,
    };
    struct CoppProfile3rd profile_next = {0};
    // On success, release `profile_next` with `copp_profile_3rd_free(profile_next)`,
    // or move it into `profile` as below and release `profile` once at cleanup.

    if (check(
            topp3_socp(topp3_problem_next, options_socp, &profile_next),
            "topp3_socp second iteration")) {
        return 1;
    }

    copp_profile_3rd_free(profile);
    profile = profile_next;
    profile_next = (struct CoppProfile3rd){0};
    ```

在计算资源允许的情况下，可以重复进行上述过程，也就是用 $a_k(s)$ 线性化三阶非凸问题并求解得到 $a_{k+1}(s),b_{k+1}(s)$，在非退化情况下最终能够收敛到 KKT 解。从在线进行的实用角度，我们推荐完成 1 到 2 次线性化即足够。

### Step 7. 三阶轨迹后处理（仅三阶需要）

我们接下来希望得到真实的轨迹 $\boldsymbol{q}=\boldsymbol{q}(t)$，特别地，应该得到插补轨迹以便于底层伺服驱动器跟踪。首先应求解 $t=t(s)$，例如：

=== "Rust"

    ```rust
    use copp::solver::topp3_socp::s_to_t_topp3;

    // t_final is the traversal time of the path.
    // t_s[i] is the time at which the path parameter s[i] is reached.
    let (t_final, t_s) = s_to_t_topp3(&s, profile.as_parts(), 0.0)?;
    ```

=== "Python"

    ```python
    # t_final is the traversal time of the path.
    # t_s[i] is the time at which the path parameter s[i] is reached.
    t_final, t_s = copp.interpolation.s_to_t_topp3(s, profile, 0.0)
    ```

=== "Matlab"

    ```matlab
    [t_final, t_s] = copp.solver.topp3_socp.s_to_t( ...
        s, profile, t0=0.0);
    ```

=== "C++"

    ```cpp
    // t_final is the traversal time of the path.
    // t_s[i] is the time at which the path parameter s[i] is reached.
    auto time = copp::interpolation::s_to_t_topp3(s, profile, 0.0);
    double t_final = time.t_final;
    const auto &t_s = time.t_s;
    ```

=== "C"

    ```c
    double t_final = 0.0;
    struct CoppVecF64 t_s = {0};
    // On success, release `t_s` later with `copp_vec_f64_free(t_s)`.

    if (check(
            copp_s_to_t_3rd(
                (struct CoppSliceF64){s, n},
                (struct CoppSliceF64){profile.a.data, profile.a.len},
                (struct CoppSliceF64){profile.b.data, profile.b.len},
                profile.num_stationary_start,
                profile.num_stationary_end,
                0.0,
                &t_final,
                &t_s),
            "copp_s_to_t_3rd")) {
        return 1;
    }
    ```

接下来求逆解 $s=s(t)$ 并插补，例如：

=== "Rust"

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

=== "Python"

    ```python
    # s_t is a uniform time grid of s(t) with dt = 1e-3s. This is useful for plotting and downstream control.
    s_t = copp.interpolation.t_to_s_topp3_uniform(
        s,
        profile,
        t_s,
        dt = 1.0e-3,
        t0=0.0,
    )
    ```

=== "Matlab"

    ```matlab
    dt = 1.0e-3;
    s_t = copp.solver.topp3_socp.t_to_s( ...
        s, profile, t_s, dt=dt, ...
        t0=0.0, include_final=true);
    ```

=== "C++"

    ```cpp
    // s_t is a uniform time grid of s(t) with dt = 1e-3s. This is useful for plotting and downstream control.
    const double dt = 1.0e-3;
    auto s_t = copp::interpolation::t_to_s_topp3_uniform(s, profile, t_s, dt);
    ```

=== "C"

    ```c
    // s_t is a uniform time grid of s(t) with dt = 1e-3s. This is useful for plotting and downstream control.
    const double dt = 1e-3;
    struct CoppVecF64 s_t = {0};
    // On success, release `s_t` later with `copp_vec_f64_free(s_t)`.

    if (check(
            copp_t_to_s_uniform_3rd(
                (struct CoppSliceF64){s, n},
                (struct CoppSliceF64){profile.a.data, profile.a.len},
                (struct CoppSliceF64){profile.b.data, profile.b.len},
                profile.num_stationary_start,
                profile.num_stationary_end,
                (struct CoppSliceF64){t_s.data, t_s.len},
                0.0,
                dt,
                true,
                &s_t),
            "copp_t_to_s_uniform_3rd")) {
        return 1;
    }
    ```

上述插补也支持非均匀时间采样方式，详见[文档章节](#docs-architecture)。最后可以求解插补轨迹 $\boldsymbol{q}=\boldsymbol{q}(t)$，一种简单的做法是：

=== "Rust"

    ```rust
    let out = path.evaluate_q(&s_t)?;
    let q_t = out.q;
    ```

=== "Python"

    ```python
    q_t = path.evaluate_q(s_t).q
    ```

=== "Matlab"

    ```matlab
    [q_t, dq_t, ddq_t, dddq_t] = path.evaluate_up_to_3rd(s_t);
    ```

    输出矩阵均为 `dim x numel(s_t)`。

=== "C++"

    ```cpp
    auto out = path.evaluate_up_to_3rd(s_t);
    const auto &q_t = out.q;
    const auto &dq_t = out.dq.value();
    const auto &ddq_t = out.ddq.value();
    const auto &dddq_t = out.dddq.value();
    ```

=== "C"

    ```c
    struct CoppMatrixF64 q_t = {0};
    struct CoppMatrixF64 dq_t = {0};
    struct CoppMatrixF64 ddq_t = {0};
    struct CoppMatrixF64 dddq_t = {0};
    // On success, release returned matrices with `copp_matrix_f64_free`.

    if (check(
            copp_path_evaluate_up_to_3rd(
                path,
                (struct CoppSliceF64){s_t.data, s_t.len},
                &q_t,
                &dq_t,
                &ddq_t,
                &dddq_t),
            "copp_path_evaluate_up_to_3rd")) {
        return 1;
    }

    // q_t is a column-major DIM x s_t.len matrix: q_t.data[row + col * q_t.rows].
    copp_matrix_f64_free(dddq_t);
    copp_matrix_f64_free(ddq_t);
    copp_matrix_f64_free(dq_t);
    copp_matrix_f64_free(q_t);
    ```

由此完成了三阶轨迹的完整求解。

### 资源释放

=== "Rust"

    Rust 版本不需要手动释放资源，相关对象会在离开作用域时自动释放。

=== "Python"

    Python 版本不需要手动释放资源，相关对象由 Python 运行时自动管理。

=== "Matlab"

    MATLAB 对象会在 `delete` 时释放 native handle，普通脚本通常不需要手动释放。长脚本或测试中可以显式释放，或者使用 `onCleanup`：

    ```matlab
    cleanup_path = onCleanup(@() path.release());
    cleanup_robot = onCleanup(@() robot.release());
    ```

=== "C++"

    C++ 版本不需要手动释放资源。`Path`、`Robot`、`Constraints`、`Profile3rd`、`Matrix` 等对象遵循 RAII，离开作用域后会自动释放 Rust 侧句柄和 C++ 侧存储。`Span`、`MatrixView` 和 `MatrixRef` 只是借用视图，不拥有内存，也不需要释放。

=== "C"

    C ABI 中，`CoppSliceF64` 和 `CoppMatrixViewF64` 只是借用用户内存，不需要释放；所有由 COPP 返回的 `CoppVecF64`、`CoppMatrixF64`、`CoppProfile3rd` 以及 `CoppPath`、`CoppRobot` 句柄都需要用匹配的函数释放。实际工程里推荐先把这些拥有型对象初始化为 `{0}` 或 `NULL`，再使用集中 `cleanup` 风格，确保中途出错也能释放已经创建的对象，例如：

    ```c
    cleanup:
        // Path evaluation outputs.
        copp_matrix_f64_free(dddq_t);
        copp_matrix_f64_free(ddq_t);
        copp_matrix_f64_free(dq_t);
        copp_matrix_f64_free(q_t);

        // Interpolation outputs and solver profiles.
        copp_vec_f64_free(s_t);
        copp_vec_f64_free(t_s);
        copp_profile_3rd_free(profile);
        copp_vec_f64_free(a_ra);

        // Long-lived handles should usually be released last.
        copp_robot_free(robot);
        copp_path_free(path);
        return rc;
    ```

    如果只求解二阶轨迹，则没有 `profile`、`dddq_t` 等三阶对象；如果保留多次三阶迭代结果，例如 `profile_qp1` 和 `profile_qp2`，则每个未被移动走所有权的 `CoppProfile3rd` 都需要各自调用一次 `copp_profile_3rd_free`。仓库中的 C 例程也采用了这种集中释放方式。

### Step-by-Step 小结

总的来说，一个最小闭环包括：构造路径 $\boldsymbol{q}(s)$，在离散网格上构造 `Robot` 和约束，选择对应的 problem builder 和 solver，得到 $a(s)$ 或 $(a(s),b(s))$，再通过 `s_to_t_*` 和 `t_to_s_*` 转回时间域，最终在 $s(t)$ 上重新采样原始路径。仓库中也提供了 TOPP2、COPP2、TOPP3、COPP3 等可运行例程。

## Benchmark 性能测试

以下测试来自仓库中的 `tests/test_random_spline.rs`。测试条件为：

- `release, --include-ignored`
- CPU: Intel(R) Core(TM) Ultra 9 285K.
- 数据集：100 条随机 7-DOF 样条路径，每条路径离散为 1000 个区间。

所有指标均以 `mean ± std` 的形式列出。

#### 时间最优 (Time-Optimal)

| 方法                   | 可用版本 |          计算时间 (ms) |         终端时间 (s) |
| ---------------------- | -------- | ---------------------: | -------------------: |
| TOPP2-RA               | 开源     |    0.615425 ± 0.244409 | 40.903420 ± 1.378671 |
| COPP2-SOCP             | 开源     |  149.969964 ± 9.364334 | 40.900039 ± 1.378613 |
| COPP2-RDDP             | PRO      |    5.436142 ± 0.465495 | 40.900135 ± 1.378613 |
| TOPP3-LP               | 开源     | 327.074029 ± 28.893341 | 41.422945 ± 1.381874 |
| TOPP3-SOCP             | 开源     | 289.654071 ± 12.862133 | 41.418608 ± 1.381202 |
| COPP3-SOCP             | 开源     | 285.004302 ± 13.471264 | 41.418608 ± 1.381202 |
| TOPP3-RA (Iteration 1) | PRO      |   10.571045 ± 0.857653 | 41.499200 ± 1.385735 |
| TOPP3-RA (Iteration 2) | PRO      |   20.300932 ± 1.237908 | 41.399867 ± 1.386791 |

#### 凸目标 (Convex-Objective)

在该测试中，TOPP 方法仍以终端时间为优化目标。

| 方法       | 可用版本 |          计算时间 (ms) |             目标函数值 |
| ---------- | -------- | ---------------------: | ---------------------: |
| TOPP2-RA   | 开源     |    0.534700 ± 0.069296 | 217.444861 ± 12.462360 |
| COPP2-SOCP | 开源     | 270.059250 ± 52.073677 |   96.517354 ± 3.641154 |
| COPP2-RDDP | PRO      |   12.667700 ± 0.429214 |   96.525785 ± 3.639733 |
| TOPP3-LP   | 开源     |  348.000000 ± 9.326314 | 211.611085 ± 12.367224 |
| TOPP3-SOCP | 开源     | 301.227000 ± 12.938498 | 211.974066 ± 12.323865 |
| COPP3-SOCP | 开源     | 301.227000 ± 12.938498 |   96.634962 ± 3.613264 |
| COPP3-RDDP | PRO      |   65.823050 ± 0.087893 |   98.708998 ± 3.354004 |

## 文档与架构 { #docs-architecture }

### 文档

=== "Rust"

    我们推荐使用 [docs.rs 最新文档](https://docs.rs/copp/latest/copp/)，也支持本地文档：

    - [v0.2.1 (Latest)](rust/v0.2.1/copp/)
    - [v0.2.0](rust/v0.2.0/copp/)
    - [v0.1.0](rust/v0.1.0/copp/)

    如果需要查看 main branch 上尚未发布的更新，我们推荐在 `copp` 仓库根目录本地生成文档：

    ```shell
    cargo doc --no-deps --open
    ```

    生成的文档包含数学基础、路径/约束构造方法、日志和输出约定、错误定义以及求解器接口。

=== "Python"

    Python 文档如下：

    - [v0.2.1 (Latest)](python/v0.2.1/index.html)

    如果需要本地生成 Python 文档，先安装 Sphinx，然后在 `copp` 仓库根目录运行：

    ```sh
    python -m pip install -U sphinx
    python -m sphinx -E -b html bindings/python/docs/source bindings/python/docs/build/html
    ```

    生成的入口页面是 `bindings/python/docs/build/html/index.html`。

=== "Matlab"

    MATLAB 文档可在本地通过 `publish` 生成：

    ```matlab
    cd bindings/matlab/docs
    build_docs()
    ```

    生成的入口页面位于 `bindings/matlab/docs/html/index.html`。

=== "C++"

    C++ API 文档使用 Doxygen 生成，内容来自 `bindings/cpp/include/copp/*.hpp`、`bindings/cpp/docs/*.md` 和示例代码。发布版文档后续会随 SDK 一起整理；如果需要查看开发分支上的 C++ 接口，可以在 `copp` 仓库根目录本地生成：

    ```sh
    doxygen bindings/cpp/Doxyfile
    ```

    生成的入口页面是 `bindings/cpp/docs/html/index.html`。若希望渲染 include 关系图和调用图，需要额外安装 Graphviz；若公式显示异常，优先检查 Doxygen 的 MathJax 配置。

=== "C"

    C 文档如下：

    - [v0.2.1 (Latest)](c/v0.2.1/index.html)
    - [v0.2.0](c/v0.2.0/index.html)

    如果需要本地生成 C 文档，先安装 Doxygen、Graphviz 和 PowerShell 7（即 `pwsh`，脚本会用它重新生成头文件），然后在 `copp` 仓库根目录运行对应命令：

    === "Linux"

        ```sh
        # Debian/Ubuntu 示例；还需要单独安装 PowerShell 7，确保 `pwsh` 在 PATH 中。
        sudo apt install doxygen graphviz
        sh bindings/c/scripts/generate_docs.sh
        ```

    === "Windows"

        ```powershell
        winget install Doxygen.Doxygen Graphviz.Graphviz Microsoft.PowerShell
        powershell -ExecutionPolicy Bypass -File bindings/c/scripts/generate_docs.ps1
        ```

    === "macOS"

        ```sh
        brew install doxygen graphviz powershell
        sh bindings/c/scripts/generate_docs.sh
        ```

    生成的入口页面是 `bindings/c/docs/html/index.html`。

### 项目架构

=== "Rust"

    | 模块          | 负责内容                             |
    | ------------- | ------------------------------------ |
    | `path`        | 路径构造、参数范围、二阶/三阶求导。  |
    | `robot`       | 机器人维度、逆动力学、物理约束入口。 |
    | `constraints` | 更底层的约束接口。                   |
    | `solver`      | 各个求解器。                         |
    | `diag`        | 错误类型、日志 verbosity、诊断信息。 |

=== "Python"

    | 模块                    | 负责内容                                                                                              |
    | ----------------------- | ----------------------------------------------------------------------------------------------------- |
    | `copp_py`               | 顶层包，导出常用类型、函数和子模块入口。                                                              |
    | `copp_py.path`          | `Path`、`SplineConfig`、路径求值、waypoint 样条、自动微分与用户 evaluator 路径。                      |
    | `copp_py.robot`         | `Robot`、路径采样、速度/加速度/jerk/力矩约束、高级物理约束入口和逆动力学 callback。                   |
    | `copp_py.constraints`   | `Constraints` 原始约束缓冲区、一/二/三阶底层约束、`amax_substitute` 和滑动窗口操作。                  |
    | `copp_py.solver`        | 求解器命名空间，包含 `topp2_ra`、`reach_set2`、`copp2_socp`、`topp3_lp`、`topp3_socp`、`copp3_socp`。 |
    | `copp_py.interpolation` | `Profile3rd`、二阶/三阶 `s_to_t_*`、`t_to_s_*` 和 `a_to_b_topp2` 轨迹后处理。                         |
    | `copp_py.objective`     | COPP2/COPP3 目标函数描述，包括时间、线性、热能耗散、力矩全变分等目标。                                |
    | `copp_py.clarabel`      | Clarabel SOCP 选项、设置、solver 状态和 expert 诊断结果。                                             |
    | `copp_py.core`          | 版本、错误类型、枚举、矩阵布局和通用类型约定。                                                        |

=== "Matlab"

    | 模块                                | 负责内容                                                       |
    | ----------------------------------- | -------------------------------------------------------------- |
    | `copp`                          | 顶层包，提供 `version`、`Path`、`Robot`、`Profile3rd` 等常用类型。 |
    | `copp.Path`                     | waypoint、evaluator、parametric、symbolic、CasADi 路径。       |
    | `copp.Robot`                    | 站点网格、路径采样、速度/加速度/jerk/力矩约束。                |
    | `copp.objective`                | 时间、线性、热能耗散、力矩全变分等 COPP 目标描述。             |
    | `copp.solver`                   | `topp2_ra`、`reach_set2`、`copp2_socp`、`topp3_*`、`copp3_*`。 |
    | `copp.interpolation`            | 二阶/三阶 `s_to_t_*`、`t_to_s_*` 和 `a_to_b_topp2`。           |
    | `copp.clarabel`                 | Clarabel 求解器选项、设置和 direct solve method。              |
    | `copp.diag`                     | MATLAB exception facade、verbosity 和 MEX last-error 快照。     |

=== "C++"

    | Header / Namespace                 | 负责内容                                               |
    | ---------------------------------- | ------------------------------------------------------ |
    | `copp/copp.hpp`                    | Umbrella header，包含 C++ public facade。              |
    | `copp/core.hpp`                    | `Matrix`、`Span`、`Error`、`Expected` 等核心类型。      |
    | `copp/path.hpp`                    | Waypoint / parametric / evaluator path 构造与求导。     |
    | `copp/robot.hpp`                   | `Robot`、`Constraints`、逆动力学 callback 和物理约束。 |
    | `copp/interpolation.hpp`           | TOPP2 / TOPP3 profile 与时间插值。                     |
    | `copp/solver/*.hpp`                | TOPP/COPP 求解器 namespace 和 problem/options/result。 |
    | `copp/eigen.hpp`                   | 可选 Eigen adapter；核心头不直接依赖 Eigen。           |
    | CMake target `copp::copp`          | 下游 C++ 工程链接入口；C ABI 另有 `copp::c_abi`。      |
=== "C"

    | 头文件                 | 负责内容                                               |
    | ---------------------- | ------------------------------------------------------ |
    | `copp/copp.h`          | Umbrella header，包含完整 C ABI。                      |
    | `copp/core.h`          | 状态码、last error、矩阵/向量视图、Clarabel 选项。     |
    | `copp/path.h`          | 路径句柄、样条路径、callback 路径、路径求值。          |
    | `copp/robot.h`         | 机器人句柄、路径采样、物理约束、逆动力学 callback。    |
    | `copp/formulation.h`   | TOPP/COPP problem descriptor、目标函数、profile 类型。 |
    | `copp/interpolation.h` | 二阶/三阶轨迹后处理与插补。                            |
    | `copp/topp2.h`         | TOPP2-RA 和 ReachSet2 接口。                           |
    | `copp/copp2.h`         | COPP2-SOCP 接口。                                      |
    | `copp/topp3.h`         | TOPP3-LP 和 TOPP3-SOCP 接口。                          |
    | `copp/copp3.h`         | COPP3-SOCP 接口。                                      |

## 引用

如果你的工作使用了开源 TOPP3/COPP3 功能，建议引用[如下论文](https://doi.org/10.1016/j.ijmachtools.2025.104355)：（即便是最基本的离散区间内 profile 模板也用到了该文章的贡献）

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

如果你的工作使用了 PRO 版本中的 TOPP3-RA, COPP2-RDDP, COPP3-RDDP 方法，建议引用[如下论文](https://arxiv.org/abs/2605.19089)：（其中 TOPP3-RA 基于该论文进行改进）

```tex
@article{wang2026reachability,
  title={Reachability-augmented dual dynamic programming for optimal path parameterization},
  author={Yunan Wang and Jizhou Yan and Chuxiong Hu and Zeyang Li},
  journal={arXiv preprint arXiv:2605.19089},
  year={2026}
}
```

其他情况下可引用 [`COPP` 库](https://github.com/TOPP-THU/copp)，或对应论文：

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

如果需要 COPP PRO 授权、商业合作、技术咨询或一般问题，可以联系 [hello@copp.pro](mailto:hello@copp.pro)。
