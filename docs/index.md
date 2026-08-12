# COPP Documentation

[![License: MIT](https://img.shields.io/badge/license-MIT-yellow.svg)](https://github.com/TOPP-THU/copp/blob/main/LICENSE) [![Website](https://img.shields.io/badge/website-copp.pro-2ff0d8)](https://copp.pro/) [![Docs](https://img.shields.io/badge/docs-docs.copp.pro-1f6feb)](https://docs.copp.pro/) [![Crates.io](https://img.shields.io/crates/v/copp.svg)](https://crates.io/crates/copp) [![PyPI](https://img.shields.io/pypi/v/copp-py.svg)](https://pypi.org/project/copp-py/)

[![Rust](https://img.shields.io/badge/Rust-native-b7410e)](https://docs.rs/copp/latest/copp/) [![C](https://img.shields.io/badge/C-ABI-a8b9cc)](https://github.com/TOPP-THU/copp/tree/main/bindings/c) [![Python](https://img.shields.io/badge/Python-bindings-ffd43b)](https://github.com/TOPP-THU/copp/tree/main/bindings/python) [![C++](https://img.shields.io/badge/C%2B%2B-bindings-00599c)](https://github.com/TOPP-THU/copp/tree/main/bindings/cpp) [![MATLAB](https://img.shields.io/badge/MATLAB-bindings-e16737)](https://github.com/TOPP-THU/copp/tree/main/bindings/matlab)

## Core Problem

COPP focuses on generating a time-parameterized trajectory from a given geometric path while satisfying user-specified constraints and optimizing a specified objective. In particular, trajectories planned by COPP are generally second-order smooth (bounded acceleration) or third-order smooth (bounded jerk).

### Path Parameterization

Given an $n$-dimensional robotic system and a sufficiently smooth geometric path:

$$
\boldsymbol{q}=\boldsymbol{q}(s)\in\mathbb{R}^n,\qquad s\in[0,s_\text{f}],
$$

where $s$ is a generic path parameter and $s_\text{f}$ is known. For an order-$m$ problem, $m\in\{2,3\}$, the path $\boldsymbol{q}=\boldsymbol{q}(s)$ is required to be $\mathcal{C}^m$ continuous in the $s$ domain. The goal of $m$-th order path parameterization is to construct a strictly increasing time parameterization

$$
s=s(t),\qquad t\in[0,t_\text{f}],
$$

such that $\frac{\mathrm{d}^ms}{\mathrm{d}t^m}(t)$ exists and is bounded except at finitely many time instants, where the terminal time $t_\text{f}$ is unknown. This yields a robot trajectory that strictly follows the prescribed geometric path:

$$
\boldsymbol{q}=\boldsymbol{q}(s(t))\in\mathbb{R}^n,\qquad t\in[0,t_\text{f}].
$$

After interpolation, the trajectory can be sent to a low-level servo system as a reference trajectory. If the servo period is $T_\text{s}$ (for example, $T_\text{s}=\text{1ms}$), the data sent to the low-level controller typically includes the interpolated trajectory

$$
\{\boldsymbol{q}(s(iT_\text{s}))\}_{i\in\mathbb{N}}
$$

and the corresponding feedforward quantities, such as the reference velocity terms $\{\dot{\boldsymbol{q}}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$ required by PID control, as well as reference acceleration terms $\{\ddot{\boldsymbol{q}}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$ and torque terms $\{\boldsymbol{\tau}(s(iT_\text{s}))\}_{i\in\mathbb{N}}$ when needed.

Let $\dot{\bullet}$ denote differentiation with respect to time $t$, and let $\bullet'$ denote differentiation with respect to the path parameter $s$. Define

$$
a(s)=\dot{s}^2,\qquad b(s)=\ddot{s}=\frac12a'(s),\qquad c(s)=\frac{\dddot{s}}{\dot{s}}=b'(s).
$$

In second-order problems, the state is $a(s)$ and the control is $b(s)$. In third-order problems, the state is $(a(s),b(s))$ and the control is $c(s)$.

### Optimal Path Parameterization

Compared with standard path parameterization, optimal path parameterization additionally introduces constraints and an optimization objective.

Constraints are used to improve trajectory smoothness and keep the reference trajectory within actuator performance limits, which in practice helps reduce vibration and improve tracking accuracy. First-order constraints can represent composite velocity limits $\|\boldsymbol{J(s)}\dot{\boldsymbol{q}}(s)\|\leq V(s)$, axis-wise velocity limits $\dot{\boldsymbol{q}}_\text{min}(s)\leq\dot{\boldsymbol{q}}(s)\leq\dot{\boldsymbol{q}}_\text{max}(s)$, and related bounds. The general form is

$$
a(s)\leq a^+(s).
$$

Second-order constraints can represent axis-wise acceleration limits $\ddot{\boldsymbol{q}}_\text{min}(s)\leq\ddot{\boldsymbol{q}}(s)\leq\ddot{\boldsymbol{q}}_\text{max}(s)$, axis-wise torque limits $\boldsymbol{\tau}_\text{min}(s)\leq\boldsymbol{\tau}(s)\leq\boldsymbol{\tau}_\text{max}(s)$, and related bounds. The general form is

$$
\boldsymbol{n}(s)a(s)+\boldsymbol{m}(s)b(s)\leq\boldsymbol{g}(s).
$$

Third-order constraints can represent axis-wise jerk limits $\dddot{\boldsymbol{q}}_\text{min}(s)\leq\dddot{\boldsymbol{q}}(s)\leq\dddot{\boldsymbol{q}}_\text{max}(s)$, axis-wise torque-derivative limits $\dot{\boldsymbol{\tau}}_\text{min}(s)\leq\dot{\boldsymbol{\tau}}(s)\leq\dot{\boldsymbol{\tau}}_\text{max}(s)$, and related bounds. The general form is

$$
\sqrt{a(s)}\left(\boldsymbol{r}(s)a(s)+\boldsymbol{v}(s)b(s)+\boldsymbol{w}(s)c(s)+\boldsymbol{h}(s)\right)\leq\boldsymbol{f}(s).
$$

In these formulations, $a(s)$, $b(s)$, and $c(s)$ are the decision variables; all other quantities are known from the path, model, and physical constraints. The constraints also support piecewise bounds, station-dependent bounds, and asymmetric limits.

Common objectives include the terminal time

$$
t_\text{f}=\int_0^{t_\text{f}}\mathrm{d}t,
$$

thermal energy dissipation

$$
J_\text{th}=\int_0^{t_\text{f}}\left\|\frac{\boldsymbol{\tau}(t)}{\boldsymbol{\tau}_\text{normal}(t)}\right\|_2^2\mathrm{d}t,
$$

torque total variation

$$
J_\text{tv}=\int_0^{t_\text{f}}\left\|\frac{\mathrm{d}\boldsymbol{\tau}(t)}{\boldsymbol{\tau}_\text{normal}(t)}\right\|_1,
$$

and linear objectives

$$
J_\text{lin}=\begin{cases}
\int_0^{s_\text{f}}(\alpha(s)a(s)+\beta(s)b(s))\mathrm{d}s,&\text{second-order problem},\\
\int_0^{s_\text{f}}(\alpha(s)a(s)+\beta(s)b(s)+\gamma(s)c(s))\mathrm{d}s,&\text{third-order problem}.\\
\end{cases}
$$

The most common objective is the terminal time $t_\text{f}$, which gives Time-Optimal Path Parameterization (TOPP). More generally, one can optimize a linear combination of the objectives above, or other user-defined convex objectives, yielding Convex-Objective Path Parameterization (COPP). [Empirical results](https://arxiv.org/abs/2605.19089) show that, compared with pure TOPP, the terminal-time-plus-thermal-energy objective $J=t_\text{f}+\lambda J_\text{th}$ can significantly improve trajectory smoothness with only a minimal increase in traversal time.

In summary, the problem classes supported by `copp` are:

| Problem class            | Second-order constraints (velocity, acceleration, torque) | Third-order constraints (additional jerk, torque-derivative bounds, etc.) |
| ------------------------ | --------------------------------------------------------- | ------------------------------------------------------------------------- |
| Time objective           | TOPP2                                                     | TOPP3                                                                     |
| General convex objective | COPP2                                                     | COPP3                                                                     |

## Installation

=== "Rust"

    The `copp` crate is published on [crates.io](https://crates.io/crates/copp). Add the following dependency to the root `Cargo.toml`:

    ```toml
    [dependencies]
    copp = "0.2.1"
    ```

    `copp` v0.2.1 requires Rust 1.88 or newer. We strongly recommend building in Release mode for substantially better computational performance.

=== "Python"

    The Python package is published on [PyPI: copp-py](https://pypi.org/project/copp-py/). Install it with:

    ```sh
    python -m pip install copp-py
    ```

    The import package name is `copp_py`, typically used as:

    ```python
    import copp_py as copp
    ```

    To build from local source, you need Python 3.9 or newer, Rust/Cargo, `maturin`, and the platform C/C++ compiler toolchain: Visual Studio Build Tools with the C++ workload on Windows, GCC/Clang on Linux, or Xcode Command Line Tools on macOS. Start with the repository:

    ```sh
    git clone https://github.com/TOPP-THU/copp.git
    cd copp
    python -m pip install -U pip
    python -m pip install -U maturin numpy jax
    maturin develop --release --features python
    ```

    `maturin develop` installs the Rust extension module directly into the current Python environment. Verify the build with:

    ```sh
    python -c "import copp_py as copp; print(copp.version())"
    ```

=== "Matlab"

    Prebuilt MATLAB toolboxes are published with every release on the [COPP GitHub Releases page](https://github.com/TOPP-THU/copp/releases). Each release ships one `.mltbx` per platform, named `copp-matlab-<version>-<platform>.mltbx`, where `<platform>` is one of `windows-x86_64`, `linux-x86_64`, `macos-aarch64`, or `macos-x86_64`. Each `.mltbx` is accompanied by a `.sha256` checksum file. MATLAB R2024b or newer is required.

    Download the asset matching your platform. If you are unsure which one that is, run `computer("arch")` in MATLAB and use the table below:

    | `computer("arch")` | Release asset platform |
    | ------------------ | ---------------------- |
    | `win64`            | `windows-x86_64`       |
    | `glnxa64`          | `linux-x86_64`         |
    | `maca64`           | `macos-aarch64`        |
    | `maci64`           | `macos-x86_64`         |

    Pick the macOS asset carefully. MATLAB toolboxes only distinguish Windows, macOS, and Linux, so both macOS packages declare plain macOS support and MATLAB will not stop you from installing the Apple-silicon package on an Intel Mac; the mismatch would only surface later, when the MEX gateway fails to load.

    Install the downloaded toolbox by double-clicking it in MATLAB, or from the Command Window:

    ```matlab
    matlab.addons.toolbox.installToolbox("copp-matlab-<version>-<platform>.mltbx", true)
    copp.version()
    ```

    `copp.version()` reads the version out of the linked native library through the MEX gateway, so a successful call also confirms that the toolbox loaded correctly. Do not unpack `.mltbx` files manually: they are MATLAB toolbox installers, and installing them manages the MATLAB path for you.

    If you have the repository checked out, `bindings/matlab/install_copp.m` automates the same steps. It detects the platform, downloads the matching release asset, and installs it:

    ```matlab
    cd bindings/matlab
    install_copp()                    % latest release
    install_copp(Version="v0.2.2")    % a specific release
    ```

    To build from source instead, you additionally need Rust/Cargo and a MATLAB-supported C++ compiler. Start with the repository:

    ```sh
    git clone https://github.com/TOPP-THU/copp.git
    cd copp
    cargo build --release --lib --features matlab
    ```

    The Cargo `matlab` feature reuses the C ABI internally, so it does not enable MATLAB code in default pure-Rust builds. Then, in the MATLAB Command Window:

    ```matlab
    cd bindings/matlab
    mex -setup C++   % only needed the first time, or when changing compiler
    build()
    addpath(pwd)
    rehash
    copp.version()
    ```

    `build()` links statically by default, which is the recommended build for ordinary use. `build(Linkage="dynamic")` produces a smaller MEX file but requires the native library to be discoverable at runtime.

    Two path constructors need optional dependencies: `copp.Path.from_symbolic` requires the Symbolic Math Toolbox, and `copp.Path.from_casadi` requires CasADi. Everything else works with base MATLAB.

=== "C++"

    The C++ SDK suits projects that prefer RAII, `namespace copp`, `std::vector` / `copp::Matrix`, and an optional Eigen adapter. Prebuilt C++ SDKs for released versions are available from [GitHub Releases](https://github.com/TOPP-THU/copp/releases). C++ currently has no single de-facto official distribution platform comparable to crates.io for Rust or PyPI for Python; the common choices are GitHub Releases plus a CMake package, or a vcpkg / Conan recipe provided later.

    To build from source, start with the repository:

    ```sh
    git clone https://github.com/TOPP-THU/copp.git
    cd copp
    ```

    You need Cargo, CMake, and a C++17-capable compiler toolchain: Visual Studio Build Tools with the C++ workload on Windows, GCC/Clang on Linux, or Xcode Command Line Tools on macOS. If you use `copp/eigen.hpp`, Eigen is required as well; if you only use the core C++ API, the Eigen adapter can be turned off in CMake.

    The C++ facade is gated behind the Cargo `cpp` feature. First build the native library from the repository root:

    ```sh
    cargo build --release --lib --features cpp
    ```

    Then configure and build the CMake project:

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

    To install it as a package discoverable by downstream CMake projects:

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

    Replace `<install-prefix>` with your own installation path, for example `C:/copp-install` or `$PWD/copp-install`.

    Use it from a downstream project as follows:

    ```cmake
    find_package(copp CONFIG REQUIRED)

    add_executable(app main.cpp)
    target_link_libraries(app PRIVATE copp::copp)
    ```

    Pass the installation prefix to CMake when configuring the downstream project:

    ```sh
    cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
    cmake --build app/build --config Release
    ```

    If static linking is desired, add `-DCOPP_LINK_STATIC=ON` to the corresponding COPP CMake configure command above. If you prefer a shared library, add `-DCOPP_LINK_STATIC=OFF` instead and make sure `copp.dll`, `libcopp.so`, or `libcopp.dylib` is discoverable at runtime.

=== "C"

    The C ABI is suitable for C/C++ projects, downstream language bindings, and existing robotics software stacks. Prebuilt C ABI SDKs for released versions are available from [GitHub Releases](https://github.com/TOPP-THU/copp/releases). To build from source, start with the repository:

    ```sh
    git clone https://github.com/TOPP-THU/copp.git
    cd copp
    ```

    You need Cargo, CMake, and a C compiler toolchain: Visual Studio Build Tools with the C++ workload on Windows, GCC/Clang on Linux, or Xcode Command Line Tools on macOS. The C ABI is gated behind the Cargo `c` feature. First build the native library from the repository root:

    ```sh
    cargo build --release --lib --features c
    ```

    Then configure and build the CMake project:

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

    To install it as a package discoverable by downstream CMake projects:

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

    Replace `<install-prefix>` with your own installation path, for example `C:/copp-install` or `$PWD/copp-install`.

    Use it from a downstream project as follows:

    ```cmake
    find_package(copp CONFIG REQUIRED)

    add_executable(app main.c)
    target_link_libraries(app PRIVATE copp::copp)
    ```

    Pass the installation prefix to CMake when configuring the downstream project:

    ```sh
    cmake -S app -B app/build -DCMAKE_PREFIX_PATH=<install-prefix>
    cmake --build app/build --config Release
    ```

    If static linking is desired, add `-DCOPP_LINK_STATIC=ON` to the corresponding COPP CMake configure command above.

    To regenerate the C headers, install `cbindgen` and run the PowerShell header-generation script. Most users can directly use the generated headers under `bindings/c/include/copp/`.

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

## Algorithm Selection

Algorithm selection in `copp` mainly depends on three factors: whether the problem is second-order or third-order, whether the objective is time-optimal or a general convex objective, and whether the higher-performance PRO algorithms are needed.

| Problem class | Algorithm  | Availability | Notes                                                                                                                                                                                                                                                                                                                                 |
| ------------- | ---------- | ------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| TOPP2         | TOPP2-RA   | Open-source  | Ultra-fast reachability-analysis-based method. In common benchmarks, its relative error versus global optimization baselines is typically below $10^{-4}$, making it a good default entry point for second-order time-optimal problems.                                                                                               |
| COPP2         | COPP2-SOCP | Open-source  | Formulated as an SOCP and solved with `clarabel`. It is globally optimal under the convex formulation, but usually costs substantially more runtime than RA-style methods.                                                                                                                                                            |
| COPP2         | COPP2-RDDP | PRO          | Fast original method that preserves global-optimal solution quality while running substantially faster than COPP2-SOCP.                                                                                                                                                                                                               |
| TOPP3         | TOPP3-SOCP | Open-source  | `clarabel`-based conic formulation that usually returns high-quality KKT solutions; computational cost may be high on some datasets.                                                                                                                                                                                                  |
| TOPP3         | TOPP3-LP   | Open-source  | Linear-objective approximation of TOPP3-SOCP. It is usually faster, but can become sub-optimal under tight jerk constraints; it is mainly recommended when jerk bounds are loose.                                                                                                                                                     |
| TOPP3         | TOPP3-RA   | PRO          | Ultra-fast reachability-analysis-based third-order method. It can be sub-optimal under tight jerk constraints and is mainly recommended when jerk bounds are loose.                                                                                                                                                                   |
| COPP3         | COPP3-SOCP | Open-source  | `clarabel`-based conic formulation with strong practical optimality, at relatively high computational cost.                                                                                                                                                                                                                           |
| COPP3         | COPP3-RDDP | PRO          | Fast original method that returns KKT-quality solutions comparable to TOPP3-SOCP while running substantially faster than TOPP3-SOCP, TOPP3-LP, and COPP3-SOCP. COPP3-RDDP can also be used as a high-quality TOPP3 solver. On long-path problems, it may provide better practical optimality and numerical stability than COPP3-SOCP. |

More specifically, choose algorithms as follows:

| Scenario / primary priority                                                    | Recommended algorithm   | Availability | Why this is recommended                                                                | Typical caveat                                        | Alternative                   |
| ------------------------------------------------------------------------------ | ----------------------- | ------------ | -------------------------------------------------------------------------------------- | ----------------------------------------------------- | ----------------------------- |
| Second-order, time-optimal planning with very low runtime                      | TOPP2-RA                | Open-source  | Excellent speed-performance trade-off; near-global-optimal in typical benchmarks.      | Objective is fixed to minimum time.                   |                               |
| Second-order, convex objective with strong global solution quality             | COPP2-SOCP              | Open-source  | Convex conic formulation with global optimality under model assumptions.               | Higher runtime than RA/RDDP methods.                  | COPP2-RDDP for major speedup. |
| Second-order, convex objective with maximum computational efficiency           | COPP2-RDDP              | PRO          | Preserves global-optimal solution quality while substantially improving runtime.       | PRO license required.                                 | COPP2-SOCP.                   |
| Third-order problems requiring the strongest open-source optimality quality    | TOPP3-SOCP / COPP3-SOCP | Open-source  | Strong KKT-quality solutions and broad applicability.                                  | On specific datasets, computational cost may be high. | COPP3-RDDP for major speedup. |
| Third-order, time-optimal planning with a faster open-source approximation     | TOPP3-LP                | Open-source  | Use it when your own path dataset shows better runtime/performance.                    | May become sub-optimal under tight jerk bounds.       | TOPP3-SOCP or COPP3-RDDP.     |
| Third-order, time-optimal planning with loose jerk bounds and very low runtime | TOPP3-RA                | PRO          | Very low computational cost.                                                           | May become sub-optimal under tight jerk bounds.       | COPP3-RDDP or TOPP3-SOCP.     |
| Third-order, high-quality and high-stability planning for difficult long paths | COPP3-RDDP              | PRO          | Strong practical optimality, high speed, and often better robustness on long horizons. | PRO license required.                                 | COPP3-SOCP.                   |

## Step-by-Step Workflow

We use time-optimal parameterization of a two-dimensional path as a simple example. Complete runnable examples are available under the `examples` directory for each language, and advanced APIs are described in the [documentation and architecture](#docs-architecture) section.

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

### Step 1. Construct the Geometric Path

Strictly speaking, the geometric path $\boldsymbol{q}=\boldsymbol{q}(s)$ is supplied by the user. The `copp` library provides path-related modules as convenient helpers.

#### Option A. Analytic Expression with Automatic Differentiation

The simplest approach is to construct the path from an analytic expression, for example:

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
        phase = jnp.array([0.0, 0.5 * jnp.pi], dtype=jnp.float64)
        return jnp.sin(freq * s + phase)

    path = copp.Path.from_jax(q_fn, 0.0, 1.0)
    ```

    Python automatic differentiation supports `jax`, `autograd`, `casadi`, and `sympy`. Choose one and install the corresponding dependency as needed.


=== "Matlab"

    ```matlab
    path = copp.Path.from_parametric( ...
        @(s) [sin(2*pi*s); cos(2*pi*s)], ...
        s_range=[0, 1]);
    ```

    `from_parametric` uses the MATLAB-side `Jet3` automatic differentiation; the length of the vector returned by the formula is the path dimension `dim`.

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
    static enum CoppStatus evaluate_parametric_path(
        void *user_data,
        size_t dim,
        struct CoppJet3 s,
        struct CoppJet3 *q)
    {
        (void)user_data;
        if (dim != DIM || q == NULL) {
            return COPP_STATUS_INVALID_ARGUMENT;
        }

        // `s` arrives seeded with ds/ds = 1, and the inline Jet3 helpers in
        // `copp/path.h` propagate derivatives up to third order. Only q(s)
        // has to be written; dq, ddq, and dddq come out of the helpers.
        const struct CoppJet3 w = copp_mul_f64(s, 6.28318530717958647692);
        q[0] = copp_sin(w);
        q[1] = copp_cos(w);
        return COPP_STATUS_OK;
    }

    struct CoppPath *path = NULL;
    // On success, release `path` later with `copp_path_free(path)`.
    if (check(
            copp_path_from_parametric(DIM, 0.0, 1.0, evaluate_parametric_path, NULL, &path),
            "copp_path_from_parametric")) {
        return 1;
    }
    ```

    `copp_path_from_parametric` and `CoppJet3` require v0.2.2 or newer. Besides `copp_sin`, `copp_cos`, and `copp_mul_f64`, the header provides `copp_add`, `copp_sub`, `copp_mul`, `copp_div`, `copp_neg`, `copp_exp`, `copp_log`, `copp_sqrt`, `copp_powi`, `copp_constant`, and their scalar variants. On earlier versions, supply the derivatives manually through `copp_path_from_evaluator_3rd`, as shown in Option C below.

#### Option B. Build a Spline from Waypoints

If waypoints are available, a spline path can be constructed as follows:

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

#### Option C. User-Provided Derivatives

In the most general case, users can provide derivatives manually. For TOPP2/COPP2, the evaluator should provide $\boldsymbol{q}(s)$, $\boldsymbol{q}'(s)$, and $\boldsymbol{q}''(s)$ at the requested $s$ values:

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
    let path = Path::from_evaluator_3rd(NormalizedEvaluator, 0.0, 1.0)?;
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


    # If only TOPP2/COPP2 is required and TOPP3/COPP3 is not called, then the path can be constructed by `from_evaluator_2nd` without dependence on `evaluate_up_to_3rd`.
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

    The MATLAB evaluator is a batch callback: the input `s` is always `1 x N`, and the outputs `q,dq,ddq,dddq` are always `dim x N`.

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

    // If only TOPP2/COPP2 is required, use `Path::from_evaluator_2nd` instead
    // and omit the `dddq` output.
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

### Step 2. Discretize Path Data

Path-parameterization problems are solved on a discrete $s$ grid, for example:

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

Create the robot model:

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

Provide the $s$ grid and path data:

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

    If only TOPP2/COPP2 is solved, `set_q_from_path_2nd` can be used instead.

=== "C++"

    ```cpp
    // If only TOPP2/COPP2 is required, replace `set_q_from_path_3rd` with
    // `set_q_from_path_2nd`, which does not depend on third-order path derivatives.
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

For more flexible use cases, paths can be added or removed online, and the robot model can include inverse-kinematics information. These advanced APIs are described in the [documentation and architecture](#docs-architecture) section.

### Step 3. Construct Constraints

In most cases, we recommend using high-level APIs with clear physical meaning, for example:

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
    upper = np.ones(DIM, dtype=np.float64)
    lower = -upper

    robot.add_velocity_limits(upper, lower, start_idx_s=0, length=n)
    robot.add_acceleration_limits(upper, lower, start_idx_s=0, length=n)
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
    // The axial velocity/acceleration is -1 <= value <= 1 for each axis.
    double upper[DIM] = {1.0, 1.0};
    double lower[DIM] = {-1.0, -1.0};

    if (check(
            copp_add_axial_velocity_limits(
                robot,
                0,
                n,
                (struct CoppSliceF64){upper, DIM},
                (struct CoppSliceF64){lower, DIM}),
            "copp_add_axial_velocity_limits")) {
        return 1;
    }
    if (check(
            copp_add_axial_acceleration_limits(
                robot,
                0,
                n,
                (struct CoppSliceF64){upper, DIM},
                (struct CoppSliceF64){lower, DIM}),
            "copp_add_axial_acceleration_limits")) {
        return 1;
    }
    ```

For third-order trajectories, additional third-order constraints are needed, for example:

=== "Rust"

    ```rust
    // The axial jerk is -1 <= jerk <= 1 for each axis in this example.
    let jerk_max = vec![1.0; DIM];
    let jerk_min = vec![-1.0; DIM];
    robot.with_axial_jerk((jerk_max.as_slice(), n), (jerk_min.as_slice(), n), 0)?;
    ```

=== "Python"

    ```python
    robot.add_jerk_limits(upper, lower, start_idx_s=0, length=n)
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
    if (check(
            copp_add_axial_jerk_limits(
                robot,
                0,
                n,
                (struct CoppSliceF64){upper, DIM},
                (struct CoppSliceF64){lower, DIM}),
            "copp_add_axial_jerk_limits")) {
        return 1;
    }
    ```

Lower-level and more flexible constraint-construction APIs are described in the [documentation and architecture](#docs-architecture) section.

### Step 4. Call the Solver

Here we solve a TOPP2 problem with `topp2_ra`. First define the problem:

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

    MATLAB-side indices are 1-based; `[1,n]` here covers every station.

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

Then build solver options and call the solver:

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

This gives the profile $a=a(s)$.

### Step 5. Post-Process the Second-Order Trajectory (Second-Order Only)

Next we convert the result into the actual trajectory $\boldsymbol{q}=\boldsymbol{q}(t)$, in particular an interpolated trajectory suitable for tracking by a low-level servo controller. First compute $t=t(s)$:

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

Then invert the timing relation to obtain $s=s(t)$ and interpolate:

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

The interpolation routines also support non-uniform time samples; see the [documentation and architecture](#docs-architecture) section for details. Finally, evaluate the interpolated trajectory $\boldsymbol{q}=\boldsymbol{q}(t)$. A simple approach is:

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

This completes the full second-order trajectory pipeline. For a third-order trajectory, skip the second-order post-processing step and continue with the following workflow.

### Step 6. Construct and Solve the Third-Order Problem (Third-Order Only)

Construct the third-order problem as follows. The non-convex third-order constraints are linearized using the previously computed second-order profile `a_ra`:

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

    The third-order MATLAB `Problem` lazily linearizes around `a_ra` when `solve` is called.

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

Using `topp3_socp` as an example, call the solver as follows:

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

This already produces a feasible, near-optimal third-order trajectory $a_1(s),b_1(s)$ and can be used directly in the next step. If more computational budget is available, use $a_1(s)$ as the next linearization point and solve a new linearized third-order problem:

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

When computational resources allow, the above procedure can be repeated: linearize the third-order non-convex problem at $a_k(s)$ and solve for $a_{k+1}(s),b_{k+1}(s)$. Under non-degenerate conditions, this process can converge to a KKT solution. For practical online use, one to two linearization iterations are usually sufficient.

### Step 7. Post-Process the Third-Order Trajectory (Third-Order Only)

Next we convert the third-order profile into the actual trajectory $\boldsymbol{q}=\boldsymbol{q}(t)$, in particular an interpolated trajectory suitable for tracking by a low-level servo controller. First compute $t=t(s)$:

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

Then invert the timing relation to obtain $s=s(t)$ and interpolate:

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

The interpolation routines also support non-uniform time samples; see the [documentation and architecture](#docs-architecture) section for details. Finally, evaluate the interpolated trajectory $\boldsymbol{q}=\boldsymbol{q}(t)$. A simple approach is:

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

    All output matrices have shape `dim x numel(s_t)`.

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

This completes the full third-order trajectory pipeline.

### Resource Management

=== "Rust"

    The Rust API does not require manual resource release. Objects are released automatically when they leave scope.

=== "Python"

    The Python API does not require manual resource release. Objects are managed by the Python runtime.

=== "Matlab"

    MATLAB objects release their native handle on `delete`, so ordinary scripts usually do not need to free anything manually. Long scripts and tests can release explicitly, or use `onCleanup`:

    ```matlab
    cleanup_path = onCleanup(@() path.release());
    cleanup_robot = onCleanup(@() robot.release());
    ```

=== "C++"

    The C++ API does not require manual resource release. `Path`, `Robot`, `Constraints`, `Profile3rd`, `Matrix`, and similar objects follow RAII and release both the Rust-side handle and the C++-side storage when they leave scope. `Span`, `MatrixView`, and `MatrixRef` are borrowed views that do not own memory and do not need to be released.

=== "C"

    In the C ABI, `CoppSliceF64` and `CoppMatrixViewF64` only borrow user-owned memory and do not need to be released. All owned objects returned by COPP, including `CoppVecF64`, `CoppMatrixF64`, `CoppProfile3rd`, and the `CoppPath` and `CoppRobot` handles, must be released with their matching free functions. In real projects, we recommend initializing owned objects to `{0}` or `NULL` and using a centralized `cleanup` block so that resources already created are still released after an early error:

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

    For second-order trajectories, objects such as `profile` and `dddq_t` are not present. If multiple third-order iteration results are retained, such as `profile_qp1` and `profile_qp2`, each `CoppProfile3rd` whose ownership has not been moved must be released once with `copp_profile_3rd_free`. The C examples in the repository follow the same centralized cleanup style.

### Step-by-Step Summary

Overall, a minimal closed loop consists of: construct the path $\boldsymbol{q}(s)$; build a `Robot` and constraints on a discrete grid; select the corresponding problem builder and solver; obtain $a(s)$ or $(a(s),b(s))$; convert back to the time domain with `s_to_t_*` and `t_to_s_*`; and finally resample the original path at $s(t)$. The repository also provides runnable examples for TOPP2, COPP2, TOPP3, and COPP3.

## Benchmark

The open-source rows are reproducible from `tests/test_random_spline.rs` in the repository. The PRO rows (COPP2-RDDP, TOPP3-RA, COPP3-RDDP) come from the same benchmark run on the PRO release and are therefore not part of the open-source test file. Test conditions:

- `release, --include-ignored`
- CPU: Intel(R) Core(TM) Ultra 9 285K.
- Dataset: 100 random 7-DOF spline paths, each discretized into 1000 intervals.

All metrics are reported as `mean ± std`. Computation times are machine- and run-dependent, so absolute values will differ from your own runs; the relative comparison between methods is the meaningful part.

#### Time-Optimal

| Method                 | Availability |  Computation time (ms) |   Traversal time (s) |
| ---------------------- | ------------ | ---------------------: | -------------------: |
| TOPP2-RA               | Open-source  |    0.615425 ± 0.244409 | 40.903420 ± 1.378671 |
| COPP2-SOCP             | Open-source  |  149.969964 ± 9.364334 | 40.900039 ± 1.378613 |
| COPP2-RDDP             | PRO          |    5.436142 ± 0.465495 | 40.900135 ± 1.378613 |
| TOPP3-LP               | Open-source  | 327.074029 ± 28.893341 | 41.422945 ± 1.381874 |
| TOPP3-SOCP             | Open-source  | 289.654071 ± 12.862133 | 41.418608 ± 1.381202 |
| COPP3-SOCP             | Open-source  | 285.004302 ± 13.471264 | 41.418608 ± 1.381202 |
| TOPP3-RA (Iteration 1) | PRO          |   10.571045 ± 0.857653 | 41.499200 ± 1.385735 |
| TOPP3-RA (Iteration 2) | PRO          |   20.300932 ± 1.237908 | 41.399867 ± 1.386791 |

#### Convex-Objective

In this test, TOPP methods still use traversal time as the optimization objective.

| Method     | Availability |  Computation time (ms) |        Objective value |
| ---------- | ------------ | ---------------------: | ---------------------: |
| TOPP2-RA   | Open-source  |    0.534700 ± 0.069296 | 217.444861 ± 12.462360 |
| COPP2-SOCP | Open-source  | 270.059250 ± 52.073677 |   96.517354 ± 3.641154 |
| COPP2-RDDP | PRO          |   12.667700 ± 0.429214 |   96.525785 ± 3.639733 |
| TOPP3-LP   | Open-source  |  348.000000 ± 9.326314 | 211.611085 ± 12.367224 |
| TOPP3-SOCP | Open-source  | 301.227000 ± 12.938498 | 211.974066 ± 12.323865 |
| COPP3-SOCP | Open-source  | 301.227000 ± 12.938498 |   96.634962 ± 3.613264 |
| COPP3-RDDP | PRO          |   65.823050 ± 0.087893 |   98.708998 ± 3.354004 |

## Documentation and Architecture { #docs-architecture }

### Documentation

=== "Rust"

    We recommend using the [latest docs.rs documentation](https://docs.rs/copp/latest/copp/). Local documentation is also available:

    - [v0.2.2 (Latest)](rust/v0.2.2/copp/)
    - [v0.2.1](rust/v0.2.1/copp/)
    - [v0.2.0](rust/v0.2.0/copp/)
    - [v0.1.0](rust/v0.1.0/copp/)

    To inspect unreleased updates on the main branch, we recommend generating the documentation locally from the `copp` repository root:

    ```shell
    cargo doc --no-deps --open
    ```

    The generated documentation includes mathematical foundations, path and constraint construction methods, logging and output conventions, error definitions, and solver interfaces.

=== "Python"

    Python documentation:

    - [v0.2.2 (Latest)](python/v0.2.2/index.html)
    - [v0.2.1](python/v0.2.1/index.html)

    To generate the Python documentation locally, install Sphinx and build the HTML pages from the `copp` repository root:

    ```sh
    python -m pip install -U sphinx
    python -m sphinx -E -b html bindings/python/docs/source bindings/python/docs/build/html
    ```

    The generated entry page is `bindings/python/docs/build/html/index.html`.

=== "Matlab"

    MATLAB documentation:

    - [v0.2.2 (Latest)](matlab/v0.2.2/index.html)

    It can also be generated locally through `publish`. Run these commands in the **MATLAB Command Window**, not in a system shell:

    ```matlab
    cd bindings/matlab/docs
    build_docs()
    ```

    `build_docs` adds `bindings/matlab` to the MATLAB path itself, so no manual `addpath` is needed. It does, however, execute the code in the documentation pages (`EvalCode` defaults to `true`), and those pages call the COPP API, so the MEX gateway must already be built. Check it first:

    ```matlab
    copp.version()
    ```

    If that errors, build the MEX before generating the documentation, as described in the [installation](#installation) section.

    To generate the documentation without leaving a system shell, use MATLAB batch mode:

    ```powershell
    matlab -batch "cd('bindings/matlab/docs'); build_docs()"
    ```

    The generated entry page is `bindings/matlab/docs/html/index.html`.

=== "C++"

    C++ documentation:

    - [v0.2.2 (Latest)](cpp/v0.2.2/index.html)

    The C++ API documentation is generated with Doxygen from `bindings/cpp/include/copp/*.hpp`, `bindings/cpp/docs/*.md`, and the example code. To inspect the C++ interface on the development branch, generate it locally from the `copp` repository root:

    ```sh
    doxygen bindings/cpp/Doxyfile
    ```

    The generated entry page is `bindings/cpp/docs/html/index.html`. Rendering include graphs and call graphs additionally requires Graphviz; if formulas do not display correctly, check the Doxygen MathJax configuration first.

=== "C"

    C documentation:

    - [v0.2.2 (Latest)](c/v0.2.2/index.html)
    - [v0.2.1](c/v0.2.1/index.html)
    - [v0.2.0](c/v0.2.0/index.html)

    To generate the C documentation locally, install Doxygen, Graphviz, and PowerShell 7 (`pwsh`, used to regenerate headers), then run the corresponding command from the `copp` repository root:

    === "Linux"

        ```sh
        # Debian/Ubuntu example. Install PowerShell 7 separately so that `pwsh` is available.
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

    The generated entry page is `bindings/c/docs/html/index.html`.

### Project Architecture

=== "Rust"

    | Module        | Responsibility                                                           |
    | ------------- | ------------------------------------------------------------------------ |
    | `path`        | Path construction, parameter range, second/third derivatives.            |
    | `robot`       | Robot dimension, inverse dynamics, and physical constraint entry points. |
    | `constraints` | Lower-level constraint interfaces.                                       |
    | `solver`      | Solvers for each problem class.                                          |
    | `diag`        | Error types, logging verbosity, and diagnostic information.              |

=== "Python"

    | Module                  | Responsibility                                                                                                                                    |
    | ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
    | `copp_py`               | Top-level package exporting common types, functions, and submodule entry points.                                                                  |
    | `copp_py.path`          | `Path`, `SplineConfig`, path evaluation, waypoint splines, automatic differentiation, and user-evaluator paths.                                   |
    | `copp_py.robot`         | `Robot`, path sampling, velocity/acceleration/jerk/torque constraints, advanced physical constraint entry points, and inverse-dynamics callbacks. |
    | `copp_py.constraints`   | Raw `Constraints` buffers, first/second/third-order low-level constraints, `amax_substitute`, and sliding-window operations.                      |
    | `copp_py.solver`        | Solver namespace containing `topp2_ra`, `reach_set2`, `copp2_socp`, `topp3_lp`, `topp3_socp`, and `copp3_socp`.                                   |
    | `copp_py.interpolation` | `Profile3rd`, second/third-order `s_to_t_*`, `t_to_s_*`, and `a_to_b_topp2` trajectory post-processing.                                           |
    | `copp_py.objective`     | COPP2/COPP3 objective descriptions, including time, linear, thermal-energy, and torque-total-variation objectives.                                |
    | `copp_py.clarabel`      | Clarabel SOCP options, settings, solver status, and expert diagnostic results.                                                                    |
    | `copp_py.core`          | Version, error types, enums, matrix layouts, and common type conventions.                                                                         |

=== "Matlab"

    | Module               | Responsibility                                                        |
    | -------------------- | --------------------------------------------------------------------- |
    | `copp`               | Top-level package providing `version`, `Path`, `Robot`, `Profile3rd`. |
    | `copp.Path`          | Waypoint, evaluator, parametric, symbolic, and CasADi paths.          |
    | `copp.Robot`         | Station grid, path sampling, velocity/acceleration/jerk/torque limits. |
    | `copp.objective`     | COPP objectives: time, linear, thermal energy, torque total variation. |
    | `copp.solver`        | `topp2_ra`, `reach_set2`, `copp2_socp`, `topp3_*`, `copp3_*`.         |
    | `copp.interpolation` | Second/third-order `s_to_t_*`, `t_to_s_*`, and `a_to_b_topp2`.        |
    | `copp.clarabel`      | Clarabel solver options, settings, and direct solve method.           |
    | `copp.diag`          | MATLAB exception facade, verbosity, and MEX last-error snapshots.     |

=== "C++"

    | Header / Namespace        | Responsibility                                                   |
    | ------------------------- | ---------------------------------------------------------------- |
    | `copp/copp.hpp`           | Umbrella header containing the C++ public facade.                |
    | `copp/core.hpp`           | Core types such as `Matrix`, `Span`, `Error`, and `Expected`.    |
    | `copp/path.hpp`           | Waypoint / parametric / evaluator path construction and derivatives. |
    | `copp/robot.hpp`          | `Robot`, `Constraints`, inverse-dynamics callbacks, and physical constraints. |
    | `copp/interpolation.hpp`  | TOPP2 / TOPP3 profiles and time interpolation.                   |
    | `copp/solver/*.hpp`       | TOPP/COPP solver namespaces with problem/options/result types.   |
    | `copp/eigen.hpp`          | Optional Eigen adapter; the core headers do not depend on Eigen. |
    | CMake target `copp::copp` | Link entry point for downstream C++ projects; the C ABI is also available as `copp::c_abi`. |

=== "C"

    | Header                 | Responsibility                                                                      |
    | ---------------------- | ----------------------------------------------------------------------------------- |
    | `copp/copp.h`          | Umbrella header containing the complete C ABI.                                      |
    | `copp/core.h`          | Status codes, last error, matrix/vector views, and Clarabel options.                |
    | `copp/path.h`          | Path handles, spline paths, callback paths, and path evaluation.                    |
    | `copp/robot.h`         | Robot handles, path sampling, physical constraints, and inverse-dynamics callbacks. |
    | `copp/formulation.h`   | TOPP/COPP problem descriptors, objectives, and profile types.                       |
    | `copp/interpolation.h` | Second/third-order trajectory post-processing and interpolation.                    |
    | `copp/topp2.h`         | TOPP2-RA and ReachSet2 interfaces.                                                  |
    | `copp/copp2.h`         | COPP2-SOCP interface.                                                               |
    | `copp/topp3.h`         | TOPP3-LP and TOPP3-SOCP interfaces.                                                 |
    | `copp/copp3.h`         | COPP3-SOCP interface.                                                               |

## Citing

If your work uses the open-source TOPP3/COPP3 functionality, please cite [the following paper](https://doi.org/10.1016/j.ijmachtools.2025.104355), as even the basic interval-wise profile template uses contributions from this work:

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

If your work uses the TOPP3-RA, COPP2-RDDP, or COPP3-RDDP methods from the PRO release, please cite [the following paper](https://arxiv.org/abs/2605.19089). TOPP3-RA is also improved based on this work:

```tex
@article{wang2026reachability,
  title={Reachability-augmented dual dynamic programming for optimal path parameterization},
  author={Yunan Wang and Jizhou Yan and Chuxiong Hu and Zeyang Li},
  journal={arXiv preprint arXiv:2605.19089},
  year={2026}
}
```

For other use cases, please cite the [`COPP` library](https://github.com/TOPP-THU/copp) or the corresponding paper:

```tex
@misc{thu2026copp,
  title = {COPP: Convex-Objective Path Parameterization},
  author = {Wang, Yunan and He, Suqin and Lin, Shize and Hu, Chuxiong},
  year = {2026},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/TOPP-THU/copp}}
}
```

## Contact

For COPP PRO licensing, commercial collaboration, technical consulting, or general inquiries, please contact [hello@copp.pro](mailto:hello@copp.pro).
