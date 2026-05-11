# robotransforms-core

`robotransforms-core` is the native C++ core for the `robotransforms` family of libraries.

The goal of this repository is to hold the shared geometry and motion primitives that can be reused by:

- Python bindings
- Node bindings
- future native solvers such as a Ceres-backed optimizer

## Purpose

This repo is the new source of truth for native transform math.

It exists to solve two problems:

1. Keep the core geometry logic in one place instead of duplicating it across language-specific packages.
2. Make the math usable with templated scalar types so the same implementation can eventually support plain `double` values and autodiff types.

The current Euclidean implementation was ported by referencing both of the older implementations:

- the older C++ implementation in `robotransforms-python`
- the pure JavaScript implementation in `robotransforms-node`

Where those implementations disagreed, this repo intentionally follows the JS location-based transform semantics for `lrQ`, `lrq`, and `lrrv`, because those paths are the better current behavioral reference and the old C++ code has known bugs there.

## Current Scope

The current header provides:

- quaternion, reduced-quaternion, and rotation-vector conversions
- quaternion, reduced-quaternion, and rotation-vector application
- quaternion, reduced-quaternion, and rotation-vector inversion
- quaternion, reduced-quaternion, and rotation-vector composition
- `lrQ`, `lrq`, and `lrrv` conversion, application, inversion, and composition

The current public header is:

- `include/robotransforms/euclidean.hpp`
- generated `robotransforms/version.hpp` during CMake configure

The library is currently header-only and exposed through a CMake `INTERFACE` target named `robotransforms_euclidean`.

## Usage

Example:

```cpp
#include <iostream>

#include "robotransforms/euclidean.hpp"
#include "robotransforms/version.hpp"

namespace rt = robotransforms::euclidean;

int main() {
    rt::LrQ<double> pose{1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0};
    rt::Vec3<double> point{4.0, 6.0, 8.0};

    auto local_point = rt::apply_lrQ(pose, point);
    auto inverse_pose = rt::invert_lrQ(pose);
    auto identity_pose = rt::compose_lrQ(pose, inverse_pose);

    std::cout << robotransforms::version_string << "\n";

    std::cout << local_point[0] << ", "
              << local_point[1] << ", "
              << local_point[2] << "\n";

    std::cout << identity_pose[0] << ", "
              << identity_pose[1] << ", "
              << identity_pose[2] << ", "
              << identity_pose[3] << "\n";
}
```

If you are consuming this repo from another CMake project:

```cmake
add_subdirectory(robotransforms-core)
target_link_libraries(your_target PRIVATE robotransforms_euclidean)
```

## Build

This repo uses CMake.

Configure:

```sh
cmake -S . -B build
```

Build:

```sh
cmake --build build
```

## Test

Run the current test suite with:

```sh
ctest --test-dir build --output-on-failure
```

Or configure, build, and test in one shot:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

The current tests focus on the Euclidean port and specifically validate the behavior that differed across the older C++ and JS implementations.

## Development

Current development workflow:

1. Add or update behavior in `include/robotransforms/euclidean.hpp`.
2. Add focused regression coverage in `tests/euclidean_test.cpp`.
3. Rebuild and rerun `ctest`.

Development guidelines for this repo:

- prefer templated scalar-friendly implementations over `double`-only code
- keep language binding code out of this repo
- keep solver-specific code out of this repo
- add tests for any behavior that is being ported from older repos, especially where implementations previously disagreed

## Layout

```text
robotransforms-core/
  CMakeLists.txt
  include/
    robotransforms/
      euclidean.hpp
  tests/
    euclidean_test.cpp
```

## Next Planned Work

Likely next steps for this repo are:

1. add install and export rules for downstream consumers
2. expand Euclidean coverage if more parity is needed
3. port dead-reckoning into the same core once the Euclidean surface is stable
4. wire Python and future solver repos to consume this core instead of owning duplicate native geometry code