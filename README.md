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

- Euler angle conversions and application
- rotation-matrix conversions, application, inversion, and composition
- homogeneous-matrix conversions, application, inversion, and composition
- quaternion, reduced-quaternion, and rotation-vector conversions
- quaternion, reduced-quaternion, and rotation-vector application
- quaternion, reduced-quaternion, and rotation-vector inversion
- quaternion, reduced-quaternion, and rotation-vector composition
- `srq`, `sre`, `lrQ`, `lrq`, `lrrv`, and `lre` conversion, application, inversion, and composition

The current public header is:

- `include/robotransforms/euclidean.hpp`
- generated `robotransforms/version.hpp` during CMake configure

The library is currently header-only and exposed through a CMake `INTERFACE` target named `robotransforms_euclidean`.

## Naming And Conventions

The transform names encode both the translation convention and the rotation parameterization.

### `lr*` means `location then rotate`

`lr*` transforms store the location of the terminal coordinate system's origin, expressed in the initial coordinate system, followed by a rotation from the initial coordinate system to the terminal coordinate system.

In other words, an `lr*` transform answers:

- where is the terminal origin, expressed in the initial frame?
- how is the terminal frame rotated relative to the initial frame?

The suffix indicates the rotation representation:

- `lrQ`: full quaternion `[re, i, j, k]`
- `lrq`: reduced quaternion `[i, j, k]` with positive real part implied
- `lrrv`: rotation vector `[e1, e2, e3]`
- `lre`: Euler angles `[yaw, pitch, roll]`

Operationally, applying an `lr*` transform means subtracting the terminal location and then re-expressing the vector in the terminal coordinate system.

### `sr*` means `shift then rotate`

`sr*` transforms store the shift that must be applied to the initial coordinates to move them to the terminal origin, with that shift expressed in the initial coordinate system, followed by the rotation from the initial coordinate system to the terminal coordinate system.

So for `sr*`:

- the translation component is the shift to apply directly in the initial frame
- the rotation component is still the rotation from the initial frame to the terminal frame

This is why `sr*` and `lr*` are related, but they are not just different names for the same stored translation.

### Rotations mean "the frame rotates, the vector stays put"

Throughout this library, applying a rotation means the vector is treated as stationary while the coordinate system rotates, and the result is the vector re-expressed in the new coordinate system.

So "apply a pitch of 10 degrees" means:

- the coordinate system pitches up by 10 degrees
- then the same geometric vector is written in that new coordinate system

This is equivalent to what many other libraries describe as "unrotating" the vector.

This convention is used consistently across quaternions, reduced quaternions, rotation vectors, Euler angles, rotation matrices, and homogeneous transforms.

### Quaternion convention

Quaternion rotation conventions are easy to mix up because `SU(2)` is a double cover of `SO(3)` and because authors differ on the sign convention for the imaginary terms.

This library uses the convention where a rotated coordinate expression is computed as:

`v' = q* [0, v] q`

where:

- `q*` is the quaternion conjugate
- `[0, v]` is the pure imaginary quaternion built from the vector

This choice is intentional because it makes quaternion composition read left-to-right in the library APIs:

- the first rotation is on the left
- the second rotation is on the right

That same left-to-right composition rule is used throughout the transform composition helpers.

### Euler angle convention

Euler angles are always ordered as:

- `yaw`
- `pitch`
- `roll`

In this library the sign and axis conventions are:

- `yaw` is a negative rotation around `z`
- `pitch` is a positive rotation around `x`
- `roll` is a positive rotation around `y`

That convention is the one implemented by the Euclidean conversion and application helpers in this repository, and it should be preserved by downstream bindings.

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

Generate a single-header distribution file:

```sh
cmake --build build --target dist_header
```

That target writes:

- `build/dist/robotransforms-core.hpp`
- `build/dist/robotransforms-core-<version>.hpp`

The generated dist header bakes in the configured version constants and bundles the current public core into a single file that is convenient to attach to a GitHub release.

## Release Workflow

This repository uses a tag-driven GitHub Actions release flow.

The workflow is configured to trigger on pushes of tags matching:

- `v*`

That means tags such as:

- `v0.2.0`
- `v0.2.1`
- `v1.0.0`

will trigger the release workflow automatically.

The workflow currently does the following:

1. checks out the tagged commit
2. configures the CMake build
3. builds the test target
4. runs the test suite
5. builds the single-header dist output
6. creates or updates the GitHub Release for that tag
7. uploads the generated dist headers as release assets

The generated release assets include:

- `robotransforms-core.hpp`
- `robotransforms-core-<project-version>.hpp`

### Creating a release

The intended release flow is:

```sh
git add .
git commit -m "Release 0.2.0"
git push origin main
git tag -a v0.2.0 -m "Release v0.2.0"
git push origin v0.2.0
```

Pushing the tag is what triggers GitHub Actions.

### Tag format and versioning

The workflow trigger uses Git tags beginning with `v`, while the project version inside the code is the numeric semantic version from `CMakeLists.txt`.

In practice, the intended mapping is:

- Git tag: `v0.2.0`
- project version: `0.2.0`

Those should be kept in sync for releases.

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