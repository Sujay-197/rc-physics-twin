# SlipX on Windows: first build and conformance run

**Date:** 2026-10-19  **SlipX commit:** `ebdbf3fe1c7704b47df9660e76603d1e18be0bd0` (upstream `main`, the pin in `THIRD_PARTY.md`)

Phase 1, first item: build `slipx_core` standalone and run the conformance check. Upstream source was not modified.

## Toolchain

The machine this first ran on had Visual Studio Build Tools 2026 (MSVC 14.51) but no Windows SDK, so MSVC cannot compile or link yet. This run used a self-contained toolchain in a scratch Python venv instead, with no admin rights needed:

| Tool | Version | Source |
|---|---|---|
| Compiler | Zig 0.16.0 (`zig c++`, Clang 21.1.0), target `x86_64-windows-gnu`, CPU `x86_64_v2` (pinned so the binaries run on machines other than the builder), C runtime UCRT | PyPI `ziglang` |
| CMake | 4.4.4 | PyPI `cmake` |
| Ninja | 1.13.2 | PyPI `ninja` |
| GoogleTest | v1.14.0 | fetched by upstream's CMake (`cmake/SlipxTesting.cmake`) |
| Python | 3.14.6 | system |
| Host | Windows 11 Pro 10.0.26300, AMD64 | |

CMake used a toolchain file pointing `CMAKE_C_COMPILER`, `CMAKE_CXX_COMPILER`, `CMAKE_AR` and `CMAKE_RANLIB` at one-line `.cmd` wrappers around `zig cc`, `zig c++`, `zig ar` and `zig ranlib`. Build type was upstream's default, `RelWithDebInfo`; flags were `-O2 -g -DNDEBUG -ffp-contract=off`.

**This is not the toolchain the Unity plugin will ship with.** MSVC is the natural one for a Unity native plugin, and the hashes below describe this Clang/mingw build only. Repeat the run with MSVC once the Windows SDK is installed.

## Results

### `slipx_core` alone (`-DSLIPX_CORE_ONLY=ON`)

Built cleanly into `libslipx_core.a`. **216 of 216 core tests passed** (ctest, 3.8 s).

### All C++ layers (core, sense, scene, sim, race; no Python bindings)

Everything built except one upstream test executable, whose source does not compile under Clang 21:

```
src/orchestration/slipx_race/tests/test_wrong_way.cpp:62:52: error: definition or redeclaration
of 'quiet_NaN' not allowed inside a function
   62 |       WrongWayMonitor(std::numeric_limits<double>::quiet_NaN()),
```

Inside `EXPECT_THROW`, the statement parses as a declaration (a most-vexing-parse). It is a test in the race layer, outside the core and the simulator. **501 of 502 tests passed**; the remaining one is `test_wrong_way_NOT_BUILT`.

Other warnings: `-Wsign-conversion` at `src/core/slipx_core/include/slipx/contact.hpp:185-186` (`int` index into a `size_type`), repeated in each translation unit that includes it, and one in GoogleTest's own headers.

### Conformance (`tools/check_conformance.py`, the `make conformance` target)

`workflow pin matches the published L1/rk4 hash.`

| Case | Hash (this build) |
|---|---|
| L0 / rk4 | `f656be2edf4d1201` |
| L0 / semi_implicit_euler | `f542326a7d1132d0` |
| L1 / rk4 | `d8e60c176ea8a981` |
| L1 / semi_implicit_euler | `600616380d658a2c` |
| L2 / rk4 | `69add79003d93539` |
| L2 / semi_implicit_euler | `951e69f4c203cd69` |

Build key from the manifest: `AMD64  Clang  21  ucrt  RelWithDebInfo`.

- **No reference row exists for this build.** Upstream publishes rows only for x86_64 Linux (GCC 11, GCC 13, Clang 18 on glibc 2.39), and promises bit-identity within one build only, not across platforms or C libraries. So "0 of 6 checked against a reference" is the expected outcome, not a failure; the script exits 0.
- **Run-to-run determinism held.** Three consecutive runs gave byte-identical output for all six cases.
- These hashes certify that the arithmetic repeats. They say nothing about any real car: the conformance scenario uses SlipX's `VehicleParams` defaults, which upstream labels provisional.

Two Windows gaps in upstream tooling showed up:

1. `tools/check_conformance.py` looks for `slipx_conformance` with no `.exe` suffix and reports "binary not found" on Windows. For this run, the built `.exe` was copied to an extension-less name inside the scratch build directory; the script and the source tree were left unchanged.
2. The Python console script `slipx-conformance` runs the same scenario through the pybind11 extension. It was **not** run: building that extension with zig/mingw against the MSVC-built CPython is not a supported pairing. It belongs with the MSVC run.

## Still to do

- Install the Windows SDK into the existing Build Tools (needs an elevated prompt), then repeat this run with MSVC, including the Python bindings and `slipx-conformance`.
- Both upstream gaps above (the `test_wrong_way.cpp` parse and the missing `.exe` lookup) are small fixes that suit upstream PRs from the fork.
