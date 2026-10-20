# Third-party code

Status of each upstream. **SlipX is vendored in `core/`**; nothing else is. Before vendoring anything else, confirm the upstream `LICENSE` file and copy it in alongside the code.

| Upstream | License (as read) | Where read | Plan | Verified against LICENSE file |
|---|---|---|---|---|
| SlipX (ibrahimsel/slipx) | Apache-2.0 | Upstream `LICENSE` and `NOTICE` at pinned commit `ebdbf3f` (see below) | **Vendored** in `core/` at `ebdbf3f` with upstream `LICENSE` and `NOTICE` (see below); upstream PRs go through the fork [Sujay-197/slipx](https://github.com/Sujay-197/slipx) | **Yes**, 2026-10-19 |
| Gym-Khana (TeoIlie/Gym-Khana) | MIT | PyPI metadata | Dependency or fork under `sim_py/`; keep copyright notice | No |
| ApexSim (pzimbrod/apexsim) | Apache-2.0 | GitHub README | Reference only | No |
| TLabVehiclePhysics (TLabAltoh) | MIT | Third-party directory listing | Reference only | No |

## SlipX: pinned commit and what its license files say

Checked 2026-10-19 against the upstream repository, not package metadata.

- **Pinned commit:** `ebdbf3fe1c7704b47df9660e76603d1e18be0bd0` (upstream `main`, committed 2026-09-13, "docs(readme): the banner is the demo race, recorded from RViz"). The fork `Sujay-197/slipx` `main` was at the same commit when checked. It is 53 commits after tag `v0.2.0` (`a6e47e5`); no file under `src/core/` changed in that range, and the range includes Windows build fixes (`5b95d01`, `503c030`).
- **`LICENSE`:** the full Apache License 2.0 text. Compared line by line with <https://www.apache.org/licenses/LICENSE-2.0.txt>, the only difference is the appendix boilerplate line, filled in as `Copyright [2026] [Ibrahim Sel]` (the brackets are left in upstream). The terms are unmodified.
- **`NOTICE`:** present. Copyright line: `Copyright 2026 The SlipX Authors`. It states that `slipx_core` links only the C++ standard library, and lists the permissive dependencies of the layers above it: PyYAML (MIT) and jsonschema (MIT) for `slipx_schema`, pybind11 (BSD-3-Clause) for the Python bindings, and the optional sink extras mcap (Apache-2.0) and rerun-sdk (Apache-2.0). It says no copyleft dependency appears in the repository.
- **Source headers:** files under `src/core/` carry `Copyright 2026 The SlipX Authors` and `SPDX-License-Identifier: Apache-2.0`.
- **Copy into this repo when code is brought in:** upstream `LICENSE` and `NOTICE` verbatim. Section 4(d) of Apache-2.0 requires the `NOTICE` text to travel with redistributions.

## SlipX: what we changed in `core/`

`core/` is upstream `ebdbf3f` plus the changes below. Apache-2.0 section 4(b) requires modified files to carry a prominent notice; each file listed here carries a one-line "Modified" (or "Added") "... by the rc-physics-twin project" comment under its licence header.

| File | Change | Upstream status |
|---|---|---|
| `core/src/orchestration/slipx_race/tests/test_wrong_way.cpp` | Brace-initialise the NaN threshold test so Clang 21 compiles it | Sent as ibrahimsel/slipx#1 |
| `core/tools/check_conformance.py` | Look for `slipx_conformance.exe` on Windows | Sent as ibrahimsel/slipx#2 |
| `core/src/core/slipx_core/include/slipx/ground.hpp` (new) | Per-wheel ground samples (height, normal, `mu_scale`) | Ours (ADR 0005); not sent |
| `core/src/core/slipx_core/include/slipx/vehicle_model.hpp`, `src/vehicle_model.cpp` | `VehicleModel::step_on`; tiers without a ground model accept level ground only | Ours (ADR 0005); not sent |
| `core/src/core/slipx_core/include/slipx/load_transfer.hpp` | Optional `g_normal` argument (default is the old arithmetic) | Ours (ADR 0005); not sent |
| `core/src/core/slipx_core/src/l2_double_track.cpp` | Grade, banking and per-wheel grip at L2; level ground unchanged bit for bit | Ours (ADR 0005); not sent |
| `core/src/core/slipx_core/tests/test_ground.cpp` (new), `tests/CMakeLists.txt` | 18 tests for the above | Ours (ADR 0005); not sent |

Upstream lineage note: Gym-Khana says it is based on the UPenn `f1tenth_gym`. Check that lineage's license before copying code that may originate there.

## Apache-2.0 obligations (SlipX fork)

- Keep the upstream `LICENSE` and any `NOTICE` text.
- Carry prominent notices in every file you modify.
- Do not use upstream names or marks to endorse this project.

## MIT obligations (Gym-Khana)

- Keep the copyright and permission notice in copies or substantial portions.
