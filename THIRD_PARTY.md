# Third-party code

Status of each upstream. **SlipX is vendored in `core/`**; nothing else is. Before vendoring anything else, confirm the upstream `LICENSE` file and copy it in alongside the code.

| Upstream | License (as read) | Where read | Plan | Verified against LICENSE file |
|---|---|---|---|---|
| SlipX (ibrahimsel/slipx) | Apache-2.0 | Upstream `LICENSE` and `NOTICE` at pinned commit `ebdbf3f` (see below) | **Vendored** in `core/` at `ebdbf3f` with upstream `LICENSE` and `NOTICE` (see below); upstream PRs go through the fork [Sujay-197/slipx](https://github.com/Sujay-197/slipx) | **Yes**, 2026-10-19 |
| Gym-Khana (TeoIlie/Gym-Khana) | MIT | PyPI metadata | Dependency or fork under `sim_py/`; keep copyright notice | No |
| ApexSim (pzimbrod/apexsim) | Apache-2.0 | GitHub README | Reference only | No |
| TLabVehiclePhysics (TLabAltoh) | MIT | Third-party directory listing | Reference only | No |
| OpenStreetMap data (campus extract) | ODbL 1.0 | [openstreetmap.org/copyright](https://www.openstreetmap.org/copyright) | **Included**: `campus/osm/amrita.osm.json` and what `tools/campus_scenery.py` derives from it (`campus/scenery.json`, the campus scenery meshes); attribution in the files and below | Yes, 2026-10-20 |

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
| `core/src/core/slipx_core/include/slipx/params.hpp`, `src/params.cpp` | `wheel_inertia` (default 0) and its validation | Ours (ADR 0006); not sent |
| `core/src/core/slipx_core/src/l2_double_track.cpp` | Wheel rotational dynamics when `wheel_inertia > 0`; inertia 0 unchanged bit for bit | Ours (ADR 0006); not sent |
| `core/src/orchestration/slipx_sim/src/simulation.cpp` | `wheel_inertia` in the manifest's params digest, only when set | Ours (ADR 0006); not sent |
| `core/src/core/slipx_core/tests/test_wheels.cpp` (new) | 20 tests for the above | Ours (ADR 0006); not sent |
| `params.hpp`, `params.cpp`, `l2_double_track.cpp`, `slipx_sim/src/simulation.cpp` | `ackermann` fraction (default 0 = parallel steer, unchanged bit for bit); per-wheel front angles at L2; in the manifest digest when set | Ours (ADR 0007); not sent |
| `core/src/core/slipx_core/tests/test_ackermann.cpp` (new) | 10 tests for the above | Ours (ADR 0007); not sent |
| `core/src/core/slipx_core/include/slipx/state.hpp` | `DriveInput::reverse` (default false): the ESC's reverse, an input | Ours (ADR 0010); not sent |
| `core/src/core/slipx_core/src/l2_double_track.cpp` | With wheel dynamics, `reverse` drives the motor backwards on its mirrored curve instead of braking; the slip angle of a wheel rolling backwards is measured from its backward direction (a wheel rolling forwards unchanged bit for bit; conformance hashes unchanged) | Ours (ADR 0010); not sent |
| `core/src/core/slipx_core/tests/test_wheels.cpp` | 6 tests for the above | Ours (ADR 0010); not sent |
| (none: `slipx/contact.hpp` used unchanged) | `native/src/rc_twin.cpp` calls SlipX's `segment_contact` and `resolve_contact` (ADR-0055) for walls; the header is not modified | Ours (ADR 0009) |

Upstream lineage note: Gym-Khana says it is based on the UPenn `f1tenth_gym`. Check that lineage's license before copying code that may originate there.

## Apache-2.0 obligations (SlipX fork)

- Keep the upstream `LICENSE` and any `NOTICE` text.
- Carry prominent notices in every file you modify.
- Do not use upstream names or marks to endorse this project.

## MIT obligations (Gym-Khana)

- Keep the copyright and permission notice in copies or substantial portions.

## OpenStreetMap (campus scenery)

`campus/osm/amrita.osm.json` is an OpenStreetMap extract of the Amrita Vishwa Vidyapeetham campus, Coimbatore (Overpass API, 2026-10-20). `campus/scenery.json` and the scenery meshes in `unity/RcTwin/Assets/RcTwin/Generated/Campus/` are derived from it. Map data (c) OpenStreetMap contributors, available under the Open Database License (ODbL) 1.0, https://www.openstreetmap.org/copyright. A derived database shared publicly must stay under the ODbL.

Map screenshots used as references for tree cover and bare ground are third-party imagery. They and everything made from them (`campus/reference/`, `Generated/Campus/Local/`) stay local and out of git.


## Campus look (Poly Haven, Inter)

The campus scene's look (`unity/RcTwin/Assets/RcTwin/Editor/CampusLook.cs`, `CampusDressing.cs`) uses the assets below. None is in git: `tools/fetch_assets.py` downloads them into `assets_src/` (MD5-checked against Poly Haven's API) and `tools/prepare_assets.py` (Blender) makes the reduced 1/10-scale copies under `Generated/Look/`, both gitignored. A built player carries the prepared copies.

| Asset | Used for | Authors | License |
|---|---|---|---|
| [Kloppenheim 06 (Pure Sky)](https://polyhaven.com/a/kloppenheim_06_puresky) | Sky, sun direction | Greg Zaal, Jarod Guest | CC0 1.0 |
| [Asphalt 02](https://polyhaven.com/a/asphalt_02) | Road | Rob Tuytel | CC0 1.0 |
| [Concrete Pavement](https://polyhaven.com/a/concrete_pavement) | Sidewalks, roofs | Charlotte Baglioni | CC0 1.0 |
| [Leafy Grass](https://polyhaven.com/a/leafy_grass) | Grass, ground relief | Charlotte Baglioni | CC0 1.0 |
| [Jacaranda Tree](https://polyhaven.com/a/jacaranda_tree) | Large trees | Rob Tuytel, Rico Cilliers | CC0 1.0 |
| [Tree Small 02](https://polyhaven.com/a/tree_small_02) | Trees; its leaves make the bushes and hedges | Rico Cilliers | CC0 1.0 |
| [Shrub 02](https://polyhaven.com/a/shrub_02) | Undergrowth | Rico Cilliers | CC0 1.0 |
| [Shrub 03](https://polyhaven.com/a/shrub_03) | Small plants | Rico Cilliers | CC0 1.0 |
| [Street Lamp 01](https://polyhaven.com/a/street_lamp_01) | Street lamps | Josh Dean | CC0 1.0 |
| [Boulder 01](https://polyhaven.com/a/boulder_01) | Rocks | Rico Cilliers | CC0 1.0 |
| [Modular Street Seating](https://polyhaven.com/a/modular_street_seating) | Benches | Stuart Attenborrow | CC0 1.0 |
| [Inter 4.1](https://github.com/rsms/inter/releases/tag/v4.1) | The shuttle HUD's typeface | The Inter Project Authors | SIL OFL 1.1 |

CC0 asks for no attribution; the authors are credited here anyway. Inter is under the SIL Open Font License 1.1: its full text is in [`third_party/inter/OFL.txt`](third_party/inter/OFL.txt) and is copied beside the fonts into the player (`Generated/Look/Resources/RcTwinLook/LICENSE.txt`). The fonts are used unmodified and are not sold on their own. The facades, mountains and bushes' arrangement are generated by this project's code.
