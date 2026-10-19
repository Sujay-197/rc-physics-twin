# Third-party code

Status of each upstream. **Nothing is vendored yet.** Before vendoring, confirm the upstream `LICENSE` file and copy it into `third_party/<name>/`.

| Upstream | License (as read) | Where read | Plan | Verified against LICENSE file |
|---|---|---|---|---|
| SlipX (ibrahimsel/slipx) | Apache-2.0 | PyPI metadata and README | Fork into `core/`; keep `LICENSE`/`NOTICE`; mark modified files | No |
| Gym-Khana (TeoIlie/Gym-Khana) | MIT | PyPI metadata | Dependency or fork under `sim_py/`; keep copyright notice | No |
| ApexSim (pzimbrod/apexsim) | Apache-2.0 | GitHub README | Reference only | No |
| TLabVehiclePhysics (TLabAltoh) | MIT | Third-party directory listing | Reference only | No |

Upstream lineage note: Gym-Khana says it is based on the UPenn `f1tenth_gym`. Check that lineage's license before copying code that may originate there.

## Apache-2.0 obligations (SlipX fork)

- Keep the upstream `LICENSE` and any `NOTICE` text.
- Carry prominent notices in every file you modify.
- Do not use upstream names or marks to endorse this project.

## MIT obligations (Gym-Khana)

- Keep the copyright and permission notice in copies or substantial portions.
