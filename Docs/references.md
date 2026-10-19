# References

Links gathered 2026-10-19. Licenses noted as read from metadata; verify against `LICENSE` files before copying code.

## Dynamics and simulation

- SlipX: 1/10-scale vehicle dynamics, C++17 core, tiers L0 to L3, Apache-2.0. https://github.com/ibrahimsel/slipx and https://pypi.org/project/slipx
- Gym-Khana (gymkhana): Gymnasium env for racing and drift on Ackermann vehicles, PAC2002 tire parameters for a 1/10 car, SB3 and ONNX export, MIT. https://github.com/TeoIlie/Gym-Khana and https://gym-khana.readthedocs.io/
- F1TENTH simulator: lightweight 2D simulator of the UPenn racecar, ROS 2 containerized variant at https://github.com/f1tenth/f1tenth_gym_ros
- ApexSim: lap-time simulation library with track CSV (x, y, elevation, banking), Apache-2.0. https://github.com/pzimbrod/apexsim
- TLabVehiclePhysics: Unity wheel collider with Pacejka and look-up tables, MIT. https://github.com/tlabaltoh/tlabvehiclephysics

## Hardware reference

- XTENTH-CAR: 1/10 Traxxas Slash 4X4 VXL platform with brushless motor and VESC. https://arxiv.org/pdf/2212.01691

## Our own repos

- Fleet sim: https://github.com/Sujay-197/campus_fleet_man_unity (see `Docs/NEXT_STEPS.md`, `rc_transit/`, `Docs/specs/2026-08-16-physical-rc-transit-design.md`)

## Concepts to learn or cite

- Pacejka Magic Formula (tire force from slip angle and slip ratio)
- Load transfer and combined slip (friction ellipse)
- RMA-style rapid motor adaptation (history-conditioned policy for parameter inference)
- Domain randomization for sim-to-real
