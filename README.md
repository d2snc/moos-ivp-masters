# MOOS-IvP Code for Master's Research

## Overview
This repository contains MOOS-IvP applications developed for master's research on COLREGs-compliant collision avoidance algorithms for Unmanned Surface Vessels (USVs).

## Submitted Paper
**"Toward Safe Autonomy at Sea: Implementation and Field Validation of COLREGs-Compliant Collision-Avoidance for Unmanned Surface Vessels"**
- Authors: Douglas Silva de Lima, Gustavo Alencar Bisinotto, Eduardo Aoun Tannuri
- Submitted to: Journal of Marine Science and Engineering (October 2025)
- Institution: Mechatronics Engineering Department, Universidade de São Paulo

## Modules

### `pContactSpawn`
Spawns virtual contacts that approach the vessel in a simulated manner based on specified parameters.
- Contacts are generated to **simulate COLREGs (COLREGS/RIPEAM) rules** for testing purposes.
- Enables controlled encounter geometries (head-on, crossing, overtaking).

### `pAstarColAvd`
Performs **collision avoidance** using a modified **A\* path-planning algorithm** while respecting COLREGs rules.
- Implements COLREGs-aware forbidden sectors
- Features dynamic re-planning capabilities

### `pColAvdvo`
Performs **collision avoidance** using a modified **Velocity Obstacles (VO) algorithm** with COLREGs compliance.
- Computes velocity-based collision cones in real-time
- Applies COLREGs-compliant maneuver selection

## Spawning Contacts

Use the following command to spawn a virtual contact:
```bash
uPokeDB SPAWN_CONTACT="heading=270,relative_bearing=0,distance=500,speed=5"
```

### Parameters:
- `heading`: Contact's heading in degrees (0-360)
- `relative_bearing`: Bearing relative to ownship (0-360)
- `distance`: Initial distance to contact in meters
- `speed`: Contact speed in m/s

## Running Missions

Three mission bundles are provided, each testing a different collision avoidance algorithm:

1. **`col_avd_moos`** → Standard MOOS-IvP COLREGs Behavior (BHV_AvdColregsV22)
2. **`astar`** → Modified A* path-planning algorithm
3. **`velocity_obstacle`** → Modified Velocity Obstacles (VO) algorithm

### How to Run:
```bash
cd missions/<mission_name>
./launch.sh
```

## Key Features

- **COLREGs Compliance**: All algorithms implement International Regulations for Preventing Collisions at Sea (Rules 13-17)
- **Field Validated**: Tested in real-world conditions at Guanabara Bay, Rio de Janeiro, Brazil
- **Three Encounter Types**: Head-on, crossing, and overtaking scenarios
- **Performance Metrics**: CPA (Closest Point of Approach), distance traveled, mission time, and trajectory deviation

## Experimental Results Summary

| Algorithm | Mean CPA (m) | Mean Distance (m) | Mean Time (s) | Distance Increase (%) |
|-----------|--------------|-------------------|---------------|----------------------|
| COLREGs Behavior | 30.0 | 179.4 | 174.7 | 27.2 |
| Velocity Obstacle | 13.0 | 193.1 | 186.0 | 37.6 |
| A* Algorithm | 30.0 | 203.5 | 202.3 | 46.5 |

## System Requirements

- MOOS-IvP (tested with release 19.8+)
- Ubuntu 20.04 or later
- ROS2 (for hardware integration)

## Citation

If you use this code in your research, please cite our work:
```bibtex
@article{delima2025colregs,
  title={Toward Safe Autonomy at Sea: Implementation and Field Validation of COLREGs-Compliant Collision-Avoidance for Unmanned Surface Vessels},
  author={de Lima, Douglas Silva and Bisinotto, Gustavo Alencar and Tannuri, Eduardo Aoun},
  journal={Submitted to Journal of Marine Science and Engineering},
  year={2025}
}
```

## Acknowledgments

This work was partially supported by:
- FINEP (Brazilian Innovation Agency)
- CASNAV (Naval Systems Analysis Center)
- CNPq – Brazilian National Council for Scientific and Technological Development (process #300884/2025-7)
- TPN-USP Ship Maneuvering Simulation Center

## License

This project is open-source and available under the terms specified in the repository.

## Contact

For questions or collaborations:
- Douglas Silva de Lima: douglasli1995@usp.br
- Gustavo Alencar Bisinotto: gustavo.bisinotto@usp.br
- Eduardo Aoun Tannuri: eduat@usp.br

---

**Repository**: https://github.com/d2snc/moos-ivp-masters
