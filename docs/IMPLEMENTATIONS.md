# Other 3D Dubins implementations for aerial logistics

Search date: 2026-09-21. Repositories are grouped by what they actually implement. Fork-only duplicates are omitted unless they add a distinct implementation.

## Python implementation in this folder

Start with [`core.py`](../dubins_airplane/core.py) and [`example.py`](../examples/example.py). The standard-library planner implements **Owen–Beard–McLain, Section 4** between `(x, y, z, yaw)` states: low, medium (fitted extra arc), and high (complete turns and fitted radius). [`KNOWLEDGE_GUIDE.md`](KNOWLEDGE_GUIDE.md) maps the code to the equations and explains its limits and differences from the NTNU implementation.

## Direct 3D path solvers

| Repository | Language | What is implemented |
|---|---|---|
| [comrob/Dubins3D.jl](https://github.com/comrob/Dubins3D.jl) | Julia | Váňa et al. decoupled/local-optimization method; bounded curvature and pitch angle. Companion to the 2020 ICRA paper. |
| [Rylandl/dubins3d.rs](https://github.com/Rylandl/dubins3d.rs) | Rust | Rust reimplementation of `Dubins3D.jl`; produces sampled 3D paths with turn-radius and pitch limits. |
| [aabecker/dubins3D](https://github.com/aabecker/dubins3D) | Mathematica | Analytic 3D CSC solver via the RRPRR inverse-kinematics formulation from Baez, Navkar, and Becker. |
| [DeepakPrakashKumar/3D-Motion-Planning-for-Generalized-Dubins-with-Pitch-Yaw-constraints](https://github.com/DeepakPrakashKumar/3D-Motion-Planning-for-Generalized-Dubins-with-Pitch-Yaw-constraints) | Python | Generalized spatial Dubins construction with separate pitch/yaw bounds; includes sphere, cylinder, and cone cases. |
| [ntnu-arl/DubinsAirplane](https://github.com/ntnu-arl/DubinsAirplane) | Python | Owen–Beard–McLain Dubins-airplane paths, including low-, medium-, and high-altitude cases. |
| [ompl/ompl](https://github.com/ompl/ompl) | C++ with Python bindings | Production implementations of `OwenStateSpace`, `VanaStateSpace`, and `VanaOwenStateSpace`; see the [3D state-space sources](https://github.com/ompl/ompl/tree/main/src/ompl/base/spaces) and [DubinsAirplane demo](https://github.com/ompl/ompl/blob/main/demos/DubinsAirplane.py). Useful as the steering layer of an aerial-logistics planner. |

## Planners built around 3D Dubins steering

| Repository | Language | What is implemented |
|---|---|---|
| [robotics-uncc/VisualTour3DDubins](https://github.com/robotics-uncc/VisualTour3DDubins) | Python/Rust | 3D Dubins-airplane visual-inspection tours and DTSPN planning in urban environments. |
| [zgoddard3/hybrid-astar](https://github.com/zgoddard3/hybrid-astar) | Python | Compact Hybrid A* demo whose motion primitives are Dubins-airplane curves. |
| [ethz-asl/terrain-navigation](https://github.com/ethz-asl/terrain-navigation) | C++/ROS | RRT* global planning in a Dubins-airplane state space for low-altitude fixed-wing terrain navigation. |
| [ethz-asl/terrain-navigation2](https://github.com/ethz-asl/terrain-navigation2) | C++/ROS 2 | ROS 2 continuation of the terrain-navigation planner. |
| [StanfordASL/GMT](https://github.com/StanfordASL/GMT) | CUDA C/MATLAB | Group Marching Tree examples with a 4D Dubins-airplane steering problem. |
| [exbibyte/sample_planning](https://github.com/exbibyte/sample_planning) | Rust | Sampling-based kinodynamic planning with selectable `dubins` and `airplane` dynamics, motion primitives, and 3D obstacles. |
| [bellenliu/RRTstar_3D-Dubins](https://github.com/bellenliu/RRTstar_3D-Dubins) | Python | Small experimental RRT* / 3D-Dubins repository. Documentation is minimal, so treat it as prototype code. |

## Models, simulations, and supporting implementations

These repositories are useful for reproducing papers or integrating a solver, but are not all stand-alone shortest-path solvers.

| Repository | Language | Relevance |
|---|---|---|
| [byu-magicc/mavsim_public](https://github.com/byu-magicc/mavsim_public) | MATLAB/Simulink/Python | Companion material for *Small Unmanned Aircraft* and the Owen–Beard–McLain Dubins-airplane architecture. |
| [hasanisci/gym-dubins-ac](https://github.com/hasanisci/gym-dubins-ac) | Python | Gym environment for Dubins-aircraft dynamics; useful for control/RL rather than analytic path generation. |
| [RationalCyPhy/RTAEval](https://github.com/RationalCyPhy/RTAEval) | Python | 2D/3D Dubins-aircraft dynamics and safety/tracking demonstrations in the RTAEval framework. |
| [c-hague/3DVisibilityGraphs](https://github.com/c-hague/3DVisibilityGraphs) | Python/C++ | Supporting aerial visibility-volume and inspection-tour planning associated with 3D Dubins aircraft. |

## Aerial repository that is not a full 3D Dubins solver

- [NandanV23/dubins-uav-planner](https://github.com/NandanV23/dubins-uav-planner): six classical 2D Dubins word types coupled to a 3-DOF UAV simulator.

## Notes

- GitHub search results contain many forks of `mavsim_public` and `DubinsAirplane`; the table points to the upstream or most clearly documented repository.
- “Exact” depends on the aircraft model. The analytic CSC solver, Dubins-airplane constructions, and pitch/yaw generalized solver solve different boundary-value problems.
- Repository availability and contents can change after the search date.
