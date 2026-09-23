# Paper catalogue

This catalogue links to the original sources of 15 papers relevant to aerial logistics. Local PDF copies may be kept in this directory for research, but `*.pdf` is excluded from Git until redistribution rights are checked. “Core” means the paper develops an aircraft-applicable 3D Dubins/Dubins-airplane construction; “application” means it uses such a construction inside UAV routing, mission planning, inspection, obstacle avoidance, or guidance.

| Year | Paper | Scope | Source |
|---:|---|---|---|
| 2007 | Time-Optimal Paths for a Dubins Airplane | Core optimal-control formulation | [LaValle/Illinois PDF](https://msl.cs.uiuc.edu/~lavalle/papers/ChiLav07b.pdf) |
| 2010 | Optimal Geometrical Path in 3D with Curvature Constraint | Core geometric/numerical spatial construction | [PDF mirror](https://sector3.imm.uran.ru/magistr/literat/05653663.pdf) |
| 2013/2014 | Implementing Dubins Airplane Paths on Fixed-Wing UAVs | Core Dubins-airplane architecture and guidance | [Author preprint mirror](https://sector3.imm.uran.ru/magistr/literat/BeardMcLain__.pdf) |
| 2016 | Novel Three-Dimensional Optimal Path Planning Method for Vehicles with Constrained Pitch and Yaw | Core pitch/yaw-constrained construction | [Cambridge PDF](https://www.cambridge.org/core/services/aop-cambridge-core/content/view/0BF09514B974A55F4B571BE7942255F6/S026357471600076Xa.pdf/novel-three-dimensional-optimal-path-planning-method-for-vehicles-with-constrained-pitch-and-yaw.pdf) |
| 2017 | Data Collection Planning with Dubins Airplane Model and Limited Travel Budget | Application to 3D data collection/orienteering | [CVUT PDF](https://comrob.fel.cvut.cz/papers/ecmr17dop3d.pdf) |
| 2020 | Minimal 3D Dubins Path with Bounded Curvature and Pitch Angle | Core fast decoupled/local-optimization method | [CVUT PDF](https://comrob.fel.cvut.cz/papers/icra20dubins3d.pdf) |
| 2021 | Finding 3D Dubins Paths with Pitch Angle Constraint Using Non-linear Optimization | Core numerical reference solver | [CVUT PDF](https://comrob.fel.cvut.cz/papers/ecmr21dubins3d-nlp.pdf) |
| 2022 | Three-Dimensional Dubins-Path-Guided Continuous Curvature Path Smoothing | Dubins-guided 3D smoothing | [MDPI PDF](https://mdpi-res.com/d_attachment/applsci/applsci-12-11336/article_deploy/applsci-12-11336-v2.pdf) |
| 2023 | Planning Visual Inspection Tours for a 3D Dubins Airplane Model | Application to inspection tours/DTSPN | [arXiv](https://arxiv.org/abs/2301.05309) |
| 2024 | An Analytic Solution to the 3D CSC Dubins Path Problem | Core analytic CSC solution | [arXiv](https://arxiv.org/abs/2405.08710) |
| 2024 | Safe Low-Altitude Navigation in Steep Terrain with Fixed-Wing Aerial Vehicles | Application using RRT* in a Dubins-airplane space | [arXiv](https://arxiv.org/abs/2401.04831) |
| 2024 | Closed-Form Solutions for Minimum-Time Paths of a Dubins Airplane in Steady Wind | Core wind-aware minimum-time solution | [arXiv](https://arxiv.org/abs/2412.04797) |
| 2025 | Reparametrization of 3D CSC Dubins Paths Enabling 2D Search | Core CSC parameter reduction | [arXiv](https://arxiv.org/abs/2503.11560) |
| 2025 | A Novel Model for 3D Motion Planning for a Generalized Dubins Vehicle with Pitch and Yaw Rate Constraints | Core generalized 3D model | [arXiv](https://arxiv.org/abs/2509.24143) |
| 2026 | Simple Trajectory Smoothing for UAV Reference Path Planning Based on Decoupling, Spatial Modeling and Linear Programming | Dubins-airplane-based smoothing | [arXiv](https://arxiv.org/abs/2603.21713) |

## Relevant full texts found but not downloaded

These sources were found, but their hosts blocked automated download or returned an HTML interstitial instead of a PDF. The links are retained so the papers are not lost from the bibliography.

- Yucong Lin and Srikanth Saripalli, “Path Planning Using 3D Dubins Curve for Unmanned Aerial Vehicles” (2014): [IEEE record](https://ieeexplore.ieee.org/document/6842268), [ResearchGate full-text page](https://www.researchgate.net/publication/269299848_Path_planning_using_3D_Dubins_Curve_for_Unmanned_Aerial_Vehicles).
- Weinan Wu et al., “Integrated Method for Multi-UAV Task Assignment and Trajectory Planning with Deadlock Based on Three-Dimensional Dubins Path” (2025): [open article and PDF link](https://www.nature.com/articles/s41598-025-09753-x).

## Selection boundary

The folder focuses on methods that formulate, derive, evaluate, or directly use spatial Dubins/Dubins-airplane paths for aircraft and UAV operations. Underwater vehicles, ground robots, sphere-only motion, general 3D RRT/A*, ordinary planar Dubins papers, and papers that only cite 3D Dubins are excluded.
