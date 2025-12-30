> :information_source: The repository CLocoMani_Humanoid is being developed in parallel with the author's R&D progress.

# Computational Loco-Manipulation of Humanoid Robots
* Integrated whole-body kinematics and operation dynamics based on Sequential Convex Programming.*

<p align="center">
<img alt="CLocoMani_Humanoid"
    title="CLocoMani_Humanoid"
    src="media/CLocoMani_Humanoid.png"
    width="800px" />
</p>

<p align="center">
    <a href="https://opensource.org/licenses/BSD-3-Clause"><img src="https://img.shields.io/badge/License-BSD_3--Clause-blue.svg" alt="License BSD 3-Clause" /></a>
</p>

The <b>Computational Loco-Manipulation of Humanoid Robots</b> (CLocoMani_Humanoid) is a C++ implementation of trajectory generation 
of <b>integrated whole-body kinematics and operation dynamics</b> [1], by real-time SCP in MCP loop with first-order Quadratic SOCP solver, 
solely for academic purposes. 

Its architecture and infrastructure mainly refers to wb_humanoid_mpc [2] and OCS2 [4].

---

## Overview
CLocoMani_Humanoid implements the following designs and algorithms:
1. Integrated Dynamics including centroidal dynamics, whole-body kinematics, task manipulation dynamics.
2. Real-time trajectory generation based on SCP.

## Implementation Highlights
1. Analytical derivatives for hand-parser of centroidal dynamics and whole-body kinematics.
2. SCP in MPC: real-time handle with nonlinear OCP completely with hard constraints including {Affine Equality, Non-Negative, SOC}, rather than soft cost.
3. Dynamics: FOH discretization to less nodes and smooth control, better than ZOH.

---

## Reference
[1] Sleiman, J. P., Farshidian, F., Minniti, M. V., & Hutter, M. (2021). A unified mpc framework for whole-body dynamic locomotion and manipulation. IEEE Robotics and Automation Letters, 6(3), 4688-4695. \
[2] Manuel Yves Galliker, Whole-body Humanoid MPC: Realtime Physics-Based Procedural Loco-Manipulation Planning and Control, [https://github.com/1x-technologies/wb_humanoid_mpc]. \
[3] Carpentier, J., & Mansard, N. (2018, June). Analytical derivatives of rigid body dynamics algorithms. In Robotics: Science and systems (RSS 2018). \
[4] Farshidian, F. (2023). OCS2: An open source library for optimal control of switched systems. Accessed: May, 23.

## License
BSD 3-Clause License



