[![Build](https://github.com/TAU-CGL/se3-localization/actions/workflows/build.yml/badge.svg)](https://github.com/TAU-CGL/se3-localization/actions/workflows/build.yml)
[![Tests](https://github.com/TAU-CGL/se3-localization/actions/workflows/tests.yml/badge.svg)](https://github.com/TAU-CGL/se3-localization/actions/workflows/tests.yml)

# UAV Few Distance Measurements Localization (UAV-FDML)

![https://raw.githubusercontent.com/TAU-CGL/uav-fdml/refs/heads/main/figures/main_sbs.png](https://raw.githubusercontent.com/TAU-CGL/uav-fdml/refs/heads/main/figures/main_sbs.png)

Few-measurement Localization with Approximations of Preimages for Indoor UAVs. Contains the original source code (see `v1` branch) of "Indoor Localization of UAVs Using Only Few Distance Measurement by Output Sensitive Preimage Intersection" (to appear in ICRA 2025).
Current branch is the further improved version, supporting real-time online localization of indoor drones, equipped with a few sensors pointing at arbitrary directions. Under review as "Demonstrating Effective Indoor UAV Localization with a Low-Cost ToF Sensor Crown" for RSS 2025.

See [https://www.cgl.cs.tau.ac.il/projects/demonstrating-effective-indoor-uav-localization-with-a-low-cost-tof-sensor-crown/](https://www.cgl.cs.tau.ac.il/projects/demonstrating-effective-indoor-uav-localization-with-a-low-cost-tof-sensor-crown/) for the project site.

## Installation and Usage

Please visit the [Wiki](https://github.com/TAU-CGL/uav-fdml/wiki) for detailed instruction for installation and usage of the software. This program was built and tested for macOS, Windows and Linux machines.

## The header `fdml.h`

Note that the entire UAV-FDML method is contained in a single header, located in the `include` directory.
The header `fdml_utils.h` containes utilities as well as the code needed for running an experiment for the paper.

The strict prerequisites for FDML are only CGAL and OpenMP. However, much more is needed to be installed for the demo visualization and experiments code to work. 
Namely, we use LightEngine3, which is a graphics engine developed in our lab for visualization.

See also `apps/experiments/src/experiment_accuracy.cpp` for a short minimal usage example of the code, without visualizations.

