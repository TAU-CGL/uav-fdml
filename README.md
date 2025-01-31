[![Build](https://github.com/TAU-CGL/se3-localization/actions/workflows/build.yml/badge.svg)](https://github.com/TAU-CGL/se3-localization/actions/workflows/build.yml)
[![Tests](https://github.com/TAU-CGL/se3-localization/actions/workflows/tests.yml/badge.svg)](https://github.com/TAU-CGL/se3-localization/actions/workflows/tests.yml)

# UAV Few Distance Measurements Localization (UAV-FDML)
Few-measurement Localization with Approximations of Preimages for Indoor UAVs.

## The header `fdml.h`

Note that the entire UAV-FDML method is contained in a single header, located in the `include` directory.
The header `fdml_utils.h` containes utilities as well as the code needed for running an experiment for the paper.

The strict prerequisites for FDML are only CGAL and OpenMP. However, much more is needed to be installed for the demo visualization and experiments code to work. 

Namely, we use LightEngine3, which is a graphics engine developed in our lab for visualization.
