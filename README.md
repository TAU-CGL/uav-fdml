[![Build](https://github.com/TAU-CGL/se3-localization/actions/workflows/build.yml/badge.svg)](https://github.com/TAU-CGL/se3-localization/actions/workflows/build.yml)
[![Tests](https://github.com/TAU-CGL/se3-localization/actions/workflows/tests.yml/badge.svg)](https://github.com/TAU-CGL/se3-localization/actions/workflows/tests.yml)

# UAV-Localization
Few-measurementLocalization with Approximations of Preimaged for Indoor UAVs

## The header `fdml.h`

Note that the entire UAV-FDML method is contained in a single header, located in the `include` directory.
The header `fdml_utils.h` containes utilities as well as the code needed for running an experiment for the paper.

The strict prerequisites for FDML are only CGAL and OpenMP. However, much more is needed to be installed for the demo visualization and experiments code to work. 

Namely, we use LightEngine3, which is a graphics engine developed in our lab for mesh processing and visualization.



## Installation

### Ubuntu 24.04 LTS

Tested also on WSL on Windows 11.

0. After cloning, make sure to also update submodules:

    ```
    git submodule init
    git submodule update --recursive --init --remote
    ```

1. Install prerequisites:

    ```
    sudo add-apt-repository universe
    sudo apt update
    sudo apt install -y build-essential cmake libfmt-dev libcgal-dev liblua5.4-dev \
        libglew-dev libsdl2-dev libassimp-dev libbullet-dev libglm-dev libcxxopts-dev libboost-dev \
        libboost-container-dev libgtest-dev py0ppipthon3 python3-pip python-is-python3 \
        python3-tqdm python3-pandas
    ```

2. Build the project:

    ```
    cmake -B build && make -C build
    ```

