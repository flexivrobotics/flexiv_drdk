# Flexiv DRDK

![CMake Badge](https://github.com/flexivrobotics/flexiv_drdk/actions/workflows/cmake.yml/badge.svg)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://www.apache.org/licenses/LICENSE-2.0.html)

Flexiv DRDK (Dual Robot Development Kit) is built on top of Flexiv RDK, with additional features tailored for a dual robot setup like simultaneous commanding, self-collision avoidance, bimanual primitives, etc.

## Environment Compatibility

| **OS**                | **Platform**  | **C++ compiler kit** | **Python interpreter** |
| --------------------- | ------------- | -------------------- | ---------------------- |
| Linux (Ubuntu 20.04+) | x86_64, arm64 | GCC   v9.4+          | 3.8, 3.10, 3.12        |

**IMPORTANT**: You might need to turn off your computer's firewall or whitelist the DRDK programs to be able to establish connection with the robot.

## Quick Start - Python

### Install the Python package

On all supported platforms, the Python package of DRDK and its dependencies for a specific Python version can be installed using the `pip` module:

    python3.x -m pip install numpy spdlog flexivdrdk

NOTE: replace `3.x` with a specific Python version.

### Use the installed Python package

After the ``flexivdrdk`` Python package is installed, it can be imported from any Python script. Test with the following commands in a new Terminal, which should start Flexiv DRDK:

    python3.x
    import flexivdrdk
    robot_pair = flexivdrdk.RobotPair(["Rizon4-100001", "Rizon4-100002"])

The program will start searching for a pair of robots with serial numbers `Rizon4-100001` for the left arm and `Rizon4-100002` for the right arm, then exit after a couple of seconds if either of the specified robots is not found in the local network.

## Quick Start - C++

### Prepare build tools

#### Linux

1. Install compiler kit using package manager:

       sudo apt install build-essential

2. Install CMake using package manager:

       sudo apt install cmake

### Install the C++ library

The following steps are identical on all supported platforms.

1. Choose a directory for installing the C++ library of DRDK and its dependencies. This directory can be under system path or not, depending on whether you want DRDK to be globally discoverable by CMake. For example, a new folder named ``drdk_install`` under the home directory.
2. In a new Terminal, run the provided script to compile and install all dependencies to the installation directory chosen in step 1:

       cd flexiv_drdk/thirdparty
       bash build_and_install_dependencies.sh ~/drdk_install

3. In a new Terminal, configure the ``flexiv_drdk`` CMake project:

       cd flexiv_drdk
       mkdir build && cd build
       cmake .. -DCMAKE_INSTALL_PREFIX=~/drdk_install

   NOTE: ``-D`` followed by ``CMAKE_INSTALL_PREFIX`` sets the absolute path of the installation directory, which should be the one chosen in step 1.

4. Install ``flexiv_drdk`` C++ library to ``CMAKE_INSTALL_PREFIX`` path, which may or may not be globally discoverable by CMake depending on the location:

       cd flexiv_drdk/build
       cmake --build . --target install --config Release

### Use the installed C++ library

After the library is installed as ``flexiv_drdk`` CMake target, it can be linked from any other CMake projects. For example:

    cd my_project
    mkdir build && cd build
    cmake .. -DCMAKE_PREFIX_PATH=~/drdk_install
    cmake --build . --config Release -j 4

NOTE: ``-D`` followed by ``CMAKE_PREFIX_PATH`` tells the user project's CMake where to find the installed C++ library. This argument can be skipped if the DRDK library and its dependencies are installed to a globally discoverable location.