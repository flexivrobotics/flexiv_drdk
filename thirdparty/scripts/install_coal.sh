#!/bin/bash
set -e
echo "Installing coal"

# Repo name
REPO=coal
# Use a specific version
VER_TAG=v3.0.2

# Clone source code
if [ ! -d $REPO ] ; then
  git clone --recurse-submodules https://github.com/coal-library/$REPO.git --branch $VER_TAG
  cd $REPO
else
  cd $REPO
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DBUILDING_ROS2_PACKAGE=OFF \
         -DBUILD_PYTHON_INTERFACE=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed coal"
