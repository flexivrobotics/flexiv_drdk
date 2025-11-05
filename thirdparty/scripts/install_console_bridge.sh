#!/bin/bash
set -e
echo "Installing console_bridge"

# Repo name
REPO=console_bridge
# Use a specific version
VER_TAG=1.0.2

# Clone source code
if [ ! -d $REPO ] ; then
  git clone https://github.com/ros/$REPO.git --branch $VER_TAG
  cd $REPO
else
  cd $REPO
  git checkout $VER_TAG
fi

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed console_bridge"
