#!/bin/bash
set -e
echo "Installing urdfdom_headers"

# Repo name
REPO=urdfdom_headers
# Use a specific version
VER_TAG=2.0.0

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
cmake .. $SHARED_CMAKE_ARGS \
         -DAPPEND_PROJECT_NAME_TO_INCLUDEDIR=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed urdfdom_headers"
