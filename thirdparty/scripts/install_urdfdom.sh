#!/bin/bash
# Depends on: console_bridge, urdfdom_headers, tinyxml2
set -e
echo "Installing urdfdom"

# Repo name
REPO=urdfdom
# Use a specific version
VER_TAG=5.0.2

# Clone source code
if [ ! -d $REPO ] ; then
  git clone https://github.com/ros/$REPO.git --branch $VER_TAG
  cd $REPO
else
  cd $REPO
  git checkout $VER_TAG
fi

# Apply patch to remove forced building shared library
git reset --hard
git apply $SCRIPTPATH/patches/$REPO.patch

# Configure CMake
mkdir -p build && cd build
cmake .. $SHARED_CMAKE_ARGS \
         -DAPPEND_PROJECT_NAME_TO_INCLUDEDIR=OFF

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed urdfdom"
