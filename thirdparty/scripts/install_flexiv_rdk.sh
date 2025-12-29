#!/bin/bash
set -e
echo "Installing flexiv_rdk"

# Use a specific version
VER_TAG=release/v1.9

# Clone source code with only 1 layer of history
if [ ! -d flexiv_rdk ] ; then
  git clone https://github.com/flexivrobotics/flexiv_rdk.git --depth 1 --branch $VER_TAG
  cd flexiv_rdk
else
  cd flexiv_rdk
  git checkout $VER_TAG
fi

# Save path to flexiv_rdk root
ROOT_DIR=$(pwd)

# Dependency installation script to run based on script argument
DEP_SCRIPT=build_and_install_dependencies
for arg in "$@"; do
  case "$arg" in
    --skip-ros2)
      DEP_SCRIPT=build_and_install_dependencies_not_in_ros2
      ;;
  esac
done

# Build and install nested dependencies
cd thirdparty
bash $DEP_SCRIPT.sh $INSTALL_DIR $NUM_JOBS

# Configure CMake
cd $ROOT_DIR
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR

# Build and install
cmake --build . --target install --config Release -j $NUM_JOBS

echo "Installed flexiv_rdk"
