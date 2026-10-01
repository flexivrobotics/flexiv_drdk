#!/bin/sh
# Build and install all dependencies of flexiv_drdk.
echo ">>>>> Start: flexiv_drdk/thirdparty/build_and_install_dependencies.sh <<<<<"

# Absolute path of this script
export SCRIPTPATH="$(dirname $(readlink -f $0))"
set -e

# Check script arguments
if [ "$#" -lt 1 ]; then
    echo "Error: invalid script argument"
    echo "Required argument: [install_directory_path]"
    echo "    install_directory_path: directory to install all dependencies, should be the same as the install directory of flexiv_drdk"
    echo "Optional argument: [num_parallel_jobs] [--skip-rdk]"
    echo "    num_parallel_jobs: number of parallel jobs used to build, use 4 if not specified"
    echo "    --skip-rdk: skip building and install flexiv_rdk"
    exit
fi

# Get dependencies install directory from script argument, should be the same as the install directory of flexiv_drdk
export INSTALL_DIR=$1
echo "Dependencies will be installed to: $INSTALL_DIR"

# Use specified number for parallel build jobs, otherwise use 4
if [ -n "$2" ] ;then
    export NUM_JOBS=$2
else
    export NUM_JOBS=4
fi
echo "Number of parallel build jobs: $NUM_JOBS"

# Set shared cmake arguments
export SHARED_CMAKE_ARGS="-DCMAKE_BUILD_TYPE=Release \
                          -DBUILD_SHARED_LIBS=ON \
                          -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
                          -DCMAKE_PREFIX_PATH=$INSTALL_DIR \
                          -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR \
                          -DBUILD_TESTING=OFF"

# OS type
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS_NAME="Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS_NAME="Darwin"
else
    OS_NAME="Windows"
fi
export OS_NAME

# Clone all dependencies in a subfolder
mkdir -p cloned && cd cloned

# Dependency installation script to run based on script argument
SKIP_RDK=false
for arg in "$@"; do
  case "$arg" in
    --skip-rdk)
      SKIP_RDK=true
      ;;
  esac
done

# Build and install all dependencies to INSTALL_DIR.
# DRDK is shipped as a self-contained shared library that statically embeds and symbol-hides its
# thirdparty dependencies, so none of those has to be built here. flexiv_rdk is the only remaining
# dependency, because rdk:: types appear in the public flexiv::drdk API.
if $SKIP_RDK; then
    echo "Skipping flexiv_rdk"
else
    bash $SCRIPTPATH/scripts/install_flexiv_rdk.sh
fi

echo ">>>>> Finished: flexiv_drdk/thirdparty/build_and_install_dependencies.sh <<<<<"
