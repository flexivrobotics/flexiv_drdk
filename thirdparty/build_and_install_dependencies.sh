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
    echo "Optional argument: [num_parallel_jobs]"
    echo "    num_parallel_jobs: number of parallel jobs used to build, use 4 if not specified"
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

# Build and install all dependencies to INSTALL_DIR
bash $SCRIPTPATH/scripts/install_flexiv_rdk.sh
bash $SCRIPTPATH/scripts/install_boost.sh
bash $SCRIPTPATH/scripts/install_assimp.sh
bash $SCRIPTPATH/scripts/install_coal.sh

echo ">>>>> Finished: flexiv_drdk/thirdparty/build_and_install_dependencies.sh <<<<<"
