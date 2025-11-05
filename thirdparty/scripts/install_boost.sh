#!/bin/bash
set -e
echo "Installing boost"

# Repo name
REPO=boost_1_83_0
# Use a specific version
VER_TAG=1.83.0

# Download source code, 1.83 required to work with MSVC v143
if [ ! -d $REPO ] ; then
  # download is faster than clone
  URL="https://archives.boost.io/release/$VER_TAG/source/$REPO.tar.bz2"
  echo "-- Downloading: $URL"
  if [ $OS_NAME == "Windows" ]; then
    curl -L -o $REPO.tar.bz2 $URL
  else
    wget $URL --no-clobber --quiet --show-progress --progress=bar:force 2>&1
  fi
  # Unzip
  echo "-- Extracting: $REPO.tar.bz2"
  tar --bzip2 -xf "$REPO.tar.bz2"
  cd $REPO
else
  cd $REPO
fi

# Build and install
./bootstrap.sh
./b2 -j4 --prefix=$INSTALL_DIR --with-serialization --with-filesystem variant=release link=static install

echo "Installed boost"
