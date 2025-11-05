#!/bin/bash
set -e
echo "Installing assimp"

PKG=libassimp-dev

# Skip if already installed
if dpkg -s $PKG >/dev/null 2>&1; then
    echo "$PKG is already installed, skip installation"
    exit 0
fi

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
  # Install assimp on Linux using apt
  if [ -w /usr/lib/ ]; then
    apt-get update && apt-get install -y $PKG
  else
    sudo apt-get update && sudo apt-get install -y $PKG
  fi
else
  echo "ERROR: only Linux is supported"
  exit 1
fi

echo "Installed assimp"
