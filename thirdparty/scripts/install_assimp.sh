#!/bin/bash
set -e
echo "Installing assimp"

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
  # Install assimp on Linux using apt
  if [ -w /usr/lib/ ]; then
    apt-get update && apt-get install -y libassimp-dev
  else
    sudo apt-get update && sudo apt-get install -y libassimp-dev
  fi
elif [[ "$OSTYPE" == "darwin"* ]]; then
  # Install assimp on macOS using Homebrew
  brew install assimp
else
  echo "ERROR: Please build and install assimp manually from source"
  exit 1
fi

echo "Installed assimp"
