#!/bin/bash
set -e

BASEDIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"

# TODO: why doesn't uv do this?
export PYTHONPATH=$BASEDIR

# *** dependencies install ***
if ! command -v uv &>/dev/null; then
  echo "'uv' is not installed. Installing 'uv'..."
  curl -LsSf https://astral.sh/uv/install.sh | sh
fi

export UV_PROJECT_ENVIRONMENT="$BASEDIR/.venv"
uv sync --all-extras
source "$PYTHONPATH/.venv/bin/activate"

# Make sure ninja is installed
if ! command -v ninja &> /dev/null; then
  echo "Installing ninja-build..."
  pip install ninja
fi

# Setup meson build directory if it doesn't exist
if [ ! -d "$BASEDIR/builddir" ] || [ ! -f "$BASEDIR/builddir/build.ninja" ]; then
  echo "Setting up meson build directory..."
  meson setup builddir
fi

# Build the project
echo "Building with meson..."
if [ -f "$BASEDIR/builddir/build.ninja" ]; then
  meson compile -C builddir
else
  echo "Error: Failed to create meson build directory properly. Check if meson and ninja are installed."
fi
