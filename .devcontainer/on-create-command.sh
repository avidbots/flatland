#!/usr/bin/env bash
set -euo pipefail

mkdir -p /opt/overlay_ws/src /tmp/.ccache
git config --global --add safe.directory /opt/overlay_ws/src/flatland