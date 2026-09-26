#!/usr/bin/env bash
set -euo pipefail

sudo mkdir -p /opt/overlay_ws/{src,build,install,log} /tmp/.ccache
sudo chown "$(id -u):$(id -g)" /opt/overlay_ws /opt/overlay_ws/src
sudo chown -R "$(id -u):$(id -g)" /opt/overlay_ws/{build,install,log} /tmp/.ccache
git config --global --add safe.directory /opt/overlay_ws/src/flatland