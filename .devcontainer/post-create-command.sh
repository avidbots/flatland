#!/usr/bin/env bash
set -euo pipefail

setup_line='if [ -f /opt/overlay_ws/install/setup.bash ]; then source /opt/overlay_ws/install/setup.bash; else for setup_file in /opt/ros/*/setup.bash; do if [ -f "$setup_file" ]; then source "$setup_file"; break; fi; done; fi'
if grep -Fxq 'source /opt/overlay_ws/install/setup.bash' "$HOME/.bashrc"; then
    sed -i '\|^source /opt/overlay_ws/install/setup.bash$|d' "$HOME/.bashrc"
fi
if ! grep -Fxq "$setup_line" "$HOME/.bashrc"; then
    printf '\n%s\n' "$setup_line" >> "$HOME/.bashrc"
fi