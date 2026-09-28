#!/usr/bin/env bash
set -euo pipefail

setup_line='source /opt/flatland_upstream_ws/install/setup.bash; if [ -f /opt/overlay_ws/install/setup.bash ]; then source /opt/overlay_ws/install/setup.bash; fi'
if grep -Fxq 'source /opt/overlay_ws/install/setup.bash' "$HOME/.bashrc"; then
    sed -i '\|^source /opt/overlay_ws/install/setup.bash$|d' "$HOME/.bashrc"
fi
if ! grep -Fxq "$setup_line" "$HOME/.bashrc"; then
    printf '\n%s\n' "$setup_line" >> "$HOME/.bashrc"
fi