#!/usr/bin/env bash
set -euo pipefail

setup_line='source /opt/overlay_ws/install/setup.bash'
if ! grep -Fxq "$setup_line" "$HOME/.bashrc"; then
    printf '\n%s\n' "$setup_line" >> "$HOME/.bashrc"
fi