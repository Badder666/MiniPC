#!/usr/bin/env bash
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/custom_controller_sim_${UID}.lock"
if ! flock -n 9; then
    echo "控制器 MuJoCo 仿真已运行。"
    exit 0
fi
exec "$HOME/venvs/controller_sim/bin/python" "$workspace_dir/simulation/controller_physics.py" "$@"
