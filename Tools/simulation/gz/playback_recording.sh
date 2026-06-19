#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
	echo "Usage: $0 <record_path> [gz sim playback args...]" >&2
	exit 1
fi

record_path=$1
shift || true

script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "${script_dir}/../../.." && pwd)
gz_env="${repo_root}/build/px4_sitl_default/rootfs/gz_env.sh"

if [ ! -f "${gz_env}" ]; then
	echo "Missing ${gz_env}. Build px4_sitl first so Gazebo env is generated." >&2
	exit 1
fi

export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# shellcheck disable=SC1090
. "${gz_env}"

# Playback only needs model / world resource paths for meshes.
# The full PX4 server config loads live sensor systems again, which can crash
# when replaying recorded state logs for models with sensors.
unset GZ_SIM_SERVER_CONFIG_PATH

exec gz sim --playback "${record_path}" "$@"
