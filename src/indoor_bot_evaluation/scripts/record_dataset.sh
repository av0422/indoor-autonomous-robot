#!/usr/bin/env bash
#
# Record a ROS 2 bag of indoor_bot's sensor and TF topics for offline
# perception development. Run from the workspace root (/ws in the Docker
# container) so the bag lands under data/bags/, which is git-ignored.
#
# Usage:
#   record_dataset.sh [name]
#
# name defaults to 'indoor_room'. The bag is written to
# data/bags/<name>_<timestamp>/ using the mcap storage format.

set -uo pipefail

name="${1:-indoor_room}"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_dir="data/bags/${name}_${timestamp}"

topics=(
  /clock
  /tf
  /tf_static
  /scan
  /odom
  /joint_states
  /imu
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/depth/image_raw
)

echo "Recording topics:"
printf '  %s\n' "${topics[@]}"
echo "Output directory: ${output_dir}"

mkdir -p "$(dirname "${output_dir}")"

ros2 bag record --storage mcap --output "${output_dir}" "${topics[@]}"

if [ -d "${output_dir}" ]; then
  echo "Bag size: $(du -sh "${output_dir}" | cut -f1)"
else
  echo "Warning: output directory ${output_dir} was not created." >&2
fi
