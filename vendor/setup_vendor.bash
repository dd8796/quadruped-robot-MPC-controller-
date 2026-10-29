#!/bin/bash
# A executer APRES source /opt/ros/humble/setup.bash et install/setup.bash.
# ros-humble-pointcloud-to-laserscan n'est pas installable ici (pas de sudo) -- .deb telecharge
# via `apt-get download` (fonctionne sans root) puis extrait avec `dpkg-deb -x` dans ce dossier.
# Sur une machine avec sudo, `sudo apt install ros-humble-pointcloud-to-laserscan` suffit et ce
# script devient inutile.
VENDOR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="${VENDOR_DIR}/pointcloud_to_laserscan:${AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="${VENDOR_DIR}/pointcloud_to_laserscan/lib:${LD_LIBRARY_PATH}"
