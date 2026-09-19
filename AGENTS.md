# Patasmonkey UGV repository

This repository contains the main Patasmonkey ROS 2 stack.

## Target runtime

Target platform:
- Jetson AGX Xavier

Target OS / ROS environment:
- Ubuntu 20.04
- ROS 2 Foxy

Development Foxy environment:
- Docker container: patasmonkey_foxy_dev
- Docker image: patasmonkey:foxy-dev

## Workspace layout

Primary repository:

Host:
- ~/Projects/patasmonkey_ws

Container:
- /workspaces/patasmonkey_ws

General ROS / external-driver workspace:

Host:
- ~/ros2_ws

Container:
- /workspaces/ros2_ws

The Patasmonkey repository and ros2_ws should be treated together when diagnosing
cross-package problems.

Do not assume that a Patasmonkey issue must be fixed inside this repository.

For example, localization or visual-odometry problems may involve:
- pm_localization
- pm_bringup
- pm_config
- open_vins
- depthai_driver
- camera / IMU drivers
- system-installed ROS dependencies

Investigate the full dependency chain before choosing the modification target.

## Build procedure

Do not build this workspace directly on the Ubuntu 22.04 host.

Build inside the Docker container.

Before building:

1. source /opt/ros/foxy/setup.bash
2. source /workspaces/ros2_ws/install/setup.bash if it exists
3. do NOT source /workspaces/patasmonkey_ws/install/setup.bash
4. cd /workspaces/patasmonkey_ws

Run builds as UID/GID 1000:1000 when practical to avoid creating root-owned files.

Use /home/developer as HOME when that directory exists in the container.
For the currently running legacy container, HOME=/tmp may be used temporarily.

Example for the current legacy container:

docker exec \
  --user 1000:1000 \
  --env HOME=/tmp \
  patasmonkey_foxy_dev \
  bash -lc '
    source /opt/ros/foxy/setup.bash
    if [ -f /workspaces/ros2_ws/install/setup.bash ]; then
        source /workspaces/ros2_ws/install/setup.bash
    fi
    cd /workspaces/patasmonkey_ws
    colcon build --packages-select <package>
  '

After the Docker image has been rebuilt and /home/developer exists:

docker exec \
  --user 1000:1000 \
  --env HOME=/home/developer \
  patasmonkey_foxy_dev \
  bash -lc '<command>'


Prefer package-specific builds where possible.

## Running ROS nodes

After successful builds:

1. source /opt/ros/foxy/setup.bash
2. source /workspaces/ros2_ws/install/setup.bash if it exists
3. source /workspaces/patasmonkey_ws/install/setup.bash if it exists

Then run ros2 commands, launch files, or diagnostics as required.

## External ROS dependencies

If investigation indicates that the required change belongs to a package under:

- ~/ros2_ws/src

modify that source repository rather than forcing the change into Patasmonkey.

Before modifying such a repository:
- inspect git status
- inspect the current branch
- inspect the remote
- preserve unrelated local changes
- explain why that repository is the appropriate modification target

If the required package exists only under /opt/ros/foxy:

- do not permanently edit /opt/ros/foxy
- identify the upstream source repository
- check whether it already exists under ~/ros2_ws/src
- if necessary, bring the source repository into ~/ros2_ws/src
- use a dedicated development branch or fork
- build it as an overlay
- rebuild downstream packages if compatibility may be affected

## ROS interface policy

Do not change the following unless required by the task:

- topic names
- frame names
- message types
- QoS settings
- parameter names
- launch interfaces
- public package interfaces

Preserve ROS 2 Foxy compatibility.

## Change policy

Before editing:
- inspect git status

While editing:
- keep changes minimal and task-focused
- avoid unrelated refactoring
- do not discard unrelated user changes

After editing:
- inspect git diff
- report modified files
- run relevant builds/tests when practical
- report build/test results