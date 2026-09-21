# Patasmonkey UGV repository

This repository contains the main Patasmonkey ROS 2 stack.

## Target runtime

- Platform: Jetson AGX Xavier
- OS / ROS: Ubuntu 20.04 / ROS 2 Foxy
- Development container: `patasmonkey_foxy_dev`
- Development image: `patasmonkey:foxy-dev`

## Workspace layout

| Workspace | Host | Container |
|---|---|---|
| Primary Patasmonkey repository | `~/Projects/patasmonkey_ws` | `/workspaces/patasmonkey_ws` |
| General ROS / external drivers | `~/ros2_ws` | `/workspaces/ros2_ws` |

Treat both workspaces together when diagnosing cross-package problems. Do not
assume a Patasmonkey issue must be fixed in this repository. Localization and
visual-odometry issues may involve `pm_localization`, `pm_bringup`, `pm_config`,
`open_vins`, `depthai_driver`, camera/IMU drivers, or system-installed ROS
dependencies. Investigate the full dependency chain before choosing where to
modify code.

## Build procedure

Do not build this workspace directly on the Ubuntu 22.04 host. Build inside the
Docker container, preferably package-by-package and as UID/GID 1000:1000 to
avoid root-owned files.

Before building:

1. Source `/opt/ros/foxy/setup.bash`.
2. Source `/workspaces/ros2_ws/install/setup.bash` if it exists.
3. Do **not** source `/workspaces/patasmonkey_ws/install/setup.bash`.
4. Change to `/workspaces/patasmonkey_ws`.

Use `/home/developer` as `HOME` when it exists in the container; the current
legacy container may temporarily use `HOME=/tmp`.

Current legacy-container example:

```bash
docker exec --user 1000:1000 --env HOME=/tmp patasmonkey_foxy_dev bash -lc '
  source /opt/ros/foxy/setup.bash
  if [ -f /workspaces/ros2_ws/install/setup.bash ]; then
    source /workspaces/ros2_ws/install/setup.bash
  fi
  cd /workspaces/patasmonkey_ws
  colcon build --packages-select <package>
'
```

After rebuilding the image with `/home/developer`:

```bash
docker exec --user 1000:1000 --env HOME=/home/developer \
  patasmonkey_foxy_dev bash -lc '<command>'
```

## Running ROS nodes

After a successful build, source—in order—`/opt/ros/foxy/setup.bash`, the
general ROS workspace install if present, and the Patasmonkey workspace install
if present. Then run the required ROS 2 commands, launch files, or diagnostics.

## External ROS dependencies

When a required change belongs to a package under `~/ros2_ws/src`, modify that
source repository instead of forcing the change into Patasmonkey. First inspect
its git status, current branch, and remote; preserve unrelated local changes;
and explain why it is the correct modification target.

For packages available only under `/opt/ros/foxy`:

- Never edit `/opt/ros/foxy` permanently.
- Identify the upstream source and check for it under `~/ros2_ws/src`.
- If needed, bring it into `~/ros2_ws/src` on a dedicated development branch or fork.
- Build it as an overlay and rebuild downstream packages when compatibility may be affected.

## ROS interface policy

Unless the task requires it, do not change topic names, frame names, message
types, QoS settings, parameter names, launch interfaces, or public package
interfaces. Preserve ROS 2 Foxy compatibility.

## Change policy

- Before editing: inspect git status.
- While editing: keep changes minimal and task-focused, avoid unrelated
  refactoring, and never discard unrelated user changes.
- After editing: inspect git diff, report modified files, run relevant
  builds/tests when practical, and report their results.
