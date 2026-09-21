# Patasmonkey UGV repository

Main Patasmonkey ROS 2 stack.

## Environment

- Target: Jetson AGX Xavier, Ubuntu 20.04, ROS 2 Foxy.
- Dev container: `patasmonkey_foxy_dev` (`patasmonkey:foxy-dev`).
- Main repo: host `~/Projects/patasmonkey_ws`, container `/workspaces/patasmonkey_ws`.
- External/general ROS workspace: host `~/ros2_ws`, container `/workspaces/ros2_ws`.

Treat both workspaces as one development area when diagnosing cross-package issues.

## Build rules

Do not run Foxy builds directly on the Ubuntu 22.04 host. Build inside
`patasmonkey_foxy_dev`, preferably package-by-package, as UID/GID `1000:1000`.

Expected overlay order:

1. `/opt/ros/foxy`
2. `/workspaces/ros2_ws`
3. `/workspaces/patasmonkey_ws`

For building `ros2_ws`, source only `/opt/ros/foxy/setup.bash`.

For building `patasmonkey_ws`:
- source `/opt/ros/foxy/setup.bash`
- source `/workspaces/ros2_ws/install/setup.bash` if present
- do not source `/workspaces/patasmonkey_ws/install/setup.bash` before build

Use `HOME=/home/developer` when available; otherwise the current legacy container
may use `HOME=/tmp`.

For runtime, source the three layers above in order.

## Dependency policy

Packages under `~/ros2_ws/src` may be modified when they are the correct source
of a problem. Before editing, inspect their git status, branch, and remote, and
preserve unrelated changes.

Do not permanently edit `/opt/ros/foxy`. If a system-installed ROS package must
be changed, use its upstream source under `~/ros2_ws/src` and build it as an
overlay.

## ROS compatibility

Preserve ROS 2 Foxy compatibility.

Unless required by the task, do not change public ROS interfaces such as topic
names, frame names, message types, QoS, parameter names, or launch interfaces.

## Change discipline

- Inspect git status before editing.
- Keep changes minimal and task-focused.
- Never discard unrelated user changes.
- After editing, inspect git diff and run relevant builds/tests when practical.
- Report modified files and test/build results.

## Hardware safety

Do not execute commands that may cause vehicle motion without explicit user
approval. This includes non-zero velocity commands, motor actuation, ODrive
calibration, autonomous motion, or any calibration that can move the vehicle.

Read-only diagnostics, logs, builds, static inspection, and non-actuating ROS
inspection are allowed.

## Jetson access

The Jetson may be accessed from the development host with:

    ssh pmjet1

Use it for Jetson-specific inspection, builds, runtime checks, and diagnostics
when required. Do not request, print, store, or modify SSH passwords or private
keys.

## Session records

For completed user requests, save one UTF-8 record under
`notes/codex_sessions/` named:

    YYYYMMDD_HHMMSS_<brief-description>.txt

Include only:
- `User prompt`: the relevant user prompt(s), verbatim
- `Codex final response`: the exact final response

Do not include internal reasoning or raw tool output. Do not create a record for
an aborted request with no final response.
