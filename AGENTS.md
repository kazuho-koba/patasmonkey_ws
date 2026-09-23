# Patasmonkey UGV repository

ROS 2 Foxy stack for Patasmonkey UGV. Target is Jetson AGX Xavier / Ubuntu
20.04; the development host uses the `patasmonkey_foxy_dev` container.

## Workspaces and builds

- Main workspace: host `~/Projects/patasmonkey_ws`, container
  `/workspaces/patasmonkey_ws`.
- External ROS workspace: host `~/ros2_ws`, container `/workspaces/ros2_ws`.
  Treat both as one development area when diagnosing cross-package behavior.
- Do **not** build Foxy on the Ubuntu 22.04 host. Build in
  `patasmonkey_foxy_dev`, preferably package-by-package as UID/GID `1000:1000`.
- Build `ros2_ws` after sourcing only `/opt/ros/foxy/setup.bash`.
- Build `patasmonkey_ws` after sourcing `/opt/ros/foxy/setup.bash`, then
  `/workspaces/ros2_ws/install/setup.bash` when present; do not source the
  main workspace overlay before its build.
- At runtime source overlays in that same order. Prefer `HOME=/home/developer`
  (legacy images may require `HOME=/tmp`).

## Compatibility and dependencies

- Preserve ROS 2 Foxy compatibility and existing public ROS interfaces
  (topics, frames, message types, QoS, parameters, and launch interfaces)
  unless the task explicitly requires a change.
- Do not edit `/opt/ros/foxy` permanently. When an installed dependency needs a
  fix, use its source under `~/ros2_ws/src` and overlay-build it.
- Before editing packages under `~/ros2_ws/src`, inspect their status, branch,
  and remote; preserve unrelated changes.

## Code and change discipline

- Inspect git status before editing; keep changes minimal and task-focused;
  never discard unrelated user changes. Inspect the diff and run proportional
  builds/tests after editing when practical.
- Add deliberate, useful comments to new or materially changed code. Document
  non-obvious intent, coordinate frames and units, algorithm assumptions,
  parameter/threshold semantics, data ownership or real-time performance
  choices, and safety-relevant behavior. Use module/class/function docstrings
  where they clarify an API or processing stage.
- Do not add comments that merely restate obvious syntax. Keep comments current
  when changing behavior; code and comments must agree.
- Report modified files plus relevant build/test results.

## Hardware and Jetson access

- Never execute commands that can move the vehicle without explicit approval:
  non-zero velocity commands, motor actuation, ODrive calibration, autonomous
  motion, or any motion-capable calibration. Read-only diagnostics, logs,
  builds, and non-actuating ROS inspection are allowed.
- Access the Jetson with `ssh pmjet1` when needed. Do not request, print,
  store, or alter SSH passwords or private keys.

## Session records

For each completed user request, save one UTF-8 record under
`notes/codex_sessions/` named:

    YYYYMMDD_HHMMSS_<brief-description>.md

Include only the relevant user prompt(s), verbatim, and the exact final
response. Do not include internal reasoning or raw tool output; do not create a
record for an aborted request with no final response.
