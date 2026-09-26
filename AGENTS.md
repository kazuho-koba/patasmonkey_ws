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
- Add deliberate, useful comments to new or materially changed code. Explain
  both non-obvious intent and the actual processing flow: inputs/units,
  filtering, coordinate transforms, intermediate values, branch/timeout
  conditions, and state updates. Also document algorithm assumptions,
  parameter/threshold semantics, data ownership or real-time performance
  choices, and safety-relevant behavior. Use module/class/function docstrings
  where they clarify an API or processing stage.
- README、docstring、コードコメントは日本語で記述する。ROS名、API名、変数名、
  数式、一般に定着した技術用語は英語表記を保ってよい。
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
- For a long-running ROS launch or rosbag recorder over SSH, keep the control
  session open until the remote launch and recorder have exited. Stop them with
  SIGINT on the remote process, wait for shutdown, and do not use local SSH
  session termination, broad `pkill`, or SIGKILL as the normal stop method.
- Before treating a recorded bag as complete, verify the recorder is stopped,
  `metadata.yaml` exists, and `ros2 bag info <bag-directory>` succeeds. For a
  detached launch, retain its exact remote PID and stop only that process tree.

## Session records

Group conversation records and reports by the exact session title by default.
For the ongoing local-planning/traversability workstream, use the stable topic
folder `perception` for both session records and reports. Use:

- `notes/codex_sessions/perception/YYYYMMDD_HHMMSS_<brief-description>.md`
- `notes/reports/perception/<report-name>.md`

Keep report figures and other required assets beside their report. Do not mix
records or reports from different sessions. These records, reports, and their
assets are local-only; do not add them to Git or publish them. For each completed
user request, save one record named:

    YYYYMMDD_HHMMSS_<brief-description>.md

Use the local creation time, including seconds, in the filename. If multiple
records are created within the same second, add a distinguishing suffix.

Include only the relevant user prompt(s), verbatim, and the exact final
response. Do not include internal reasoning or raw tool output; do not create a
record for an aborted request with no final response.
