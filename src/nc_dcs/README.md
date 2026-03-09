# nc_dcs (Docking Control System)

Navifra

`nc_dcs` is a ROS1 Noetic package responsible for managing the docking and undocking operations of a forklift robot. It executes a sequence of actions (lift, move, perception, etc.) based on a configurable state machine.

## Overview

The system receives a mission trigger via the `/nc_task_manager/fork_docking` topic, selects the appropriate sequence based on the `n_drive_type`, and executes the defined steps. It handles communication with the perception system, path planner, and lift controller.

## Features

*   **YAML-Based Sequence Configuration**: Define complex operation sequences without code changes.
*   **Dynamic Sequence Selection**: Automatically selects sequences based on `n_drive_type`.
*   **Rack-Based Height Management**: Supports defining lift heights based on rack types and operations (approach, pickup, carry, etc.).
*   **Task Control**: Supports PAUSE, RESUME, and CANCEL commands.
*   **Parallel Actions**: Supports executing multiple actions simultaneously (e.g., lift, sideshift, tilt).
*   **FSM Architecture**: Robust state machine for managing execution flow.

## Architecture

### Subscribers

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/nc_task_manager/fork_docking` | `core_msgs::ForkLift` | Triggers a docking mission. Contains `n_drive_type`, `n_target_height`, etc. |
| `/nc_task_manager/task_cmd` | `std_msgs::String` | Control commands: `PAUSE`, `RESUME`, `CANCEL`. |
| `/perception/pose` | `geometry_msgs::PoseStamped` | Target pose received from the perception system. |
| `/path_plan/status` | `std_msgs::String` | Status updates from the path planner. |
| `/cheonil/read_register` | `core_msgs::CheonilReadRegister` | Current status of the lift (height, fork width, tilt). |
| `/localization/robot_pos` | `geometry_msgs::PoseWithCovarianceStamped` | Current robot pose. |
| `/navifra/live_path` | `move_msgs::CoreCommand` | Live path data (used for path following). |
| `/path_plan/docking_path` | `geometry_msgs::PoseArray` | Path plan data. |
| `/navifra/alarm` | `core_msgs::NaviAlarm` | Alarms, specifically `GOAL_ARRIVED` for move completion. |

### Publishers

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/perception/cmd` | `std_msgs::String` | JSON-formatted command to start/stop perception. |
| `/path_plan/cmd` | `dcs_msgs::DockingInfo` | Command for the path planner (docking target, goback). |
| `/path_plan/pose` | `geometry_msgs::PoseStamped` | Forwards the perception pose to the path planner. |
| `/forkinfo` | `core_msgs::WiaForkInfo` | Command to control the fork (height, width, tilt). |
| `/fork_lift_reached` | `std_msgs::Bool` | Publishes `true` when the sequence is completed, `false` if cancelled. |
| `/navifra/live_path` | `move_msgs::CoreCommand` | Forwards path plan data. |

### Services

*   **Client**: Calls `/path_plan/done` (`std_srvs::Trigger`) when a docking move is completed.

## Configuration

### 1. Sequences (`config/sequences.yaml`)

Defines the steps for each `drive_type`.

```yaml
sequences:
  unloading:
    drive_type: -1
    name: "UNLOADING"
    steps:
      - action_type: "fork_updown"
        params:
          height: 1500
        wait_for: "completed"
      - action_type: "perception"
        params:
          command: "start"
        wait_for: "completed"
      # ...
```

### 2. Fork Parameters (`config/fork_parameters.yaml`)

Defines height presets for different rack types.

```yaml
fork_parameters:
  rack_heights:
    rack_1:
      rack_type: 1
      name: "Standard Rack"
      heights:
        approach_height: 500
        pickup_height: 1500
        carry_height: 200
        place_height: 1550
        clearance_height: 1600
```

## Action Types

### `fork_updown`
Controls the lift height.
*   `height`: Absolute target height in mm.
*   `use_mission_height`: If `true`, uses `n_target_height` from the `ForkLift` message.
*   `use_rack_heights`: If `true`, calculates height based on `n_rack_type` and `operation`.
*   `operation`: Used with `use_rack_heights` (e.g., "approach", "pickup", "carry", "place", "clearance").

### `fork_side`
Controls the fork width (sideshift).
*   `position`: "wide" or "narrow".

### `fork_tilt`
Controls the fork tilt.
*   `direction`: "up" or "down".

### `move`
Controls robot movement.
*   `type`: "docking" (approach target) or "goback" (reverse).
*   `use_perception_pose`: If `true`, uses the pose from `/perception/pose`.
*   `continuous_publish`: If `true`, continuously updates the target.
*   `distance`: Distance for "goback" (0 uses default).

### `perception`
Controls the vision system.
*   `command`: Command string (e.g., "start", "stop").
*   `target_type`: Type of target (default: "pallet").

## Usage

### Launching

```bash
roslaunch nc_dcs docking_control.launch
```

### Testing

Trigger a mission manually:

```bash
rostopic pub /nc_task_manager/fork_docking core_msgs/ForkLift "{n_drive_type: 1, n_target_height: 1500}"
```

Control execution:

```bash
rostopic pub /nc_task_manager/task_cmd std_msgs/String "data: 'PAUSE'"
rostopic pub /nc_task_manager/task_cmd std_msgs/String "data: 'RESUME'"
rostopic pub /nc_task_manager/task_cmd std_msgs/String "data: 'CANCEL'"
```
