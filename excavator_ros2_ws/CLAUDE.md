# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
# Build all packages
source /opt/ros/humble/setup.bash
cd excavator_ros2_ws
colcon build

# Build single package
colcon build --packages-select excavator_ros2_bridge

# Source workspace after build
source install/setup.bash
```

## Runtime Commands

```bash
# Terminal 1: Launch bridge (controller_manager + MuJoCo physics + controllers)
ros2 launch excavator_ros2_bridge excavator_bringup.launch.py

# Terminal 2: Launch viewer (independent MuJoCo simulator + GLFW window)
ros2 run excavator_viewer mujoco_viewer

# Terminal 3: Launch auto digging (state machine)
ros2 run excavator_auto_dig auto_dig_node

# Terminal 4: Start auto digging cycle
ros2 service call /auto_dig/start std_srvs/srv/Trigger

# Manual velocity control
ros2 topic pub /velocity_controller/commands std_msgs/Float64MultiArray "{data: [0.0, 0.0, 0.3, 0.0]}"
```

## Architecture

This is a ROS2 Humble workspace that wraps an existing MuJoCo hydraulic excavator simulator (at `../excavator_simulator_mujoco/`) with ros2_control. The original project is **not modified**.

### Key Design Decision

**Bridge and viewer are independent MuJoCo simulators.** They subscribe to the same `/velocity_controller/commands` topic and each run their own `mj_step()` physics. They do NOT synchronize state with each other. This is critical — do not try to make the viewer track the bridge's joint positions.

### Package Overview

**excavator_ros2_bridge** — ros2_control hardware interface that drives MuJoCo physics
- `MujocoWrapper`: C API wrapper for mj_loadXML, mj_step, qpos/ctrl read/write
- `ExcavatorHardwareInterface`: `hardware_interface::SystemInterface` plugin. `read()` calls `mj_step()`, `write()` sets `d->ctrl` from velocity commands
- `bridge_node_main.cpp`: Hosts `controller_manager` which runs the 100Hz update loop
- Controllers: `joint_state_broadcaster` (publishes `/joint_states`), `velocity_controller` (subscribes to `/velocity_controller/commands`)

**excavator_viewer** — Standalone GLFW viewer running its own MuJoCo physics
- Subscribes to `/velocity_controller/commands` (NOT `/joint_states`)
- Sets `d->ctrl` from received commands, calls `mj_step()`, renders
- Handles soil plugin hfield GPU upload after soil deformation
- Wireframe rendering: `scn.flags[mjRND_WIREFRAME] = 1`

**excavator_auto_dig** — Auto digging state machine
- Publishes to `/velocity_controller/commands`
- Subscribes to `/joint_states` for position feedback
- States: IDLE → APPROACH → DIG → LIFT → SWING → DUMP → RETURN (loops)

**excavator_simulate_ros2** — Original MuJoCo Simulate viewer with ROS2 integration (reference implementation)

### MuJoCo Model Specifics

- Model file: `config/excavator_control.xml` (copied from original, paths adjusted)
- Hydraulic excavator: 4 velocity actuators driving piston slide joints
- Equality `<connect>` constraints convert piston linear motion to arm rotation
- **`mj_step()` is required** to resolve equality constraints — `mj_forward()` does NOT work
- Soil plugin (`mujoco.soil`) deforms terrain hfield; must call `mjr_uploadHField()` after update
- Joint mapping: Rotation→chassis, Boom→chassis piston rod, Arm→boom piston rod, Bucket→arm piston rod
- Actuator kv gains: Rotation/Boom=700000, Arm/Bucket=100000

### Critical Paths

- MuJoCo headers/libs: `../excavator_simulator_mujoco/build/_deps/mujoco-src/include/` and `build/lib/`
- Soil plugin: `../excavator_simulator_mujoco/build/bin/mujoco_plugin/`
- Mesh/texture: `../excavator_simulator_mujoco/model/excavator/mesh/` and `texture/`

## Common Pitfalls

- Never use `mj_forward()` for physics — it skips constraint solving, equality constraints won't work
- `mj_step()` with zero `d->ctrl` causes collapse under gravity — actuators must provide force
- The viewer and bridge are separate MuJoCo instances — do not try to sync their qpos
- Slide joints have `ref` attribute — `qpos=0` means actual position = ref value, not zero
- `scn.flags[mjRND_WIREFRAME]` not `opt.flags` — rendering flags are on the scene, not options
