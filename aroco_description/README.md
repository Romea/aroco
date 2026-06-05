# aroco_description

## 1) Overview

`aroco_description` provides the robot-specific description layer for the Aroco mobile base.

It extends `romea_mobile_base_description` with the concrete configuration, URDF/Xacro files, meshes and `ros2_control` descriptions required to instantiate the Aroco robot.

The Aroco mobile base uses:

| Configuration file | Mobile base architecture | Command type |
|---|---|---|
| `config/aroco.yaml` | `2AS4WD` | `two_axle_steering` |

This architecture has front and rear axle steering and four driving wheels. It is controlled as a two-axle-steering mobile base.

## 2) Robot configuration

The `config/` directory contains:

| File | Purpose |
|---|---|
| `aroco.yaml` | full Aroco mobile base configuration |
| `teleop.yaml` | default two-axle-steering teleoperation configuration |

The robot configuration follows the structure defined by `romea_mobile_base_description` and contains:

* the mobile base architecture (`2AS4WD`);
* geometry, wheel dimensions and chassis bounding box;
* front and rear axle steering command and feedback information;
* wheel speed command and feedback information;
* inertia and control point;
* link and joint names used in the URDF and `ros2_control` descriptions.

## 3) URDF and ros2_control descriptions

The URDF description is built from:

| Path | Role |
|---|---|
| `urdf/aroco.urdf.xacro` | main URDF entry point |
| `urdf/aroco.xacro` | Aroco mobile base macro |
| `urdf/aroco.simulation.xacro` | simulator-specific Gazebo or Gazebo Classic plugin insertion |
| `urdf/visual/` | visual Xacro fragments for chassis, wheels and arms |
| `meshes/` | Aroco chassis and wheel meshes |

The Aroco macro reuses the `base2ASxxx.chassis.xacro` template from `romea_mobile_base_description` and specializes it with Aroco geometry, link names, joint names and visual meshes.

The `ros2_control` description is built from:

| Path | Role |
|---|---|
| `ros2_control/aroco.ros2_control.urdf.xacro` | main `ros2_control` entry point |
| `ros2_control/aroco.ros2_control.xacro` | Aroco `ros2_control` macro |

Depending on the selected mode, the `ros2_control` description selects:

| Mode | Hardware plugin |
|---|---|
| `live` | `aroco_hardware/ArocoHardware` |
| `simulation`, `simulation_gazebo` | `romea_mobile_base_gazebo/GazeboSystemInterface2AS4WD` |
| `simulation_gazebo_classic` | `romea_mobile_base_gazebo/GazeboSystemInterface2AS4WD` |

## 4) Python API

The installed Python module provides helper functions used by `aroco_bringup` and by the meta-bringup workflow.

| Function | Purpose |
|---|---|
| `get_specifications_path_file()` | returns the Aroco configuration file path |
| `get_specifications_configuration()` | loads the full robot configuration |
| `get_configuration()` | returns the compact mobile base configuration completed with manufacturer, model and version |
| `generate_configuration_file(configuration, extended)` | serializes the compact configuration |
| `generate_urdf_description(...)` | generates the Aroco URDF description |
| `generate_ros2_control_description(...)` | generates the Aroco `ros2_control` description |

Example:

```python
from aroco_description import get_configuration

configuration = get_configuration()
```

When `mode` is set to `simulation`, the Python API maps it to `simulation_gazebo` before generating the URDF or `ros2_control` description.

## 5) Relation with other packages

`aroco_description` is the Aroco specialization of `romea_mobile_base_description`:

* `romea_mobile_base_description` provides the generic `2AS4WD` description and `ros2_control` templates;
* `aroco_description` provides the Aroco configuration, meshes and Xacro specialization;
* `aroco_hardware` provides the live `ros2_control` hardware plugin;
* `aroco_bringup` uses this package to generate configuration, URDF and `ros2_control` artifacts for live and simulation modes.
