# aroco

## Overview

`aroco` groups the ROS2 packages that describe, launch and control the Aroco mobile base in live and simulation modes.

This repository-level README gives a map of the stack. Detailed information about each package can be found in the corresponding package README.

## Packages

| Package | Role |
| --- | --- |
| `aroco` | Metapackage that groups the Aroco ROS2 packages. |
| `aroco_description` | Robot-specific description layer for Aroco, including configuration files, URDF/Xacro descriptions, meshes and ros2_control descriptions. |
| `aroco_bringup` | Main integration entry point for generating Aroco configuration files, URDF descriptions, ros2_control descriptions and launch files. |
| `aroco_hardware` | Live `ros2_control` hardware plugin for the Aroco mobile base, built on the generic `2AS4WD` hardware abstraction. |

## CAN connection

- Setup can
  - `sudo ip link set can0 type can bitrate 500000`
  - `sudo ip link set can0 up`
- Test can : `candump can0`

## Usage

Quickstart on real robot : `ros2 launch aroco_bringup aroco_test.launch.py mode:=live`

In most cases, start with `aroco_bringup`. It is the user-facing entry point of the stack and the package used by `romea_mobile_base_meta_bringup` when an Aroco model is selected from a mobile base meta-description.

The Aroco stack is a robot-specific specialization of `romea_mobile_base`. The mobile base architecture is `2AS4WD`; `aroco_description` provides the concrete geometry and generated descriptions, `aroco_hardware` provides the live hardware implementation, and `aroco_bringup` connects these pieces to the generic mobile base launch workflow.

## License

This project is released under the Apache License 2.0. See the `LICENSE` file for details.

## Authors

The `aroco` project was developed by Jean Laneurit in the context of the BaudetRob ANR project.

## Contact

For questions or comments about this project, please contact [Jean Laneurit](mailto:jean.laneurit@inrae.fr).
