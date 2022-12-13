from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    joystick_type = LaunchConfiguration("joystick_type").perform(context)
    joystick_device = LaunchConfiguration("joystick_device").perform(context)

    actions = []

    if mode == "simulation":

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("gazebo_ros"),
                                "launch",
                                "gazebo.launch.py",
                            ]
                        )
                    ]
                )
            )
        )


    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("aroco_bringup"),
                            "launch",
                            "aroco_base.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={"mode": mode}.items(),
        )
    )

    actions.append(PushRosNamespace("aroco"))

    teleop_configuration_filename = (
        get_package_share_directory("aroco_description") + "/config/teleop.yaml"
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_teleop_drivers")
                + "/launch/teleop.launch.py"
            ),
            launch_arguments={
                "robot_type": "aroco",
                "joystick_type": joystick_type,
                "joystick_driver": "joy",
                "teleop_configuration_filename": teleop_configuration_filename,
            }.items(),
        )
    )

    actions.append(PushRosNamespace("joystick"))

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("romea_joystick_bringup"),
                            "launch",
                            "drivers/joy.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "device": joystick_device,
                "dead_zone": "0.05",
                "autorepeat_rate": "10.0",
                "frame_id": "joy",
            }.items(),
        )
    )


    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="simulation"))

    declared_arguments.append(DeclareLaunchArgument("joystick_type", default_value="xbox"))

    declared_arguments.append(
        DeclareLaunchArgument("joystick_device", default_value="/dev/input/js0")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )