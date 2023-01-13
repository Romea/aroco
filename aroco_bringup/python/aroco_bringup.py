# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
# Add license

from ament_index_python.packages import get_package_share_directory
import aroco_description


def urdf_description(prefix, mode):

    controller_manager_yaml_file = (
        get_package_share_directory("aroco_bringup") + "/config/controller_manager.yaml"
    )

    return aroco_description.urdf(prefix, mode, controller_manager_yaml_file)
