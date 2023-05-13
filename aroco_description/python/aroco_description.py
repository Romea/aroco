#!/usr/bin/env python3

import xacro

from ament_index_python.packages import get_package_share_directory


def urdf(prefix, mode, controller_conf_yaml_file, ros_prefix):

    xacro_file = (
        get_package_share_directory("aroco_description") + "/urdf/aroco.urdf.xacro"
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "prefix": prefix,
            "mode": mode,
            "controller_conf_yaml_file": controller_conf_yaml_file,
            "ros_prefix": ros_prefix
        },
    )

    return urdf_xml.toprettyxml()
