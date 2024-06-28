# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import logging
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml

from spot_wrapper.wrapper import SpotWrapper


def get_ros_param_dict(config_file_path: str) -> Dict[str, Any]:
    """Get a dictionary of parameter_name: parameter_value from a ROS config yaml file.

    Args:
        config_file_path (str): Path to the config yaml.

    Raises:
        FileNotFoundError: If the path to the config file doesn't exist.
        KeyError, YAMLError: If your yaml is formatted incorrectly

    Returns:
        dict[str, Any]: dictionary of parameter_name: parameter_value.
    """
    if not config_file_path:
        return {}
    with open(config_file_path, "r") as config_yaml:
        config_dict = yaml.safe_load(config_yaml)
        ros_params = config_dict["/**"]["ros__parameters"]
        return ros_params


def get_login_parameters(config_file_path: str) -> Tuple[str, str, str, Optional[int], Optional[str]]:
    """Obtain the username, password, hostname, port, and certificate of Spot from the environment variables or,
    if they are not set, the configuration file yaml.

    Args:
        config_file_path (str): Path to the configuration yaml

    Raises:
        ValueError: If any of username, password, hostname is not set.

    Returns:
        Tuple[str, str, str, Optional[int], Optional[str]]: username, password, hostname, port, certificate
    """
    # Get value from environment variables
    username = os.getenv("BOSDYN_CLIENT_USERNAME")
    password = os.getenv("BOSDYN_CLIENT_PASSWORD")
    hostname = os.getenv("SPOT_IP")
    portnum = os.getenv("SPOT_PORT")
    port = int(portnum) if portnum else None
    certificate = os.getenv("SPOT_CERTIFICATE")

    ros_params = get_ros_param_dict(config_file_path)
    # only set username/password/hostname if they were not already set as environment variables.
    if (not username) and ("username" in ros_params):
        username = ros_params["username"]
    if (not password) and ("password" in ros_params):
        password = ros_params["password"]
    if (not hostname) and ("hostname" in ros_params):
        hostname = ros_params["hostname"]
    if (not port) and ("port" in ros_params):
        port = ros_params["port"]
    if (not certificate) and ("certificate" in ros_params):
        certificate = ros_params["certificate"]
    if (not username) or (not password) or (not hostname):
        raise ValueError(
            "One or more of your login credentials has not been specified! Got invalid values of "
            "[Username: '{}' Password: '{}' Hostname: '{}']. Ensure that your environment variables are set or "
            "update your config_file yaml.".format(username, password, hostname)
        )
    return username, password, hostname, port, certificate


def default_camera_sources(has_arm: bool) -> List[str]:
    camera_sources = ["frontleft", "frontright", "left", "right", "back"]
    if has_arm:
        camera_sources.append("hand")
    return camera_sources


def get_camera_sources(config_file_path: str, has_arm: bool) -> List[str]:
    """Get the list of cameras to stream from. This will be taken from the config yaml if it exists and is correctly
    formatted, and if not, it will default to all available cameras.

    Args:
        config_file_path (str): Path to your configuration yaml.
        has_arm (bool): Whether or not your Spot has an arm.

    Returns:
        List[str]: List of cameras the driver will stream from.
    """
    default_sources = default_camera_sources(has_arm)
    ros_params = get_ros_param_dict(config_file_path)
    if "cameras_used" in ros_params:
        camera_sources = ros_params["cameras_used"]
        if isinstance(camera_sources, List):
            if "hand" in camera_sources and not has_arm:
                print(
                    f'Selected camera sources {camera_sources} contains "hand", but your robot doesn\'t have an arm --'
                    " removing this from your camera sources"
                )
                camera_sources.remove("hand")
            return camera_sources
        else:
            print(
                f"Inputted camera sources {camera_sources} is not correctly formatted as a list! Defaulting to all"
                " cameras enabled."
            )
            return default_sources
    else:
        return default_sources


def spot_has_arm(config_file_path: str, spot_name: str) -> bool:
    """Check if Spot has an arm querying the robot through SpotWrapper

    Args:
        config_file_path (str): Path to configuration yaml
        spot_name (str): Name of spot

    Returns:
        bool: True if spot has an arm, False otherwise
    """
    logger = logging.getLogger("spot_driver_launch")
    username, password, hostname, port, certificate = get_login_parameters(config_file_path)
    spot_wrapper = SpotWrapper(
        username=username,
        password=password,
        hostname=hostname,
        port=port,
        cert_resource_glob=certificate,
        robot_name=spot_name,
        logger=logger,
    )
    return spot_wrapper.has_arm()
