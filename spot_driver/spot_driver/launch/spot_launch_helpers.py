# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import logging
import os
from typing import Optional, Tuple

import yaml

from spot_wrapper.wrapper import SpotWrapper


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

    # parse the yaml to determine if login information is set there
    if os.path.isfile(config_file_path):
        with open(config_file_path, "r") as config_yaml:
            try:
                config_dict = yaml.safe_load(config_yaml)
                if ("/**" in config_dict) and ("ros__parameters" in config_dict["/**"]):
                    ros_params = config_dict["/**"]["ros__parameters"]
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
            except yaml.YAMLError as exc:
                print("Parsing config_file yaml failed with: {}".format(exc))
    if (not username) or (not password) or (not hostname):
        raise ValueError(
            "One or more of your login credentials has not been specified! Got invalid values of "
            "[Username: '{}' Password: '{}' Hostname: '{}']. Ensure that your environment variables are set or "
            "update your config_file yaml.".format(username, password, hostname)
        )
    return username, password, hostname, port, certificate


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
