#!/usr/bin/env python3
from tkinter import messagebox
from typing import Any, List, Optional

import synchros2.process as ros_process
from rclpy.parameter import Parameter
from rclpy.qos import QoSPresetProfiles
from synchros2.node import Node

from spot_msgs.msg import (  # type: ignore
    BatteryStateArray,
)


class SpotAlerts(Node):
    """Monitors spot states and will generate a pop-up message to alert the user of warnings"""

    def __init__(self, **kwargs: Any) -> None:
        super().__init__("spot_alerts", **kwargs)
        # Subscribers #
        self.battery_states = self.create_subscription(
            BatteryStateArray, "status/battery_states", self.battery_callback, QoSPresetProfiles.SENSOR_DATA.value
        )

        # Parameters #
        self.declare_parameter("low_battery", False)

    def battery_callback(self, battery_array_msg: BatteryStateArray) -> None:
        """A callback function to check the battery percentage and send a pop-up alert when <= 10%
        Args:
            battery_array_msg: BatteryStateArray message received from the publisher containing BatteryState messages
        Returns:
            None
        """
        for battery_msg in battery_array_msg.battery_states:
            if battery_msg.charge_percentage <= 10 and not self.get_parameter("low_battery").value:
                low_battery_param = Parameter("low_battery", Parameter.Type.BOOL, True)
                self.set_parameters([low_battery_param])
                messagebox.showwarning(
                    title="Warning: Low Battery {}".format(battery_msg.identifier),
                    message=(
                        "Battery is at {} %. Approximately {} minutes remaining.\n Please charge your Spot soon."
                    ).format(battery_msg.charge_percentage, round(battery_msg.estimated_runtime.sec / 60)),
                )
            elif battery_msg.charge_percentage > 10 and self.get_parameter("low_battery").value:
                low_battery_param = Parameter("low_battery", Parameter.Type.BOOL, False)
                self.set_parameters([low_battery_param])


@ros_process.main(prebaked=False)
def main(args: Optional[List[str]] = None) -> None:
    ros_process.spin(SpotAlerts)


if __name__ == "__main__":
    main()
