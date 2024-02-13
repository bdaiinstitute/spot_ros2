from tkinter import messagebox

import bdai_ros2_wrappers.process as ros_process
from bdai_ros2_wrappers.node import Node
from rclpy.parameter import Parameter

from spot_msgs.msg import (  # type: ignore
    BatteryStateArray,
)


class SpotAlerts(Node):
    """Monitors spot states and will generate a pop-up message to alert the user of warnings"""

    def __init__(self) -> None:
        super().__init__("spot_alerts")
        # Subscribers #
        self.battery_states = self.create_subscription(
            BatteryStateArray, "status/battery_states", self.battery_callback
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
        for battery_msg in battery_array_msg:
            if battery_msg.charge_percentage <= 10 and not self.get_parameter("low_battery").value:
                low_battery_param = Parameter("low_battery", Parameter.Type.BOOL, True)
                self.set_parameters([low_battery_param])
                messagebox.showwarning(
                    title="Warning: Low Battery {}".format(battery_msg.identifier),
                    message=(
                        "Battery is at {} %. Approximately {} minutes remaining.\n Please charge your Spot soon."
                    ).format(battery_msg.charge_percentage, round(battery_msg.estimated_runtime.sec / 60)),
                )


@ros_process.main(prebaked=False)
def main() -> None:
    ros_process.spin(SpotAlerts)


if __name__ == "__main__":
    main()
