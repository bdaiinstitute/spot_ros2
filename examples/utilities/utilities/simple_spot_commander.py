# Copyright [2023] Boston Dynamics AI Institute, Inc.

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

TRIGGER_SERVICES = [
    'claim', 'release', 'stop', 'self_right', 'sit', 'stand', 'power_on', 'power_off', 'estop/hard', 'estop/gentle',
    'estop/release'
]


class SimpleSpotCommander(Node):

    def __init__(self, namespace=None):
        super().__init__('spot_commander', namespace=namespace)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._command_map = {}
        for service in TRIGGER_SERVICES:
            self._command_map[service] = self.create_client(Trigger, service)
            self.get_logger().info('Waiting for service ' + str(service))
            self._command_map[service].wait_for_service()
            self.get_logger().info('Found service ' + str(service))

    def command(self, command):
        try:
            self._future = self._command_map[command].call_async(Trigger.Request())
        except KeyError:
            err = 'No command ' + str(command)
            self.get_logger().error(err)
            return Trigger.Response(success=False, message=err)
        self._executor.spin_until_future_complete(self._future)
        return self._future.result()


def main():
    rclpy.init()
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("robot", help="Name of the robot if the ROS driver is inside that namespace")
    args = parser.parse_args()

    commander = SpotCommander(args.robot)

    while rclpy.ok():
        cmd = input('Please enter a command:\n' + ' '.join(TRIGGER_SERVICES) + '\n> ')
        result = commander.command(cmd)
        if not result.success:
            print('Error was', result.message)
        else:
            print('Successfully executed command')


if __name__ == '__main__':
    main()
