import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

TRIGGER_SERVICES = ['claim', 'release', 'stop', 'self_right', 'sit', 'stand', 'power_on',
                    'power_off', 'estop/hard', 'estop/gentle', 'estop/release']


class SpotCommander(object):

    def __init__(self, node):
        self._node = node
        self._command_map = {}
        for service in TRIGGER_SERVICES:
            self._command_map[service] = node.create_client(Trigger, service)
            self.get_logger().info('Waiting for service ' + str(service))
            self._command_map[service].wait_for_service()
            self.get_logger().info('Found service ' + str(service))

    def get_logger(self):
        return self._node.get_logger()

    def command(self, command):
        try:
            self._future = self._command_map[command].call_async(Trigger.Request())
        except KeyError:
            err = 'No command ' + str(command)
            self.get_logger().error(err)
            return Trigger.Response(success=False, message=err)
        rclpy.spin_until_future_complete(self._node, self._future)
        return self._future.result()


def main():
    rclpy.init()
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("robot",
                        help="Name of the robot if the ROS driver is inside that namespace")
    args = parser.parse_args()

    node = Node('spot_commander', namespace=args.robot)
    commander = SpotCommander(node)

    while rclpy.ok():
        cmd = input('Please enter a command:\n' + ' '.join(TRIGGER_SERVICES) + '\n> ')
        result = commander.command(cmd)
        if not result.success:
            print('Error was', result.message)
        else:
            print('Successfully executed command')


if __name__ == '__main__':
    main()
