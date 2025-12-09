"""Client node consuming the CapitalFullName service."""

import sys
from typing import Optional

import rclpy
from rclpy.node import Node

from custom_msg_and_srv_py.srv import CapitalFullName


class CapitalFullNameClient(Node):
    """Send requests to the CapitalFullName service."""

    def __init__(self) -> None:
        super().__init__('capital_full_name_client')
        self.client = self.create_client(CapitalFullName, 'capitalize_full_name')
        self._future: Optional[rclpy.task.Future] = None

    def send_request(self, name: str, surname: str) -> str:
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Service "capitalize_full_name" not available')

        request = CapitalFullName.Request()
        request.name = name
        request.surname = surname
        self._future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self._future)
        if self._future.result() is None:
            raise RuntimeError(f'Service call failed: {self._future.exception()}')
        return self._future.result().capitalfullname


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CapitalFullNameClient()
    try:
        name = sys.argv[1] if len(sys.argv) > 1 else 'john'
        surname = sys.argv[2] if len(sys.argv) > 2 else 'doe'
        capitalised = node.send_request(name, surname)
        node.get_logger().info('Capital full name: "%s"', capitalised)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error('Failed to complete request: %s', exc)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
