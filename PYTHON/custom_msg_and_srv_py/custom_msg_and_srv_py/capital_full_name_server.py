#!/usr/bin/env python3
"""Service node that returns the capitalised full name."""

import rclpy
from rclpy.node import Node

from custom_msg_and_srv_py.formatting import build_capital_full_name
from custom_msg_and_srv.srv import CapitalFullName


class CapitalFullNameServer(Node):
    """Provide a service that returns an upper-case full name."""

    def __init__(self) -> None:
        super().__init__('capital_full_name_server')
        self.srv = self.create_service(
            CapitalFullName, 'capitalize_full_name', self.handle_capital_full_name)
        self.get_logger().info('CapitalFullName service ready on "capitalize_full_name".')

    def handle_capital_full_name(
        self, request: CapitalFullName.Request, response: CapitalFullName.Response
    ) -> CapitalFullName.Response:
        _, _, capital_full_name = build_capital_full_name(request.name, request.surname)
        response.capitalfullname = capital_full_name
        self.get_logger().info(
            f'Request: name="{request.name}" surname="{request.surname}" -> "{response.capitalfullname}"'
        )
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CapitalFullNameServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down CapitalFullNameServer.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
