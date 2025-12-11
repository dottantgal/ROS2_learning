#  *
#  * @file class_action_client.py
#  *
#  * @brief A class defined ROS2 action client node that sends as goal the number of string
#  *        concatenation the action server should perform. 
#  *        The server will send back feedbacks and the final result
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from custom_action.action import Concatenate


class ConcatenateActionClient(Node):
    """Class-based action client node."""

    def __init__(self):
        super().__init__('class_action_client')
        self._goal_done = False
        self._client = ActionClient(self, Concatenate, 'concatenation')
        self._timer = self.create_timer(0.5, self.send_goal)

    def goal_done(self):
        """Check if goal is done."""
        return self._goal_done

    def send_goal(self):
        """Send goal to the action server."""
        self._timer.cancel()
        self._goal_done = False

        if not self._client:
            self.get_logger().error('Action client not initialized')
            return

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('!!ATTENTION!! Action server not available')
            self._goal_done = True
            return

        goal_msg = Concatenate.Goal()
        goal_msg.num_concatenations = 9

        self.get_logger().info('Sending goal')

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback):
        """
        Feedback callback.
        """
        self.get_logger().info(
            f'Feedback received: {feedback.feedback.partial_concatenation}'
        )

    def goal_response_callback(self, future):
        """
        Goal response callback.
        """
        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('Goal was rejected by server')
            self._goal_done = True
            return

        self.get_logger().info('Goal accepted by server, waiting for result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """
        Result callback.
        """
        self._goal_done = True
        wrapped_result = future.result()
        
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Result received')
            self.get_logger().info(f'{wrapped_result.result.final_concatenation}')
        elif wrapped_result.status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal was aborted')
        elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().error('Goal was canceled')
        else:
            self.get_logger().error('Unknown result code')


def main(args=None):
    rclpy.init(args=args)
    action_client = ConcatenateActionClient()

    try:
        while not action_client.goal_done():
            rclpy.spin_once(action_client, timeout_sec=0.1)
    except KeyboardInterrupt:
        if rclpy.ok():
            action_client.get_logger().info("Interrupted while waiting for action result...")
    finally:
        action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()

