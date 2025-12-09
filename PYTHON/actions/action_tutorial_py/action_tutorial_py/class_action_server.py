#  *
#  * @file class_action_server.py
#  *
#  * @brief A class defined ROS2 action server node that concatenates a string 
#  *         based on the number of string concatenation sent by a client within a goal request
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from custom_action.action import Concatenate


class ConcatenateActionServer(Node):
    """Class-based action server node."""

    def __init__(self):
        super().__init__('class_action_server')
        self._action_server = ActionServer(
            self,
            Concatenate,
            'concatenation',
            self.execute_callback,
            goal_callback=self.handle_goal,
            cancel_callback=self.handle_cancel
        )

    def handle_goal(self, goal_request):
        """
        Response to the client goal request.
        """
        self.get_logger().info(
            f'Got goal request with {goal_request.num_concatenations} string concatenations'
        )
        # Conditional to reject numbers of concatenations
        if (goal_request.num_concatenations > 10) and (goal_request.num_concatenations < 2):
            return rclpy.action.GoalResponse.REJECT
        return rclpy.action.GoalResponse.ACCEPT_AND_EXECUTE

    def handle_cancel(self, goal_handle):
        """
        Response to cancel requests from the client.
        """
        self.get_logger().info('Got request to cancel goal')
        return rclpy.action.CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        The goal is executed.
        """
        self.get_logger().info('Executing the concatenation')
        
        goal = goal_handle.request
        feedback_msg = Concatenate.Feedback()
        my_string = "HELLOWORLD"
        concatenation = my_string
        concatenation = concatenation + " " + my_string
        result = Concatenate.Result()
        
        for i in range(1, goal.num_concatenations):
            if not rclpy.ok():
                break
            
            # Check if there is a cancel request
            if goal_handle.is_cancel_requested:
                result.final_concatenation = concatenation
                goal_handle.canceled()
                self.get_logger().info('Goal Canceled')
                return
            
            # Update the final concatenation
            concatenation = concatenation + " " + my_string
            # Update and publish feedback of the partial concatenation
            feedback_msg.partial_concatenation = concatenation
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Publish Feedback')
            
            rclpy.spin_once(self, timeout_sec=1.0)
        
        # Check if goal is done
        if rclpy.ok():
            result.final_concatenation = concatenation
            goal_handle.succeed()
            self.get_logger().info('Goal Succeeded')


def main(args=None):
    rclpy.init(args=args)
    action_server = ConcatenateActionServer()
    rclpy.spin(action_server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

