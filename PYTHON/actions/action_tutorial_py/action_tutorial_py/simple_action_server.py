#  *
#  * @file simple_action_server.py
#  *
#  * @brief A basic ROS2 action server node that concatenates a string 
#  *         based on the number of string concatenation sent by a client within a goal request
#  *
#  * @author Antonio Mauro Galiano
#  * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
#  *
#

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse
from custom_action.action import Concatenate
import threading


class SimpleActionServer(Node):
    """Simple action server node."""

    def __init__(self):
        super().__init__('simple_action_server')
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
            return GoalResponse.REJECT
        # In ROS 2 Jazzy Python, use ACCEPT instead of ACCEPT_AND_EXECUTE
        return GoalResponse.ACCEPT

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
                return result
            
            # Update the final concatenation
            concatenation = concatenation + " " + my_string
            # Update and publish feedback of the partial concatenation
            feedback_msg.partial_concatenation = concatenation
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info('Publish Feedback')
            
            rclpy.spin_once(self, timeout_sec=1.0)
        
        # Check if goal is done
        result.final_concatenation = concatenation
        if rclpy.ok():
            goal_handle.succeed()
            self.get_logger().info('Goal Succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = SimpleActionServer()
    
    # Create an action server with three callbacks
    # 'handle_goal' and 'handle_cancel' are called by the Executor (rclpy.spin)
    # 'execute' is called whenever 'handle_goal' returns by accepting a goal
    # Note: In Python, the execute callback runs in a separate thread automatically
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        if rclpy.ok():
            action_server.get_logger().info("Shutting down action server...")
    finally:
        action_server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

