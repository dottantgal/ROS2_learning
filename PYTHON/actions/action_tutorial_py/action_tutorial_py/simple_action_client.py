#  *
#  * @file simple_action_client.py
#  *
#  * @brief A basic ROS2 action client node that sends as goal the number of string concatenation
#  *        the action server should perform. 
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


def main(args=None):
    rclpy.init(args=args)
    client_node = Node('simple_action_client')
    
    # Action client definition
    # Be sure to set the same action name used for the server
    action_client = ActionClient(client_node, Concatenate, 'concatenation')
    
    # Waiting for the server online response
    if not action_client.wait_for_server(timeout_sec=10.0):
        client_node.get_logger().error('!!ATTENTION!! Action server not available')
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    
    # Creation of the goal message
    goal_msg = Concatenate.Goal()
    goal_msg.num_concatenations = 9  # The number of concatenation to perform
    
    client_node.get_logger().info('Sending goal')
    
    # Create feedback callback as a closure to capture the node
    def feedback_callback(feedback):
        """
        The callback for the feedbacks.
        The server sends back the partial sequence of the string concatenation.
        """
        client_node.get_logger().info(f'Feedback received: {feedback.feedback.partial_concatenation}')
    
    # Ask server to achieve some goal and wait until it's accepted
    # Here some options such as feedbacks are set
    send_goal_future = action_client.send_goal_async(
        goal_msg,
        feedback_callback=feedback_callback
    )
    
    # The goal is sent and the server answers, it's necessary a SUCCESS to continue the communication
    try:
        rclpy.spin_until_future_complete(client_node, send_goal_future)
    except KeyboardInterrupt:
        if rclpy.ok():
            client_node.get_logger().info("Interrupted while waiting for goal response...")
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    
    goal_handle = send_goal_future.result()
    if not goal_handle:
        client_node.get_logger().error('send goal call failed :(')
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    
    # The server elaborates the goal and decide to reject or not the goal
    if not goal_handle.accepted:
        client_node.get_logger().error('Goal was rejected by server')
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    
    client_node.get_logger().info('Goal accepted by server, waiting for result')
    
    # Wait for the server to be done with the goal
    result_future = goal_handle.get_result_async()
    client_node.get_logger().info('Waiting for result')
    try:
        rclpy.spin_until_future_complete(client_node, result_future)
    except KeyboardInterrupt:
        if rclpy.ok():
            client_node.get_logger().info("Interrupted while waiting for result...")
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    
    # The clients gets the final result from the server
    wrapped_result = result_future.result()
    if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
        client_node.get_logger().info('concatenation received')
        client_node.get_logger().info(f'{wrapped_result.result.final_concatenation}')
    elif wrapped_result.status == GoalStatus.STATUS_ABORTED:
        client_node.get_logger().error('Goal was aborted')
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    elif wrapped_result.status == GoalStatus.STATUS_CANCELED:
        client_node.get_logger().error('Goal was canceled')
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    else:
        client_node.get_logger().error('Unknown result code')
        client_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return 1
    
    client_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()

