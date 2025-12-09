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
from custom_action.action import Concatenate


def feedback_callback(goal_handle, feedback):
    """
    The callback for the feedbacks.
    The server sends back the partial sequence of the string concatenation.
    """
    node = rclpy.get_node("simple_action_client")
    node.get_logger().info(f'Feedback received: {feedback.partial_concatenation}')


def main(args=None):
    rclpy.init(args=args)
    client_node = Node('simple_action_client')
    
    # Action client definition
    # Be sure to set the same action name used for the server
    action_client = ActionClient(client_node, Concatenate, 'concatenation')
    
    # Waiting for the server online response
    if not action_client.wait_for_server(timeout_sec=10.0):
        client_node.get_logger().error('!!ATTENTION!! Action server not available')
        return 1
    
    # Creation of the goal message
    goal_msg = Concatenate.Goal()
    goal_msg.num_concatenations = 9  # The number of concatenation to perform
    
    client_node.get_logger().info('Sending goal')
    
    # Ask server to achieve some goal and wait until it's accepted
    # Here some options such as feedbacks are set
    send_goal_future = action_client.send_goal_async(
        goal_msg,
        feedback_callback=feedback_callback
    )
    
    # The goal is sent and the server answers, it's necessary a SUCCESS to continue the communication
    rclpy.spin_until_future_complete(client_node, send_goal_future)
    
    goal_handle = send_goal_future.result()
    if not goal_handle:
        client_node.get_logger().error('send goal call failed :(')
        return 1
    
    # The server elaborates the goal and decide to reject or not the goal
    if not goal_handle.accepted:
        client_node.get_logger().error('Goal was rejected by server')
        return 1
    
    client_node.get_logger().info('Goal accepted by server, waiting for result')
    
    # Wait for the server to be done with the goal
    result_future = goal_handle.get_result_async()
    client_node.get_logger().info('Waiting for result')
    rclpy.spin_until_future_complete(client_node, result_future)
    
    # The clients gets the final result from the server
    wrapped_result = result_future.result()
    if wrapped_result.status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
        client_node.get_logger().info('concatenation received')
        client_node.get_logger().info(f'{wrapped_result.result.final_concatenation}')
    elif wrapped_result.status == rclpy.action.GoalStatus.STATUS_ABORTED:
        client_node.get_logger().error('Goal was aborted')
        return 1
    elif wrapped_result.status == rclpy.action.GoalStatus.STATUS_CANCELED:
        client_node.get_logger().error('Goal was canceled')
        return 1
    else:
        client_node.get_logger().error('Unknown result code')
        return 1
    
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()

