/**
 * @file tf2_publisher_node.h
 *
 * @brief Node creation of the dynamic TF2 publisher
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


#include <dynamic_tf2_publisher/tf2_publisher.h>


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tf2Publisher>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception in spin: %s", e.what());
  }
  
  // Graceful shutdown: node will clean up timer and broadcaster automatically via RAII
  node.reset();
  rclcpp::shutdown();
  return 0;
}
