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
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
