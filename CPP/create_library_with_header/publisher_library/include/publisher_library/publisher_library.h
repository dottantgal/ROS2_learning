/**
 * @file publisher_library.h
 *
 * @brief Header file for the publisher library
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */

#ifndef PUBLISHER_LIBRARY_H
#define PUBLISHER_LIBRARY_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherLibrary : public rclcpp::Node
{
public:
  PublisherLibrary();
  ~PublisherLibrary();

private:
  size_t counter_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  void TimerCallback();

};

#endif