# ROS2 C++ learning
##### _A useful collection of rclcpp examples to dive into ROS2_
This repository has the goal to help the learning process of ROS2 basic, middle and advance features through a collection of example nodes ready to compile and use.
All the nodes have been tested with ROS2 Foxy [to be continued]
## Folders tree
All the folders cover a specific feature and there are their own CMakeLists.txt and package.xml files to compile them 
###### CPP
* 01 Start with simple nodes
    * my_first_node.cpp
    * node_with_class.cpp
    * node_timer_without_class.cpp
    * node_timer_with_class.cpp
* 02 Publisher and subscriber
    * simple_publisher_node.cpp
    * simple_subscriber_node.cpp
    * simple_publisher_class_node.cpp
    * simple_subscriber_class_node.cpp
    * sub_pub_pipeline.cpp
    * publish_custom_message.cpp
    * msg/EmployeeSalary.msg
* 03 Custom msg and srv
    * srv/CapitalFullName.srv
* 04 Service and client
    * service_node.cpp
    * service_node_class.cpp
    * client_node.cpp
    * client_node_class.cpp
* 05 Parameters
    * set_parameters.cpp

![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)
