/**
 * @file message_sync.cpp
 *
 * @brief ROS2 node to synchronize messages
 *        In this case I choosed a PCL and a pure camera info messages, but it can be use
 *        for every type of message and more than two
 *
 * @author Antonio Mauro Galiano
 * Contact: https://www.linkedin.com/in/antoniomaurogaliano/
 *
 */


// This first line is necessary if you are going to use PCL library within your code
// otherwise you will get a "reference to 'placeholders' is ambiguous" error due to the BOOST 
// used by PCL. Otherwise you should use lambda functions
//#define BOOST_BIND_NO_PLACEHOLDERS

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using std::placeholders::_1;
using std::placeholders::_2;

class MessageSync : public rclcpp::Node{

  public:
    MessageSync() : Node("ros2_message_sync")
    { 
      // If one of the messages is published from bag files is better to allign the timestamp,
      // otherwise the synchronizer callback will never called
      // Maybe there is o will be available some other method using the clock or the time sim, but I actually
      // didn't find anything useful
      pubPclMod_ = this->create_publisher<PointCloud2Type>("/laser/pcl_mod", 10);
      subPcl_ = this->create_subscription<PointCloud2Type>(
          "/laser/pcl", 10, std::bind(&MessageSync::pclTopicCallback,
          this, std::placeholders::_1));    

      // A best practice would be the one where you look at the qos of the topics make them consistent
      // so it could be necessary to manage the qos
      // These lines are just an example
      // rclcpp::QoS qos(10);
      // auto rmw_qos_profile = qos.get_rmw_qos_profile();
      // pclSubFilter_.subscribe(this, "/livox/lidar_mod", rmw_qos_profile );

      // The two subscribers for the topics that must be synchronized
      pclSubFilter_.subscribe(this, "/laser/pcl_mod");
      cameraInfoSubFilter_.subscribe(this, "/camera/camera_info");

      // Here is initialized the synchronizer registering the callback to synchronize the messages
      sync.reset(new Sync(MySyncPolicy(10),pclSubFilter_,cameraInfoSubFilter_));
      sync->registerCallback(std::bind(&MessageSync::syncMessageCallback, 
                this, _1, _2));

    }

  private:
    typedef sensor_msgs::msg::PointCloud2 PointCloud2Type;
    typedef sensor_msgs::msg::CameraInfo CameraInfoType;

    // Here is defined the synchronization policy, in this case is the approximate time
    typedef message_filters::sync_policies::ApproximateTime<PointCloud2Type, CameraInfoType> MySyncPolicy;
    // The synchronizer based on the chosen policy
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    // The synchronizer pointer
    std::shared_ptr<Sync> sync;

    rclcpp::Publisher<PointCloud2Type>::SharedPtr pubPclMod_;
    rclcpp::Subscription<PointCloud2Type>::SharedPtr subPcl_;

    message_filters::Subscriber<PointCloud2Type> pclSubFilter_;
    message_filters::Subscriber<CameraInfoType> cameraInfoSubFilter_;

    void pclTopicCallback(const PointCloud2Type::ConstSharedPtr msg);
    void syncMessageCallback(
        const PointCloud2Type::ConstSharedPtr& pcl,
        const CameraInfoType::ConstSharedPtr& camInfo);
};


// Callback to sub the topic from the bag file and allign the time stamp 
void MessageSync::pclTopicCallback(const PointCloud2Type::ConstSharedPtr msg)
  {
    PointCloud2Type newMsg;

    newMsg = *msg;
    newMsg.header.stamp = rclcpp::Clock().now();
    
    RCLCPP_INFO(this->get_logger(),"Received pcl and published with clock().now()");
    pubPclMod_->publish(newMsg);
  }

// Synchronization callback
 void MessageSync::syncMessageCallback(
      const PointCloud2Type::ConstSharedPtr& pcl,
      const CameraInfoType::ConstSharedPtr& camInfo)
    {
      RCLCPP_INFO_STREAM(this->get_logger(), "Message sync time pcl=" << pcl->header.stamp.sec << " camera_info=" << camInfo->header.stamp.sec);
    } 


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MessageSync>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
