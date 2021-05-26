#pragma once

#include <proprietary_template/ExampleSrv.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace nrg_ros_samples {
class ExamplePub {
public:
  ExamplePub(ros::NodeHandle &nh);

  const bool startPublishing(const double duration = 1);
  const bool stopPublishing();

  const bool sendServiceRequest(const std::string message);

private:
  ros::NodeHandle nh_;

  ros::Publisher pub_;
  ros::ServiceClient client_;

  ros::Timer timer_;
  ros::Time start_time_;

  void publishMessage(const ros::TimerEvent &timer_event) const;
};
} // namespace nrg_ros_samples
