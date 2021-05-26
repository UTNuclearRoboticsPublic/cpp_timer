#pragma once

#include <proprietary_template/ExampleSrv.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>

namespace nrg_ros_samples {
class ExampleSub {
public:
  ExampleSub(ros::NodeHandle &nh);

  std::string getLastMsg();

private:
  ros::NodeHandle nh_;

  ros::Subscriber sub_;
  ros::ServiceServer server_;

  mutable std::mutex callback_mutex_;
  std_msgs::StringConstPtr last_msg_;

  /** \brief Callback for subscriber */
  void subscriberCB(const std_msgs::StringConstPtr &msg);

  /** \brief Callback for service */
  bool serverCB(proprietary_template::ExampleSrv::Request &req,
                proprietary_template::ExampleSrv::Response &res);
};
} // namespace nrg_ros_samples
