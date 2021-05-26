#include <proprietary_template/example_pub.h>

namespace nrg_ros_samples {
ExamplePub::ExamplePub(ros::NodeHandle &nh) : nh_(nh) {
  // Set up publisher
  pub_ = nh_.advertise<std_msgs::String>("chatter", 1000);

  // Set up service client
  client_ = nh_.serviceClient<proprietary_template::ExampleSrv>("foo");

  start_time_ = ros::Time::now();
};

const bool ExamplePub::startPublishing(const double duration) {
  if (duration <= 0) {
    ROS_WARN_STREAM("Timer Duration must be greater than 0, passed value was: "
                    << duration);
    return false;
  }
  timer_ = nh_.createTimer(ros::Duration(duration), &ExamplePub::publishMessage,
                           this);
  return true;
}

const bool ExamplePub::stopPublishing() {
  timer_.stop();
  return true;
}

const bool ExamplePub::sendServiceRequest(const std::string message) {
  // Set up the service message
  proprietary_template::ExampleSrv srv;
  srv.request.request_string = message;

  // Call the service
  if (!client_.call(srv)) {
    ROS_ERROR_STREAM("No response from service");
    return false;
  }

  ROS_INFO_STREAM("Service returned: " << srv.response.response_string);
  return srv.response.success;
}

void ExamplePub::publishMessage(const ros::TimerEvent &timer_event) const {
  std_msgs::String msg;
  msg.data = std::string("Up time: ") +
             std::to_string((ros::Time::now() - start_time_).toSec()) +
             std::string(" seconds.");
  pub_.publish(msg);
}
} // namespace nrg_ros_samples
