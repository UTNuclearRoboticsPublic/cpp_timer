#include <proprietary_template/example_sub.h>

namespace nrg_ros_samples {
ExampleSub::ExampleSub(ros::NodeHandle &nh) : nh_(nh) {
  // Set up subscriber
  sub_ = nh_.subscribe("chatter", 10, &ExampleSub::subscriberCB, this);

  // Set up service server
  server_ = nh_.advertiseService("foo", &ExampleSub::serverCB, this);
};

std::string ExampleSub::getLastMsg() {
  const std::lock_guard<std::mutex> lock(callback_mutex_);
  return last_msg_->data;
}

void ExampleSub::subscriberCB(const std_msgs::StringConstPtr &msg) {
  // Lock mutex before saving
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    last_msg_ = msg;
  }
  ROS_INFO_STREAM("Heard message: " << msg->data);
}

bool ExampleSub::serverCB(proprietary_template::ExampleSrv::Request &req,
                          proprietary_template::ExampleSrv::Response &res) {
  // Lock mutex before saving
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    std_msgs::String msg;
    msg.data = req.request_string;

    last_msg_ = boost::make_shared<const std_msgs::String>(msg);
  }
  ROS_INFO_STREAM("Got service message with request: " << req.request_string);
  res.success = true;
  res.response_string = "Received service request";
  return true;
}
} // namespace nrg_ros_samples
