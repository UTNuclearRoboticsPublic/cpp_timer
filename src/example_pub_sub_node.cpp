#include <proprietary_template/example_pub.h>
#include <proprietary_template/example_sub.h>

#include <thread>

int main(int argc, char **argv) {
  // ROS Initialization
  ros::init(argc, argv, "classes_as_node");
  ros::NodeHandle nh;

  // Create and start the publisher
  nrg_ros_samples::ExamplePub class_publisher(nh);
  class_publisher.startPublishing(1);

  // Create the subscriber
  nrg_ros_samples::ExampleSub class_subscriber(nh);

  // In a new thread, use the publisher to call the service
  std::thread service_thread([&class_publisher] {
    ros::Duration(5).sleep();
    class_publisher.sendServiceRequest("Service request");
  });

  ros::spin();
  if (service_thread.joinable()) {
    service_thread.join();
  }
  return EXIT_SUCCESS;
}
