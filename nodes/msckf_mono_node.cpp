#include <ros/ros.h>
#include <msckf_mono/ros_interface.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "msckf_mono_node");
  msckf_mono::RosInterface ri();
  ros::spin();
}
