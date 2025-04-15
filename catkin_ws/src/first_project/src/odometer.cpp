#include "ros/ros.h"
#include "std_msgs/String.h" // Replace with the actual message type of your topic
#include "geometry_msgs/PointStamped.h"

void messageCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    ROS_INFO("Received Point: x=%f, y=%f, z=%f",
        msg->point.x, msg->point.y, msg->point.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("speedsteer", 10, messageCallback);

  ros::spin();

  return 0;
}
