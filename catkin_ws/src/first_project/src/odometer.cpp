#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include <math.h>
#include <tf/transform_datatypes.h>

// Vehicle parameters
static const double WHEELBASE = 1.3;  // Rear wheels baseline (in meters)
static const double VEHICLE_LENGTH = 1.765;  // Distance from front to rear wheels (in meters)
static const double STEERING_FACTOR = 32;  // Steering factor (adjusted per vehicle)

class Odometer
{
public:
  Odometer()
  {
    // Initialize state variables
    x_ = 0.0; 
    y_ = 0.0; 
    theta_ = 0.0;
    last_time_ = ros::Time::now();

    // Publisher for odometry
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

    // Subscriber to vehicle status on "/speedsteer"
    sub_ = nh_.subscribe("/speedsteer", 10, &Odometer::speedsteerCallback, this);
  }

  void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
  {
    ros::Time current_time = msg->header.stamp;
    double dt = (current_time - last_time_).toSec();
    // if (dt <= 0.0) {
    //   // In case of timing issues, skip integration
    //   dt = 0.01;
    // }
    last_time_ = current_time;

    // Extract vehicle status:
    // x: steering angle in deg, y: speed in km/h
    double steer_deg = msg->point.x;
    double steer_rad = steer_deg * M_PI / 180.0;  // Convert to radians
    double speed_kmh = msg->point.y;
    // Convert km/h to m/s
    double speed = speed_kmh / 3.6;

    // Calculate the turning radius using the steering factor
    // This is a simplified model: Steering angle (steer_rad) * steering factor gives turning radius.
    double turning_radius = VEHICLE_LENGTH / tan(steer_rad) * STEERING_FACTOR;

    // Calculate the rate of change of the vehicle's position and orientation (kinematic equations)
    double angular_velocity = speed / turning_radius;  // In radians per second

    // Update the pose (x, y, theta) by integrating over time
    x_ += speed * cos(theta_) * dt;
    y_ += speed * sin(theta_) * dt;
    theta_ += angular_velocity * dt;

    // Prepare the Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "vehicle";

    // Set position from integration
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    // Convert orientation (theta) to a quaternion
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
    odom.pose.pose.orientation = odom_quat;

    // Publish velocity info in the twist message
    odom.twist.twist.linear.x = speed;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.z = angular_velocity;
    // Other twist components remain zero
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;

    odom_pub_.publish(odom);

    ROS_INFO("HERE!!!!!!");
    // PRINT ODOM NAV_MSG
    ROS_INFO("Position: (%.2f, %.2f), Orientation: (%.2f, %.2f, %.2f, %.2f)",
             odom.pose.pose.position.x,
             odom.pose.pose.position.y,
             odom.pose.pose.orientation.x,
             odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w);
    
    // Also broadcast the transform over tf from "odom" to "vehicle"
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "vehicle";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    tf_broadcaster_.sendTransform(odom_trans);

    // Log the computed odometry for debugging
    ROS_INFO("Speed: %.2f m/s, Steer: %.2f deg, Pos: (%.2f, %.2f), Theta: %.2f rad",
             speed, steer_deg, x_, y_, theta_);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Vehicle state variables
  double x_, y_, theta_;
  ros::Time last_time_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometer");
  Odometer odom_node;
  ros::spin();
  return 0;
}
