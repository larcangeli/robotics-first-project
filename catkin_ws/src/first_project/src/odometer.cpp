// FIRST NODE: ODOMETER

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>
#include <math.h>

// Vehicle parameters
static const double WHEELBASE = 1.3;        // Rear wheels baseline (in meters)
static const double VEHICLE_LENGTH = 1.765; // Distance from front to rear wheels (in meters)
static const double STEERING_FACTOR = 32;   // Steering factor (adjusted per vehicle)

class Odometer
{
private:
  ros::NodeHandle nh_;
  ros::Publisher odom_pub_;
  ros::Subscriber sub_;
  ros::Subscriber gps_yaw_sub_;
  ros::Publisher debug_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  
  // Steering bias for correction
  double steer_bias_ = 0.0; 
  static constexpr double K = 0.1;  // learning rate

  // Vehicle state variables
  double x_, y_, theta_;
  ros::Time last_time_;

  // GPS yaw comparison
  double gps_yaw_;
  bool got_yaw_;
  double yaw_diff;

public:
  Odometer()
  {
    // Initialize state variables
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    last_time_ = ros::Time::now();
    gps_yaw_ = 0.0;
    got_yaw_ = false;

    // Publisher for odometry
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

    // Subscriber to vehicle status on "/speedsteer"
    sub_ = nh_.subscribe("/speedsteer", 10, &Odometer::speedsteerCallback, this);
    debug_pub_ = nh_.advertise<std_msgs::String>("/debug_topic", 10);
    // Subscriber to GPS yaw
    gps_yaw_sub_ = nh_.subscribe("/gps_yaw", 10, &Odometer::yawCallback, this);
    


  }

  void yawCallback(const std_msgs::Float64::ConstPtr &msg)
  {
    gps_yaw_ = msg->data;
    got_yaw_ = true;
  }

  void speedsteerCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
  {
    ros::Time current_time = msg->header.stamp;
    double dt = (current_time - last_time_).toSec();
    last_time_ = current_time;

    // Extract vehicle status:
    // x: steering angle in deg, y: speed in km/h
    double steer_deg = msg->point.x / STEERING_FACTOR;

    steer_deg -= steer_bias_;

    if (got_yaw_)
    {
      yaw_diff = theta_ - gps_yaw_;

      //wrap to [-pi, pi]
      while (yaw_diff > M_PI)   yaw_diff -= 2*M_PI;
      while (yaw_diff < -M_PI)  yaw_diff += 2*M_PI;

      steer_bias_ += K * yaw_diff;    
      
      ROS_INFO("[BIAS] STEER BIAS!!!!!!!!!!!!!!!!!!!!!!: %.4f deg", steer_bias_);

    }


    double steer_rad = steer_deg * M_PI / 180.0; // Convert to radians
    double speed_kmh = msg->point.y;
    // Convert km/h to m/s
    double speed = speed_kmh / 3.6;

    // Calculate the turning radius using the steering factor
    // This is a simplified model: Steering angle (steer_rad) * steering factor gives turning radius.
    double turning_radius = VEHICLE_LENGTH / tan(steer_rad) + WHEELBASE / 2;

    // Calculate the rate of change of the vehicle's position and orientation (kinematic equations)
    double angular_velocity = speed / turning_radius; // In radians per second

    // Update the pose (x, y, theta) by integrating over time using Eueler's method
    /*
    theta_ += angular_velocity * dt;
    x_ += speed * cos(theta_) * dt;
    y_ += speed * sin(theta_) * dt;
    */
    // Update the pose (x, y, theta) by integrating over time using Runge-Kutta method

    theta_ += angular_velocity * dt;
    x_ += speed * cos(theta_ + angular_velocity * dt / 2) * dt;
    y_ += speed * sin(theta_ + angular_velocity * dt / 2) * dt;

    // Update the pose (x, y, theta) by integrating over time using exact approximation
    // This is a more accurate method for non-linear motion
    /*
    theta_new = theta_ + angular_velocity * dt;
    x_ += speed/angular_velocity * (sin(theta_new) - sin(theta_));
    y_ += speed/angular_velocity * (cos(theta_new) - cos(theta_));
    theta_ = theta_new;
    */

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

    double theta_deg = theta_ * 180.0 / M_PI;

    std_msgs::String debug;
    debug.data = "VEHICULE_YAW: " + std::to_string(theta_);
    debug_pub_.publish(debug);

    ROS_INFO("[ODOM] Time: %.2f | Pos: (%.2f, %.2f) | theta: %.2f rad | q: (%.2f, %.2f, %.2f, %.2f) | v: %.2f m/s | steer: %.2f deg",
             current_time.toSec(),
             x_, y_,
             theta_,
             odom.pose.pose.orientation.x,
             odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z,
             odom.pose.pose.orientation.w,
             speed, steer_deg);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometer");
  Odometer odom_node;
  ros::spin();
  return 0;
}