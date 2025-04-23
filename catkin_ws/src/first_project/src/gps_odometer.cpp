// SECOND NODE: GPS ODOMETER
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float64.h"
#include "tf/transform_datatypes.h"
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <boost/bind.hpp>

class GpsOdometer
{
public:
  GpsOdometer()
  {
    // Initialize flags
    got_reference_ = false;
    last_enu_x_ = 0.0;
    last_enu_y_ = 0.0;
    heading_initialized_ = false;

    // Load reference point parameters
    ros::NodeHandle private_nh("~");
    private_nh.param("lat_r", lat_ref_, 0.0);
    private_nh.param("lon_r", lon_ref_, 0.0);
    private_nh.param("alt_r", alt_ref_, 0.0);

    // Convert reference point to radians
    lat_ref_rad_ = lat_ref_ * M_PI / 180.0;
    lon_ref_rad_ = lon_ref_ * M_PI / 180.0;

    // Convert reference point to ECEF
    referenceECEF(lat_ref_, lon_ref_, alt_ref_, x_ref_, y_ref_, z_ref_);

    // ROS setup
    gps_sub_.subscribe(nh_, "/swiftnav/front/gps_pose", 10);
    speedsteer_sub_.subscribe(nh_, "/speedsteer", 10);

    sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), gps_sub_, speedsteer_sub_);
    sync_->registerCallback(boost::bind(&GpsOdometer::syncedCallback, this, _1, _2));

    pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 10);
    yaw_pub_ = nh_.advertise<std_msgs::Float64>("/gps_heading", 10);
  }

  void syncedCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                      const geometry_msgs::PointStamped::ConstPtr &steer_msg)
  {
    if (gps_msg->status.status < 0)
    {
      ROS_WARN("GPS signal not valid");
      return;
    }

    double x, y, z;
    gpsToECEF(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, x, y, z);

    double enu_x, enu_y, enu_z;
    ecefToENU(x, y, z, enu_x, enu_y, enu_z);

    ros::Time current_time = gps_msg->header.stamp;

    // rotate ENU axes
    double north = enu_y;
    double east = enu_x;
    enu_x = north;
    enu_y = east;

    // Estimate heading
    double yaw = 0.0;
    if (heading_initialized_)
    {
      double dx = enu_x - last_enu_x_;
      double dy = enu_y - last_enu_y_;
      if (dx != 0.0 || dy != 0.0)
      {
        yaw = atan2(dy, dx);
      }
    }
    last_enu_x_ = enu_x;
    last_enu_y_ = enu_y;
    heading_initialized_ = true;

    // Prepare Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "gps";

    odom.pose.pose.position.x = enu_x;
    odom.pose.pose.position.y = enu_y;
    odom.pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);
    odom.pose.pose.orientation = quat;

    pub_.publish(odom);

    // Publish yaw angle
    std_msgs::Float64 yaw_msg;
    yaw_msg.data = yaw;
    yaw_pub_.publish(yaw_msg);

    // Broadcast TF
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "gps";
    tf_msg.transform.translation.x = enu_x;
    tf_msg.transform.translation.y = enu_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = quat;
    tf_broadcaster_.sendTransform(tf_msg);

    // Log the computed odometry for debugging
    ROS_INFO("[GPS_ODOM] Time: %.2f | Pos: (%.2f, %.2f) | yaw: %.2f rad | q: (%.2f, %.2f, %.2f, %.2f)",
             current_time.toSec(),
             enu_x, enu_y,
             yaw,
             quat.x, quat.y, quat.z, quat.w);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher yaw_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;
  message_filters::Subscriber<geometry_msgs::PointStamped> speedsteer_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::PointStamped> MySyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  // Reference point (ECEF)
  double lat_ref_, lon_ref_, alt_ref_;
  double lat_ref_rad_, lon_ref_rad_;
  double x_ref_, y_ref_, z_ref_;

  // Heading tracking
  bool heading_initialized_;
  double last_enu_x_, last_enu_y_;
  bool got_reference_;

  void gpsToECEF(double lat, double lon, double alt, double &x, double &y, double &z)
  {
    double a = 6378137.0;
    double b = 6356752.0;
    double e2 = 1 - (b * b) / (a * a);

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));

    x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    z = ((b * b) / (a * a) * N + alt) * sin(lat_rad);
  }

  void referenceECEF(double lat, double lon, double alt, double &x, double &y, double &z)
  {
    gpsToECEF(lat, lon, alt, x, y, z);
  }

  void ecefToENU(double x, double y, double z, double &enu_x, double &enu_y, double &enu_z)
  {
    double dx = x - x_ref_;
    double dy = y - y_ref_;
    double dz = z - z_ref_;

    enu_x = -sin(lon_ref_rad_) * dx + cos(lon_ref_rad_) * dy;
    enu_y = -sin(lat_ref_rad_) * cos(lon_ref_rad_) * dx - sin(lat_ref_rad_) * sin(lon_ref_rad_) * dy + cos(lat_ref_rad_) * dz;
    enu_z = cos(lat_ref_rad_) * cos(lon_ref_rad_) * dx + cos(lat_ref_rad_) * sin(lon_ref_rad_) * dy + sin(lat_ref_rad_) * dz;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_odometer");
  GpsOdometer node;
  ros::spin();
  return 0;
}