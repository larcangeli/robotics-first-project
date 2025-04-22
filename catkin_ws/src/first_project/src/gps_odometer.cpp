// SECOND NODE: GPS ODOMETER
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <math.h>

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

    // Load parameters (reference point)
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
    sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 10, &GpsOdometer::gpsCallback, this);
    pub_ = nh_.advertise<nav_msgs::Odometry>("/gps_odom", 10);
  }

  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
  {
    if (msg->status.status < 0) {
      ROS_WARN("GPS signal not valid");
      return;
    }

    double x, y, z;
    gpsToECEF(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    double enu_x, enu_y, enu_z;
    ecefToENU(x, y, z, enu_x, enu_y, enu_z);

    ros::Time current_time = msg->header.stamp;

    double north = enu_y;
    double east  = enu_x;
    enu_x = north;
    enu_y = east;

    // Estimate heading
    double yaw = 0.0;
    if (heading_initialized_) {
      double dx = enu_x - last_enu_x_;
      double dy = enu_y - last_enu_y_;
      if (dx != 0.0 || dy != 0.0) {
        yaw = atan2(dy, dx);
      }
    }
    last_enu_x_ = enu_x;
    last_enu_y_ = enu_y;
    heading_initialized_ = true;

    // Fill Odometry message
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

    ROS_INFO("Published GPS odometry: (%.2f, %.2f), yaw: %.2f rad", enu_x, enu_y, yaw);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Reference point (ECEF)
  double lat_ref_, lon_ref_, alt_ref_;
  double lat_ref_rad_, lon_ref_rad_;
  double x_ref_, y_ref_, z_ref_;

  // For heading
  bool heading_initialized_;
  double last_enu_x_, last_enu_y_;
  bool got_reference_;

  void gpsToECEF(double lat, double lon, double alt, double& x, double& y, double& z)
  {
    // Constant from WGS-84 ellipsoid parameters
    double a = 6378137.0;        // semi-major axis (equator radius in meters)
    double b = 6356752.0;        // semi-minor axis (pole radius)
    double e2 = 1 - (b*b)/(a*a); // eccentricity squared    

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / sqrt(1 - e2 * sin(lat_rad) * sin(lat_rad));

    x = (N + alt) * cos(lat_rad) * cos(lon_rad);
    y = (N + alt) * cos(lat_rad) * sin(lon_rad);
    z = ((b*b)/(a*a) * N + alt) * sin(lat_rad);
  }

  void referenceECEF(double lat, double lon, double alt, double& x, double& y, double& z)
  {
    gpsToECEF(lat, lon, alt, x, y, z);
  }

  void ecefToENU(double x, double y, double z, double& enu_x, double& enu_y, double& enu_z)
  {
    double dx = x - x_ref_;
    double dy = y - y_ref_;
    double dz = z - z_ref_;

    enu_x = -sin(lon_ref_rad_)*dx + cos(lon_ref_rad_)*dy;
    enu_y = -sin(lat_ref_rad_)*cos(lon_ref_rad_)*dx - sin(lat_ref_rad_)*sin(lon_ref_rad_)*dy + cos(lat_ref_rad_)*dz;
    enu_z = cos(lat_ref_rad_)*cos(lon_ref_rad_)*dx + cos(lat_ref_rad_)*sin(lon_ref_rad_)*dy + sin(lat_ref_rad_)*dz;
      
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_odometer");
  GpsOdometer node;
  ros::spin();
  return 0;
}
