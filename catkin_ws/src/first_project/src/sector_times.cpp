// THIRD NODE: SECTOR TIME
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <first_project/SectorTimes.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <math.h>

class SectorTimeNode
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;
        
        message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;
        message_filters::Subscriber<geometry_msgs::PointStamped> speedsteer_sub_;
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::PointStamped> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

    public:
        SectorTimeNode() {
            pub_ = nh_.advertise<first_project::SectorTimes>("/sector_times", 10);
            
            gps_sub_.subscribe(nh_, "/swiftnav/front/gps_pose", 10);
            speedsteer_sub_.subscribe(nh_, "/speedsteer", 10);
            
            sync_.reset(new Sync(MySyncPolicy(10), gps_sub_, speedsteer_sub_));
            sync_->registerCallback(boost::bind(&SectorTimeNode::syncCallback, this, _1, _2));
        }

        void syncCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
            const geometry_msgs::PointStamped::ConstPtr& speedsteer_msg)
        {
            first_project::SectorTimes msg;
        
            msg.current_sector = 2;
            msg.current_sector_time = 32.5;  
            msg.current_sector_mean_speed = 75.3;
            
            pub_.publish(msg);
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sector_times");
    SectorTimeNode node;
    ros::spin();
    return 0;
}
