// THIRD NODE: SECTOR TIME
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <first_project/sector_times.h>
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <math.h>

struct SectorGate {
    float north1;
    float east1;
    float north2;
    float east2;

    // ax + by + c = 0
    float a; // coefficient of x
    float b; // coefficient of y
    float c; // bias
};

class SectorTimeNode
{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_;

        struct SectorGate first_gate;
        struct SectorGate second_gate;
        struct SectorGate third_gate;

        first_project::sector_times msg;

        message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub_;
        message_filters::Subscriber<geometry_msgs::PointStamped> speedsteer_sub_;
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::PointStamped> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

    public:
        SectorTimeNode() {
            first_gate.north1 = 45.613456;//45.616024;
            first_gate.east1 = 9.279773;//9.279818;
            first_gate.north2 = 45.623362;//45.615885;
            first_gate.east2 = 9.285824;//9.281463;

            float delta_x = first_gate.east1 - first_gate.east2;
            float delta_y = first_gate.north1 - first_gate.north2;
            first_gate.a = -delta_y; // coefficient of x
            first_gate.b = delta_x; // coefficient of y
            first_gate.c = -first_gate.north1*delta_x + first_gate.east1*delta_y;

            second_gate.north1 = 45.630451;
            second_gate.east1 = 9.289480;
            second_gate.north2 = 45.629847;
            second_gate.east2 = 9.289963;

            delta_x = second_gate.east1 - second_gate.east2;
            delta_y = second_gate.north1 - second_gate.north2;
            second_gate.a = -delta_y; // coefficient of x
            second_gate.b = delta_x; // coefficient of y
            second_gate.c = -second_gate.north1*delta_x + second_gate.east1*delta_y;

            third_gate.north1 = 45.623781;
            third_gate.east1 = 9.286902;
            third_gate.north2 = 45.623241;
            third_gate.east2 = 9.287604;

            delta_x = third_gate.east1 - third_gate.east2;
            delta_y = third_gate.north1 - third_gate.north2;
            third_gate.a = -delta_y; // coefficient of x
            third_gate.b = delta_x; // coefficient of y
            third_gate.c = -third_gate.north1*delta_x + third_gate.east1*delta_y;

            msg.current_sector = 1;
            msg.current_sector_time = 0.0;  
            msg.current_sector_mean_speed = 0.0;
            
            pub_ = nh_.advertise<first_project::sector_times>("/sector_times", 10);
            
            gps_sub_.subscribe(nh_, "/swiftnav/front/gps_pose", 10);
            speedsteer_sub_.subscribe(nh_, "/speedsteer", 10);
            
            sync_.reset(new Sync(MySyncPolicy(10), gps_sub_, speedsteer_sub_));
            sync_->registerCallback(boost::bind(&SectorTimeNode::syncCallback, this, _1, _2));
        }

        void syncCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg,
            const geometry_msgs::PointStamped::ConstPtr& speedsteer_msg)
        {
            msg.current_sector_mean_speed = (msg.current_sector_mean_speed*(msg.current_sector_time/0.1) + speedsteer_msg->point.y)/(msg.current_sector_time/0.1 + 1);
            msg.current_sector_time += 0.1;

            // I pass through it when the ax + by + c becomes positive and I was in sector 1
            if(msg.current_sector == 1 && gateCoefficient(gps_msg->latitude, gps_msg->longitude, second_gate.a, second_gate.b, second_gate.c) <= 0) {
                msg.current_sector = 2;
                msg.current_sector_mean_speed = 0;
                msg.current_sector_time = 0;
            }

            // I pass through it when the ax + by + c becomes negative and I was in sector 2
            if(msg.current_sector == 2 && gateCoefficient(gps_msg->latitude, gps_msg->longitude, third_gate.a, third_gate.b, third_gate.c) >= 0) {
                msg.current_sector = 3;
                msg.current_sector_mean_speed = 0;
                msg.current_sector_time = 0;
            }

            // I pass through it when the ax + by + c becomes positive and I was in secotor 3
            if(msg.current_sector == 3 && gateCoefficient(gps_msg->latitude, gps_msg->longitude, first_gate.a, first_gate.b, first_gate.c) <= 0) {
                msg.current_sector = 1;
                msg.current_sector_mean_speed = 0;
                msg.current_sector_time = 0;
            }
            
            pub_.publish(msg);
        }

        float gateCoefficient(float latit, float longit, float a, float b, float c) {
            return a*longit + b*latit + c;
        }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sector_times");
    SectorTimeNode node;
    ros::spin();
    return 0;
}
