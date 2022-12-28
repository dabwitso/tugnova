#include <iostream>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>

class GPSNode
{
    public:
    GPSNode(ros::NodeHandle nh_) : n(nh_)
    {
        gps_sub = n.subscribe("/extended_fix", 100, &GPSNode::gps_callback, this);
    }

    gps_common::GPSFix gpsMsg;

    void gps_callback(const gps_common::GPSFixConstPtr &msg)
    {
        printf("Received Data");
        gpsMsg = *msg;
    }

    private:
    ros::NodeHandle n;
    ros::Subscriber gps_sub;
    ros::Publisher gps_pub;
};

int main(int argc, char** argv)
{
    double gpsLat = 0;
    double gpsLong = 0;

    ros::init(argc, argv, "gps_node");
    ros::NodeHandle nh_;

    GPSNode *p = new GPSNode(nh_);

    gpsLat = p->gpsMsg.latitude;
    gpsLong = p->gpsMsg.longitude;

    std::cout << "Current Latitude: " << gpsLat << std::endl;
    std::cout << "Current Longitude: " << gpsLong << std::endl;

    ros::spin();

    return 0;
}