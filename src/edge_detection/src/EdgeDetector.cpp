#include <edge_detection/EdgeDetector.hpp>

//External 
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

using namespace edge_detection;

// Your class methods definition goes here





void SensorPointCloud2Message(const sensor_msgs::PointCloud2::ConstPtr& cloud){



}



int main(int argc, char **argv)
{
	edge_detection::EdgeDetector *detector = new edge_detection::EdgeDetector();
    // Create the executable for testing the code here

	//detector->InitSubscriber();


	ros::init(argc, argv, "Subscriber");
	ros::NodeHandle nh;
    ros::Subscriber topic_sub = nh.subscribe("/camera/depth_registered/points", 1, SensorPointCloud2Message);
    ros::spin();
    //ROS_INFO("Test1");

 





	return 0;
}
