#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "visualization_msgs/Marker.h"

#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <stdlib.h>
#include <iostream>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/conversions.h>

using namespace std;
//PCL

//void writeMsgToLog(const std_msgs::String::ConstPtr& msg){
//    ROS_INFO("Test55");
//    ROS_INFO("Test Message");
//}


void writeMsgToLog(const sensor_msgs::PointCloud2::ConstPtr& points){


    ROS_INFO("Ros Info");

    const size_t number_of_points = points->height * points->width;
    ROS_INFO("Number of Points : %d", number_of_points);

    //sensor_msgs::PointCloud2Iterator<float> iter_x(*points, "x");
    //sensor_msgs::PointCloud2Iterator<float> iter_y(*points, "y");



    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRGB (new pcl::PointCloud<pcl::PointXYZ>);

    //pcl::fromPCLPointCloud2(points, cloudRGB);

    //pcl::copyPointCloud(*points, *cloudRGB);

    //sensor_msgs::Image image;

    //pcl::toROSMsg(points, *image);

    //pcl::PointCloud<pcl::PointXYZRRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRRGB>);
    //pcl::fromROSMsg (*points, *cloud);
    //int index = 5;
    //pcl::io::savePCDFile("/home/vboxuser/catkin_ws/pclXYZLCloud.pcd", *cloud, true);


    //pcl::PCLPointCloud2 pcl_pc2;
    //pcl_conversions::toPCL(*points,pcl_pc2);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    ROS_INFO("Done");

/*
    sensor_msgs::PointCloud2ConstIterator<float> iter(*points, "x");


    ROS_INFO("X: %d", iter[0]);
    
    for (size_t i = 0; i< number_of_points; i++, ++iter)
    {
        double x = iter[0];
        double y = iter[1];
        double z = iter[2];

        ROS_INFO("X: %d Y: %d Z: %d", x, y, z);


    }

    */



    //ROS_INFO("Height: %d Width: %d Row_Step: %d Point_step: %d Data_Length:%d",  points->height, points->width, points->row_step, points->point_step, points->data.size());


}



int main(int argc, char **argv){
    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle nh;
    ROS_INFO("Test1");

    /*
    // Send Marker
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle n;
    ros::Rate r(1);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    uint32_t shape = visualization_msgs::Marker::CUBE;

    while (ros::ok())
   {
       visualization_msgs::Marker marker;

       marker.header.frame_id = "/root_link";
       marker.header.stamp = ros::Time::now();
   
       // Set the namespace and id for this marker.  This serves to create a unique ID
       // Any marker sent with the same namespace and id will overwrite the old one
       marker.ns = "basic_shapes";
       marker.id = 0;
   
       // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
       marker.type = shape;
   
       // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
       marker.action = visualization_msgs::Marker::ADD;
   
       // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
       marker.pose.position.x = 0;
       marker.pose.position.y = 0;
       marker.pose.position.z = 0;
       marker.pose.orientation.x = 0.0;
       marker.pose.orientation.y = 0.0;
       marker.pose.orientation.z = 0.0;
       marker.pose.orientation.w = 1.0;
   
       // Set the scale of the marker -- 1x1x1 here means 1m on a side
       marker.scale.x = 1.0;
       marker.scale.y = 1.0;
       marker.scale.z = 1.0;
   
       // Set the color -- be sure to set alpha to something non-zero!
       marker.color.r = 1.0f;
       marker.color.g = 0.0f;
       marker.color.b = 0.0f;
       marker.color.a = 1.0;
   
       marker.lifetime = ros::Duration();
   
       // Publish the marker
       while (marker_pub.getNumSubscribers() < 1)
       {
         if (!ros::ok())
         {
           return 0;
         }
         ROS_WARN_ONCE("Please create a subscriber to the marker");
         sleep(1);
      }
       marker_pub.publish(marker);
       ROS_INFO("Test INFO");
       // Cycle between different shapes
       switch (shape)
       {
       case visualization_msgs::Marker::CUBE:
         shape = visualization_msgs::Marker::SPHERE;
        break;
      case visualization_msgs::Marker::SPHERE:
        shape = visualization_msgs::Marker::ARROW;
        break;
      case visualization_msgs::Marker::ARROW:
        shape = visualization_msgs::Marker::CYLINDER;
        break;
      case visualization_msgs::Marker::CYLINDER:
        shape = visualization_msgs::Marker::CUBE;
        break;
      }
  
      r.sleep();
    }
    */
    //

    ros::Subscriber topic_sub = nh.subscribe("/camera/depth_registered/points", 1, writeMsgToLog);

    ros::spin();

    return 0;

}

