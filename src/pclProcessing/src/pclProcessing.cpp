
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>

#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/point_cloud_image_extractors.h>
    
#include <pcl/PCLImage.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/png_io.h>
int main ()
{

    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("/home/vboxuser/catkin_ws/pclXYZRGBCloud.pcd", *cloud);


    //Point Cloud Image Extractor
    pcl::PCLImage image;
    pcl::io::PointCloudImageExtractorFromLabelField<pcl::PointXYZ> pcie;
    bool success = pcie.extract(*cloud, image);

    if(success)
        pcl::io::savePNGFile("/home/vboxuser/catkin_ws/imageTest.png", image);

    
    /*
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud);
    voxel_filter.setLeafSize(0.05, 0.05, 0.05);
    voxel_filter.filter(*voxel_cloud);


    pcl::io::savePCDFile("/home/vboxuser/catkin_ws/pclVoxel.pcd", *voxel_cloud);
  

    //edge Detection

    pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setNormalSmoothingSize (20.0f);
    ne.setBorderPolicy (ne.BORDER_POLICY_MIRROR);
    ne.setInputCloud (cloud);
    ne.compute (*normal);


    pcl::OrganizedEdgeFromNormals<pcl::PointXYZ, pcl::Normal, pcl::Label> oed;
  //OrganizedEdgeFromRGBNormals<PointXYZ, Normal, Label> oed;
    oed.setInputNormals (normal);
    oed.setInputCloud (cloud);
    oed.setDepthDisconThreshold (0.5);
    oed.setMaxSearchNeighbors (10);
    oed.setEdgeType (oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED | oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> label_indices;
    oed.compute (labels, label_indices);


    pcl::PointCloud<pcl::PointXYZ>::Ptr occluding_edges (new pcl::PointCloud<pcl::PointXYZ>),
    occluded_edges (new pcl::PointCloud<pcl::PointXYZ>),
    nan_boundary_edges (new pcl::PointCloud<pcl::PointXYZ>),
    high_curvature_edges (new pcl::PointCloud<pcl::PointXYZ>),
    rgb_edges (new pcl::PointCloud<pcl::PointXYZ>);

    copyPointCloud (*cloud, label_indices[0].indices, *nan_boundary_edges);
    copyPointCloud (*cloud, label_indices[1].indices, *occluding_edges);
    copyPointCloud (*cloud, label_indices[2].indices, *occluded_edges);
    copyPointCloud (*cloud, label_indices[3].indices, *high_curvature_edges);
    copyPointCloud (*cloud, label_indices[4].indices, *rgb_edges);
    //

    pcl::io::savePCDFile("/home/vboxuser/catkin_ws/pclnan_boundary_edge.pcd", *voxel_cloud);
    pcl::io::savePCDFile("/home/vboxuser/catkin_ws/pcl_occluding_edge.pcd", *voxel_cloud);
    pcl::io::savePCDFile("/home/vboxuser/catkin_ws/pcloccluded_edge.pcd", *voxel_cloud);
    pcl::io::savePCDFile("/home/vboxuser/catkin_ws/pclcurvature_edge.pcd", *voxel_cloud);
    pcl::io::savePCDFile("/home/vboxuser/catkin_ws/pcl_reg_edge.pcd", *voxel_cloud);
  





    */

    

    return 0;
}