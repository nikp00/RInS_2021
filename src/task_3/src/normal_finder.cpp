#include <iostream>
#include <ros/ros.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/point_cloud.h"
#include <pcl/features/integral_image_normal.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/PointStamped.h"

#include "task_3/GetNormalService.h"

tf2_ros::Buffer tf2_buffer;
ros::Publisher pubx;

double min_z, max_z, min_y, max_y;

typedef pcl::PointXYZRGB PointT;

pcl::PassThrough<PointT> pass;
pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);
}

bool find_normal(task_3::GetNormalService::Request &req, task_3::GetNormalService::Response &res)
{

    // pass.setKeepOrganized(true);
    // pass.setInputCloud(cloud);
    // pass.filter(*cloud_filtered);

    // ne.setSearchMethod(tree);
    // ne.setInputCloud(cloud_filtered);
    // ne.setRadiusSearch(req.radius);
    // ne.compute(*cloud_normals);

    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);

    std::cout << "X: " << req.x << ", Y: " << req.y << std::endl;
    std::cout << cloud_normals->height << " x " << cloud_normals->width << std::endl;
    std::cout << "Original: ";
    std::cout << cloud->points.size();
    std::cout << ", Filtered: ";
    std::cout << cloud_normals->points.size();
    std::cout << ", Normal: ";
    std::cout << cloud_normals->at(req.x, req.y) << std::endl;
    pcl::Normal n = cloud_normals->at(req.x, req.y);

    cloud->at(req.x, req.y).r = 255;
    cloud->at(req.x, req.y).g = 0;
    cloud->at(req.x, req.y).b = 0;
    pcl::PCLPointCloud2 outcloud;
    pcl::toPCLPointCloud2(*cloud, outcloud);
    pubx.publish(outcloud);

    res.nx = n.normal_x;
    res.ny = n.normal_y;
    res.nz = n.normal_z;
    return true;
}

int main(int argc, char **argv)
{

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Initialize ROS
    ros::init(argc, argv, "cylinder_segmentation");
    ros::NodeHandle nh;

    std::cout << "Started" << std::endl;

    // For transforming between coordinate frames
    tf2_ros::TransformListener tf2_listener(tf2_buffer);

    nh.param<double>("min_y", min_y, -0.3);
    nh.param<double>("max_y", max_y, 0.09);
    nh.param<double>("min_z", min_z, 0.2);
    nh.param<double>("max_z", max_z, 2.0);
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);
    pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);

    ros::ServiceServer service = nh.advertiseService("get_normal", find_normal);

    ros::spin();

    return EXIT_SUCCESS;
}
