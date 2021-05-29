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
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "std_msgs/ColorRGBA.h"
#include "geometry_msgs/PointStamped.h"
#include "task_3/CylinderSegmentation.h"

ros::Publisher pubx;
ros::Publisher puby;
ros::Publisher cylinder_pose_publisher;

tf2_ros::Buffer tf2_buffer;

double min_z, max_z, min_y, max_y;

typedef pcl::PointXYZRGB PointT;

void cloud_cb(const pcl::PCLPointCloud2ConstPtr &cloud_blob)
{
    // All the objects needed

    ros::Time time_rec, time_test;
    time_rec = ros::Time::now();

    pcl::PassThrough<PointT> pass;
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointT> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    Eigen::Vector4f centroid;

    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>), cloudF(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>), cloud_normalsF(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Read in the cloud data
    pcl::fromPCLPointCloud2(*cloud_blob, *cloud);

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(min_y, max_y);
    pass.filter(*cloud_filtered);

    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z, max_z);
    pass.filter(*cloud_filtered);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    int initial_size = cloud_filtered->height * cloud_filtered->width;
    int count = 0;

    std::cout << "------------------------" << std::endl;
    std::cout << "Size before: " << cloud_filtered->height * cloud_filtered->width;
    std::cout << ", " << cloud_normals->height * cloud_normals->width << std::endl;
    while (cloud_filtered->height * cloud_filtered->width > initial_size * 0.1 && count++ < 3)
    {
        seg.setInputCloud(cloud_filtered);
        seg.setInputNormals(cloud_normals);
        // Obtain the plane inliers and coefficients
        (void)seg.segment(*inliers_plane, *coefficients_plane);

        // Remove the planar inliers, extract the rest
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers_plane);
        extract.setNegative(true);

        extract_normals.setInputCloud(cloud_normals);
        extract_normals.setIndices(inliers_plane);
        extract_normals.setNegative(true);

        extract.filter(*cloudF);
        cloud_filtered.swap(cloudF);

        extract_normals.filter(*cloud_normalsF);
        cloud_normals.swap(cloud_normalsF);
    }

    std::cout << "Size after: " << cloud_filtered->height * cloud_filtered->width;
    std::cout << ", " << cloud_normals->height * cloud_normals->width << ", count: " << count << std::endl;

    if (cloud_filtered->height * cloud_filtered->width < 10)
        return;

    pcl::PCLPointCloud2 outcloud_plane;
    pcl::toPCLPointCloud2(*cloud_filtered, outcloud_plane);
    pubx.publish(outcloud_plane);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.005);
    seg.setRadiusLimits(0.115, 0.125);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    (void)seg.segment(*inliers_cylinder, *coefficients_cylinder);

    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
    extract.filter(*cloud_cylinder);
    if (cloud_cylinder->points.empty())
        std::cout << "Can't find the cylindrical component." << std::endl;
    else
    {
        float radius_val = coefficients_cylinder->values[6];
        float radius_target = 0.12;

        uint32_t r = 0;
        uint32_t g = 0;
        uint32_t b = 0;
        uint32_t rgb;
        int size = cloud_cylinder->points.size();
        int i = 0;
        int increment = 500;
        for (; i < size; i += increment)
        {
            rgb = *reinterpret_cast<int *>(&cloud_cylinder->points[i].rgb);
            r += (rgb >> 16) & 0x0000ff;
            g += (rgb >> 8) & 0x0000ff;
            b += (rgb)&0x0000ff;
        }

        r = r / (i / increment);
        g = g / (i / increment);
        b = b / (i / increment);

        printf("r: %d, g: %d, b: %d, samples: %d \n", r, g, b, i / increment);

        std::cout << "Radius: " << radius_val << ", Size: " << size << std::endl;

        if (abs(radius_val - radius_target) > 0.005 || size < 2500 || size > 10000)
            return;

        std::cout << "Cylinder found" << std::endl;
        pcl::compute3DCentroid(*cloud_cylinder, centroid);

        //Create a point in the "camera_rgb_optical_frame"
        geometry_msgs::PointStamped point_camera;
        geometry_msgs::PointStamped point_map;
        visualization_msgs::Marker marker;
        geometry_msgs::TransformStamped tss;

        point_camera.header.frame_id = "camera_rgb_optical_frame";
        point_camera.header.stamp = ros::Time::now();

        point_map.header.frame_id = "map";
        point_map.header.stamp = ros::Time::now();

        point_camera.point.x = centroid[0];
        point_camera.point.y = centroid[1];
        point_camera.point.z = centroid[2];

        try
        {
            time_test = ros::Time::now();
            tss = tf2_buffer.lookupTransform("map", "camera_rgb_optical_frame", time_rec);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("Transform warning: %s\n", ex.what());
        }

        tf2::doTransform(point_camera, point_map, tss);

        task_3::CylinderSegmentation msg;
        std_msgs::ColorRGBA color;

        color.r = r;
        color.g = g;
        color.b = b;
        color.a = 1;

        msg.point = point_map;
        msg.color = color;

        cylinder_pose_publisher.publish(msg);

        pcl::PCLPointCloud2 outcloud_cylinder;
        pcl::toPCLPointCloud2(*cloud_cylinder, outcloud_cylinder);
        puby.publish(outcloud_cylinder);
    }
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

    // Create a ROS publisher for the output point cloud
    pubx = nh.advertise<pcl::PCLPointCloud2>("planes", 1);
    puby = nh.advertise<pcl::PCLPointCloud2>("cylinder", 1);

    cylinder_pose_publisher = nh.advertise<task_3::CylinderSegmentation>("detected_cylinder", 1);

    // Spin
    ros::spin();
}
