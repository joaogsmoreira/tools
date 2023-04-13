#include "ros/ros.h"
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>

ros::Publisher filtered;

void callback(const sensor_msgs::PointCloud2& input_cloud) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_pcl(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_pcl(new pcl::PointCloud<pcl::PointXYZI>);
    
    // From sensor_msgs::PointCloud2 to pcl::PointCloud
    pcl::PCLPointCloud2 input_pcl2;
    pcl_conversions::toPCL(input_cloud, input_pcl2);
    pcl::fromPCLPointCloud2(input_pcl2, *input_pcl);

    std::shared_ptr<std::vector<int>> indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*input_pcl, *output_pcl, *indices);

    sensor_msgs::PointCloud2 output_cloud;
    pcl::toROSMsg(*output_pcl, output_cloud);

    filtered.publish(output_cloud);

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "remove_bag_nans");
    ros::NodeHandle a, b;
    ros::Rate loop_rate(10);

    ros::Subscriber subPclFromM3 = a.subscribe("/rslidar_points", 10000, callback);
    filtered = b.advertise<sensor_msgs::PointCloud2>("/filtered_cloud", 10000);

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}