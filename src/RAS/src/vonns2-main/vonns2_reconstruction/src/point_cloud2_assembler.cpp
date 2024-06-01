// C++ libraries
#include <iostream>
// ROS2 libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// PCL libraries
#include <pcl/common/common.h>
#include <pcl/io/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

class PointCloudAssembler: public rclcpp::Node{
    public: 
        PointCloudAssembler(): Node("point_cloud2_assembler"){
            std::cout << "PointCloud2 Subscriber Started!" << std::endl;

            point_cloud2_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/point_cloud2",
                10,
                std::bind(&PointCloudAssembler::cloud2_callback, this, std::placeholders::_1)
            );
        }
        
    private:
        // PointCloud2's subscriber
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_sub;

        // Current PointCloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl{new pcl::PointCloud<pcl::PointXYZ>}; 

        // Final PointCloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_concatenated{new pcl::PointCloud<pcl::PointXYZ>};

        void cloud2_callback(sensor_msgs::msg::PointCloud2::SharedPtr cloud_ros){
            std::cout << "ROS PointCloud2 Recibed!" << std::endl;
            pcl::fromROSMsg(*cloud_ros, *cloud_pcl);

            // Concatenate current PointCloud2 to the final PointCloud2
            *cloud_concatenated += *cloud_pcl;
            pcl::io::savePCDFile("concatenated_cloud.pcd", *cloud_concatenated);
            std::cout << "Concatenated PointCloud2 Saved!" << std::endl;
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    
    auto point_cloud2_assembler_node = std::make_shared<PointCloudAssembler>();
    rclcpp::spin(point_cloud2_assembler_node);
    rclcpp::shutdown();
    return (0);
}
