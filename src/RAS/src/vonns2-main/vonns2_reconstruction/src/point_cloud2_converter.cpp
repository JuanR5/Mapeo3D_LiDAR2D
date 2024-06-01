// C++ libraries
#include <iostream>
#include <string>
#include <deque>
// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "laser_geometry/laser_geometry.hpp"
// ROS2 messages
#include <std_msgs/msg/int32.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class PointCloud2Converter : public rclcpp::Node {
public:
    PointCloud2Converter() : Node("point_cloud2_converter") {
        std::cerr << "Start..." << std::endl;
        // Getting fixed frame id for PointCloud2
        fixed_frame_id = "base_link";

        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        laser_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&PointCloud2Converter::scan_callback, this, std::placeholders::_1)
        );

        point_cloud2_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud2", 10);
    }

    // Subscribing to the LaserScans sent by LiDAR or similar-type devices
    void scan_callback(sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg) {
        std::cerr << "\nIncoming Data from LiDAR" << std::endl;
        
        // Imprimir el frame_id del mensaje LaserScan recibido
        std::cerr << "LaserScan frame_id: " << laser_scan_msg->header.frame_id << std::endl;

        sensor_msgs::msg::LaserScan scan_in = *laser_scan_msg;
        sensor_msgs::msg::PointCloud2 cloud_out;
        laser_projector.projectLaser(scan_in, cloud_out);

        // Imprimir el frame_id del PointCloud2 antes de la transformación
        std::cerr << "PointCloud2 frame_id before transform: " << cloud_out.header.frame_id << std::endl;

        // Cambiar el frame_id del PointCloud2 si es necesario
        if (cloud_out.header.frame_id != fixed_frame_id) {
            try {
                // Calcular la transformación entre los marcos especificados
                tf_stamped = tf_buffer->lookupTransform(
                    fixed_frame_id, // Marco fijo
                    cloud_out.header.frame_id, // Marco actual de PointCloud2
                    cloud_out.header.stamp, // Tiempo actual de PointCloud2
                    rclcpp::Duration::from_seconds(1.5) // Tiempo de espera para el cálculo de transformaciones
                );

                // Transformar PointCloud2 basado en la transformación anterior
                tf2::doTransform(cloud_out, cloud_out, tf_stamped);
                std::cerr << "Transform successful" << std::endl;
            } catch (tf2::TransformException &ex) {
                std::cerr << "Transform not computed: " << ex.what() << std::endl;
            }
        }

        // Imprimir el frame_id del PointCloud2 después de la transformación
        std::cerr << "PointCloud2 frame_id after transform: " << cloud_out.header.frame_id << std::endl;
        point_cloud2_pub->publish(cloud_out);
    }

private:
    std::string fixed_frame_id;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    geometry_msgs::msg::TransformStamped tf_stamped;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_pub;

    laser_geometry::LaserProjection laser_projector;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto point_cloud2_converter_node = std::make_shared<PointCloud2Converter>();
    rclcpp::spin(point_cloud2_converter_node);

    rclcpp::shutdown();
    return 0;
}