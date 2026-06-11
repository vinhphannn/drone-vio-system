#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <cmath>

#include "drone_core/drivers/imu_driver.h"
#include "drone_core/drivers/camera_driver.h"

#include <thread>
#include <atomic>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class HardwareSyncNode : public rclcpp::Node {
public:
    HardwareSyncNode() : Node("drone_hardware_sync") {
        // Publishers cho OpenVINS
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 200);
        cam_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

        // Khởi tạo Cấu hình Driver
        drone::ImuDriverConfig imu_cfg;
        imu_cfg.device = "/dev/ttyACM0";
        imu_cfg.cpu_core = 1;

        drone::CameraDriverConfig cam_cfg;
        cam_cfg.device_index = 0;
        cam_cfg.width = 640;
        cam_cfg.height = 480;
        cam_cfg.fps = 30.0;
        cam_cfg.cpu_core = 2;
        cam_cfg.time_offset_ms = 34.4; // Peak correlation gave 34.4ms

        imu_driver_ = std::make_unique<drone::ImuDriver>(imu_cfg, imu_ring_);
        cam_driver_ = std::make_unique<drone::CameraDriver>(cam_cfg, cam_ring_);

        // Khởi chạy Hardware Threads
        imu_driver_->start();
        cam_driver_->start();

        RCLCPP_INFO(this->get_logger(), "Hardware Drivers started! Publishing to /camera/image_raw and /imu/data");

        // Khởi chạy Sync Thread
        sync_thread_ = std::thread(&HardwareSyncNode::sync_loop, this);
    }

    ~HardwareSyncNode() {
        running_ = false;
        if (sync_thread_.joinable()) {
            sync_thread_.join();
        }
        cam_driver_->stop();
        imu_driver_->stop();
    }

private:
    void sync_loop() {
        while (rclcpp::ok() && running_) {
            // 1. Rút cạn IMU Ring Buffer và Publish
            drone::ImuMeasurement imu_meas;
            while (imu_ring_.pop(imu_meas)) {
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp.sec = imu_meas.timestamp_ns / 1000000000ULL;
                msg.header.stamp.nanosec = imu_meas.timestamp_ns % 1000000000ULL;
                msg.header.frame_id = "imu_link";
                
                // Publish RAW IMU measurements! DO NOT ROTATE!
                // VIO needs the exact raw readings to match the physical IMU-Camera transform!
                msg.linear_acceleration.x = imu_meas.accel_x;
                msg.linear_acceleration.y = imu_meas.accel_y;
                msg.linear_acceleration.z = imu_meas.accel_z;

                msg.angular_velocity.x = imu_meas.gyro_x;
                msg.angular_velocity.y = imu_meas.gyro_y;
                msg.angular_velocity.z = imu_meas.gyro_z;

                // Không có Magnetometer hoặc Orientation (Để OpenVINS tự tính)
                msg.orientation_covariance[0] = -1.0; 

                imu_pub_->publish(msg);
            }

            // 2. Rút cạn Camera Ring Buffer và Publish
            drone::CameraFrame cam_frame;
            while (cam_ring_.pop(cam_frame)) {
                cv::Mat img(cam_frame.height, cam_frame.width, CV_8UC1, cam_frame.data.data());
                
                std_msgs::msg::Header header;
                header.stamp.sec = cam_frame.timestamp_ns / 1000000000ULL;
                header.stamp.nanosec = cam_frame.timestamp_ns % 1000000000ULL;
                header.frame_id = "camera_link";

                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", img).toImageMsg();
                cam_pub_->publish(*msg);
            }

            // Nghỉ ngắn để không miss frame (1600Hz = 625us)
            std::this_thread::sleep_for(200us);
        }
    }

    drone::ImuRingBuffer imu_ring_;
    drone::CamRingBuffer cam_ring_;
    std::unique_ptr<drone::ImuDriver> imu_driver_;
    std::unique_ptr<drone::CameraDriver> cam_driver_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr cam_pub_;

    std::atomic<bool> running_{true};
    std::thread sync_thread_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HardwareSyncNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
