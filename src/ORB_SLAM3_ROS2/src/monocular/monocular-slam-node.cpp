#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    
    m_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "/camera/image_raw",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "Monocular SLAM Node initialized" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    
    // Publish odometry if tracking is successful
    if (!Tcw.translation().hasNaN())
    {
        Sophus::SE3f Twc = Tcw.inverse();
        Eigen::Vector3f t = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = "map"; // World frame
        odom_msg.child_frame_id = "camera_link";

        odom_msg.pose.pose.position.x = t.x();
        odom_msg.pose.pose.position.y = t.y();
        odom_msg.pose.pose.position.z = t.z();

        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        m_odom_publisher->publish(odom_msg);
    }
}
