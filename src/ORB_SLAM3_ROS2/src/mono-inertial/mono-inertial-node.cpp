#include "mono-inertial-node.hpp"

#include <opencv2/core/core.hpp>

using std::placeholders::_1;

MonoInertialNode::MonoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K, P, R, D;
        fsSettings["LEFT.K"] >> K;
        fsSettings["LEFT.P"] >> P;
        fsSettings["LEFT.R"] >> R;
        fsSettings["LEFT.D"] >> D;

        int rows = fsSettings["LEFT.height"];
        int cols = fsSettings["LEFT.width"];

        if (K.empty() || P.empty() || R.empty() || D.empty() || rows == 0 || cols == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K, D, R, P.rowRange(0, 3).colRange(0, 3), cv::Size(cols, rows), CV_32F, M1_, M2_);
    }

    m_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    subImu_ = this->create_subscription<ImuMsg>("/imu/data", 1000, std::bind(&MonoInertialNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("/camera/image_raw", 100, std::bind(&MonoInertialNode::GrabImage, this, _1));

    syncThread_ = new std::thread(&MonoInertialNode::SyncWithImu, this);
}

MonoInertialNode::~MonoInertialNode()
{
    // Delete sync thread
    syncThread_->join();
    delete syncThread_;

    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    bufMutex_.lock();
    imuBuf_.push(msg);
    bufMutex_.unlock();
}

void MonoInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
        imgBuf_.pop();
    imgBuf_.push(msg);

    bufMutexImg_.unlock();
}

cv::Mat MonoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonoInertialNode::SyncWithImu()
{
    while (1)
    {
        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);

        cv::Mat im;
        double tIm = 0;
        
        bufMutexImg_.lock();
        if (imgBuf_.empty()) {
            bufMutexImg_.unlock();
            continue;
        }
        tIm = Utility::StampToSec(imgBuf_.front()->header.stamp);
        bufMutexImg_.unlock();

        bufMutex_.lock();
        if (imuBuf_.empty() || tIm > Utility::StampToSec(imuBuf_.back()->header.stamp)) {
            bufMutex_.unlock();
            continue;
        }
        bufMutex_.unlock();

        bufMutexImg_.lock();
        im = GetImage(imgBuf_.front());
        imgBuf_.pop();
        bufMutexImg_.unlock();

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        bufMutex_.lock();
        if (!imuBuf_.empty())
        {
            // Load imu measurements from buffer
            vImuMeas.clear();
            while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tIm)
            {
                double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                imuBuf_.pop();
            }
        }
        bufMutex_.unlock();

        if (bClahe_)
        {
            clahe_->apply(im, im);
        }

        if (doRectify_)
        {
            cv::remap(im, im, M1_, M2_, cv::INTER_LINEAR);
        }

        Sophus::SE3f Tcw = SLAM_->TrackMonocular(im, tIm, vImuMeas);
        
        // Publish odometry if tracking is successful
        if (!Tcw.translation().hasNaN())
        {
            Sophus::SE3f Twc = Tcw.inverse();
            Eigen::Vector3f t = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.stamp = rclcpp::Time((int64_t)(tIm * 1e9));
            odom_msg.header.frame_id = "map";
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
}
