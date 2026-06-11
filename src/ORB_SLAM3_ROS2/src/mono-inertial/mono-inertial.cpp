#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "mono-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        std::cerr << endl << "Usage: ros2 run orbslam3 mono-inertial path_to_vocabulary path_to_settings do_rectify do_equalize" << endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool doRectify = false;
    bool doEqualize = false;

    if(argc >= 4)
    {
        string strDoRectify = argv[3];
        if(strDoRectify == "true" || strDoRectify == "True" || strDoRectify == "1") doRectify = true;
    }

    if(argc >= 5)
    {
        string strDoEqualize = argv[4];
        if(strDoEqualize == "true" || strDoEqualize == "True" || strDoEqualize == "1") doEqualize = true;
    }

    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);

    auto node = std::make_shared<MonoInertialNode>(&SLAM, argv[2], argv[3], argv[4]);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
