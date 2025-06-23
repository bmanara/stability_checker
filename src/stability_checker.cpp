#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"


using namespace pinocchio;

class StabilityCheckerNode : public rclcpp::Node
{
    public:
        StabilityCheckerNode() : Node("stability_checker_node")
        {
            RCLCPP_INFO(this->get_logger(), "Stability Checker Node has been started.");

            this->declare_parameter("urdf_file_path", "no file provided");
            std::string urdf_file_path = this->get_parameter("urdf_file_path").as_string();
            RCLCPP_INFO(this->get_logger(), "Supplied stability checker URDF file path: %s", urdf_file_path.c_str());

            Model model;
            try 
            {
                pinocchio::urdf::buildModel(urdf_file_path, model);
                Data data(model);
            } catch (const std::exception & e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to build model from URDF: %s", e.what());
                return;
            }
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StabilityCheckerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
