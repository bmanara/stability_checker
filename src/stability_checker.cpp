#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
                pinocchio::urdf::buildModel(urdf_file_path, pinocchio::JointModelFreeFlyer(), model);
                RCLCPP_INFO(this->get_logger(), "Model built successfully with %d joints and %d frames.", model.njoints, model.nframes);
            } catch (const std::exception & e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to build model from URDF: %s", e.what());
                return;
            }

            Data data(model);

            joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&StabilityCheckerNode::joint_state_callback, this, std::placeholders::_1)
            );
        }

    private:
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

        void joint_state_callback(const sensor_msgs::msg::JointState)
        {
            RCLCPP_INFO(this->get_logger(), "Received joint state message. Attempting to compute stability...");
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
