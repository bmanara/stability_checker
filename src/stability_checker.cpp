#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"
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

            try 
            {
                pinocchio::urdf::buildModel(urdf_file_path, pinocchio::JointModelFreeFlyer(), model);
                RCLCPP_INFO(this->get_logger(), "Model built successfully with %d joints and %d frames.", model.njoints, model.nframes);
            } catch (const std::exception & e) 
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to build model from URDF: %s", e.what());
                return;
            }

            data = pinocchio::Data(model);

            joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&StabilityCheckerNode::joint_state_callback, this, std::placeholders::_1)
            );

            com_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
                "/robot_com", 10
            );
        }

    private:
        pinocchio::Model model; // Make model a member variable
        pinocchio::Data data; // Data object for computations

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_publisher_;

        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            // Create a configuration vector from the joint state message
            Eigen::VectorXd q = pinocchio::neutral(model);
            for (size_t i = 0; i < msg->name.size(); ++i) {
                const std::string& joint_name = msg->name[i];
                if (model.existJointName(joint_name)) {
                    auto joint_id = model.getJointId(joint_name);
                    auto q_index = model.joints[joint_id].idx_q();
                    q[q_index] = msg->position[i];
                } else {
                    RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in model.", joint_name.c_str());
                }
            }

            // Compute relative (to base_link) center of mass
            pinocchio::centerOfMass(model, data, q);
            const Eigen::Vector3d& com_position = data.com[0];

            // Publish result as PointStamped message
            auto com_msg = geometry_msgs::msg::PointStamped(); 
            com_msg.header.stamp = this->get_clock()->now();
            com_msg.header.frame_id = "base_link"; // Assuming base_link is the reference frame
            com_msg.point.x = com_position.x();
            com_msg.point.y = com_position.y();
            com_msg.point.z = com_position.z();

            com_publisher_->publish(com_msg);
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
