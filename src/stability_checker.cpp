#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"


using namespace pinocchio;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class StabilityCheckerNode : public rclcpp::Node
{
    public:
        StabilityCheckerNode() : Node("stability_checker_node")
        {
            rclcpp::QoS qos = rclcpp::QoS(10);
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

            joint_state_subscription.subscribe(this, "joint_states", qos.get_rmw_qos_profile());
            odometry_subscription.subscribe(this, "odom", qos.get_rmw_qos_profile());

            joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states_out", qos);
            odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_out", qos);
            com_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("robot_com", qos);

            timer_ = this->create_wall_timer(1000ms, std::bind(&StabilityCheckerNode::TimerCallback, this));

            uint32_t queue_size = 10;
            sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::JointState, nav_msgs::msg::Odometry>>(joint_state_subscription, odometry_subscription, queue_size);
            sync_->registerCallback(std::bind(&StabilityCheckerNode::SyncCallback, this, _1, _2));
        }

    private:
        pinocchio::Model model; // Make model a member variable
        pinocchio::Data data; // Data object for computations

        // Subscribers for joint states and odometry
        message_filters::Subscriber<sensor_msgs::msg::JointState> joint_state_subscription;
        message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_subscription;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr com_publisher_;

        std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::JointState, nav_msgs::msg::Odometry>> sync_;
        rclcpp::TimerBase::SharedPtr timer_;

        void SyncCallback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state_msg,
                          const nav_msgs::msg::Odometry::ConstSharedPtr odometry_msg)
        {
            RCLCPP_INFO(this->get_logger(), "Received synchronized messages with %u and %u as times.", joint_state_msg->header.stamp.sec, odometry_msg->header.stamp.sec);

            // Create a configuration vector from the joint state message
            Eigen::VectorXd q = pinocchio::neutral(model);
            for (size_t i = 0; i < joint_state_msg->name.size(); ++i) {
                const std::string& joint_name = joint_state_msg->name[i];
                if (model.existJointName(joint_name)) {
                    auto joint_id = model.getJointId(joint_name);
                    auto q_index = model.joints[joint_id].idx_q();
                    q[q_index] = joint_state_msg->position[i];
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

        void TimerCallback()
        {
            sensor_msgs::msg::JointState joint_state_msg;
            nav_msgs::msg::Odometry odometry_msg;
            auto now = rclcpp::Clock().now();
            double time_sec = now.seconds();

            // TODO: Fill up with dummy data for testing
            joint_state_msg.header.stamp = now;
            joint_state_msg.header.frame_id = "";

            joint_state_msg.name = {
                "joint_1", "joint_2", "joint_3", 
                "joint_4", "joint_5", "joint_6"
                // If your URDF has different names like "shoulder_pan_joint", use those instead.
            };
            
            // --- Joint Positions, Velocities, and Efforts ---
            // We will make one joint oscillate to test dynamic effects.
            double oscillation_frequency_rad_s = 0.8;
            double amplitude_rad = 0.7; // ~40 degrees

            double joint_2_angle    = amplitude_rad * sin(oscillation_frequency_rad_s * time_sec);
            double joint_2_velocity = amplitude_rad * oscillation_frequency_rad_s * cos(oscillation_frequency_rad_s * time_sec);

            joint_state_msg.position.resize(joint_state_msg.name.size());
            joint_state_msg.velocity.resize(joint_state_msg.name.size());
            joint_state_msg.effort.resize(joint_state_msg.name.size()); // Optional, can be empty

            std::fill(joint_state_msg.position.begin(), joint_state_msg.position.end(), 0.0);
            joint_state_msg.position[1] = joint_2_angle; // Index 1 corresponds to "abb_joint_2"

            std::fill(joint_state_msg.velocity.begin(), joint_state_msg.velocity.end(), 0.0);
            joint_state_msg.velocity[1] = joint_2_velocity;

            std::fill(joint_state_msg.effort.begin(), joint_state_msg.effort.end(), 0.0);

            odometry_msg.header.stamp = now;
            odometry_msg.header.frame_id = "odom";
            odometry_msg.child_frame_id = "base_link";

            odometry_msg.pose.pose.position.x = 0.5 * time_sec;  // Moves forward at 0.5 m/s
            odometry_msg.pose.pose.position.y = 0.2 * sin(0.2 * time_sec); // Weaves left and right
            odometry_msg.pose.pose.position.z = 0.0;             // Stays on the ground

            odometry_msg.pose.pose.orientation.x = 0.0;
            odometry_msg.pose.pose.orientation.y = 0.0;
            odometry_msg.pose.pose.orientation.z = 0.0;
            odometry_msg.pose.pose.orientation.w = 1.0;

            odometry_msg.twist.twist.linear.x = 0.5; // Constant forward velocity
            odometry_msg.twist.twist.linear.y = 0.2 * 0.2 * cos(0.2 * time_sec); // Derivative of sin is cos
            odometry_msg.twist.twist.linear.z = 0.0;

            odometry_msg.twist.twist.angular.x = 0.0;
            odometry_msg.twist.twist.angular.y = 0.0;
            odometry_msg.twist.twist.angular.z = 0.0; // Not turning in this simple test

            // Publish the messages
            joint_state_publisher_->publish(joint_state_msg);
            odometry_publisher_->publish(odometry_msg);
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
