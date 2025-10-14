#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <cmath>

class ServoStopNode : public rclcpp::Node
{
public:
    ServoStopNode() : Node("servo_stop_node")
    {
        // 力センサの購読
        force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/calibrated_force_data", 1,
            std::bind(&ServoStopNode::force_callback, this, std::placeholders::_1));

        // サーボ指令の購読
        servo_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 1,
            std::bind(&ServoStopNode::servo_callback, this, std::placeholders::_1));

        // サーボ指令の発行
        servo_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds_filtered", 1);

        RCLCPP_INFO(this->get_logger(), "ServoStopNode started.");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr servo_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_pub_;

    double fx_{0.0}, fy_{0.0}, fz_{0.0};
    double threshold_{6.0};

    // 力センサの値を保持
    void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        fx_ = msg->wrench.force.x;
        fy_ = msg->wrench.force.y;
        fz_ = msg->wrench.force.z;
    }

    // Twist を購読 → 閾値チェック → publish
    void servo_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TwistStamped cmd = *msg;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";

        if (std::abs(fx_) >= threshold_ || std::abs(fy_) >= threshold_ || std::abs(fz_) >= threshold_)
        {
            RCLCPP_WARN(this->get_logger(), "Force exceeded threshold! Sending STOP command.");
            cmd.twist.linear.x = 0.0;
            cmd.twist.linear.y = 0.0;
            cmd.twist.linear.z = 0.0;
            cmd.twist.angular.x = 0.0;
            cmd.twist.angular.y = 0.0;
            cmd.twist.angular.z = 0.0;
        }
        cmd.twist.linear.x =  cmd.twist.linear.x*-1;
        cmd.twist.linear.y = cmd.twist.linear.y*-1;
        cmd.twist.linear.z = cmd.twist.linear.z*-1;
        cmd.twist.angular.x = cmd.twist.angular.x*-1;
        cmd.twist.angular.y = cmd.twist.angular.y*-1;
        cmd.twist.angular.z = cmd.twist.angular.z*-1;

        servo_pub_->publish(cmd);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoStopNode>());
    rclcpp::shutdown();
    return 0;
}