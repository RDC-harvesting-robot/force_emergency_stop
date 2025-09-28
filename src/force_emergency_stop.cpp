#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
# include <chrono>

using namespace std::chrono_literals;

class TrajectoryCancelNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using Trigger = std_srvs::srv::Trigger;

  TrajectoryCancelNode() : Node("trajectory_cancel_node")
  {

    threshold_ = this->declare_parameter<double>("threshold", 3.0);

    client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

    force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/calibrated_force_data", 10,
      std::bind(&TrajectoryCancelNode::force_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&TrajectoryCancelNode::try_cancel, this));

    srv_ = this->create_service<Trigger>(
      "cancel_trajectory",
      std::bind(&TrajectoryCancelNode::handle_cancel_service, this,
                std::placeholders::_1, std::placeholders::_2));
  }

private:
  void try_cancel()
  {
    if (std::abs(fx) >= threshold_ || std::abs(fy) >= threshold_ || std::abs(fz) >= threshold_) {
      while(rclcpp::ok()){
        client_->async_cancel_all_goals();
        RCLCPP_WARN(this->get_logger(), "Force threshold exceeded! Sent cancel request.");
        rclcpp::sleep_for(1000ms);
      }

    }

    const double delta = 1e-3;
    if (std::abs(fx - fx_buff) < delta &&
        std::abs(fy - fy_buff) < delta &&
        std::abs(fz - fz_buff) < delta) {
      error_count++;
    } else {
      error_count = 0;
    }

    if (error_count >= 10) {
      RCLCPP_WARN(this->get_logger(), "Force sensor might not be sending data.");
    }

    fx_buff = fx;
    fy_buff = fy;
    fz_buff = fz;
  }

  void force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    fx = msg->wrench.force.x;
    fy = msg->wrench.force.y;
    fz = msg->wrench.force.z;
  }

  void handle_cancel_service(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response)
  {
    (void)request;  // unused
    client_->async_cancel_all_goals();
    RCLCPP_INFO(this->get_logger(), "Service call received: canceled trajectory.");
    response->success = true;
    response->message = "Trajectory canceled via service.";
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  rclcpp::Service<Trigger>::SharedPtr srv_;

  double fx = 0, fy = 0, fz = 0;
  double fx_buff = 0, fy_buff = 0, fz_buff = 0;
  int error_count = 0;

  double threshold_{3.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryCancelNode>();
  rclcpp::spin(node);
  return 0;
}
