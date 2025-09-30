#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class KeyToActionCancel : public rclcpp::Node {
public:
  KeyToActionCancel()
  : Node("key_to_action_cancel")
  {
    
    action_name_  = this->declare_parameter<std::string>(
      "action_name", "/scaled_joint_trajectory_controller/follow_joint_trajectory");
    trigger_key_  = this->declare_parameter<std::string>("trigger_key", "a");   
    stop_key_     = this->declare_parameter<std::string>("stop_key", "s");      
    period_ms_    = this->declare_parameter<int>("period_ms", 10);              
    stop_on_q_    = this->declare_parameter<bool>("stop_on_q", true);
    if (trigger_key_.empty()) trigger_key_ = "a";
    if (stop_key_.empty())    stop_key_    = "s";
    trigger_char_ = trigger_key_.front();
    stop_char_    = stop_key_.front();


    client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name_);
    RCLCPP_INFO(get_logger(), "Action client to: %s", action_name_.c_str());

    tcgetattr(STDIN_FILENO, &oldt_);
    termios newt = oldt_;
    newt.c_lflag &= ~(ICANON | ECHO);          
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int flags = fcntl(STDIN_FILENO, F_GETFL);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    // --- 入力監視スレッド ---
    running_.store(true);
    key_thread_ = std::thread([this](){ key_loop(); });

    RCLCPP_INFO(get_logger(),
      "Press '%c' to START cancel loop, '%c' to STOP. (q to quit: %s)",
      trigger_char_, stop_char_, stop_on_q_ ? "enabled" : "disabled");
  }

  ~KeyToActionCancel() override {
    running_.store(false);
    stop_cancel_loop_.store(true);
    if (cancel_thread_.joinable()) cancel_thread_.join();
    if (key_thread_.joinable())    key_thread_.join();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt_);
  }

private:
  void key_loop() {
    while (rclcpp::ok() && running_.load()) {
      char c;
      ssize_t n = read(STDIN_FILENO, &c, 1);
      if (n == 1) {
        if (stop_on_q_ && c == 'q') {
          rclcpp::shutdown();
          break;
        }
        if (c == trigger_char_) {
          start_cancel_loop();
        } else if (c == stop_char_) {
          stop_cancel_loop_.store(true);
        }
      } else {
        rclcpp::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void start_cancel_loop() {

    bool expected = false;
    if (!cancel_running_.compare_exchange_strong(expected, true)) {
      RCLCPP_DEBUG(get_logger(), "Cancel loop already running.");
      return;
    }

    stop_cancel_loop_.store(false);

    if (cancel_thread_.joinable()) cancel_thread_.join();
    cancel_thread_ = std::thread([this]() {
      auto period = std::chrono::milliseconds(std::max(1, period_ms_));

      if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(get_logger(), "Action server not available: ");
      }

      while (rclcpp::ok() && !stop_cancel_loop_.load()) {
        client_->async_cancel_all_goals();
        RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 100,
                             "Sending cancel requests ");
        rclcpp::sleep_for(period);
      }
      cancel_running_.store(false);
      RCLCPP_INFO(get_logger(), "Cancel loop stopped.");
    });
  }


  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  std::string action_name_;

  std::string trigger_key_;
  std::string stop_key_;
  char trigger_char_{'a'};
  char stop_char_{'s'};
  bool stop_on_q_{true};
  int period_ms_{10};

  std::thread key_thread_;
  std::thread cancel_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> cancel_running_{false};
  std::atomic<bool> stop_cancel_loop_{true};
  termios oldt_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyToActionCancel>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
