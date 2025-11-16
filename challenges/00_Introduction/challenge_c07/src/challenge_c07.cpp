#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c07"), step_state_(0) {
    sub1_ = create_subscription<std_msgs::msg::String>(
        "c07_step1", 10, std::bind(&ChallengeNode::on_step1, this, std::placeholders::_1));
    sub3_ = create_subscription<std_msgs::msg::String>(
        "c07_step3", 10, std::bind(&ChallengeNode::on_step3, this, std::placeholders::_1));
    publisher_ = create_publisher<std_msgs::msg::String>("c07_flag", 10);
    this->declare_parameter<bool>("step2", false);
    RCLCPP_INFO(this->get_logger(), "Challenge C07 (Multi-Step Unlock) started!");
    RCLCPP_INFO(this->get_logger(), "Complete steps in order: step1 -> set parameter -> step3");
  }

 private:
  void on_step1(const std_msgs::msg::String::SharedPtr msg) {
    if (step_state_ == 0 && msg->data == "start") {
      step_state_ = 1;
      RCLCPP_INFO(this->get_logger(), "Step 1 complete");
    }
  }

  void on_step3(const std_msgs::msg::String::SharedPtr msg) {
    bool step2 = this->get_parameter("step2").as_bool();
    if (step_state_ == 1 && step2 && msg->data == "finish") {
      std_msgs::msg::String out;
      out.data = generate_flag();
      publisher_->publish(out);
      RCLCPP_INFO(this->get_logger(), "All steps complete — flag published on /c07_flag");
      step_state_ = 0;
    } else {
      RCLCPP_WARN(this->get_logger(), "Sequence not satisfied yet.");
    }
  }

  std::string generate_flag() const {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

  int step_state_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub3_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}