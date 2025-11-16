#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c03") {
    this->declare_parameter<std::string>("secret", "wrong");
    publisher_ = create_publisher<std_msgs::msg::String>("c03_flag", 10);
    timer_ = create_wall_timer(500ms, std::bind(&ChallengeNode::check_parameter, this));
    RCLCPP_INFO(this->get_logger(), "Challenge C03 started!");
  }

 private:
  void check_parameter() {
    auto value = this->get_parameter("secret").as_string();
    if (value == "opensesame") {
      std_msgs::msg::String msg;
      msg.data = generate_flag();
      publisher_->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Correct parameter! Flag published.");
    }
  }

  std::string generate_flag() const {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}