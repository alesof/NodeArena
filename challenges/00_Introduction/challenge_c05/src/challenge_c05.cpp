#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c05") {
    subscriber_ = create_subscription<std_msgs::msg::String>(
        "c05_input", 10, std::bind(&ChallengeNode::relay_callback, this, std::placeholders::_1));
    publisher_ = create_publisher<std_msgs::msg::String>("c05_output", 10);
    RCLCPP_INFO(this->get_logger(), "Challenge C05 (Topic Relay) started!");
  }

 private:
  void relay_callback(const std_msgs::msg::String::SharedPtr msg) {
    auto out = std_msgs::msg::String();
    out.data = msg->data;
    publisher_->publish(out);
    if (msg->data == expected_trigger_) {
      std_msgs::msg::String flag;
      flag.data = generate_flag();
      publisher_->publish(flag);
      RCLCPP_INFO(this->get_logger(), "Trigger received — flag relayed on /c05_output");
    }
  }

  std::string generate_flag() const {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

  const std::string expected_trigger_ = "release_flag";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}