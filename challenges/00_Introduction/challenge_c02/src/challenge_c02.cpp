#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c02") {
    subscriber_ =
        create_subscription<std_msgs::msg::String>("c02_input", 10, std::bind(&ChallengeNode::callback, this, _1));
    publisher_ = create_publisher<std_msgs::msg::String>("c02_flag", 10);
    RCLCPP_INFO(this->get_logger(), "Challenge C02 started!");
  }

 private:
  void callback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "helloworld!") {
      std_msgs::msg::String flag_msg;
      flag_msg.data = generate_flag();
      publisher_->publish(flag_msg);
      RCLCPP_INFO(this->get_logger(), "Flag published on /c02_flag");
    } else {
      RCLCPP_WARN(this->get_logger(), "Incorrect input. Try again!");
    }
  }

  std::string generate_flag() const {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

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