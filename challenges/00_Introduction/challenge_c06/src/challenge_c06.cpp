#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

static std::string caesar_shift(const std::string& s, int shift) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c >= 'a' && c <= 'z') {
      out.push_back(char((c - 'a' + shift + 26) % 26 + 'a'));
    } else if (c >= 'A' && c <= 'Z') {
      out.push_back(char((c - 'A' + shift + 26) % 26 + 'A'));
    } else {
      out.push_back(c);
    }
  }
  return out;
}

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c06") {
    this->declare_parameter<int>("decoder_key", 3);
    publisher_ = create_publisher<std_msgs::msg::String>("c06_code", 10);
    timer_ = create_wall_timer(1s, std::bind(&ChallengeNode::publish_encoded, this));
    RCLCPP_INFO(this->get_logger(), "Challenge C06 (Parameter-Gated Topic) started!");
    RCLCPP_INFO(this->get_logger(), "Hint: look for a parameter that explains how the code is encoded.");
  }

 private:
  void publish_encoded() {
    const std::string plain = generate_flag();
    int shift = this->get_parameter("decoder_key").as_int();
    std_msgs::msg::String msg;
    msg.data = caesar_shift(plain, shift);
    publisher_->publish(msg);
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}