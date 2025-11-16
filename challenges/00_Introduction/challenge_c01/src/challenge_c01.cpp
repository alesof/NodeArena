#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c01") {
    publisher_ = create_publisher<std_msgs::msg::String>("c01", 10);
    timer_ = create_wall_timer(1s, std::bind(&ChallengeNode::timer_cb, this));
    RCLCPP_INFO(this->get_logger(), "Challenge C01 started!");
  }

  void timer_cb() {
    std_msgs::msg::String msg;
    msg.data = generate_flag();
    publisher_->publish(msg);
  }

  std::string generate_flag() {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

 private:
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