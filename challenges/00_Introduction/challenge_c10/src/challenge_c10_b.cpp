#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class NodeB : public rclcpp::Node {
 public:
  NodeB() : Node("c10_b") {
    sub_ = create_subscription<std_msgs::msg::String>("c10_use_hint", 10,
                                                      std::bind(&NodeB::on_hint, this, std::placeholders::_1));
    publisher_ = create_publisher<std_msgs::msg::String>("c10_flag", 10);
    RCLCPP_INFO(this->get_logger(), "C10 Node B (flag holder) started!");
  }

 private:
  void on_hint(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == expected_hint_) {
      std_msgs::msg::String out;
      out.data = generate_flag();
      publisher_->publish(out);
      RCLCPP_INFO(this->get_logger(), "Correct hint received — final flag published on /c10_flag");
    } else {
      RCLCPP_WARN(this->get_logger(), "Incorrect hint received.");
    }
  }

  std::string generate_flag() const {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  const std::string expected_hint_ = "blue_key";
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeB>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}