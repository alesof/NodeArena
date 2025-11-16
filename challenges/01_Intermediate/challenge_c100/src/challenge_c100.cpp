#include <cmath>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c100") {
    publisher_ = create_publisher<geometry_msgs::msg::Quaternion>("c100/orientation", 10);
    subscriber_ = create_subscription<geometry_msgs::msg::Quaternion>(
        "c100/inverse", 10, std::bind(&ChallengeNode::callback, this, std::placeholders::_1));
    timer_ = create_wall_timer(1s, std::bind(&ChallengeNode::publish_rotation, this));

    RCLCPP_INFO(this->get_logger(), "Challenge c100 started!");
  }

 private:
  void publish_rotation() {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(M_PI / 4);
    q.w = std::cos(M_PI / 4);
    publisher_->publish(q);
  }

  void callback(const geometry_msgs::msg::Quaternion::SharedPtr msg) {
    // Check if msg is inverse of original rotation (simple quaternion inverse check)
    if (std::fabs(msg->x) < 1e-6 && std::fabs(msg->y) < 1e-6 && std::fabs(msg->z + std::sin(M_PI / 4)) < 1e-6 &&
        std::fabs(msg->w - std::cos(M_PI / 4)) < 1e-6) {
// Flag is embedded at compile time
#ifdef FLAG_VALUE
      RCLCPP_INFO(this->get_logger(), "%s", FLAG_VALUE);
#else
      RCLCPP_INFO(this->get_logger(), "CTF{flag_not_found}");
#endif
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}