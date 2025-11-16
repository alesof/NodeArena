#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class ChallengeNode : public rclcpp::Node {
public:
    ChallengeNode() : Node("c201") {
        RCLCPP_INFO(this->get_logger(), "🧩 Challenge c201 started!");
        RCLCPP_INFO(this->get_logger(), "Flag: CTF{wowie}");
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChallengeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
