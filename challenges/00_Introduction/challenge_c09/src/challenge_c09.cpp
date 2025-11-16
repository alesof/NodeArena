#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c09") {
    this->declare_parameter<int>("access_level", 0);
    service_ = create_service<std_srvs::srv::Trigger>(
        "c09_get_flag", std::bind(&ChallengeNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Challenge C09 (Parameter-Protected Service) started!");
    RCLCPP_INFO(this->get_logger(), "A service returns the flag only if the correct parameter value is set.");
  }

 private:
  void handle_request(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                      std_srvs::srv::Trigger::Response::SharedPtr res) {
    int level = this->get_parameter("access_level").as_int();
    if (level == required_level_) {
      res->success = true;
      res->message = generate_flag();
      RCLCPP_INFO(this->get_logger(), "Access granted — flag returned.");
    } else {
      res->success = false;
      res->message = "Access denied";
      RCLCPP_WARN(this->get_logger(), "Access denied — wrong parameter value.");
    }
  }

  std::string generate_flag() const {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  const int required_level_ = 42;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}