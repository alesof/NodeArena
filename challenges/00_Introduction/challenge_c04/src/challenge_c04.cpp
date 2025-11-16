#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c04") {
    service_ = create_service<std_srvs::srv::Trigger>(
        "c04_unlock", std::bind(&ChallengeNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Challenge C04 started!");
  }

 private:
  void handle_request(const std_srvs::srv::Trigger::Request::SharedPtr req,
                      std_srvs::srv::Trigger::Response::SharedPtr resp) {
    (void)req;
    resp->success = true;
    resp->message = generate_flag();
    RCLCPP_INFO(this->get_logger(), "Service called. Flag sent.");
  }

  std::string generate_flag() const {
#ifdef FLAG_VALUE
    return FLAG_VALUE;
#else
    return "CTF{flag_not_found}";
#endif
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}