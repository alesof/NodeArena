#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class ChallengeNode : public rclcpp::Node {
 public:
  ChallengeNode() : Node("c08") {
    service_ = create_service<std_srvs::srv::Trigger>(
        "c08_request", std::bind(&ChallengeNode::handle_request, this, std::placeholders::_1, std::placeholders::_2));
    response_sub_ = create_subscription<std_msgs::msg::String>(
        "c08_response", 10, std::bind(&ChallengeNode::on_response, this, std::placeholders::_1));
    publisher_ = create_publisher<std_msgs::msg::String>("c08_flag", 10);
    challenge_word_ = "open_sesame_42";
    RCLCPP_INFO(this->get_logger(), "Challenge C08 (Service + Topic Validation) started!");
  }

 private:
  void handle_request(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                      std_srvs::srv::Trigger::Response::SharedPtr res) {
    res->success = true;
    res->message = challenge_word_;
    RCLCPP_INFO(this->get_logger(), "Service called — returned challenge word.");
  }

  void on_response(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == challenge_word_) {
      std_msgs::msg::String out;
      out.data = generate_flag();
      publisher_->publish(out);
      RCLCPP_INFO(this->get_logger(), "Valid response received — flag published on /c08_flag");
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr response_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string challenge_word_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChallengeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}