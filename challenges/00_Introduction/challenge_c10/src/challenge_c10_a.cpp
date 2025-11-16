#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

class NodeA : public rclcpp::Node {
 public:
  NodeA() : Node("c10_a") {
    service_ = create_service<std_srvs::srv::Trigger>(
        "c10_hint", std::bind(&NodeA::handle, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "C10 Node A (hint provider) started!");
  }

 private:
  void handle(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
              std_srvs::srv::Trigger::Response::SharedPtr res) {
    res->success = true;
    res->message = expected_hint_;
    RCLCPP_INFO(this->get_logger(), "Hint requested — hint returned.");
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
  const std::string expected_hint_ = "blue_key";
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NodeA>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}