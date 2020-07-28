#include "rclcpp/rclcpp.hpp"
#include "my_reconros_services/srv/sort.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("srvclient");
  rclcpp::Client<my_reconros_services::srv::Sort>::SharedPtr client =
    node->create_client<my_reconros_services::srv::Sort>("sorter");

  auto request = std::make_shared<my_reconros_services::srv::Sort::Request>();
  request->unsorted.resize(2048);
  std::generate(request->unsorted.begin(), request->unsorted.end(), std::rand);
  

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sorted: %d", (std::is_sorted(result.get()->sorted.begin(), result.get()->sorted.end())));
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}