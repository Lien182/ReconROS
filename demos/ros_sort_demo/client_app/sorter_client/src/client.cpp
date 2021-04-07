#include "rclcpp/rclcpp.hpp"
#include "sorter_msgs/msg/sortdata.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;


class ClientNode : public rclcpp::Node
{
  public:
    ClientNode() : Node("SorterClientNode"), count_(0)
    {
      publisher_ = this->create_publisher<sorter_msgs::msg::Sortdata>("unsorted", 10);
      timer_ = this->create_wall_timer(2s, std::bind(&ClientNode::timer_callback, this));

 	    subscription_ = this->create_subscription<sorter_msgs::msg::Sortdata>("sorted", 10, std::bind(&ClientNode::topic_callback, this, std::placeholders::_1));
    }

  private:
    void timer_callback()
    {
      static int cnt = 0; 
      auto message = sorter_msgs::msg::Sortdata();
      message.sortdata.resize(2048);
      std::generate(message.sortdata.begin(), message.sortdata.end(), std::rand);

      RCLCPP_INFO(this->get_logger(), "Unsorted data published, Cnt='%d'", cnt++);
      publisher_->publish(message);
    }


	  void topic_callback(const sorter_msgs::msg::Sortdata::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Sorted: %d", (std::is_sorted(msg->sortdata.begin(), msg->sortdata.end())));
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sorter_msgs::msg::Sortdata>::SharedPtr publisher_;
    size_t count_;

    rclcpp::Subscription<sorter_msgs::msg::Sortdata>::SharedPtr subscription_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClientNode>());
  rclcpp::shutdown();
  return 0;
}

