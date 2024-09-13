#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("number_counter")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::Int64>("/number", 10, std::bind(&NumberCounterNode::callbackNumber, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::String>("/number_count", 10);
    }

private:
    void callbackNumber(const std_msgs::msg::Int64::SharedPtr msg)
    {
        ++counter_[msg->data];

        auto count_data = std_msgs::msg::String();
        count_data.data = std::to_string(msg->data) +  " : " + std::to_string(counter_[msg->data]);

        publisher_->publish(count_data);
    }

    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::unordered_map<int, int> counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node{std::make_shared<NumberCounterNode>()};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}