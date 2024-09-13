#include <random>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

namespace {
    class RandomNumberGenerator{
    public:
        RandomNumberGenerator() : rd{}, gen(rd()) {

        }
    
    int generateInt(int min = 0, int max = 10) {
        std::uniform_int_distribution<int> dist(min, max);
        return dist(gen);
    }

    private:
        std::random_device rd;  // Generate random seed number
        std::mt19937 gen;       // Mersenne Twister 19937 generator
    };
}

class NumberPublisherNode : public rclcpp::Node
{
public:
    NumberPublisherNode() : Node("number_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been created");
    }

private:
    void publishNumber()
    {
        auto msg{std_msgs::msg::Int64()};
        msg.data = rng.generateInt();
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    ::RandomNumberGenerator rng;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node{std::make_shared<NumberPublisherNode>()};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}