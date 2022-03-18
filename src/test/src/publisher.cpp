#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher: public rclcpp::Node{
    private:
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        int count;
        rclcpp::TimerBase::SharedPtr timer_;
    public:
        MinimalPublisher()
        : Node("publisher") ,count(0){
            publisher_ = this->create_publisher<std_msgs::msg::String>("topic",10);
            timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::publish, this));
        }

        void publish(){
            std_msgs::msg::String msg;
            msg.data = "Hello " + std::to_string(count++);
            publisher_->publish(msg);
        }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
