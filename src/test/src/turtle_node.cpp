#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "termios.h"
#include <unistd.h>
#include <cmath>

#include <iostream>
using namespace std::chrono_literals;
using std::placeholders::_1;

class TurtleNode: public rclcpp::Node{
    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_pose;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_odometry;
        rclcpp::TimerBase::SharedPtr timer;

        float w,v,dt;
        geometry_msgs::msg::Point odom;
        void callback_pose(const turtlesim::msg::Pose::SharedPtr p);

    public:
        TurtleNode(): Node("turtle_node"){
            RCLCPP_INFO(this->get_logger(),"Starting Turtle_Node!");
            
            pub_odometry = this->create_publisher<geometry_msgs::msg::Point>("odometry",10);
            sub_pose = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleNode::callback_pose, this, _1));
            timer = this->create_wall_timer(10ms, std::bind(&TurtleNode::publish, this));

            dt = 0.01;
            odom.x = 5.54444;
            odom.y = 5.54444;
            odom.z = 0; //theta
        }

        void odometry();
        void publish();

};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TurtleNode>());
    rclcpp::shutdown();
    return 0;
}

void TurtleNode::callback_pose(const turtlesim::msg::Pose::SharedPtr p){
    w = p->angular_velocity;
    v = p->linear_velocity;
}

void TurtleNode::odometry(){
    odom.z = odom.z + w*dt;
    odom.x = odom.x + cos(odom.z)*v*dt;
    odom.y = odom.y + sin(odom.z)*v*dt;
}

void TurtleNode::publish(){
    odometry();
    pub_odometry->publish(this->odom);
}
