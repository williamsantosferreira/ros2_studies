#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <iostream>
using namespace std::chrono_literals;
using std::placeholders::_1;


class TurtleGoal: public rclcpp::Node{
    private:




        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_pose;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_w;

        rclcpp::TimerBase::SharedPtr timer;

        double x_goal;
        double y_goal;
        double v;
        double w;

        double tol;

        double K;

        geometry_msgs::msg::Twist msg;

        void callback_pose(const turtlesim::msg::Pose::SharedPtr pose);
    public:
        TurtleGoal(): Node("turtle_goal"){
            x_goal = 8;
            y_goal = 8;
            
            K = 3;
            
            tol = 0.1;
            RCLCPP_INFO(this->get_logger(),"Starting Turtle_Goal!");
            pub_w = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
            sub_pose = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleGoal::callback_pose, this, _1));
            timer = this->create_wall_timer(10ms, std::bind(&TurtleGoal::publish, this));
        }

        void publish();
};

int main(int argc, char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<TurtleGoal>());
    rclcpp::shutdown();
    return 0;
}

void TurtleGoal::callback_pose(const turtlesim::msg::Pose::SharedPtr pose){
    double ex, ey, u, theta;

    ex = x_goal - pose->x;
    ey = y_goal - pose->y;

    theta = atan2(ey,ex);

    u = theta - pose->theta;



    if(abs(ex) < tol && abs(ey)<0.1){
        v = 0;
        w = 0;
    }
    else{
        v = sqrt(pow(ex,2)+pow(ey,2));
        w = K*u;
    }

    msg.linear.x = v;
    msg.angular.z = w;

}

void TurtleGoal::publish(){
    pub_w->publish(msg);
}

