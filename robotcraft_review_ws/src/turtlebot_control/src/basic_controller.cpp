#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

class TurtleController
{
private:
    ros::NodeHandle private_nh;
    ros::Publisher cmd_vel_pub;
    double linear_speed;
    double angular_speed;

    geometry_msgs::Twist calculateCommand()
    {
        geometry_msgs::Twist msg;
        // Control code goes here //
        msg.linear.x = linear_speed;
        msg.angular.z = angular_speed;
        return msg;
    }

public:
    TurtleController() : private_nh("~"), linear_speed(0.5), angular_speed(0.5) {
        // Initialize ROS
        ros::NodeHandle n;

        // Create a publisher object, able to push messages
        cmd_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);

        // Get parameters from the parameter server
        if (!private_nh.getParam("linear_speed", linear_speed)) {
            ROS_WARN("Failed to get param 'linear_speed', defaulting to 0.5");
        }
        if (!private_nh.getParam("angular_speed", angular_speed)) {
            ROS_WARN("Failed to get param 'angular_speed', defaulting to 0.5");
        }
    }

    void run() {
        // Send messages in a loop
        ros::Rate loop_rate(10);
        while (ros::ok()) {
            // Calculate the command to apply
            geometry_msgs::Twist msg = calculateCommand();

            // Publish the new command
            cmd_vel_pub.publish(msg);  // Publish message
            
            std::cout << "Linear Speed: " << linear_speed << std::endl;
            std::cout << "Angular Speed: " << angular_speed << std::endl;

            // And throttle the loop
            loop_rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "basic_controller");

    // Create our controller object and run it
    TurtleController controller;
    controller.run();

    // And make good on our promise
    return 0;
}

