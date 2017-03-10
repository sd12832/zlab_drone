/* Sharan Duggirala Zen Labs */

// NORMAL INCLUDES 
#include <stdio.h>
#include <iostream>
#include <string>

// ROS INCLUDES 
#include <ros/ros.h>
#include <std_msgs/Bool.h> 

int main(int argc, char** argv) {

	ros::init(argc, argv,"emergency_land");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    std_msgs::Bool msg; 

    std::cout << "This is the Emergency Land node" << std::endl 
                        << std::endl;
    std::cout << "Please type in 'C', if you would like to land " << std::endl;
    std::cout << "-----------------------------------------------------------";
    

    ros::Publisher command;
    command = node.advertise<std_msgs::Bool>("/emer_land", 1);

    bool land = false;
    std::string input = "";

   	while (ros::ok()) { 

        std::cout << std::endl << std::endl;
        std::cout << "Waiting for command:" << std::endl << std::endl; 
        getline(std::cin, input);
        if ((input[0] == 'c') || (input[0] == 'C')) { 
            land = true; 
            msg.data = land; 
            command.publish(msg); 
            return 0;
        }
        land = false;
    }

}