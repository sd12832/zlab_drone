/* Sharan Duggirala Zen Labs */

// NORMAL INCLUDES 
#include <stdio.h>
#include <iostream>
#include <string>

// ROS INCLUDES 
#include <ros/ros.h>
#include <std_msgs/String.h> 

int main(int argc, char** argv) {

	ros::init(argc, argv,"demo_usrInput");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    std_msgs::String msg; 

    std::cout << "Welcome to Sharan's demo drone fly node." << std::endl 
                        << std::endl;
    std::cout << "Please take note of all the commands below: " << std::endl;
    std::cout << "1. To Takeoff: Type in t <height in m>" << std::endl; 
    std::cout << "2. To Land: Type in l" << std::endl;
    std::cout << "3. To Move Forward: Type in MF <distance in cm>" 
                        << std::endl; 
    std::cout << "4. To Move Backwards: Type in MB <distance in cm>" 
                        << std::endl;
    std::cout << "5. To Move to the Right: Type in MR <distance in cm>" 
                        << std::endl; 
    std::cout << "6. To Move to the Left: Type in ML <distance in cm>" 
                        << std::endl << std::endl;  
    std::cout << "TO CLOSE THE NODE TYPE IN 'C'." << std::endl << std::endl; 
    std::cout << "-----------------------------------------------------------";
    

    ros::Publisher command;
    command = node.advertise<std_msgs::String>("/command", 1);

    std::string input = "";

   	while (ros::ok()) { 

        std::cout << std::endl << std::endl;
        std::cout << "Type your Command Below:" << std::endl << std::endl; 
        getline(std::cin, input);
        if ((input[0] == 'C') || (input[0] == 'c')) {
            return 0; 
        } else {
            msg.data = input;
            command.publish(msg); 
        }

    }

}