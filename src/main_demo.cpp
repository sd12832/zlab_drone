/* Sharan Duggirala Zen Labs */

// NORMAL INCLUDES 
#include <string>

// ROS INCLUDES 
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <zlab_drone/Tak_land.h>
#include <ardrone_autonomy/Navdata.h>

// Global Variables
bool emr_land = false; 
bool circle = false; 
int new_msg = 0; 
int nCommand = 0; 
bool repeatmove = false; 

//Ros Message Intializations
std_msgs::Bool emr_in; 
std_msgs::Bool circle_in; 
std_msgs::String command_in;
zlab_drone::Tak_land tak_land_out;
sensor_msgs::Joy joy_out;  
geometry_msgs::Twist twist_msg_out; 

// Callback Functions 
void emr_callback(const std_msgs::Bool& emr_in) {
	bool e_bool = emr_in.data;

	if (e_bool == true) {
		emr_land == true; 
	} else {
		std::cout << "Fatal Error: False Landing should not be Published"
						<< std::endl;
	}
	// New Message Initializations
	new_msg = 1; 
}

void circle_callback(const std_msgs::Bool& circle_in) {
	bool c_bool = circle_in.data; 

	if (c_bool == true) {
		circle == true;
	} else {
		std::cout << "Fatal Error: False C Detection should not be Published"
						<< std::endl;
	}
	// New Message Initializations
	new_msg = 1; 
}

void command_callback(const std_msgs::String& command_in) {
	// Reference: 
	std::string command = command_in.data;

	if (command == "t") {
		nCommand = 1; 
	} else if (command == "l") {
		nCommand = 2; 
	} else if (command == "MF") {
		nCommand = 3; 
	} else if (command == "MB") {
		nCommand = 4; 
	} else if (command == "MR") {
		nCommand = 5; 
	} else if (command == "ML") {
		nCommand = 6; 
	} else {
		std::cout << "Unknown command. Please try again." << std::endl;
		nCommand = 7; 
	}
	// New Message Initializations
	new_msg = 1; 
}

int main(int argc, char** argv) {

	// Ros Initializations 
	ros::init(argc, argv,"main_demo");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

    // Variable Initializations 
    uint8_t emr_tak_land [3];

    // Subscribers and Publishers
    ros::Publisher pub_tak_land;
    ros::Publisher pub_twist;
    ros::Subscriber emrLand_sub;
    ros::Subscriber circle_sub; 
    ros::Subscriber command_sub;    

    pub_tak_land = node.advertise<zlab_drone::Tak_land>("/tak_land", 1);
    pub_twist = node.advertise<geometry_msgs::Twist>("/twist", 1);
    emrLand_sub = node.subscribe("emr_land", 1, emr_callback);
    circle_sub = node.subscribe("/demo_circle", 1, circle_callback);
    command_sub = node.subscribe("/command", 1, command_callback);

    while (ros::ok()) {
    	if (!new_msg) {

    		ros::spinOnce();
	    	loop_rate.sleep();

    	} else {

	    	if (emr_land == true) {

	    		// Publish Landing Command
	    		tak_land_out.takeoff = 0;
	    		tak_land_out.landing = 1;
	    		tak_land_out.move = 0;

	    		pub_tak_land.publish(tak_land_out);

	    		// Change back the variable
	    		emr_land = false;

	    	} else if (circle == true) {

	    		if (repeatmove == false) {

		    		// Wait Here
		    		std::cout << "Detected Circle" << std::endl; 
		    		std::cout << "Now Sleeping for 30 seconds" << std::endl;
		    		ros::Duration(30).sleep();

		    		// Change back the variable 
		    		circle = false; 

		    		repeatmove = true; 

	    		} else {
	    			// Do nothing 
	    		}


	    	} else {


	    		// Handle the various commands
	    		if (nCommand == 7) {
	    			// Do nothing

	    		} else if (nCommand == 6) {

	    			// Move Left //

	    			// Initialize State and Movement
		    		tak_land_out.takeoff = (uint8_t)0;
	    			tak_land_out.landing = (uint8_t)0;
	    			tak_land_out.move = (uint8_t)1;

				    twist_msg_out.linear.x = 0.25;
	    			twist_msg_out.linear.y = 0;
				    twist_msg_out.linear.z = 0;
				    twist_msg_out.angular.x = 0;
				    twist_msg_out.angular.y = 0;
				    twist_msg_out.angular.z = 0.75;

				    // Publish this information
				    pub_tak_land.publish(tak_land_out);
				    pub_twist.publish(twist_msg_out);

	    		}  else if (nCommand == 5) {

	    			// Move Right //

	    			// Initialize State and Movement
		    		tak_land_out.takeoff = (uint8_t)0;
	    			tak_land_out.landing = (uint8_t)0;
	    			tak_land_out.move = (uint8_t)1;

				    twist_msg_out.linear.x = 0.25;
	    			twist_msg_out.linear.y = 0;
				    twist_msg_out.linear.z = 0;
				    twist_msg_out.angular.x = 0;
				    twist_msg_out.angular.y = 0;
				    twist_msg_out.angular.z = -0.75;

				    // Publish this information
				    pub_tak_land.publish(tak_land_out);
				    pub_twist.publish(twist_msg_out);

	    		}  else if (nCommand == 4) {

	    			// Move Back //

	    			// Initialize State and Movement
		    		tak_land_out.takeoff = (uint8_t)0;
	    			tak_land_out.landing = (uint8_t)0;
	    			tak_land_out.move = (uint8_t)1;

				    twist_msg_out.linear.x = -0.25;
	    			twist_msg_out.linear.y = 0;
				    twist_msg_out.linear.z = 0;
				    twist_msg_out.angular.x = 0;
				    twist_msg_out.angular.y = 0;
				    twist_msg_out.angular.z = 0;

				    // Publish this information
				    pub_tak_land.publish(tak_land_out);
				    pub_twist.publish(twist_msg_out);

	    		}  else if (nCommand == 3) {

	    			// Move Forward //

	    			// Initialize State and Movement
		    		tak_land_out.takeoff = (uint8_t)0;
	    			tak_land_out.landing = (uint8_t)0;
	    			tak_land_out.move = (uint8_t)1;

				    twist_msg_out.linear.x = 0.0001;
	    			twist_msg_out.linear.y = 0;
				    twist_msg_out.linear.z = 0;
				    twist_msg_out.angular.x = 0;
				    twist_msg_out.angular.y = 0;
				    twist_msg_out.angular.z = 0;

				    // Publish this information
				    pub_tak_land.publish(tak_land_out);
				    pub_twist.publish(twist_msg_out);

				    // Sleep for a bit
				    ros::Duration(0.5).sleep();

				    // Set back the State and Movement
				    twist_msg_out.linear.x = 0;
	    			twist_msg_out.linear.y = 0;
				    twist_msg_out.linear.z = 0;
				    twist_msg_out.angular.x = 0;
				    twist_msg_out.angular.y = 0;
				    twist_msg_out.angular.z = 0;

				    // Publish Info 2
				    pub_twist.publish(twist_msg_out);

	    		}  else if (nCommand == 2) {

	    			// Landing //

	    			// Initialize State and Movement
		    		tak_land_out.takeoff = (uint8_t)0;
	    			tak_land_out.landing = (uint8_t)1;
	    			tak_land_out.move = (uint8_t)0;

				    // Publish this information
				    pub_tak_land.publish(tak_land_out);


	    		} else if (nCommand == 1) {

	    			// Takeoff //

	    			// Initialize State and Movement
		    		tak_land_out.takeoff = (uint8_t)1;
	    			tak_land_out.landing = (uint8_t)0;
	    			tak_land_out.move = (uint8_t)0;

				    // Publish this information
				    pub_tak_land.publish(tak_land_out);

	    		} else {
	    			std::cout << "You shouldn't be here." << std::endl; 
	    		}

	    		// Make repeat move what it was again
	    		repeatmove = false; 

	    		// Initialize nCommand
	    		nCommand = 0; 
	    	}
	    	new_msg = 0;
	    } 
    }
    ROS_ERROR("ROS::ok failed- Node Closing");

}