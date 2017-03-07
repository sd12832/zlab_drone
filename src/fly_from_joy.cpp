/* Sharan Duggirala Zen Labs */
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <zlab_drone/Tak_land.h>
#include <ardrone_autonomy/Navdata.h>

double joy_x_, joy_y_, joy_z_;
double joy_x, joy_y, joy_z;
int new_msg = 0;
float forget = 0.99;
double joy_x_old, joy_y_old, joy_z_old;
bool takeoff, landing; 
int drone_state; 

geometry_msgs::Twist twist_msg;
std_msgs::Empty emp_msg;
geometry_msgs::Vector3 v3_msg; 
sensor_msgs::Joy joy_msg_in;
zlab_drone::Tak_land tak_land_in; 
geometry_msgs::Twist twist_msg_hover; 
ardrone_autonomy::Navdata navdata_in;
	
void joy_callback(const sensor_msgs::Joy& joy_msg_in) {
    
    //Take in joystick
    joy_x_ = joy_msg_in.axes[0];
    joy_y_ = joy_msg_in.axes[1];
    joy_z_ = joy_msg_in.axes[2];
	
    //msg_time=(double)ros::Time::now().toNSec();
    new_msg = 1;
}

void tak_land_callback(const zlab_drone::Tak_land& tak_land_in) {

    //Detect a Takeoff or Landing Instruction
    takeoff = tak_land_in.state[1];
    landing = tak_land_in.state[2]; 
    new_msg = 1;

}

void state_callback(const ardrone_autonomy::Navdata& navdata_in) {

    //Detect the state of the ardrone as of now
    drone_state = navdata_in.state;
}
	
float map(float value, float in_min, float in_max, float out_min, 
                                                    float out_max) {
    return (float)((value - in_min) * (out_max - out_min) 
                                    / (in_max - in_min) + out_min);
}	

int main(int argc, char** argv)
{
    ros::init(argc, argv,"fly_from_joy");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    ros::Publisher pub_twist;
    ros::Publisher pub_empty_takeoff;
    ros::Publisher pub_empty_landing;
    ros::Publisher pub_empty;
    ros::Publisher pub_v3;
    ros::Subscriber joy_sub;
    ros::Subscriber tak_land_sub;
    ros::Subscriber state_sub; 

    // Hover Message
    twist_msg_hover.linear.x = 0;
    twist_msg_hover.linear.y = 0;
    twist_msg_hover.linear.z = 0;
    twist_msg_hover.angular.x = 0;
    twist_msg_hover.angular.x = 0;
    twist_msg_hover.angular.x = 0;

    pub_twist = node.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
    //pub_v3 = node.advertise<geometry_msgs::Vector3>("joy_vel", 1);  
    joy_sub = node.subscribe("/joy", 1, joy_callback); 
    pub_empty_takeoff = node.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    pub_empty_landing = node.advertise<std_msgs::Empty>("/ardrone/land", 1);
    tak_land_sub = node.subscribe("/tak_land", 1, tak_land_callback);	
    state_sub = node.subscribe("/state_sub", 1, state_callback);     

    //ROS_INFO("Waiting for joystick message");

    while (!new_msg) { 

        if (drone_state == 4) {
            pub_twist.publish(twist_msg_hover);
        } else { 
            ros::spinOnce();
	        loop_rate.sleep();
        }
    }
    
    ROS_INFO("Starting Joy --> cmd_vel Node");
    while (ros::ok() && new_msg) { 
        
        if (takeoff == 1 && landing == 0) {
        
            // Takeoff Time
            pub_empty_takeoff.publish(emp_msg);
            ROS_INFO("DRONE TAKING OFF");
            new_msg = 0;             

        } else if ((takeoff == 0) && (landing == 1)) {

            // Landing Time
            pub_empty_landing.publish(emp_msg);
            ROS_INFO("DRONE_LANDING");
            new_msg = 0; 

        } else if (((takeoff = 0) && (landing == 0)) || ((takeoff = 1) && 
                                                    (landing == 1))) {
        
            // Error
            ROS_INFO("Takeoff and Land?! This shouldn't happen");
            return 0; 

        } else {       
            joy_x = map(joy_x_, -1024, 1024, -1, 1); 
            joy_y = map(joy_y_, -1024, 1024, -1, 1);
            joy_z = map(joy_z_, -1024, 1024, -1, 1);

            if (fabs(joy_x) < 0.01) {joy_x = 0;}
            //else {joy_x=joy_x*forget+joy_x_old*(1-forget);}  

            if (fabs(joy_y) < 0.01) {joy_y = 0;}
            //else {joy_y=joy_y*forget+joy_y_old*(1-forget);}

            if (fabs(joy_z) < 0.01) {joy_z = 0;}
            //else {joy_z=joy_z*forget+joy_z_old*(1-forget);} 

            //ROS_INFO("new message");

            twist_msg.linear.x=joy_x;
            twist_msg.linear.y=joy_y;	
            twist_msg.linear.z=joy_z;
            twist_msg.angular.z=0.0; //YAW IS DISABLED
            // We should keep a regular YAW through the initial experiments
            
            /*
            v3_msg.x=joy_x;
            v3_msg.y=joy_y;
            v3_msg.z=joy_z;
            */ 

            new_msg=0;
            //pub_v3.publish(v3_msg); 
            pub_twist.publish(twist_msg);
        } 

        ros::spinOnce();
        loop_rate.sleep();

    }

    ROS_ERROR("ROS::ok failed- Node Closing");
}
