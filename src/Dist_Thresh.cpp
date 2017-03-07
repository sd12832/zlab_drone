#include <ros/ros.h>
#include <zlab_drone/Circles.h>
#include <zlab_drone/Circle.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imageproc/imageproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Deprecated Function 
/*
bool smaller_than (int i, int j) {

    return ()

}
*/

// Global Variables 
double stack [3][3];
cv::Mat main_image; 
bool new_msg1, new_msg2, new_msg; 

static const std::string OPENCV_WINDOW = "Image Window"; 

void image_callback (const sensor_msgs::ImageConstPtr& msg) {

	// Image Processing
	cv_bridge::CvImagePtr cv_ptr
	try {
		cv_ptr = cv_bridge::toCvCopy(msg,
			 sensor_msgs::image_encodings::BGR8);
	} catch (cv::bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	main_image = cv_ptr->image; 
	
}

// Make sure the different circles are organized properly
void organizer_callback(const 
            zlab_drone::Circles::ConstPtr& msg) {

	// The Variables 
    std::vector<std::vector<float>> circles;
    int center_x;
    int center_y;
    int radius;

	// Initializing Stack to 0
	for (int dim1 = 0; dim1 < 3; dim1++) {
		for (int dim2 = 0; dim2 < 3; dim2++) {
			stack[dim1][dim2] = 0;
		}
	}

	// Gathering Data from the messages
	for (std::vector<zlab_drone::Circles>::const_iterator gp = 
		msg->circles.begin(); gp!= msg->circles.end(), ++gp) {
		for (std::vector<float>::const_iterator dp = 
		    gp->circle.begin(); dp != gp->circle.end(); ++dp) {
		    circles[gp][dp] = gp->circle(dp); 
		}
	}
		
	double euclidian_distance [circles.size()][3];  
	for (int i = 0, i  < circles.size(); ++i) {
		center_x = circles[i][0];
		center_y = circles[i][1];

		//Calculate the  distance from the center of the video

		euclidian_distance[i][0] = (); 
		euclidian_distance[i][1] = center_x; 
		euclidian_distance[i][2] = center_y; 
		
	}
	std::sort(euclidian_distance.begin(), euclidian_distance.end()); 
	stack[1] = euclidian_distance[1];
	stack[2] = euclidian_distance[2];
	stack[3] = euclidian_distance[3];

	circles_out1.publish(stack[1]);
	circles_out2.publish(stack[2]);
	circles_out3.publish(stack[3]);
	
}


int main (int argc, char** argv) {

    ros::init(argc, argv, "distance_thresholder");

    // Aim to organize the stack of the different circles that have 
    //been detected
    ros::Nodehandle node;
    ros::Subscriber circles_in;
    ros::Publisher circles_out1;
    ros::Publisher circles_out2;
    ros::Publisher circles_out3;  

    // Subscribe to the Bottom Image of the Drone for Debugging 
	// Comment this out, if not needed. 
    image_sub = node.subscribe("/ardrone/bottom/image_raw", 1, 
				image_callback);

    // Subscribe to the Circles topic
    circles_in = node.subscribe("/circles", 1, &Thresholder::organizer_callback);	
    
    // Publish the organized circles topic
    circles_out1 = node.advertise<zlab_drone::Circle>("/current", 1);
    circles_out2 = node.advertise<zlab_drone::Circle>("/closest", 1);
    circles_out3 = node.advertise<zlab_drone::Circle>("/furthest", 1);

    if (new_msg1 == true && new_msg2 == true) {
        
        // Debugging Flags 
        new_msg == true;         

    } 

	while ((ros::ok()) && (new_msg == true)) {

	// Debugging Section ---------------------------------
		for (int dim1 = 0; dim1 < 3; dim1++) {
			cv::Point center(cvRound(stack[dim1][0]), 
					cvRound(stack[dim1][1]));
			int radius = stack[dim1][2];
			cv::circle(main_image, center, radius+1, 
				cv::Scalar(0,0,255));
		}

		cv::NamedWindow(OPENCV_WINDOW);
		cv::imshow(OPENCV_WINDOW, main_image);
		cv::waitkey(3); 
	}

    ros::spin();
    return 0; 

}
