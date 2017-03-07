#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include <zlab_drone/Circles.h>
#include <zlab_drone/Circle.h>

using namespace std;
using std::string;
namespace cv
{
    using std::vector;
} 

static const std::string OPENCV_WINDOW1 = "Image window";

string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT); 
  
  switch (depth) { 
    case CV_8U:    r = "8U"; break;
    case CV_8S:    r = "8S"; break;
    case CV_16U:   r = "16U"; break;
    case CV_16S:   r = "16S"; break;
    case CV_32S:   r = "32S"; break;
    case CV_32F:   r = "32F"; break;
    case CV_64F:   r = "64F"; break;
    default:       r = "User"; break;
  }

  r += "C";
  r += (chans + '0');

  return r;

}

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher circle_pub_;
  /*
  ros::Publisher<zlab_drone::Circle> circles_out1;
  ros::Publisher<zlab_drone::Circle> circles_out2;
  ros::Publisher<zlab_drone::Circle> circles_out3;
  */ 
  
public:
  ImageConverter()
    : it_(nh_)
  {

    // Subscribe to the Bottom Raw Image
    image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, 
				&ImageConverter::imageCb, this);

    // Stabilizer Image Subscription  
    /*image_sub_ = it_.subscribe("/stabImage", 1, &ImageConverter::imageCb, 
						this);*/ 

    // Advertising a Simple Image
    /*image_pub_ = it_.advertise("/image", 1);*/

    // Advertisng the Circles being detected from this 
    /*circle_pub_ = 
    nh_.advertise< std::vector < std::vector < float > > >("/circles", 1);*/ 

	/* 
    // Publish the organized circles topic
    circles_out1 = nh_.advertise<zlab_drone::Circle>("/current", 1);
    circles_out2 = nh_.advertise<zlab_drone::Circle>("/closest", 1);
    circles_out3 = nh_.advertise<zlab_drone::Circle>("/furthest", 1);
	*/ 

    cv::namedWindow(OPENCV_WINDOW1);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv::Mat img_hsv;
    cv::Mat img_bin;
    cv::Mat img_hue;
    cv::Mat img_sat;
    cv::Mat copy_ptr; 
    cv_bridge::CvImagePtr hsv_ptr;
    cv_bridge::CvImagePtr cv_ptr;


    //-------------------------------------------------------------------------
    
    // Aquire the Sensor Camera Image 
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      //hsv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
     
    // Change it to HSV
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);
    copy_ptr = cv_ptr->image; 

    img_hue = cv::Mat::zeros(img_hsv.rows, img_hsv.cols, CV_8UC1);
    img_sat = cv::Mat::zeros(img_hsv.rows, img_hsv.cols, CV_8UC1);
    img_bin = cv::Mat::zeros(img_hsv.rows, img_hsv.cols, CV_8UC1);
    
    /* ARCHAIC CHANNEL EXTRACTION 
    cv::Mat img_split[] = {img_hue, img_sat}; 
    int from_to[] = {0,0, 1,1};
    cv::mixChannels(&img_hsv, 3, img_split, 2, from_to, 2);
    */

    cv::extractChannel(img_hsv, img_hue, 0);
    cv::extractChannel(img_hsv, img_sat, 1);

    /* Debugging Matrices Code 
    string ty1 = type2str(img_hue.type());  
    string ty2 = type2str(img_hsv.type());

    printf("Hue Matrix: %s %dx%d \n", ty1.c_str(), img_hue.cols, 
							img_hue.rows);
    printf("HSV Matrix: %s %dx%d \n", ty2.c_str(), img_hsv.cols, 
							img_hsv.rows);
    */

    //Debugging Matrices Size Code

    /*printf("No. of Channels is: %d\n",   copy_ptr.channels());
    printf("Dim1 = %d, Dim2 = %d, Dim3 = %d\n",   copy_ptr.size[0],
						copy_ptr.size[1],
						copy_ptr.size[3]);
    */
    
    // Get the Reds 
    for (int i=0; i < cv_ptr->image.rows; i++) {
	    for (int j =0; j < cv_ptr->image.cols; j++) {
            if (img_hue.at<uint8_t>(i,j) > 160 && 
		        img_sat.at<uint8_t>(i,j) > 70 ) {
    		    img_bin.at<uint8_t>(i,j) = 255; 
	    	} else {
		        img_bin.at<uint8_t>(i,j)=0;
		    /*copy_ptr->image.at<double>(i,j,1) = 0;
		    copy_ptr->image.at<double>(i,j,2) = 0;
		    copy_ptr->image.at<double>(i,j,3) = 0;*/
		    }
    	}
    }
 
    
    //-------------------------------------------------------------------------

    // Computer Vision Algorithm to find the Circles
	cv::medianBlur(img_bin, img_bin, 7);
    cv::Size strel_size;
    strel_size.width = 3;
    strel_size.height = 3; 
    cv::Mat strel = cv::getStructuringElement(cv::MORPH_ELLIPSE,strel_size);
    cv::morphologyEx(img_bin, copy_ptr, cv::MORPH_OPEN, strel, 
						cv::Point(-1, -1), 3);

    cv::bitwise_not(img_bin, img_bin);
    cv::GaussianBlur(img_bin, img_bin, cv::Size(7,7), 2, 2);

    cv::vector<cv::Vec3f> circles; 

    cv::HoughCircles(img_bin, circles, CV_HOUGH_GRADIENT, 1, 70, 140, 15, 
					12, 30);

    
    //-------------------------------------------------------------------------
    
    // Code Loop to draw the Circles on the image. [Old Method]

    for (size_t i = 0; i < circles.size(); i++) {

        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(cv_ptr->image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
        cv::circle(cv_ptr->image, center, radius+1, cv::Scalar(0, 0, 255), 
						8, 0);

    }

    //-------------------------------------------------------------------------

    // Publishing the Image with the different Circles drawn on
    // Only useful when Debugging
    //image_pub_.publish(cv_ptr->toImageMsg());

    // Publishing the actual array of arrays of Circles. 
    // Do this, if there is you are not doing the sorting algorithm in this
    // file itself. 
    
    /*
    std::vector<zlab_drone::Circle> v;
    
    for(int i = 0; i < circles.size(); ++i) {
        const cv::Vec3f& c = circles[i];
        v[i][0] = c[0];
        v[i][1] = c[1];
        v[i][2] = c[2];
    } 

    circle_pub_.publish(v);
    */
    
    //-------------------------------------------------------------------------    

    // Sort out the Circles to find the 3 closest ones
	
    int center_x;
    int center_y;
    int radius;
	double stack [3][4];
    //double euclidian_distance [circles.size()][3];
	std::vector< std::vector< double > > euclidian_distance (
		circles.size(), std::vector<double> (4, 0)); 	

    for (int i = 0; i  < circles.size(); ++i) {
        center_x = circles[i][0];
        center_y = circles[i][1];
		radius = circles[i][2]; 

        //Calculate the  distance from the center of the video
		
		int x_diff = 180 - center_x;
		int y_diff = 320 - center_y; 
        euclidian_distance[i][0] = std::sqrt((x_diff* x_diff) + 
									(y_diff * y_diff)); 
        euclidian_distance[i][1] = center_x; 
        euclidian_distance[i][2] = center_y; 
    	euclidian_distance[i][3] = radius; 
    }
	 
    std::sort(euclidian_distance.begin(), euclidian_distance.end());
	
	//Debugging 
	/*printf("No. of circles detected = %d", euclidian_distance.size());
	printf("----------------------------\nNew Stack\n\n");*/

	int cnt; 

	
	if (euclidian_distance.size() > 3) {
		cnt = 3; 
	} else if (euclidian_distance.size() == 2) {
		cnt = 2; 
	} else if (euclidian_distance.size() == 1) { 
		cnt = 1; 
	} else {
		cnt = 0;
	}
	
	for (int dim1 = 0; dim1 < cnt; dim1++) {
		for (int dim2 = 0; dim2 < 4; dim2++) {
			stack[dim1][dim2] = euclidian_distance[dim1][dim2];
		}
	}

    //------------------------------------------------------------------------- 
   
	// Publishing on to the next topic
	/*
	if (cnt == 3) {
		circles_out1.publish(stack[1]);
		circles_out2.publish(stack[2]);
		circles_out3.publish(stack[3]);
	} else if (cnt == 2) {
		circles_out1.publish(stack[1]);
		circles_out2.publish(stack[2]);	
	} else if (cnt == 1) {
		circles_out1.publish(stack[1]);
	} else {
		// Do Nothing -> Move on
	}
	*/

    //-------------------------------------------------------------------------

    // Draw out the Circles [DEBUGGING]
    
    // Circle 1 
	/*
	if (cnt == 3) {

		// In case of 3 Circles
		cv::Point center1(cvRound(stack[0][1]), cvRound(stack[0][2]));
		int radius1 = cvRound(stack[0][3]);
		cv::circle(cv_ptr->image, center1, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		cv::circle(cv_ptr->image, center1, radius1+1, cv::Scalar(0, 255, 0), 
						8, 0);	

		// Circle 2
		cv::Point center2(cvRound(stack[1][1]), cvRound(stack[1][2]));
		int radius2 = cvRound(stack[1][3]);
		cv::circle(cv_ptr->image, center2, 3, cv::Scalar(255, 215, 0), -1, 8, 0);
		cv::circle(cv_ptr->image, center2, radius2+1, cv::Scalar(255, 215, 0), 
						8, 0);

		// Circle 3
		cv::Point center3(cvRound(stack[2][1]), cvRound(stack[2][2]));
		int radius3 = cvRound(stack[2][3]);
		cv::circle(cv_ptr->image, center3, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
		cv::circle(cv_ptr->image, center3, radius3+1, cv::Scalar(0, 0, 255), 
						8, 0);

	} else if (cnt == 2) {
		
		// In case of 2 Circles
		cv::Point center1(cvRound(stack[0][1]), cvRound(stack[0][2]));
		int radius1 = cvRound(stack[0][3]);
		cv::circle(cv_ptr->image, center1, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		cv::circle(cv_ptr->image, center1, radius1+1, cv::Scalar(0, 255, 0), 
						8, 0);	

		// Circle 2
		cv::Point center2(cvRound(stack[1][1]), cvRound(stack[1][2]));
		int radius2 = cvRound(stack[1][3]);
		cv::circle(cv_ptr->image, center2, 3, cv::Scalar(255, 215, 0), -1, 8, 0);
		cv::circle(cv_ptr->image, center2, radius2+1, cv::Scalar(255, 215, 0), 
						8, 0);

	} else if (cnt == 1) {

		// In case of 1 Circle
		cv::Point center1(cvRound(stack[0][1]), cvRound(stack[0][2]));
		int radius1 = cvRound(stack[0][3]);
		cv::circle(cv_ptr->image, center1, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
		cv::circle(cv_ptr->image, center1, radius1+1, cv::Scalar(0, 255, 0), 
						8, 0);	

	} else {
		// Do Nothing -> Move on
	}
	*/
    //-------------------------------------------------------------------------

    // Code to show Image 
    cv::imshow(OPENCV_WINDOW1, cv_ptr->image);
    cv::waitKey(3);

  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
