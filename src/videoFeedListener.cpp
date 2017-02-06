#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>

using namespace std;
using std::string;
namespace cv
{
    using std::vector;
} 

static const std::string OPENCV_WINDOW1 = "Image window";
static const std::string OPENCV_WINDOW2 = "Image window2";
static const std::string OPENCV_WINDOW3 = "Image window3";

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
  
public:
  ImageConverter()
    : it_(nh_)
  {
    //Alternate Subscription Plans :P 

    image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, 
				&ImageConverter::imageCb, this); 
    /*image_sub_ = it_.subscribe("/stabImage", 1, &ImageConverter::imageCb, 
						this);*/ 
    image_pub_ = it_.advertise("/image", 1);

    cv::namedWindow(OPENCV_WINDOW1);
    //cv::namedWindow(OPENCV_WINDOW2);
    //cv::namedWindow(OPENCV_WINDOW3);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
    //cv::destroyWindow(OPENCV_WINDOW3);
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

    for (int i=0; i < cv_ptr->image.rows; i++) {
	for (int j =0; j < cv_ptr->image.cols; j++) {
            if (img_hue.at<uint8_t>(i,j) > 0  &&
		img_hue.at<uint8_t>(i,j) < 28 &&
		img_sat.at<uint8_t>(i,j) > 95 ) {
		    img_bin.at<uint8_t>(i,j) = 255; 
		} else {
		img_bin.at<uint8_t>(i,j)=0;
		/*copy_ptr->image.at<double>(i,j,1) = 0;
		copy_ptr->image.at<double>(i,j,2) = 0;
		copy_ptr->image.at<double>(i,j,3) = 0;*/
		}
	}
    }

    cv::Size strel_size;
    strel_size.width = 3;
    strel_size.height = 3; 
    cv::Mat strel = cv::getStructuringElement(cv::MORPH__ELLIPSE,strel_size);
    cv::morphologyEx(img_bin, copy_ptr, cv::MORPH_OPEN, strel, 
						cv::Point(-1, -1), 2);

    cv::bitwise_not(img_bin, img_bin);
    cv::GaussianBlur(img_bin, img_bin, cv::Size(7,7), 2, 2);

    cv::vector<cv::Vec3f> circles; 

    cv::HoughCircles(img_bin, circles, CV_HOUGH_GRADIENT, 1, 70, 140, 15, 
					20, 400);

    for (size_t i = 0; i < circles.size(); i++) {

    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    cv::circle(cv_ptr->image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    cv::circle(cv_ptr->image, center, radius+1, cv::Scalar(0, 0, 255), 2
				8, 0);

    }

    cv::imshow(OPENCV_WINDOW1, cv_ptr->image);
    //cv::imshow(OPENCV_WINDOW2, img_bin);
    //cv::imshow(OPENCV_WINDOW3, copy_ptr->image);

    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
