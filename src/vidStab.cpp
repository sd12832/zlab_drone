#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/video.hpp>

using namespace std; 
using std::string;
using std::vector;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class Tracker {

    // The ROS declarations 
    ros::NodeHandle nh1_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // Internal Declarations
    vector<cv::Point2f> trackedFeatures;
    cv::Mat prevGray, orig_warped; 

    public:
    bool freshStart;
    Mat_<float> rigidTransform = cv::Mat::eye(3,3,CV_32FC1);

    Tracker():it_(nh1_) { 
	image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1, 
					&Tracker::processImage, this);
	image_pub_ = it_.advertise("/stabImage", 1);
	
	cv::namedWindow(OPENCV_WINDOW);
    }

    ~Tracker() {
	cv::destroyWindow(OPENCV_WINDOW);
    }

    void processImage(const sensor_msgs::ImageConstPtr& msg) {
	
	// ROS declarations 
	cv_bridge::CvImagePtr cv_ptr;
	
	// Function Initializations 
	/*
    if (freshStart == true) {
	   rigidTransform = cv::Mat::eye(3,3,CV_32FC1);
    }

    */

	cv::Mat gray; 
	cv::Mat copy_img;
	vector<cv::Point2f> corners;
	
	try { 
	    cv_ptr = cv_bridge::toCvCopy(msg, 
					sensor_msgs::image_encodings::BGR8);
	} 
	catch (cv_bridge::Exception& e) {
	    ROS_INFO("cv_bridge exception");
	    return;
	}

	copy_img = cv_ptr->image;
	cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);
	
	if (trackedFeatures.size() < 200) {
	    cv::goodFeaturesToTrack(gray,corners,200,0.01,10);
	    //Debugging
	    //cout << "found" << corners.size() << "features\n";
	    for (int i = 0; i < corners.size(); ++i) {
		trackedFeatures.push_back(corners[i]);
	    }
	}

	int m = 4; 

	if (!prevGray.empty()) {

	   vector<uchar> status;
	   vector<float> errors; 

	   calcOpticalFlowPyrLK(prevGray, gray, trackedFeatures, corners,
				status, errors, Size(10,10));

	   if (countNonZero(status) < status.size() * 0.8) {
		cout << "cataclysmic error\n";
		rigidTransform = cv::Mat::eye(3,3,CV_32FC1);
		trackedFeatures.clear();
		prevGray.release();
		freshStart = true;
		return;
	   }
	   else { freshStart = false; }

	   cv::Mat_<float> newRigidTransform = estimateRigidTransform(trackedFeatures, 
							corners, false);
	   cv::Mat_<float> nrt33 = cv::Mat_<float>::eye(3,3);
	   newRigidTransform.copyTo(nrt33.rowRange(0,2));
	   rigidTransform *= nrt33;

	   trackedFeatures.clear();
	   for (int i = 0; i < status.size(); ++i) {
		if (status[i]) {
		    trackedFeatures.push_back(corners[i]);
		}
	   }
	}

	// Debugging to see the tracked features as of now
	for (int i = 0; i < trackedFeatures.size(); ++i) {
	    circle(cv_ptr->image, trackedFeatures[i], 3, Scalar(0,0,255), 
			CV_FILLED);
	}
	imshow(OPENCV_WINDOW, cv_ptr->image); 
	cv::waitKey(3);

	gray.copyTo(prevGray);

	Mat invTrans = rigidTransform.inv(DECOMP_SVD);
	warpAffine(cv_ptr->image, orig_warped, invTrans.rowRange(0,2), 
			Size());


	// Publishing (Only uncomment, if you're sure nothing is broken
	// in this section)
	/*
	cv_ptr->image = orig_warped; 
	image_pub_.publish(cv_ptr->toImageMsg()); 	
	*/
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "video_stabilizer");
  Tracker tr;
  ros::spin();
  return 0;
}


