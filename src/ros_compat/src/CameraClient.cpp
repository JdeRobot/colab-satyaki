#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros_compat/Num.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"

#include "opencv2/opencv.hpp"

#include <iostream>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <jderobot/camera.h>
#include <visionlib/colorspaces/colorspacesmm.h>

#include "parallelIce/cameraClient.h"
#include "easyiceconfig/EasyIce.h" 

using namespace cv;

void chatterCallback(const ros_compat::Num::ConstPtr& msg)
{
  ROS_INFO("receiver's frame id: [%ld]", msg->num);
}

void cameracallback(const sensor_msgs::ImageConstPtr& image_msg) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	Mat frame = cv_ptr->image;
	imshow("frame:ROS", frame);
	waitKey(33);
}

int main(int argc, char **argv) {
	Ice::CommunicatorPtr ic;
	jderobot::cameraClient* camRGB;

	try {
		ic = EasyIce::initialize(argc, argv);
		Ice::ObjectPrx base = ic->propertyToProxy("Cameraview.Camera.Proxy");
		Ice::PropertiesPtr prop = ic->getProperties();

		if (0==base)
			throw "Could not create Proxy\n";

		camRGB = new jderobot::cameraClient(ic, "Cameraview.Camera.");

		if (camRGB == NULL) {
			throw "Invalid Proxy";
		} else {
			camRGB->start();
			std::cout << "Using ICE camera server..";
			cv::Mat rgb;
	
			while(1) {
				camRGB->getImage(rgb);
				if (rgb.rows==0 || rgb.cols==0) continue;
				imshow("frame:ICE", rgb);
				waitKey(33);
			}
		}
	} catch (const char* msg) {

	} 

 	ros::init(argc, argv, "listener");

  	ros::NodeHandle n,nh;
  	image_transport::ImageTransport it(nh);
  	image_transport::Subscriber camera_sub = it.subscribe("cameratopic", 1000, cameracallback);

  	ros::Subscriber sub = n.subscribe("chatter", 1001, chatterCallback);

  	ros::spin();

  	return 0;
}
