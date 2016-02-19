#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "my_pkg/Num.h"

#include "opencv2/opencv.hpp"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n, nh;
  ros::Publisher chatter_pub = n.advertise<my_pkg::Num>("chatter", 1001);
  image_transport::ImageTransport it(nh);// = new image_transport::ImageTransport(nh);
  image_transport::Publisher camera_pub = it.advertise("cameratopic", 1000);

  ros::Rate loop_rate(10);
  int count = 0;

  VideoCapture cap(0);
  if (cap.isOpened())
  	ROS_INFO("%s", "Camera connected..");
  else {
	ROS_ERROR("%s", "Camera not detected..");
	return 1;
  }
  Mat frame;

  while (ros::ok())
  {
    my_pkg::Num msg;

	cap >> frame;
	//imshow("frame", frame);
	//if (waitKey(33)==27) break;
	sensor_msgs::ImagePtr image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

    msg.num = count;

    //ROS_INFO("%s", msg.data.c_str());
	ROS_INFO("%ld", msg.num);
    chatter_pub.publish(msg);
	camera_pub.publish(image_message);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
