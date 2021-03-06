#include "sensors.h"

using namespace cv;

roscompat* rc;

// global variables for laser
int numLaser;
std::vector<float> laserdatatemp;

// global variables for encoders
float x, y, z, h, q0, q1, q2, q3;

//global variables for camera
Mat left_frame, right_frame;

void laserCallback(const ros_compat::Num::ConstPtr& msg) {
	rc->translate_laser_messages(msg, laserdatatemp);
	numLaser = laserdatatemp.size();	
}

void pose3dCallback(const ros_compat::Pose3d::ConstPtr& msg) {
	//ROS_INFO("inside callback sample encoder data: [%lf]", msg->x);
	std::vector<float> pose_vector;
	rc->translate_pose3d_messages(msg, pose_vector);
	x = pose_vector[0];
	y = pose_vector[1];
	z = pose_vector[2];
	h = pose_vector[3];
	q0 = pose_vector[4];
	q1 = pose_vector[5];
	q2 = pose_vector[6];
	q3 = pose_vector[7];
}

void cam_left_callback(const sensor_msgs::ImageConstPtr& image_msg) {
	rc->translate_image_messages(image_msg, left_frame);
}

void* threadROS (void* v) {
	ros::NodeHandle n, n_pose, n_cam_left, n_cam_right;
	ros::Subscriber sub = n.subscribe("Laser", 101, laserCallback);
  	ros::Subscriber sub_pose = n_pose.subscribe("Pose3d", 101, pose3dCallback);
	image_transport::ImageTransport it(n_cam_left);
	image_transport::Subscriber cam_left_sub = it.subscribe("cam_topic", 101, cam_left_callback);

	ros::spin();
  	ros::AsyncSpinner* spinner = new ros::AsyncSpinner(4);
	spinner->start();
	ros::waitForShutdown();
	return NULL;
}

Sensors::Sensors(Ice::CommunicatorPtr ic)
{
    this-> ic = ic;
    Ice::PropertiesPtr prop = ic->getProperties();

    ////////////////////////////// Pose3D //////////////////////////////
    // Contact to POSE3D interface
    /*Ice::ObjectPrx basePose3D = ic->propertyToProxy("introrob.Pose3D.Proxy");
    if (0 == basePose3D) {
		pose3dON = false;
		std::cout << "Pose3D configuration not specified" <<std::endl;
        //throw "Could not create proxy with pose3D";
	}else{
		// Cast to pose3D
		try {
			p3dprx = jderobot::Pose3DPrx::checkedCast(basePose3D);
			if (0 == p3dprx)
				throw "Invalid proxy introrob.Pose3D.Proxy";

			pose3dON = true;
			std::cout << "Pose3D connected" << std::endl;
		}catch (Ice::ConnectionRefusedException& e){
			pose3dON=false;
			std::cout << "Pose3D inactive" << std::endl;
		}
	}*/
	char * pose_name = "pose3d_name";
	char * pose_argv[] = {pose_name};
	int pose_argc = 1;
	//ros::init(pose_argc, pose_argv, "pose3d_client");

	/*pthread_t thr_pose3d;
	int thr_pose_status = pthread_create(&thr_pose3d, NULL, threadROSpose3d, (void *)spinner);
	std::cout << "thread status: "<< thr_pose_status << "\n";
	if (thr_pose_status) {
		std::cerr << "Unable to create thread\n";
		exit(-1);
	}*/


	//spinner->start();
   	//ros::waitForShutdown();
	pose3dON=true;


    ////////////////////////////// CAMERA1 /////////////////////////////2

	/*jderobot::ImageDataPtr data;

    Ice::ObjectPrx baseCamera1 = ic->propertyToProxy("introrob.Camera1.Proxy");
    if (0==baseCamera1) {
		camera1ON = false;
		image1.create(400, 400, CV_8UC3);
		std::cout << "Camera1 configuration not specified" <<std::endl;
      //throw "Could not create proxy";
	}else{
    try {
		camera1 = jderobot::CameraPrx::checkedCast(baseCamera1);
		if (0==camera1)
		  throw "Invalid proxy";

		camera1ON = true;
		std::cout << "Camera1 connected" << std::endl;

		data = camera1->getImageData(camera1->getImageFormat().at(0));
		image1.create(data->description->height, data->description->width, CV_8UC3);
	}catch (Ice::ConnectionRefusedException& e){
		camera1ON=false;
		std::cout << "Camera1 inactive" << std::endl;

		//create an empty image if no camera connected (avoid seg. fault)
		image1.create(400, 400, CV_8UC3);
	}}

    ////////////////////////////// CAMERA2 /////////////////////////////2
	Ice::ObjectPrx baseCamera2 = ic->propertyToProxy("introrob.Camera2.Proxy");
    if (0==baseCamera2) {
		camera2ON = false;
		image2.create(400, 400, CV_8UC3);
		std::cout << "Camera2 configuration not specified" <<std::endl;
      //throw "Could not create proxy";
	}else{
    try {
		camera2 = jderobot::CameraPrx::checkedCast(baseCamera2);
		if (0==camera2)
		  throw "Invalid proxy";

		camera2ON = true;
		std::cout << "Camera2 connected" << std::endl;

		data = camera2->getImageData(camera2->getImageFormat().at(0));
		image2.create(data->description->height, data->description->width, CV_8UC3);
	}catch (Ice::ConnectionRefusedException& e){
		camera2ON=false;
		std::cout << "Camera2 inactive" << std::endl;

		//create an empty image if no camera connected (avoid seg. fault)
		image2.create(400, 400, CV_8UC3);
	}}*/

	camera1ON=true;

    ////////////////////////////// LASER //////////////////////////////
	// Contact to LASER interface
    /*Ice::ObjectPrx laserICE = ic->propertyToProxy("introrob.Laser.Proxy");
    if (0 == laserICE) {
		laserON = false;
		std::cout << "Laser configuration not specified" <<std::endl;
        //throw "Could not create proxy with Laser";
	}else{
    // Cast to LASER
	try {
		laserprx = jderobot::LaserPrx::checkedCast(laserICE);
		if (0 == laserprx){
		   throw std::string("Invalid proxy introrob.Laser.Proxy");
		}

		laserON = true;
		std::cout << "Laser connected" << std::endl;
	}catch (Ice::ConnectionRefusedException& e){
		laserON=false;
		std::cout << "Laser inactive" << std::endl;
	}}*/
	
	char * name = "laser_name";
	char * argv[] = {name};
	int argc = 1;
	ros::init(argc, argv, "laser_client");

	/*ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("Laser", 101, laserCallback);
  	spinner = new ros::AsyncSpinner(4);
	spinner->start();
	ros::waitForShutdown();*/

	pthread_t thr;
	int thr_status = pthread_create(&thr, NULL, threadROS, NULL);//(void *)spinner);
	std::cout << "thread status: "<< thr_status << "\n";
	if (thr_status) {
		std::cerr << "Unable to create thread\n";
		exit(-1);
	}


	//spinner->start();
   	//ros::waitForShutdown();
	laserON=true;
}

cv::Mat Sensors::getCamera1()
{
    mutex.lock();
    cv::Mat result = left_frame.clone();
    mutex.unlock();
    return result;

}

cv::Mat Sensors::getCamera2()
{
    mutex.lock();
    cv::Mat result = left_frame.clone();
    mutex.unlock();
    return result;
}

void Sensors::update()
{
	if (pose3dON) {
    	//pose3ddata = this->p3dprx->getPose3DData();
	    mutex.lock();
		robotx = x;//pose3ddata->x;
		roboty = y;//pose3ddata->y;

		double magnitude,w,x,y,z,squ,sqx,sqy,sqz;
		magnitude = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);//sqrt(this->pose3ddata->q0 * this->pose3ddata->q0 + this->pose3ddata->q1 * this->pose3ddata->q1 + this->pose3ddata->q2 * this->pose3ddata->q2 + this->pose3ddata->q3 * this->pose3ddata->q3);

		w = q0/magnitude;//this->pose3ddata->q0 / magnitude;
		x = q1/magnitude;//this->pose3ddata->q1 / magnitude;
		y = q2/magnitude;//this->pose3ddata->q2 / magnitude;
		z = q3/magnitude;//this->pose3ddata->q3 / magnitude;

		squ = w * w;
		sqx = x * x;
		sqy = y * y;
		sqz = z * z;

		double angle;

		angle = atan2( 2 * (x * y + w * z), squ + sqx - sqy - sqz) * 180.0 / M_PI;

		if(angle < 0)
		{
		    angle += 360.0;
		}

		this->robottheta = angle;

	    mutex.unlock();
	}

	if (camera1ON) {
	    //jderobot::ImageDataPtr data = camera1->getImageData(camera1->getImageFormat().at(0));
		mutex.lock();
		if (left_frame.rows!=0 && left_frame.cols!=0)
			image1 = left_frame.clone();	    
		mutex.unlock();
	}

	if (camera2ON) {
	    //jderobot::ImageDataPtr data2 = camera2->getImageData(camera2->getImageFormat().at(0));
		mutex.lock();
	    mutex.unlock();
	}

	if (laserON) {
		mutex.lock();
		if (numLaser) {
			laserData.resize(numLaser);
        	for(int i = 0; i< numLaser; i++) {
        	    laserData[i] = laserdatatemp[i];
        	}
		//std::cout<< "sample laser data:"<< laserData[0]<<"\n";
		}
		mutex.unlock();
	}
}

float Sensors::getRobotPoseX()
{

	float x;
	mutex.lock();
	if (pose3dON) 
	    x = this->robotx;
   	else
		x = 0;
	mutex.unlock();

    return x;
}

float Sensors::getRobotPoseY()
{
    float y;
	mutex.lock();
	if (pose3dON) 
	    y = this->roboty;
   	else
		y = 0;
	mutex.unlock();

    return y;
}

float Sensors::getRobotPoseTheta()
{
    float theta;
	mutex.lock();
	if (pose3dON) 
	    theta = this->robottheta;
   	else
		theta = 0;
	mutex.unlock();

    return theta;
}

std::vector<float> Sensors::getLaserData()
{
	std::vector<float> laserDataAux;
    mutex.lock();
	if (laserON)
	    laserDataAux = laserData;
	else
		laserDataAux = {0};
    mutex.unlock();
    return laserDataAux;
}

