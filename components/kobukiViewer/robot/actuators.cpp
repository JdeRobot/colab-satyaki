#include "actuators.h"

//global variables
float w, v, l;

roscompat* rc_;

void motorgetCallback(const ros_compat::Motors::ConstPtr& msg) {
	std::vector<float> motors_vector;
	rc_->translate_motor_messages(msg, motors_vector);
	w = motors_vector[0];
	v = motors_vector[1];
	l = motors_vector[2];	
}

void* threadROSMotors(void* v) {
	ros::NodeHandle n;
	ros::Subscriber mot_sub = n.subscribe("Motors_pub", 101, motorgetCallback);
	ros::spin();
	return NULL;
}

Actuators::Actuators(Ice::CommunicatorPtr ic)
{
    this->ic = ic;

    // Contact to MOTORS interface
    /*Ice::ObjectPrx baseMotors = ic->propertyToProxy("introrob.Motors.Proxy");
    if (0 == baseMotors){
        motorsON = false;
		std::cout << "Motors configuration not specified" <<std::endl;

        //throw "Could not create proxy with motors";
	}else{
		// Cast to motors
		try{
			mprx = jderobot::MotorsPrx::checkedCast(baseMotors);
			if (0 == mprx)
				throw "Invalid proxy introrob.Motors.Proxy";

			motorsON = true;
			std::cout << "Motors connected" << std::endl;
		}catch (Ice::ConnectionRefusedException& e){
			motorsON=false;
			std::cout << "Motors inactive" << std::endl;
		}
	}*/
    
	motorsON= true;

    motorVout= 0;
    motorWout = 0;
    motorLout= 0;

	pthread_t motorget_thread;
	pthread_create(&motorget_thread, NULL, threadROSMotors, NULL);

	int argc = 1;
	char * argv[] = {"motors"};
	ros::init(argc, argv, "motors_client");
	  	
	ros::NodeHandle n_mot;
  	mot_pub = n_mot.advertise<ros_compat::Motors>("Motors_sub", 101);
	loop_rate = new ros::Rate(10);
}

void Actuators::update()
{
    if (motorsON) {
	    mutex.lock();

	    motorVin = v;//this->mprx->getV();
	    motorWin = w;//this->mprx->getW();
	    motorLin = l;//this->mprx->getL();

	    mutex.unlock();
    }
}

void Actuators::setActuators()
{

	ros_compat::Motors my_motor;
	if (motorsON) {
		mutex.lock();

		if (motorWout < 5 && motorWout>-5)
		    my_motor.w = 0.;

		my_motor.w = motorWout;
		my_motor.l = motorLout;
		my_motor.v = motorVout;

		mutex.unlock();

		std::cout << "in setActuators: w = " << motorWout << " l = " << motorLout << " v = " << motorVout << "\n"; 
		mot_pub.publish(my_motor);

		ros::spinOnce();
    	loop_rate->sleep();
	}
}


///////////////// GETTER //////////////////
float Actuators::getMotorV()
{

	float v;
	mutex.lock();
	if (motorsON)
		v = motorVin;
	else
		v = 0;
	mutex.unlock();

    return v;
	
}

float Actuators::getMotorW()
{
	float w;
	mutex.lock();
	if (motorsON)
		w = motorVin;
	else
		w = 0;
	mutex.unlock();

    return w;
}

float Actuators::getMotorL()
{
    float l;
	mutex.lock();
	if (motorsON)
		l = motorVin;
	else
		l = 0;
	mutex.unlock();

    return l;
}

///////////////// SETTER //////////////////
void Actuators::setMotorSTOP()
{
	if (motorsON) {
		mutex.lock();
		this->motorVout = 0;
		this->motorWout = 0;
		this->motorLout = 0;
		mutex.unlock();
	}
}

void Actuators::setMotorV(float motorV)
{
	if (motorsON) {
		mutex.lock();
		this->motorVout = motorV;
		mutex.unlock();
	}
}

void Actuators::setMotorW(float motorW)
{
	if (motorsON) {
		mutex.lock();
		this->motorWout = motorW;
		mutex.unlock();
	}
}

void Actuators::setMotorL(float motorL)
{
	if (motorsON) {
		mutex.lock();
		this->motorLout = motorL;
		mutex.unlock();
	}

}

