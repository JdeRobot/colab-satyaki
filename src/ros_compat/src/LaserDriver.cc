#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/MultiRayShape.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/plugins/RayPlugin.hh"

#include <boost/algorithm/string.hpp>

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "ros_compat/Num.h"

// ICE utils includes
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <easyiceconfig/EasyIce.h> 

#include <jderobot/laser.h>

void *mainLaser(void* v);

namespace gazebo
{     
    class LaserDump : public RayPlugin
  	{ 
  	
		public: LaserDump() : RayPlugin()
		{
			std::cout << "LaserDump Constructor" <<std::endl;
			count = 0;
			pthread_mutex_init (&mutex, NULL);	
		}
		
	    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
		{
		  // Don't forget to load the camera plugin
		  RayPlugin::Load(_parent,_sdf);
		  this->parentSensor =  boost::dynamic_pointer_cast<sensors::RaySensor>(_parent);

		} 

		// Update the controller
		public: void OnNewLaserScans()
		{
			if(count == 0){
				count++;
				std::string name = this->parentSensor->GetParentName();
				std::cout <<" laser: " << name  << std::endl;
				
				std::vector<std::string> strs;
				boost::split(strs, name, boost::is_any_of("::"));
				
				std::cout << "strs[0]: " << strs[0] << std::endl;

    			nameLaser = std::string("--Ice.Config=" +  strs[0] + "_laser.cfg");
				pthread_t thr_gui;
				pthread_create(&thr_gui, NULL, &mainLaser, (void*)this);
			}
		
			physics::MultiRayShapePtr laser = this->parentSensor->GetLaserShape();

			pthread_mutex_lock (&mutex); 
			laserValues.resize(laser->GetSampleCount ());
			for (int i = 0; i< laser->GetSampleCount (); i++){
				laserValues[i] = laser->GetRange(i);
			}
			pthread_mutex_unlock (&mutex); 
		}
		sensors::RaySensorPtr parentSensor;
		int count;
		std::string nameLaser;
		std::vector<float> laserValues;
		pthread_mutex_t mutex;
  	};

  // Register this plugins with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(LaserDump)
}

class LaserI: virtual public jderobot::Laser {
	public:
		LaserI (gazebo::LaserDump* laser) 
		{
			this->laser = laser;
		}

		virtual ~LaserI(){};

		virtual jderobot::LaserDataPtr getLaserData(const Ice::Current&) {
		    jderobot::LaserDataPtr laserData (new jderobot::LaserData());
			pthread_mutex_lock (&laser->mutex); 
			laserData->numLaser = laser->laserValues.size();
			laserData->distanceData.resize(sizeof(int)*laserData->numLaser);
			
			//Update laser values
			for(int i = 0 ; i < laserData->numLaser; i++){
			   laserData->distanceData[i] = laser->laserValues[i]*1000;
			}
			pthread_mutex_unlock (&laser->mutex); 
			return laserData;
		};

	private:
		int laser_num_readings;
		gazebo::LaserDump* laser;
};

void* threadROSlaser (void* v) {

	char * name = "laser_name";
	char * argv[] = {name};
	int argc = 1;
	ros::init(argc, argv, "laser_driver");

	ros::NodeHandle n_laser;
  	ros::Publisher laser_pub = n_laser.advertise<ros_compat::Num>("Laser", 101);
	ros::Rate loop_rate(10);
	gazebo::LaserDump* laser = (gazebo::LaserDump*)v;

	while (ros::ok()) {
	    ros_compat::Num numLaser;

    	numLaser.num = laser->laserValues.size();
		std::vector<long int> laserdata;
		for (int i=0; i<numLaser.num; i++)
			laserdata.push_back(laser->laserValues[i]*1000);
		numLaser.numArr = laserdata;

    	ROS_INFO("%ld", numLaser.numArr[0]);
    	laser_pub.publish(numLaser);
		
    	ros::spinOnce();

    	loop_rate.sleep();
    }
	return NULL;
}

void *mainLaser(void* v) 
{
	gazebo::LaserDump* laser = (gazebo::LaserDump*)v;
	char* name = (char*)laser->nameLaser.c_str();

    Ice::CommunicatorPtr ic;
    int argc = 1;

    Ice::PropertiesPtr prop;
	char* argv[] = {name};

	ros::init(argc, argv, "laser_driver");

	ros::NodeHandle n_laser;
  	ros::Publisher laser_pub = n_laser.advertise<ros_compat::Num>("Laser", 101);
	ros::Rate loop_rate(10);

	while (ros::ok()) {
	    ros_compat::Num numLaser;

    	numLaser.num = laser->laserValues.size();
		std::vector<long int> laserdata;
		for (int i=0; i<numLaser.num; i++)
			laserdata.push_back(laser->laserValues[i]*1000);
		numLaser.numArr = laserdata;

    	//ROS_INFO("%ld", numLaser.numArr[0]);
    	laser_pub.publish(numLaser);
		
    	ros::spinOnce();

    	loop_rate.sleep();
    }
	exit(0);

    try {
        
        ic = EasyIce::initialize(argc, argv);
        prop = ic->getProperties();
        
        std::string Endpoints = prop->getProperty("Laser.Endpoints");
        std::cout << "Laser Endpoints > " << Endpoints << std::endl;
        
        Ice::ObjectAdapterPtr adapter =
            ic->createObjectAdapterWithEndpoints("Laser", Endpoints);
        Ice::ObjectPtr object = new LaserI(laser);
        adapter->add(object, ic->stringToIdentity("Laser"));
        adapter->activate();
        ic->waitForShutdown();
    } catch (const Ice::Exception& e) {
        std::cerr << e << std::endl;
    } catch (const char* msg) {
        std::cerr << msg << std::endl;
    }
    if (ic) {
        try {
            ic->destroy();
        } catch (const Ice::Exception& e) {
            std::cerr << e << std::endl;
        }
    }
}


