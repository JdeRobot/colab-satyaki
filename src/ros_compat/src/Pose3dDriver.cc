#include "Pose3dDriver.h"

namespace gazebo {

    void *Pose3DICE(void* v);

    GZ_REGISTER_MODEL_PLUGIN(Pose3D)

    Pose3D::Pose3D() {
        pthread_mutex_init(&mutex, NULL);
        pthread_mutex_init(&mutexPose3D, NULL);
        count = 0;
        std::cout << "-----------------constructor Pose3D" << std::endl;
    }

    void Pose3D::Load(physics::ModelPtr _parent, sdf::ElementPtr) {

        model = _parent;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&Pose3D::OnUpdate, this));
    }
    
    physics::ModelPtr Pose3D::getModel()
    {
    	return model;
    }

    void Pose3D::OnUpdate() {

        if (count == 0) {
            count++;
            std::string name = this->model->GetName();
            pthread_t thr_gui;
            pthread_create(&thr_gui, NULL, &Pose3DICE, (void*) this);
        }

        position = model->GetWorldPose();
        this->initial_q = position.rot;


        pthread_mutex_lock(&mutex);
        robotPose3D.x = position.pos.x;
        robotPose3D.y = position.pos.y;
        robotPose3D.q0 = position.rot.w;
        robotPose3D.q1 = position.rot.x;
        robotPose3D.q2 = position.rot.y;
        robotPose3D.q3 = position.rot.z;
        pthread_mutex_unlock(&mutex);

    }

    class Pose3DI : virtual public jderobot::Pose3D {
    public:

        Pose3DI(gazebo::Pose3D* pose) : Pose3DData(new jderobot::Pose3DData()) {
            this->pose = pose;
        }

        virtual ~Pose3DI() {
        };
        
        virtual int setPose3DData(const jderobot::Pose3DDataPtr&  Pose3DData,
        						     const Ice::Current&) {
             math::Pose position = this->pose->getModel()->GetWorldPose();
             


             position.pos.x = Pose3DData->x / Pose3DData->h;
             position.pos.y = Pose3DData->y / Pose3DData->h;
             position.pos.z = Pose3DData->z / Pose3DData->h;

             position.rot.w = Pose3DData->q0;
             position.rot.x = Pose3DData->q1;
             position.rot.y = Pose3DData->q2;
             position.rot.z = Pose3DData->q3;

             this->pose->getModel()->SetWorldPose(position);

             return 0;
	}


        virtual jderobot::Pose3DDataPtr getPose3DData(const Ice::Current&) {
            pthread_mutex_lock(&pose->mutex);

            Pose3DData->x = pose->robotPose3D.x * 1000;
            Pose3DData->y = pose->robotPose3D.y * 1000;
            Pose3DData->z = 0.0;
            Pose3DData->h = 1.0;

            Pose3DData->q0 = pose->robotPose3D.q0;
            Pose3DData->q1 = pose->robotPose3D.q1;
            Pose3DData->q2 = pose->robotPose3D.q2;
            Pose3DData->q3 = pose->robotPose3D.q3;


            pthread_mutex_unlock(&pose->mutex);

            return Pose3DData;
        };

    public:
        gazebo::Pose3D* pose;
    private:
        jderobot::Pose3DDataPtr Pose3DData;
    };

    void *Pose3DICE(void* v) {
        gazebo::Pose3D* base = (gazebo::Pose3D*)v;
        char* name = (char*) base->namePose3D.c_str();
        int argc = 1;
        char* argv[] = {name};

		ros::init(argc, argv, "pose3d_driver");
	  	
		ros::NodeHandle n_pose;
  		ros::Publisher pose_pub = n_pose.advertise<ros_compat::Pose3d>("Pose3d", 101);
		ros::Rate loop_rate(10);
		
		while (ros::ok()) {
	    	ros_compat::Pose3d pose_enc;
		
			pose_enc.x = base->robotPose3D.x * 1000;
			pose_enc.y = base->robotPose3D.y * 1000;
            pose_enc.z = 0.0;
            pose_enc.h = 1.0;

            pose_enc.q0 = base->robotPose3D.q0;
            pose_enc.q1 = base->robotPose3D.q1;
            pose_enc.q2 = base->robotPose3D.q2;
            pose_enc.q3 = base->robotPose3D.q3;

    		//ROS_INFO("x co-or pose: %lf", base->robotPose3D.x);
    		pose_pub.publish(pose_enc);
			
    		ros::spinOnce();
	
    		loop_rate.sleep();
    	}
		exit(0);
    }

}
