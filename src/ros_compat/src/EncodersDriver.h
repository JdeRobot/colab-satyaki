#include <boost/bind.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/transport/transport.hh"


// ICE utils includes
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>

#include <easyiceconfig/EasyIce.h> 

// JDErobot general ice component includes
#include <jderobot/motors.h>
#include <jderobot/encoders.h>

#include "ros/ros.h"
#include "ros_compat/Pose3d.h"

#ifndef ENCODERS_H
#define	ENCODERS_H

namespace gazebo {
    
    
    class Encoders : public ModelPlugin {
    public:
        
        Encoders();
        
        virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
        physics::ModelPtr getModel();
        
        pthread_mutex_t mutex;
        pthread_mutex_t mutexEncoders;
        int count;
        struct encoders_t {
            float x;
            float y;
            float theta;
            float cos;
            float sin;
        };
        math::Quaternion initial_q;
        encoders_t robotEncoders;
        std::string nameEncoders;
        
    private:
        
        void OnUpdate();
        physics::ModelPtr model;
        math::Pose position;
        event::ConnectionPtr updateConnection;
        
        
    };
    
}

#endif	/* POSE_H */
