/*
 * Copyright (c) 2017-21, Ubiquity Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include "move_basic/collision_checker.h"
#include "move_basic/queued_action_server.h"
#include <move_basic/MovebasicConfig.h>
#include <mobile_cmd_control/move_smoothAction.h>
#include <actionlib_msgs/GoalID.h>
#include <move_basic/imu_controlAction.h>

#include <string>
#include <math.h>

typedef actionlib::SimpleActionServer<move_basic::imu_controlAction> ImuControlServer;

class ImuControl {
    private:
        ros::NodeHandle nh;
        ros::Publisher cmdPub;
        ros::Subscriber imu_sub;
        ros::ServiceServer srv_imu_control;
        bool isFix;
        double sign;
        std::unique_ptr<ImuControlServer> mobile_imuServer;

    public:
        ImuControl(ros::NodeHandle& nh);
        void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
        void sendCmd(double angular); 
        void executeAction(const move_basic::imu_controlGoalConstPtr& msg);
};


void ImuControl::imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
    tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("Imu Orientation Roll: [%f], Pitch: [%f], Yaw: [%f]", roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);
}

void ImuControl::sendCmd(double angular){
    geometry_msgs::Twist msg;
    msg.angular.z = angular;
    cmdPub.publish(msg);
}

void ImuControl::executeAction(const move_basic::imu_controlGoalConstPtr& msg){
    isFix = false;
    ros::Rate r(5);
    bool done=false;

    while(!done){
        try{
            boost::shared_ptr<sensor_msgs::Imu const> sharedmsg;
            sensor_msgs::Imu msg;
            sharedmsg = ros::topic::waitForMessage<sensor_msgs::Imu>("/mobile_imu_control/imu", ros::Duration(3));
            if(sharedmsg != NULL){
                    msg = *sharedmsg;
                }
            tf::Quaternion q(msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w);
            tf::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            ROS_INFO("Imu Orientation Roll: [%f], Pitch: [%f], Yaw: [%f]", roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI);
            if(std::abs(yaw * 180/M_PI) < 3){
                isFix = false; 
                sendCmd(0);
                ROS_INFO("Correction Done");
                done = true;
            }
            else if(std::abs(yaw * 180/M_PI) > 10 || isFix==true){
                ROS_INFO("FIXXXX");
                isFix = true;
                if(yaw * 180/M_PI > 10){
                    sign = -1;
                } else if(yaw * 180/M_PI < -10){
                    sign=1;
                }
                sendCmd(0.3 * sign);
            }
        }
        catch(ros::Exception){
            ROS_INFO("No IMU data");
            mobile_imuServer->setAborted();
        }
        r.sleep();
    }   
    mobile_imuServer->setSucceeded();

}

ImuControl::ImuControl(ros::NodeHandle& nh){
    cmdPub = ros::Publisher(nh.advertise<geometry_msgs::Twist>("/rover_twist", 1));

    mobile_imuServer.reset(new ImuControlServer(nh, "/move_base/imu",
        boost::bind(&ImuControl::executeAction, this, _1) ,false));
    // imu_sub = nh.subscribe("/mobile_imu_control/imu",10 , imu_cb);
    mobile_imuServer->start();
};





