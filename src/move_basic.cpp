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

#include <string>

typedef actionlib::QueuedActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;
typedef actionlib::SimpleActionServer<mobile_cmd_control::move_smoothAction> MoveSmoothActionServer;

class MoveBasic {
  private:
    ros::NodeHandle nh;
    ros::Subscriber goalSub;
    ros::Subscriber stopSub;

    ros::Publisher goalPub;
    ros::Publisher cmdPub;
    ros::Publisher pathPub;
    ros::Publisher obstacleDistPub;
    ros::Publisher errorPub;

    std::unique_ptr<MoveBaseActionServer> actionServer;
    std::unique_ptr<CollisionChecker> collision_checker;
    std::unique_ptr<ObstaclePoints> obstacle_points;
    std::unique_ptr<MoveSmoothActionServer> smooth_actionServer;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener;

    double minTurningVelocity;
    double maxTurningVelocity;
    double turningAcceleration;
    double angularTolerance;
    double maxLateralVelocity;
    std::string axis;

    double maxLinearVelocity;
    double minLinearVelocity;
    double linearAcceleration;
    double linearTolerance;

    double lateralKp;
    double lateralKi;
    double lateralKd;

    double obstacleWaitThreshold;
    double forwardObstacleThreshold;
    double minSideDist;
    double localizationLatency;
    double runawayTimeoutSecs;
    bool stop;

    float forwardObstacleDist;
    float leftObstacleDist;
    float rightObstacleDist;
    tf2::Vector3 forwardLeft;
    tf2::Vector3 forwardRight;

    std::string preferredPlanningFrame;
    std::string alternatePlanningFrame;
    std::string preferredDrivingFrame;
    std::string alternateDrivingFrame;
    std::string baseFrame;

    dynamic_reconfigure::Server<move_basic::MovebasicConfig> dr_srv;

    void dynamicReconfigCallback(move_basic::MovebasicConfig& config, uint32_t);
    void stopCallback(const std_msgs::Bool::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void executeAction(const move_base_msgs::MoveBaseGoalConstPtr& goal);
    void drawLine(double x0, double y0, double x1, double y1);
    void sendCmd(double angular, double linear);
    void sendCmdXY(double angular, double linearX, double linearY);
    void abortGoal(const std::string msg);
    int getNewGoal();

    bool getTransform(const std::string& from, const std::string& to,
                      tf2::Transform& tf);
    bool transformPose(const std::string& from, const std::string& to,
                       const tf2::Transform& in, tf2::Transform& out);
    
    void move_smoothCB(const mobile_cmd_control::move_smoothGoalConstPtr &goal);
    
  public:
    MoveBasic(ros::NodeHandle &nodeHandle,std::string ax);

    void run();

    bool moveLinear(tf2::Transform& goalInDriving,
                    const std::string& drivingFrame);
    bool approachLinear(tf2::Transform& goalInDriving,
                    const std::string& drivingFrame);
    bool rotate(double requestedYaw,
                const std::string& drivingFrame);

    tf2::Transform goalInPlanning;
};


// Radians to degrees

static double rad2deg(double rad)
{
    return rad * 180.0 / M_PI;
}

// Get the sign of a number

static int sign(double n)
{
    return (n <0 ? -1 : 1);
}

// Adjust angle to be between -PI and PI

static void normalizeAngle(double& angle)
{
    if (angle < -M_PI) {
         angle += 2 * M_PI;
    }
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
}


// retreive the 3 DOF we are interested in from a Transform

static void getPose(const tf2::Transform& tf, double& x, double& y, double& yaw)
{
    tf2::Vector3 trans = tf.getOrigin();
    x = trans.x();
    y = trans.y();

    double roll, pitch;
    tf.getBasis().getRPY(roll, pitch, yaw);
}


// Constructor

MoveBasic::MoveBasic(ros::NodeHandle &nodeHandle, std::string ax): tfBuffer(ros::Duration(3.0)),
                        listener(tfBuffer)
{
    std::string ns = ros::this_node::getNamespace();
    ros::NodeHandle nh = nodeHandle;
    axis = ax;
    // Velocity parameters
    nh.param<double>("min_turning_velocity", minTurningVelocity, 0.10);
    nh.param<double>("max_turning_velocity", maxTurningVelocity, 0.5);
    nh.param<double>("max_lateral_velocity", maxLateralVelocity, 0.5);
    nh.param<double>("max_linear_velocity", maxLinearVelocity, 0.5);
    // Target Velocity 
    nh.param<double>("min_linear_velocity", minLinearVelocity, 0.3);
    nh.param<double>("linear_acceleration", linearAcceleration, 0.5);
    nh.param<double>("turning_acceleration", turningAcceleration, 0.1);

    // Within tolerance, goal is successfully reached
    nh.param<double>("angular_tolerance", angularTolerance, 0.05);
    nh.param<double>("linear_tolerance", linearTolerance, 0.05);

    // PID parameters for lateral control
    nh.param<double>("lateral_kp", lateralKp, 0.0);
    nh.param<double>("lateral_ki", lateralKi, 0.0);
    nh.param<double>("lateral_kd", lateralKd, 3.0);

    // how long to wait after moving to be sure localization is accurate
    nh.param<double>("localization_latency", localizationLatency, 1.5);

    // how long robot can be driving away from the goal
    nh.param<double>("runaway_timeout", runawayTimeoutSecs, 10.0);

    // how long to wait for an obstacle to disappear
    nh.param<double>("obstacle_wait_threshold", obstacleWaitThreshold, 60.0);

    // Minimum distance to maintain in front
    nh.param<double>("forward_obstacle_threshold", forwardObstacleThreshold, 0.5);

    // Minimum distance to maintain at each side
    nh.param<double>("min_side_dist", minSideDist, 0.3);

    nh.param<std::string>("preferred_planning_frame",
                          preferredPlanningFrame, "");
    nh.param<std::string>("alternate_planning_frame",
                          alternatePlanningFrame, "odom");
    nh.param<std::string>("preferred_driving_frame",
                          preferredDrivingFrame, "map");
    nh.param<std::string>("alternate_driving_frame",
                          alternateDrivingFrame, "odom");
    nh.param<std::string>("base_frame", baseFrame, "base_footprint");

    stop = false;

    dynamic_reconfigure::Server<move_basic::MovebasicConfig>::CallbackType f;
    f = boost::bind(&MoveBasic::dynamicReconfigCallback, this, _1, _2);
    dr_srv.setCallback(f);

    cmdPub = ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    pathPub = ros::Publisher(nh.advertise<nav_msgs::Path>("/plan", 1));

    obstacleDistPub =
        ros::Publisher(nh.advertise<geometry_msgs::Vector3>("/obstacle_distance", 1));
    errorPub =
        ros::Publisher(nh.advertise<geometry_msgs::Vector3>("/lateral_error", 1));

    goalSub = nh.subscribe(ns + "/" + axis + "_simple" + "/goal", 1,
                            &MoveBasic::goalCallback, this);

    stopSub = nh.subscribe(ns + "/" + axis + "/stop", 1,
                            &MoveBasic::stopCallback, this);

    ros::NodeHandle actionNh("");

    actionServer.reset(new MoveBaseActionServer(actionNh, axis,
	boost::bind(&MoveBasic::executeAction, this, _1)));

    actionServer->start();
    goalPub = actionNh.advertise<move_base_msgs::MoveBaseActionGoal>(
      ns + "/" + axis + "/goal", 1);

    obstacle_points.reset(new ObstaclePoints(nh, tfBuffer));
    collision_checker.reset(new CollisionChecker(nh, tfBuffer, *obstacle_points));
    std::string action_name = "move_smooth";

    if(axis == "Y")
    {
        minLinearVelocity = 0.18;
        smooth_actionServer.reset(new MoveSmoothActionServer(actionNh, action_name,
        boost::bind(&MoveBasic::move_smoothCB, this, _1) ,false));
        smooth_actionServer->start();
    }
    ROS_INFO("Move Basic ready");
}


// Lookup the specified transform, returns true on success

void MoveBasic::move_smoothCB(const mobile_cmd_control::move_smoothGoalConstPtr &goal){
    float s= goal->target_distance.data;
    float v = goal->target_speed.data;
    float sleep_t = s/v;

    if(goal->direction.data == 2){
        sendCmd(0,goal->target_speed.data);
    }
    if(goal->direction.data == 3){
        sendCmd(0,goal->target_speed.data);
    }

    sleep(sleep_t);
    // ROS_INFO("%f", sleep_t);
    // ROS_INFO(axis.c_str());

    // See if another node is publishing the next command
    std::string topic_name = "rover_twist";
    try{
        geometry_msgs::TwistConstPtr twist = ros::topic::waitForMessage<geometry_msgs::Twist>(topic_name, ros::Duration(3));
    }
    catch(ros::Exception){
        ROS_INFO("No new command and stopping ");
        sendCmd(0,0);
    }

    sendCmd(0,0.0);
    ROS_INFO("Smooth Command");
    smooth_actionServer->setSucceeded();
}

bool MoveBasic::getTransform(const std::string& from, const std::string& to,
                             tf2::Transform& tf)
{
    try {
        geometry_msgs::TransformStamped tfs =
            tfBuffer.lookupTransform(to, from, ros::Time(0));
        tf2::fromMsg(tfs.transform, tf);
        return true;
    }
    catch (tf2::TransformException &ex) {
         return false;
    }
}

// Transform a pose from one frame to another

bool MoveBasic::transformPose(const std::string& from, const std::string& to,
                              const tf2::Transform& in, tf2::Transform& out)
{
    tf2::Transform tf;
    if (!getTransform(from, to, tf)) {
        return false;
    }
    out = tf * in;
    return true;
}

// Dynamic reconfigure

void MoveBasic::dynamicReconfigCallback(move_basic::MovebasicConfig& config, uint32_t){
    minTurningVelocity = config.min_turning_velocity;
    maxTurningVelocity = config.max_turning_velocity;
    maxLateralVelocity = config.max_lateral_velocity;
    turningAcceleration = config.turning_acceleration;
    maxLinearVelocity = config.max_linear_velocity;
    minLinearVelocity = config.min_linear_velocity;
    linearAcceleration = config.linear_acceleration;

    angularTolerance = config.angular_tolerance;
    linearTolerance = config.linear_tolerance;

    lateralKp = config.lateral_kp;
    lateralKi = config.lateral_ki;
    lateralKd = config.lateral_kd;

    localizationLatency = config.localization_latency;
    runawayTimeoutSecs = config.runaway_timeout;

    minSideDist = config.min_side_dist;
    obstacleWaitThreshold = config.obstacle_wait_threshold;
    forwardObstacleThreshold = config.forward_obstacle_threshold;

    ROS_WARN("Parameter change detected");
}

// Stop robot in place and save last state

void MoveBasic::stopCallback(const std_msgs::Bool::ConstPtr& msg)
{
    stop = msg->data;
    if (stop) ROS_WARN("MoveBasic: Robot is forced to stop!");
}

// Called when a simple goal message is received

void MoveBasic::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("MoveBasic: Received simple goal");

    move_base_msgs::MoveBaseActionGoal actionGoal;
    actionGoal.header.stamp = ros::Time::now();
    actionGoal.goal.target_pose = *msg;

    // Send the goal to the action server
    goalPub.publish(actionGoal);
}


// Abort goal and print message

void MoveBasic::abortGoal(const std::string msg)
{
    actionServer->setAborted(move_base_msgs::MoveBaseResult(), msg);
    ROS_ERROR("%s", msg.c_str());
}


int MoveBasic::getNewGoal()
{
    int goalId = actionServer->current_goal.getGoal()->target_pose.header.seq;

    ROS_INFO("%d",goalId);
    return goalId;
}


// Called when an action goal is received

void MoveBasic::executeAction(const move_base_msgs::MoveBaseGoalConstPtr& msg)
{
    /*
      Plan a path that involves rotating to face the goal, going straight towards it,
      and then rotating for the final orientation.

      It is assumed that we are dealing with imperfect localization data:
      map->base_link is accurate but may be delayed and is at a slow rate
      odom->base_link is frequent, but drifts, particularly after rotating
      To counter these issues, we plan in the map frame, and wait localizationLatency
      after each step, and execute in the odom frame.
    */
    int n = getNewGoal();

    ROS_INFO("starting new goal");

    tf2::Transform goal;
    tf2::fromMsg(msg->target_pose.pose, goal);
    std::string frameId = msg->target_pose.header.frame_id;
    // ROS_INFO(goal);

    // Needed for RobotCommander
    if (frameId[0] == '/')
        frameId = frameId.substr(1);

    double x, y, yaw;
    getPose(goal, x, y, yaw);

    ROS_INFO("MoveBasic: Received goal %f %f %f %s", x, y, rad2deg(yaw), frameId.c_str());

    bool do_final_rotation = true;

    if (std::isnan(yaw)) {
        ROS_WARN("MoveBasic: Received a goal with invalid orientation, will go to it but not do final turn");
        do_final_rotation = false;
    }

    // The pose of the robot planning frame MUST be known initially, and may or may not
    // be known after that.
    // The pose of the robot in the driving frame MUST be known at all times.
    // An empty planning frame means to use what ever frame the goal is specified in.
    double goalYaw;
    std::string planningFrame;
    if (preferredPlanningFrame == "") {
       planningFrame = frameId;
       goalInPlanning = goal;
       ROS_INFO("Planning in goal frame: %s\n", planningFrame.c_str());
    }
    else if (!transformPose(frameId, preferredPlanningFrame, goal, goalInPlanning)) {
        ROS_WARN("MoveBasic: Will attempt to plan in %s frame", alternatePlanningFrame.c_str());
        if (!transformPose(frameId, alternatePlanningFrame, goal,
            goalInPlanning)) {
            abortGoal("MoveBasic: No localization available for planning");
            return;
        }
        planningFrame = alternatePlanningFrame;
        goalYaw = yaw;
    }
    else {
        planningFrame = preferredPlanningFrame;
    }

    getPose(goalInPlanning, x, y, goalYaw);
    ROS_INFO("MoveBasic: Goal in %s  %f %f %f", planningFrame.c_str(),
             x, y, rad2deg(goalYaw));

    // Publish our planned path
    nav_msgs::Path path;
    geometry_msgs::PoseStamped p0, p1;
    path.header.frame_id = frameId;
    p0.pose.position.x = x;
    p0.pose.position.y = y;
    p0.header.frame_id = frameId;
    path.poses.push_back(p0);

    tf2::Transform poseFrameId;
    if (!getTransform(baseFrame, frameId, poseFrameId)) {
         abortGoal("MoveBasic: Cannot determine robot pose in goal frame");
         return;
    }
    getPose(poseFrameId, x, y, yaw);
    p1.pose.position.x = x;
    p1.pose.position.y = y;
    p1.header.frame_id = frameId;
    path.poses.push_back(p1);

    pathPub.publish(path);

    std::string drivingFrame;
    tf2::Transform goalInDriving;
    tf2::Transform currentDrivingBase;
    // Should be at time of goal message
    if (!getTransform(preferredDrivingFrame, baseFrame, currentDrivingBase)) {
         ROS_WARN("MoveBasic: %s not available, attempting to drive using %s frame",
                  preferredDrivingFrame.c_str(), alternateDrivingFrame.c_str());
         if (!getTransform(alternateDrivingFrame,
                           baseFrame, currentDrivingBase)) {
             abortGoal("MoveBasic: Cannot determine robot pose in driving frame");
             return;
         }
         else {
             drivingFrame = alternateDrivingFrame;
         }
    }
    else {
      drivingFrame = preferredDrivingFrame;
    }

    if (!transformPose(frameId, drivingFrame, goal, goalInDriving)) {
         abortGoal("MoveBasic: Cannot determine goal pose in driving frame");
         return;
    }

    tf2::Transform goalInBase = currentDrivingBase * goalInDriving;
    {
       double x, y, yaw;
       getPose(goalInBase, x, y, yaw);
    //    ROS_INFO("MoveBasic: Goal in %s  %f %f %f", baseFrame.c_str(),
            //  x, y, rad2deg(yaw));
    }

    tf2::Vector3 linear = goalInBase.getOrigin();
    linear.setZ(0);
    double dist = linear.length();

    if (!transformPose(frameId, baseFrame, goal, goalInBase)) {
        ROS_WARN("MoveBasic: Cannot determine robot pose for rotation");
        return;
    }

    if (axis == "RZ"){
        // Initial rotation to face the goal
        // double requestedYaw = atan2(linear.y(), linear.x());

        // Direct rotation to goal

        double requestedYaw = goalYaw;
        ROS_INFO("%f", rad2deg(goalYaw));


        // if (std::abs(requestedYaw) > angularTolerance) {
        //     if (!rotate(requestedYaw, drivingFrame)) {
        //         return;
        //     }
        // }
        // else{
        //         ROS_INFO("Less than angle tolerance");
        // }
        // ROS_INFO("Completed rotation");
        if (!rotate(requestedYaw, drivingFrame)) {
            return;
        }
    }

    // Do linear portion of the goal
    if((axis=="X") || (axis=="Y")){
        // Setting Target velocity parameter
        nh.getParam("/move_base/move_basic/min_linear_velocity", minLinearVelocity);
        nh.getParam("/move_base/move_basic/max_linear_velocity", maxLinearVelocity);
        if (!moveLinear(goalInDriving, drivingFrame)) {
            return;
        }
    }

    if(axis=="XY"){
        // Setting Target velocity parameter
        nh.getParam("/move_base/move_basic/min_linear_velocity", minLinearVelocity);
        nh.getParam("/move_base/move_basic/max_linear_velocity", maxLinearVelocity);
        if (!approachLinear(goalInDriving, drivingFrame)) {
            return;
        }
    }
    sleep(localizationLatency);

    actionServer->setSucceeded();
}


// Send a motion command

void MoveBasic::sendCmd(double angular, double linear)
{
    if (stop) {
        angular = 0;
        linear = 0;
    }
    geometry_msgs::Twist msg;
    // Change according to axis
    if (axis == "X"){
        msg.linear.x = linear;
    }
    else if(axis == "Y"){
        msg.linear.y = linear;
    }
    else if(axis == "RZ"){
        msg.angular.z = angular;
    }

    cmdPub.publish(msg);
}

// Send a motion command in 2D

void MoveBasic::sendCmdXY(double angular, double linearX, double linearY)
{
    if (stop) {
        angular = 0;
        linearX = 0;
        linearY = 0;
    }
    geometry_msgs::Twist msg;
    // Change according to axis
    msg.linear.x = linearX ;
    msg.linear.y = linearY;
    msg.angular.z = angular;

    cmdPub.publish(msg);
}


// Main loop

void MoveBasic::run()
{
    ros::Rate r(20);

    while (ros::ok()) {
        ros::spinOnce();
        collision_checker->min_side_dist = minSideDist;
        forwardObstacleDist = collision_checker->obstacle_dist(true,
                                                               leftObstacleDist,
                                                               rightObstacleDist,
                                                               forwardLeft,
                                                               forwardRight);
        geometry_msgs::Vector3 msg;
        msg.x = forwardObstacleDist;
        msg.y = leftObstacleDist;
        msg.z = rightObstacleDist;
        obstacleDistPub.publish(msg);
        r.sleep();
    }
}


// Rotate relative to current orientation

bool MoveBasic::rotate(double yaw, const std::string& drivingFrame)
{
    tf2::Transform poseDriving;
    if (!getTransform(baseFrame, drivingFrame, poseDriving)) {
         abortGoal("MoveBasic: Cannot determine robot pose for rotation");
         return false;
    }

    double x, y, currentYaw;
    getPose(poseDriving, x, y, currentYaw);
    double requestedYaw = yaw;
    // normalizeAngle(requestedYaw);
    ROS_INFO("MoveBasic: Requested rotation %f", rad2deg(requestedYaw));

    int oscillations = 0;
    double prevAngleRemaining = 0;

    bool done = false;
    ros::Rate r(50);
    bool firstRotation = true;

    while (!done && ros::ok()) {
        ros::spinOnce();
        r.sleep();

        double x, y, currentYaw;
        tf2::Transform poseDriving;
        if (!getTransform(baseFrame, drivingFrame, poseDriving)) {
            abortGoal("MoveBasic: Cannot determine robot pose for rotation");
            return false;
        }
        getPose(poseDriving, x, y, currentYaw);

        double angleRemaining = requestedYaw - currentYaw;
        // ROS_INFO("requested: %f, current:%f", requestedYaw, currentYaw);
        normalizeAngle(angleRemaining);

        // double obstacle = collision_checker->obstacle_angle(angleRemaining > 0);
        double obstacle = 99999;
        double remaining = std::min(std::abs(angleRemaining), std::abs(obstacle));
        double velocity = std::max(minTurningVelocity,
            std::min(remaining, std::min(maxTurningVelocity,
                    std::sqrt(2.0 * turningAcceleration *remaining))));

        if (sign(prevAngleRemaining) != sign(angleRemaining)) {
            oscillations++;
        }
        prevAngleRemaining = angleRemaining;

        if (actionServer->isPreemptRequested()) {
            ROS_INFO("MoveBasic: Stopping rotation due to preempt");
            sendCmd(0, 0);
            actionServer->setPreempted();
            return false;
        }

        ROS_INFO("%d", oscillations);
        // ROS_INFO("remain:%f", angleRemaining);
        ROS_INFO("tolerance: %f", rad2deg(angularTolerance));

        // if (!firstRotation){
        //     if (std::abs(angleRemaining) < angularTolerance*2) {
        //         ROS_INFO("MoveBasic: Done rotation, error %f degrees", rad2deg(angleRemaining));
        //         velocity = 0;
        //         done = true;
        //     }
        //     firstRotation = false;
        // }
        if (std::abs(angleRemaining) < angularTolerance || oscillations > 2) {
            ROS_INFO("MoveBasic: Done rotation, error %f degrees", rad2deg(angleRemaining));
            velocity = 0;
            done = true;
        }
        ROS_INFO("angle: %f", rad2deg(angleRemaining));
        bool counterwise = (angleRemaining < 0.0);
        if (counterwise) {
            velocity = -velocity;
        }

        sendCmd(velocity, 0);
        ROS_DEBUG("Angle remaining: %f, Angular velocity: %f", rad2deg(angleRemaining), velocity);
    }
    return done;
}

// Move forward specified distance

bool MoveBasic::moveLinear(tf2::Transform& goalInDriving,
                           const std::string& drivingFrame)
{
    tf2::Transform poseDriving;
    if (!getTransform(drivingFrame, baseFrame, poseDriving)) {
         abortGoal("MoveBasic: Cannot determine robot pose for linear");
         return false;
    }
    tf2::Transform goalInBase = poseDriving * goalInDriving;
    tf2::Vector3 remaining = goalInBase.getOrigin();

    remaining.setZ(0);
    double requestedDistance = remaining.length();

    bool pausingForObstacle = false;
    ros::Time obstacleTime;
    ros::Duration runawayTimeout(runawayTimeoutSecs);
    ros::Time last = ros::Time::now();

    // For lateral control
    double lateralIntegral = 0.0;
    double lateralError = 0.0;
    double prevLateralError = 0.0;
    double lateralDiff = 0.0;
    double sign;

    bool done = false;
    ros::Rate r(50);

    while (!done && ros::ok()) {
        ros::spinOnce();
        r.sleep();

        if (!getTransform(drivingFrame, baseFrame, poseDriving)) {
             ROS_WARN("MoveBasic: Cannot determine robot pose for linear");
             return false;
        }
        goalInBase = poseDriving * goalInDriving;
        remaining = goalInBase.getOrigin();
        double distRemaining = sqrt(remaining.x() * remaining.x() + remaining.y() * remaining.y());


        // Change according to axis 
        if (axis == "X"){
            distRemaining = abs(remaining.x());
            if(remaining.x() < 0.0){
                sign = -1;
            }
            else{
                sign = 1;
            }
        }
        else if (axis == "Y"){
            distRemaining = abs(remaining.y());
            if(remaining.y() < 0.0){
                sign = -1;
            }
            else{
                sign = 1;
            }
        }

        // PID lateral control to keep robot on path
        double rotation = 0.0;
        lateralError = remaining.y();
        lateralDiff = lateralError - prevLateralError;
        prevLateralError = lateralError;
        lateralIntegral += lateralError;
        rotation = (lateralKp * lateralError) + (lateralKi * lateralIntegral) +
                   (lateralKd * lateralDiff);
        // Clamp rotation
        rotation = std::max(-maxLateralVelocity, std::min(maxLateralVelocity,
                                                          rotation));
        ROS_DEBUG("MoveBasic: %f L %f, R %f %f %f %f %f \n",
                  forwardObstacleDist, leftObstacleDist, rightObstacleDist,
                  remaining.x(), remaining.y(), lateralError,
                  rotation);

        // Publish messages for PID tuning
        geometry_msgs::Vector3 pid_debug;
        pid_debug.x = remaining.x();
        pid_debug.y = lateralError;
        pid_debug.z = rotation;
        errorPub.publish(pid_debug);

        // Collision Avoidance
        double obstacleDist = forwardObstacleDist;
	if (requestedDistance < 0.0) {
		obstacleDist = collision_checker->obstacle_dist(false,
                                                        	leftObstacleDist,
                                                        	rightObstacleDist,
                                                        	forwardLeft,
                                                        	forwardRight);
	}
        // ROS_INFO("%f", distRemaining);
        double velocity = std::max(minLinearVelocity,
		std::min(std::min(std::abs(obstacleDist), std::abs(distRemaining/2)),
                	std::min(maxLinearVelocity, std::sqrt(2.0 * linearAcceleration *
								    std::min(std::abs(obstacleDist), std::abs(distRemaining))))));
        // ROS_INFO("Vel:%f", velocity);

        bool obstacleDetected = (obstacleDist < forwardObstacleThreshold);
        // Disable obstacle avoidance
        obstacleDetected = false;
        if (obstacleDetected) {
            velocity = 0;
            if (!pausingForObstacle) {
                ROS_INFO("MoveBasic: PAUSING for OBSTACLE");
                obstacleTime = ros::Time::now();
                pausingForObstacle = true;
            }
            else {
                ROS_INFO("MoveBasic: Still waiting for obstacle at %f meters!", obstacleDist);
                ros::Duration waitTime = ros::Time::now() - obstacleTime;
                if (waitTime.toSec() > obstacleWaitThreshold) {
                    abortGoal("MoveBasic: Aborting due to obstacle");
                    return false;
                }
            }
        }

        if (!obstacleDetected && pausingForObstacle) {
            ROS_INFO("MoveBasic: Resuming after obstacle has gone");
            pausingForObstacle = false;
        }

        // Abort Checks
        if (actionServer->isPreemptRequested()) {
            ROS_INFO("MoveBasic: Stopping move due to preempt");
            actionServer->setPreempted();
            sendCmd(0, 0);
            return false;
        }

        /* Since we are dealing with imperfect localization we should make
         * sure we are at least runawayTimeout driving away from the goal*/
        double angleRemaining = std::atan2(remaining.y(), remaining.x());
        
        // Check Angle remaining
        // ROS_INFO("Angle remaining:%f", std::cos(angleRemaining));
        // if (std::cos(angleRemaining) < 0) {
        //     if (ros::Time::now() - last > runawayTimeout) {
        //         abortGoal("MoveBasic: Moving away from goal");
        //         sendCmd(0, 0);
        //         return false;
        //     }
        // }
        // else {
        //     // Only update time when moving towards the goal
        //     last = ros::Time::now();
        // }

        /* Finish Check */

        if (distRemaining < linearTolerance) {
            ROS_INFO("MoveBasic: Done linear, error: x: %f meters, y: %f meters", remaining.x(), remaining.y());
            velocity = 0;
            done = true;
            sendCmd(rotation, velocity);
            return done;
        }
        // ROS_INFO("Velocity %f", velocity* sign);
        sendCmd(rotation, velocity * sign);
        ROS_DEBUG("Distance remaining: %f, Linear velocity: %f", distRemaining, velocity);
    }

    return done;
}

bool MoveBasic::approachLinear(tf2::Transform& goalInDriving,
                           const std::string& drivingFrame)
{
    tf2::Transform poseDriving;
    if (!getTransform(drivingFrame, baseFrame, poseDriving)) {
         abortGoal("MoveBasic: Cannot determine robot pose for linear approach");
         return false;
    }
    tf2::Transform goalInBase = poseDriving * goalInDriving;
    tf2::Vector3 remaining = goalInBase.getOrigin();

    remaining.setZ(0);
    double requestedDistance = remaining.length();

    bool pausingForObstacle = false;
    bool doneX = false;
    bool doneY= false;
    bool doneRZ = false;
    ros::Time obstacleTime;
    ros::Duration runawayTimeout(runawayTimeoutSecs);
    ros::Time last = ros::Time::now();

    // For lateral control
    double rotation = 0.0;
    double lateralIntegral = 0.0;
    double lateralError = 0.0;
    double prevLateralError = 0.0;
    double lateralDiff = 0.0;
    double sign;
    double signX;
    double signY;
    double requestedYaw = 0;
    double x, y, currentYaw;



    bool done = false;
    ros::Rate r(50);

    while (!done && ros::ok()) {
        ros::spinOnce();
        r.sleep();

        // Fixing the Yaw angle 

        tf2::Transform poseDriving;
        if (!getTransform(baseFrame, drivingFrame, poseDriving)) {
            ROS_INFO("MoveBasic: Cannot fix robot pose for rotation");
        }
        getPose(poseDriving, x, y, currentYaw);

        double angleRemaining = requestedYaw - currentYaw;
        // ROS_INFO("requested: %f, current:%f", requestedYaw, currentYaw);
        normalizeAngle(angleRemaining);

        // double obstacle = collision_checker->obstacle_angle(angleRemaining > 0);
        double velocityRZ = 0;

        if (std::abs(angleRemaining) < angularTolerance) {
            // ROS_INFO("MoveBasic: Done rotation, error %f degrees", rad2deg(angleRemaining));
            doneRZ = true;
            }
        else{
            velocityRZ = std::max(minTurningVelocity,
            std::min(angleRemaining, std::min(maxTurningVelocity,
                    std::sqrt(2.0 * turningAcceleration *angleRemaining))));
            doneRZ= false;
        }

        ROS_INFO("Angle:%f", rad2deg(angleRemaining));


        if (!doneRZ){
            bool counterwise = (angleRemaining < 0.0);
            if (counterwise) {
                velocityRZ = -velocityRZ;
            }
            sendCmdXY(velocityRZ,0, 0);
            ROS_INFO("Correcting yaw");
        }
        else{ // Only move linear if the yaw angle has been fixed 
            ROS_INFO("Moving forward");
            if (!getTransform(drivingFrame, baseFrame, poseDriving)) {
                ROS_WARN("MoveBasic: Cannot determine robot pose for linear approach");
                return false;
            }
            goalInBase = poseDriving * goalInDriving;
            remaining = goalInBase.getOrigin();
            double distRemaining = sqrt(remaining.x() * remaining.x() + remaining.y() * remaining.y());
            // Change according to axis 
            double distRemainingX = abs(remaining.x());
            if(remaining.x() < 0.0){
                signX = -1;
            }
            else{
                signX = 1;
            }

            double distRemainingY = abs(remaining.y());
            if(remaining.y() < 0.0){
                signY = -1;
            }
            else{
                signY = 1;
            }

            double obstacleDist = 99999;


            // ROS_INFO("%f", distRemainingX);
            double velocityX = std::max(minLinearVelocity,
            std::min(std::min(std::abs(obstacleDist), std::abs(distRemainingX/2)),
                        std::min(maxLinearVelocity, std::sqrt(2.0 * linearAcceleration *
                                        std::min(std::abs(obstacleDist), std::abs(distRemainingX))))));

            // ROS_INFO("%f", distRemainingY);
            double velocityY = std::max(minLinearVelocity,
            std::min(std::min(std::abs(obstacleDist), std::abs(distRemainingY/2)),
                        std::min(maxLinearVelocity, std::sqrt(2.0 * linearAcceleration *
                                        std::min(std::abs(obstacleDist), std::abs(distRemainingY))))));
            // ROS_INFO("Vel:%f", velocity);


            // Abort Checks
            if (actionServer->isPreemptRequested()) {
                ROS_INFO("MoveBasic: Stopping move due to preempt");
                actionServer->setPreempted();
                sendCmdXY(0, 0, 0);
                return false;
            }

            /* Finish Check */
            doneX = distRemainingX < linearTolerance/2;
            doneY = distRemainingY < linearTolerance;

            if (doneX and doneY) {
                ROS_INFO("MoveBasic: Done linear, error: x: %f meters, y: %f meters", remaining.x(), remaining.y());
                velocityX = 0;
                velocityY = 0;
                done = true;
                sendCmdXY(rotation, velocityX, velocityY);
                return done;
            }
            else if(doneX){
                velocityX = 0;
            }
            else if(doneY){
                velocityY = 0;
            }
            sendCmdXY(0, velocityX * signX, velocityY * signY);
        }
    }

    return done;
}



int main(int argc, char ** argv) {
    ros::init(argc, argv, "move_basic");
    ros::NodeHandle nh("~");
    MoveBasic mb_nodeX(nh,"X");
    MoveBasic mb_nodeY(nh,"Y");
    MoveBasic mb_nodeRZ(nh, "RZ");
    MoveBasic mb_nodeXY(nh, "XY");
    mb_nodeX.run();
    mb_nodeY.run();
    mb_nodeRZ.run();
    mb_nodeXY.run();


    return 0;
}
