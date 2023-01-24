#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from tf.transformations import quaternion_from_euler
import numpy as np 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

def main():
    rospy.init_node("test_move")
    yaw = rospy.get_param("test_move/RZ", 0.0)
    x = rospy.get_param("test_move/X", 0.0)
    y = rospy.get_param("test_move/Y", 0.0)
    useOffset = rospy.get_param("test_move/offset", False)

    ns = "/move_base"
    RZ_server = SimpleActionClient(ns + '/RZ', MoveBaseAction)
    Y_server = SimpleActionClient(ns + '/Y', MoveBaseAction)
    X_server = SimpleActionClient(ns + '/X', MoveBaseAction)

    X_server.cancel_all_goals()
    Y_server.cancel_all_goals()
    RZ_server.cancel_all_goals()
    RZ_simple_pub = rospy.Publisher(ns + "/goal", PoseStamped, queue_size=10)

    pose = PoseStamped(pose = Pose(Point(x,y,0), Quaternion(*quaternion_from_euler(0,0,np.deg2rad(yaw))) ))
    pose.header.stamp = rospy.Time.now()
    if useOffset:
        pose.header.frame_id = "base_link"
    else:
        pose.header.frame_id = "map"

    goal = MoveBaseGoal()
    goal.target_pose = pose 

    rospy.sleep(1)
    RZ_server.wait_for_server()
    RZ_simple_pub.publish(pose)

    # Z
    # RZ_server.send_goal(goal)
    # RZ_server.wait_for_result()
    # rospy.loginfo("finish RZ")
    # rospy.sleep(1)

    # Y
    # pose.header.stamp = rospy.Time.now()
    # Y_server.send_goal(goal)
    # Y_server.wait_for_result()
    # rospy.loginfo("finish Y")
    # rospy.sleep(1)

    # X
    pose.header.stamp = rospy.Time.now()
    X_server.send_goal(goal)
    X_server.wait_for_result()
    rospy.loginfo("finish X")
    rospy.sleep(1)

if __name__ == "__main__":
    main()