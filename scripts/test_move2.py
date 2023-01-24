#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from tf.transformations import quaternion_from_euler
import numpy as np 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient, SimpleActionServer
import tf2_ros
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from mobile_cmd_control.msg import move_commandGoal, move_commandAction

class ARMove(object):
    def __init__(self):
        rospy.on_shutdown(self.clear_goals)
        self.BASE_CAMERA_OFFSET = 0.31
        self.target_id = 0
        ns = "move_base"
        self.RZ_server = SimpleActionClient(ns + '/RZ', MoveBaseAction)
        self.Y_server = SimpleActionClient(ns + '/Y', MoveBaseAction)
        self.X_server = SimpleActionClient(ns + '/X', MoveBaseAction)
        self.X_server.wait_for_server()
        self.Y_server.wait_for_server()
        self.RZ_server.wait_for_server()
        rospy.loginfo("move_base ok")
        self.clear_goals()
        self.target_pub = rospy.Publisher(ns + "/goal", PoseStamped, queue_size=10)
        
        self.ar_move_as = SimpleActionServer("ar_move", move_commandAction, self.move, auto_start=False)
        self.ar_move_as.start()
        rospy.loginfo("server_started")
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def wait_for_id(self):
        while True:
            msg = rospy.wait_for_message("/camera_front/fiducial_transforms", FiducialTransformArray, self.cb_fiducial, timeout=0.1)
            if msg is not None:
                for f in msg.transforms:
                    if self.target_id == f.fiducial_id:
                        break          
            else:
                rospy.sleep(0.2)
                continue
        
    def move(self, move_goal):
        # pose = PoseStamped(pose = Pose(Point(x,y,0), Quaternion(*quaternion_from_euler(0,0,np.deg2rad(yaw))) ))
        # pose.header.stamp = rospy.Time.now(
        # if cmd.enable_x == True and cmd.enable_y == True and cmd.enable_z == True:
        rospy.loginfo("running")
        goal = self.AR_offset(self.target_id, [-1 -self.BASE_CAMERA_OFFSET,0,0])
        self.move_y(goal)
        rospy.sleep(1)
        self.move_x(goal)
            
    def AR_offset(self, id, offset):
        try:
            trans = self.tfBuffer.lookup_transform('map','ar_board_' + str(id),rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        p,q = self.transform_stamped_to_pq(trans)
        q = [0,0,0,1]
        pose = PoseStamped(pose = Pose(Point(*p), Quaternion(*q )))
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.pose.position.x += offset[0]
        goal.target_pose.pose.position.y += offset[1] 
        goal.target_pose.pose.position.z += offset[2]
        rospy.loginfo(goal)
        return goal
        
        
        
    def move_x(self, goal):
        self.target_pub.publish(goal.target_pose)
        self.X_server.send_goal(goal)
        self.X_server.wait_for_result()
        rospy.loginfo("finish X")
        rospy.sleep(1)
        
    def move_y(self,goal):
        self.target_pub.publish(goal.target_pose)
        self.Y_server.send_goal(goal)
        self.Y_server.wait_for_result()
        rospy.loginfo("finish Y")
        rospy.sleep(1)
        
    def move_rz(self,goal):
        self.target_pub.publish(goal.target_pose)
        self.RZ_server.send_goal(goal)
        self.RZ_server.wait_for_result()
        rospy.loginfo("finish RZ")
        rospy.sleep(1)
        
    def clear_goals(self):
        self.X_server.cancel_all_goals()
        self.Y_server.cancel_all_goals()
        self.RZ_server.cancel_all_goals()
        
    def transform_stamped_to_pq(self,trans):
        p = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
        q = np.array([trans.transform.rotation.x, trans.transform.rotation.y,
        trans.transform.rotation.z, trans.transform.rotation.w])
        return p,q
        
    
def main():
    pass


if __name__ == "__main__":
    rospy.init_node("test_move")
    # main()
    ar_move = ARMove()
    rospy.sleep(2)

    ac = SimpleActionClient("ar_move", move_commandAction)
    rospy.loginfo("start")
    ac.wait_for_server()
    ac.send_goal(move_commandGoal())

    while not rospy.is_shutdown():
        rospy.spin()

