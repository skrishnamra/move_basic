#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from tf.transformations import quaternion_from_euler
import numpy as np 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient, SimpleActionServer
import tf2_ros
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from mobile_cmd_control.msg import move_commandAction, move_commandResult, move_commandGoal

class ARMove(object):
    def __init__(self):
        rospy.on_shutdown(self.clear_goals)
        self.BASE_CAMERA_OFFSET = 0.31
        self.target_id = 0
        ns = "/move_base"
        self.RZ_server = SimpleActionClient(ns + '/RZ', MoveBaseAction)
        self.Y_server = SimpleActionClient(ns + '/Y', MoveBaseAction)
        self.X_server = SimpleActionClient(ns + '/X', MoveBaseAction)
        self.clear_goals()
        self.target_pub = rospy.Publisher(ns + "/goal", PoseStamped, queue_size=10)
        self.X_server.wait_for_server()
        self.Y_server.wait_for_server()
        self.RZ_server.wait_for_server()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.ar_move_as = SimpleActionServer("ar_move", move_commandAction, self.move, auto_start=False)
        self.ar_move_as.start()
        rospy.sleep(2)
        
    def wait_for_id(self, target_id,timeout=30):
        time_limit = rospy.Time.now() + rospy.Duration(timeout)
        while(rospy.Time(timeout) < rospy.Time.now()):
            try:
                msg = rospy.wait_for_message("/camera_front/fiducial_transforms", FiducialTransformArray, timeout=0.1)
                for f in msg.transforms:
                    if target_id == f.fiducial_id:
                        self.clear_goals()
                        break  
            except:
                rospy.logdebug("Fiducial_transform not published")
                rospy.sleep(0.2)
                pass
           
            
    def ar_follow(self,target_id):  
        use_search = False
        # Move towards the ID if not already seen 
        # Search direction 1:Left -1:Right 
        use_search = True
        if use_search:
            search_direction = 1
            search_distance = 2
            self.search_y(self.move_offset([0.0,search_direction * search_distance,0]), target_id)
        rospy.sleep(0.5)
 
        
        # Move towards the AR Board 
        goal = self.AR_offset(target_id, [-1 -self.BASE_CAMERA_OFFSET,0.0,0.0])
        self.target_id = target_id
        self.move_y(goal)
        rospy.sleep(3.0)
        goal = self.AR_offset(target_id, [-1 -self.BASE_CAMERA_OFFSET,0.0,0.0])
        self.move_x(goal) 
        
        
    def move(self, move_goal):
        # if cmd.enable_x == True and cmd.enable_y == True and cmd.enable_z == True:
        
        # AR task
        id_list = [0,1]
        for id in id_list:
            self.ar_follow(id) 
            if not id_list[-1]:
                self.move_x(self.move_offset([-0.5,0,0]))
            rospy.sleep(2)

        # move_goal = self.init_move_goal(frame="base_link")
        # move_goal.target_pose.pose.position.x += 0.4
        # self.move_x(move_goal)
        
        
        
    def init_move_goal(self, frame="map"):
        pose = PoseStamped(pose = Pose(Point(), Quaternion(0,0,0,1)))
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = frame
        move_goal = MoveBaseGoal()
        move_goal.target_pose = pose 
        return move_goal   
           
            
    def AR_offset(self, id, offset):
        try:
            trans = self.tfBuffer.lookup_transform('map','ar_board_' + str(id),rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        p,q = self.transform_stamped_to_pq(trans)
        pose = PoseStamped(pose = Pose(Point(*p), Quaternion(*q )))
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.pose.position.x += offset[0]
        goal.target_pose.pose.position.y += offset[1] 
        goal.target_pose.pose.position.z += offset[2]
        return goal
        
    def move_offset(self,offset):
        pose = PoseStamped(pose = Pose(Point(*offset), Quaternion(0,0,0,1)))
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        
        
        goal = MoveBaseGoal()
        goal.target_pose = pose
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
        
    def search_y(self,goal,id):
        self.target_pub.publish(goal.target_pose)
        self.Y_server.send_goal(goal)
        self.wait_for_id(id)
        rospy.loginfo("Found Y")
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
        rospy.loginfo("Goals cleared")
        
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
    
    move_client = SimpleActionClient("ar_move", move_commandAction)
    move_client.wait_for_server()
    move_client.send_goal(move_commandGoal())
    

    while not rospy.is_shutdown():
        rospy.spin()