#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose
from std_msgs.msg import Float32, Int32, Int32MultiArray
from tf.transformations import quaternion_from_euler, quaternion_multiply
import numpy as np 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient, SimpleActionServer
import tf2_ros
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
from mobile_cmd_control.msg import move_commandAction, move_commandResult, move_commandGoal, move_smoothAction, move_smoothGoal, \
mobile_wait, mobile_waitRequest

class ARMove(object):
    def __init__(self):
        rospy.on_shutdown(self.clear_goals)
        self.BASE_CAMERA_OFFSET = 0.31
        self.target_id = 0
        ns = "/move_base"
        self.RZ_server = SimpleActionClient(ns + '/RZ', MoveBaseAction)
        self.Y_server = SimpleActionClient(ns + '/Y', MoveBaseAction)
        self.X_server = SimpleActionClient(ns + '/X', MoveBaseAction)
        self.move_smooth_ac = SimpleActionClient(ns + '/move_smooth', move_smoothAction)
        self.clear_goals()
        self.target_pub = rospy.Publisher(ns + "/goal", PoseStamped, queue_size=10)
        self.X_server.wait_for_server()
        self.Y_server.wait_for_server()
        self.RZ_server.wait_for_server()
        self.move_smooth_ac.wait_for_server()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.ar_move_as = SimpleActionServer("ar_move", move_commandAction, self.move, auto_start=False)
        self.ar_move_as.start()
        self.ar_move_as.register_preempt_callback(self.stop)
        self.enable_ur3 = rospy.get_param("~enable_ur3", False)
        rospy.loginfo(self.enable_ur3)
        rospy.sleep(2)


        # UR3 - Wait for Output Register on UR3 using client 
        rospy.wait_for_service('ur3/modbus_wait')
        self.srv_ur3_wait = rospy.ServiceProxy('ur3/modbus_wait', mobile_wait)
        # Publisher to Modbus Server to write register to start UR3
        self.ur3_start_pub = rospy.Publisher("ur3/command_state", Int32MultiArray, queue_size=10)

        
    def wait_for_id(self, target_id,use_smooth=False,timeout=30):
        time_limit = rospy.Time.now() + rospy.Duration(timeout)
        while(rospy.Time(timeout) < rospy.Time.now()):
            try:
                msg = rospy.wait_for_message("/camera_front/fiducial_transforms", FiducialTransformArray, timeout=0.1)
                for f in msg.transforms:
                    if target_id == f.fiducial_id:
                        if not use_smooth:
                            self.clear_goals()
                        return 
                    
            except:
                rospy.logdebug("Fiducial_transform not published")
                rospy.sleep(0.05)
                pass
           
            
    def ar_follow(self,target_id):  
        use_search = True
        self.use_smooth = True
        try:
            msg = rospy.wait_for_message("/camera_front/fiducial_transforms", FiducialTransformArray, timeout=2.0)
            for f in msg.transforms:
                if target_id == f.fiducial_id:
                    use_search = False
                    if target_id == self.id_list[0]:
                        self.move_rz(self.rotate_to_angle([0,0,0]))
                        
        except:
            rospy.loginfo("Searching for AR Marker")

        # Move towards the ID if not already seen 
        # Search direction 1:Left -1:Right 
        if use_search:
            search_direction = 1
            search_distance = 2
            self.search_y(self.move_offset([0.0,search_direction * search_distance,0]), target_id, use_smooth=self.use_smooth)
            if self.use_smooth: 
                # Move towards the AR Board 
                now = rospy.Time.now()
                goal = self.AR_offset(target_id, [-1 -self.BASE_CAMERA_OFFSET,0.0,0.0])
                self.target_id = target_id
                if target_id == self.id_list[0]:
                    rospy.sleep(0.2)
                    self.X_server.cancel_goals_at_and_before_time(now)
                    self.Y_server.cancel_goals_at_and_before_time(now)
                    self.RZ_server.cancel_goals_at_and_before_time(now)
                    rospy.sleep(1.5)
                    self.move_rz(self.rotate_to_angle([0,0,0]))
                    goal = self.AR_offset(target_id, [-1 -self.BASE_CAMERA_OFFSET,0.0,0.0])
                    self.move_y(goal)
                else:
                    self.move_y(goal, blocking=False)
                    self.move_smooth(direction=3)
                    rospy.sleep(0.3)
                    self.X_server.cancel_goals_at_and_before_time(now)
                    self.Y_server.cancel_goals_at_and_before_time(now)
                    self.RZ_server.cancel_goals_at_and_before_time(now)
                    self.Y_server.wait_for_result()
            else:
                if target_id == self.id_list[0]:
                    self.move_rz(self.rotate_to_angle([0,0,0]))
                # Move towards the AR Board 
                goal = self.AR_offset(target_id, [-1 -self.BASE_CAMERA_OFFSET,0.0,0.0])
                self.target_id = target_id
                self.move_y(goal)
            rospy.sleep(3)
        goal = self.AR_offset(target_id, [-1 -self.BASE_CAMERA_OFFSET,0.0,0.0])
        self.move_x(goal) 
        
        
    def move(self, move_goal):        
        # AR Move
        if move_goal.use_marker.data:
            # AR task
            # self.id_list = list(range(move_goal.id.data + 1))
            self.id_list = [1,2]
            for id in self.id_list:
                self.ar_follow(id) 
                if self.enable_ur3:
                    # Calls service which Blocks mobile base if UR3 is not done with movement 
                    # UR Moving
                    self.move_client.cancel_all_goals()
                    self.ur3_start_pub.publish(Int32MultiArray(data = [1,0]))
                    #Need to check if UR is Moving 
                    wait = self.srv_ur3_wait(mobile_waitRequest(timeout=Int32(20)))
                    if wait.result == False:
                        rospy.loginfo("UR still moving")
                        self.ur3_start_pub.publish(Int32MultiArray(data = [0,1]))
                    rospy.loginfo("ur_done")
                    # Tel UR3 to Stop
                    self.ur3_start_pub.publish(Int32MultiArray(data = [0,1]))
                    rospy.sleep(2.0)
                if id is not self.id_list[-1]:
                    rospy.sleep(1)
                    self.move_x(self.move_offset([-0.5,0,0]))
                rospy.sleep(2)
        # Normal Move
        else:
            if move_goal.enable_rz.data:
                self.move_rz(self.rotate_offset(move_goal.target_pose))
            if move_goal.enable_y.data:  
                self.move_y(self.move_offset(move_goal.target_pose))
                self.move_smooth(direction=3)
            if move_goal.enable_x.data:
                self.move_x(self.move_offset(move_goal.target_pose))
        self.ar_move_as.set_succeeded()
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

    # Only for y axis
    def move_smooth(self, direction, blocking=False):
        smooth_goal = move_smoothGoal()
        smooth_goal.target_distance = Float32(0.2)
        smooth_goal.target_speed = Float32(0.05)
        smooth_goal.direction = Int32(direction)
        self.move_smooth_ac.send_goal(smooth_goal)
        if blocking:
            self.move_smooth_ac.wait_for_result()     
            
    def AR_offset(self, id, offset):
        try:
            trans = self.tfBuffer.lookup_transform('map','ar_board_' + str(id),rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

        p,q = self.transform_stamped_to_pq(trans)
        pose = PoseStamped(pose = Pose(Point(*p), Quaternion(0,0,0,1) ))
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        
        
        goal = MoveBaseGoal()
        goal.target_pose = pose
        goal.target_pose.pose.position.x += offset[0]
        goal.target_pose.pose.position.y += offset[1] 
        goal.target_pose.pose.position.z += offset[2]
        return goal

    def rotate_offset(self,offset):
        if isinstance(offset, list) or isinstance(offset, np.ndarray):
            pose = PoseStamped(pose = Pose(Point(), Quaternion(*quaternion_from_euler(0,0,np.rad2deg(offset[2])))))
        else:
            try:
                trans = self.tfBuffer.lookup_transform('map','base_link',rospy.Time(0), rospy.Duration(1.0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise
            p,q = self.transform_stamped_to_pq(trans)
            q_target = [offset.pose.orientation.x, offset.pose.orientation.y, offset.pose.orientation.z, offset.pose.orientation.w]
            q = quaternion_multiply(q, q_target)
            pose = PoseStamped(pose = Pose(Point(*p), Quaternion(*q)))
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        goal = MoveBaseGoal()
        goal.target_pose = pose
        return goal

    def move_offset(self,offset):
        if isinstance(offset, list) or isinstance(offset, np.ndarray):
            pose = PoseStamped(pose = Pose(Point(*offset), Quaternion(0,0,0,1)))
        else:
            pose = PoseStamped()
            pose = offset
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        
        
        goal = MoveBaseGoal()
        goal.target_pose = pose
        return goal
    
    def rotate_to_angle(self, target_yaw):  
        try:
            trans = self.tfBuffer.lookup_transform('map','base_link',rospy.Time(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise
        q = quaternion_from_euler(*target_yaw)
        pose = PoseStamped(pose = Pose(Point(), Quaternion(*q)))
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position = trans.transform.translation
        
        goal = MoveBaseGoal()
        goal.target_pose = pose
        return goal
        
    def move_x(self, goal,blocking=True):
        self.target_pub.publish(goal.target_pose)
        self.X_server.send_goal(goal)
        if blocking:
            self.X_server.wait_for_result()
        rospy.loginfo("finish X")
        rospy.sleep(1)
        
    def move_y(self,goal,blocking=True):
        self.target_pub.publish(goal.target_pose)
        self.Y_server.send_goal(goal)
        if blocking:
            self.Y_server.wait_for_result()
        rospy.loginfo("finish Y")
        rospy.sleep(1)
        
    def search_y(self,goal,id, use_smooth=False):
        self.target_pub.publish(goal.target_pose)
        self.Y_server.send_goal(goal)
        self.wait_for_id(id, use_smooth=use_smooth)
        rospy.loginfo("Found Y")
        rospy.sleep(1)
        
    def move_rz(self,goal,blocking=True):
        self.target_pub.publish(goal.target_pose)
        self.RZ_server.send_goal(goal)
        rospy.loginfo("Doing RZ")
        if blocking:
            self.RZ_server.wait_for_result()
        rospy.loginfo("finish RZ")
        rospy.sleep(1)
        
    def clear_goals(self):
        self.X_server.cancel_all_goals()
        self.Y_server.cancel_all_goals()
        self.RZ_server.cancel_all_goals()
        rospy.loginfo("Goals cleared")

    def stop(self):
        self.clear_goals()
        rospy.loginfo("Preempt Request")


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
    
    # move_client = SimpleActionClient("ar_move", move_commandAction)
    # move_client.wait_for_server()
    # move_client.send_goal(move_commandGoal())
    

    while not rospy.is_shutdown():
        rospy.spin()