#!/usr/bin/env python

# This code refers to the example of Seed R7 ROS Repository
# Modified by Muhammad Labiyb Afakh and Muhammad Ramadhan Hadi Setyawan

# -*- coding: utf-8 -*-
import sys
import time
import rospy
##-- for smach
from smach import State,StateMachine
import smach_ros
##-- for navigation
import yaml
import tf
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
##-- for find pkg
import rospkg
##-- for moveit
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int8
import geometry_msgs.msg
##-- for hand control
from seed_r7_ros_controller.srv import*
from time import sleep
import numpy as np
import math

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal


global object_position_x, object_position_y, object_position_z
global object_flag
global object_type

object_flag = 0
object_type = 0

object_position_x=0.0
object_position_y=0.0
object_position_z=0.0

###########################################
class NaviAction:
  def __init__(self):
    rospack = rospkg.RosPack()
    rospack.list() 
    path = rospack.get_path('wrs2023_navigation')
    with open(path + '/config/waypoints.yaml') as f:
        self.config = yaml.load(f)
    rospy.on_shutdown(self.shutdown)
    self.ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    while not self.ac.wait_for_server(rospy.Duration(5)):
      rospy.loginfo("Waiting for the move_base action server to come up")
    rospy.loginfo("The server comes up")
    self.goal = MoveBaseGoal()

  def set_goal(self,_number):
    rospy.on_shutdown(self.shutdown)

    rev = dict(self.config[_number]) #List to Dictionary

    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.goal.target_pose.pose.position.x = rev['pose']['position']['x']
    self.goal.target_pose.pose.position.y = rev['pose']['position']['y']
    self.goal.target_pose.pose.position.z = rev['pose']['position']['z']
    self.goal.target_pose.pose.orientation.x = rev['pose']['orientation']['x']
    self.goal.target_pose.pose.orientation.y = rev['pose']['orientation']['y']
    self.goal.target_pose.pose.orientation.z = rev['pose']['orientation']['z']
    self.goal.target_pose.pose.orientation.w = rev['pose']['orientation']['w']

    rospy.loginfo('Sending goal')
    self.ac.send_goal(self.goal)
    succeeded = self.ac.wait_for_result(rospy.Duration(60));
    state = self.ac.get_state();
    if succeeded:
      rospy.loginfo("Succeed")
      return 'succeeded'
    else:
      rospy.loginfo("Failed")
      return 'aborted'

  def shutdown(self):
    #rospy.loginfo("The robot was terminated")
    self.ac.cancel_goal()
#--------------------------------
class GO_TO_PLACE(State):
  def __init__(self,_place):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.place_ = _place

  def execute(self, userdata):
    rospy.loginfo('Going to Place{}'.format(self.place_))
    if(na.set_goal(self.place_) == 'succeeded'):return 'succeeded'
    else: return 'aborted' 

###########################################
class HandController:
  def __init__(self):
    rospy.loginfo('waiting service')
    rospy.wait_for_service('/seed_r7_ros_controller/hand_control')

  def grasp(self):
    try:
        service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)
        response = service(0,'grasp',100)
        return True
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
        return False

  def release(self):
    try:
        service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)
        response = service(0,'release',100)
        return True
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
        return False

  ############################################################################################
  def grasp_left(self):
    try:
        service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)
        response = service(1,'grasp',100)
        return True
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
        return False

  def release_left(self):
    try:
        service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)
        response = service(1,'release',100)
        return True
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
        return False
  ############################################################################################
###########################################
class MoveitCommand:
  def __init__(self):
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

    self.group = moveit_commander.MoveGroupCommander("rarm_with_torso")
    self.group.set_pose_reference_frame("base_link")

    #self.box1 = self.box_pose(0.6,-0.3,0.36)
    #self.box2 = self.box_pose(0.7,0.3,0.76)
    #self.box3 = self.box_pose(0.4,-0.2,1)

    # Custom
    # self.box4 = self.box_pose(0.7,-0.2,0.77)
    # self.box5 = self.box_pose(0.7,0.2,0.77)

    self.pringles = self.box_pose(0,0,0)

    self.robot_model = rospy.get_param("/seed_r7_ros_controller/robot_model_plugin")

  def set_grasp_position(self, x, y, z, vel=1.0,direction="side"):
    self.group = moveit_commander.MoveGroupCommander("rarm_with_torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_planner_id( "RRTConnectkConfigDefault" )
    self.group.allow_replanning( True )

    target_pose = Pose()
    if(direction == "side"):
      self.group.set_end_effector_link("r_eef_grasp_link")
      quat = tf.transformations.quaternion_from_euler(0,0,0)
    elif(direction == "top"): 
      self.group.set_end_effector_link("r_eef_pick_link")
      quat = tf.transformations.quaternion_from_euler(-1.57,0.79,0)

    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]

    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(vel)
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'
    
  def set_grasp_position_left(self, x, y, z, vel=1.0,direction="side"):
    self.group = moveit_commander.MoveGroupCommander("larm_with_torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_planner_id( "RRTConnectkConfigDefault" )
    self.group.allow_replanning( True )

    target_pose = Pose()
    if(direction == "side"):
      self.group.set_end_effector_link("l_eef_grasp_link")
      quat = tf.transformations.quaternion_from_euler(0,0,0)
    elif(direction == "top"): 
      self.group.set_end_effector_link("l_eef_pick_link")
      quat = tf.transformations.quaternion_from_euler(-1.57,0.79,0)

    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]

    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z

    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(vel)
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'

  def set_lifter_position(self, x, z, vel=1.0):
    self.group = moveit_commander.MoveGroupCommander("torso")
    self.group.set_pose_reference_frame("base_link")
    self.group.set_end_effector_link("body_link")
    if("typef" in self.robot_model):
      distance_body_lifter = 1.065 - 0.92
    elif("typeg" in self.robot_model):
      distance_body_lifter = 0.994 - 0.857

    target_pose = Pose()

    target_pose.orientation.x = 0
    target_pose.orientation.y = 0
    target_pose.orientation.z = 0
    target_pose.orientation.w = 1

    target_pose.position.x = x
    target_pose.position.y = 0
    target_pose.position.z = z + distance_body_lifter

    self.group.set_start_state_to_current_state()
    self.group.set_pose_target(target_pose)
    self.group.set_max_velocity_scaling_factor(vel)
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("can't be solved lifter ik")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'

  def set_initial_pose(self):
    self.group = moveit_commander.MoveGroupCommander("upper_body")
    self.group.set_named_target("reset-pose")
    self.group.set_max_velocity_scaling_factor(1.0)
    
    plan = self.group.plan()
    if type(plan) is tuple: # for noetic
        plan = plan[1]
    self.group.go()

    if(len(plan.joint_trajectory.points)==0):
      rospy.logwarn("IK can't be solved")
      self.group.clear_pose_targets()
      return 'aborted'
    else: 
      self.group.execute(plan)
      return 'succeeded'

  def wait_for_state_update(self, box_name,box_is_known=False, box_is_attached=False, timeout=4):
    scene = self.scene

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      is_known = box_name in scene.get_known_object_names()

      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return 'succeeded'

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return 'aborted'

  def box_pose(self, x, y, z):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    return box_pose

  def add_objects(self):
    self.scene.add_box("shelf1", self.box_pose(0.7,0,0.1), size=(0.75, 1.5, 0.1))
    self.scene.add_box("shelf2", self.box_pose(0.7,0,0.55), size=(0.75, 1.5, 0.1))
    # self.scene.add_box("shelf2", self.box_pose(0.8,0,0.7), size=(0.5, 1.0, 0.01))
    # self.scene.add_box("shelf3", self.box_pose(0.8,0,1.1), size=(0.5, 1.0, 0.01))

    # self.scene.add_box("wall1", self.box_pose(0.8,0.5,0.75), size=(0.5, 0.01, 1.5))
    # self.scene.add_box("wall2", self.box_pose(0.8,-0.5,0.75), size=(0.5, 0.01, 1.5))
    # self.scene.add_box("box1", self.box1, size=(0.05, 0.1, 0.1))
    # self.scene.add_box("box2", self.box2, size=(0.05, 0.05, 0.1))
    # self.scene.add_box("box3", self.box3, size=(0.05, 0.05, 0.1))

    #Custom
    #self.scene.add_box("box4", self.box4, size=(0.05, 0.07, 0.1))
    #self.scene.add_box("pringles", self.pringles, size=(0.07, 0.07, 0.1))
    #self.scene.add_box("pringles", self.caffe_latte, size=(0.075, 0.075, 0.11))
    self.scene.add_box("pringles", self.pringles, size=(0.07, 0.07, 0.1))
    

    return 'succeeded'

  def remove_objects(self):
    self.scene.remove_world_object()
    return 'succeeded'

  def attach_objects(self,object_name):
    self.group.set_end_effector_link("r_eef_pick_link")
    eef_link = self.group.get_end_effector_link()
    grasping_group = 'rhand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(eef_link, object_name, touch_links=touch_links)

    return self.wait_for_state_update(object_name,box_is_attached=True, box_is_known=False, timeout=4)

  def detach_objects(self, object_name):
    self.group.set_end_effector_link("r_eef_pick_link")
    eef_link = self.group.get_end_effector_link()

    self.scene.remove_attached_object(eef_link, name=object_name)

    return self.wait_for_state_update(object_name,box_is_known=True, box_is_attached=False, timeout=4)
  
###############################################################################################################

  def attach_objects_left(self,object_name):
    self.group.set_end_effector_link("l_eef_pick_link")
    eef_link = self.group.get_end_effector_link()
    grasping_group = 'lhand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(eef_link, object_name, touch_links=touch_links)

    return self.wait_for_state_update(object_name,box_is_attached=True, box_is_known=False, timeout=4)

  def detach_objects_left(self, object_name):
    self.group.set_end_effector_link("l_eef_pick_link")
    eef_link = self.group.get_end_effector_link()

    self.scene.remove_attached_object(eef_link, name=object_name)

    return self.wait_for_state_update(object_name,box_is_known=True, box_is_attached=False, timeout=4)

###############################################################################################################

#---------------------------------
class MANIPULATE(State):
  def __init__(self,x,y,z,vel=0.5,direction="side"):
    State.__init__(self, outcomes=['succeeded','aborted'])
    # global object_position_x, object_position_y, object_position_z

    # self.x = object_position_x
    # self.y = object_position_y
    # self.z = object_position_z
    # rospy.loginfo('INIT at ({},{},{})'.format(x,y,z))

    self.x_place = x
    self.y_place = y
    self.z_place = z

    self.vel = vel
    self.direction = direction


  def execute(self, userdata):
    global object_position_x, object_position_y, object_position_z

    self.x = (mc.pringles.pose.position.x) / 1000 
    self.y = (mc.pringles.pose.position.y) / 1000 
    self.z = (mc.pringles.pose.position.z) / 1000 

    self.x = (0.08 + (math.cos(0.506) * self.x + math.sin(0.506) * self.z)) + self.x_place
    self.y = (-0.07 + self.y) + self.y_place
    self.z = (1.075 + (-math.sin(0.506) * self.x + math.cos(0.506) * self.z)) + self.z_place


    
    #rospy.loginfo('Manipulate at ({},{},{}) in scale Pos {}'.format(mc.pringles.pose.position.x,mc.pringles.pose.position.y,mc.pringles.pose.position.z,self.vel))
    rospy.loginfo('Manipulate at ({},{},{}) in scale Pos {}'.format(self.x,self.y,self.z,self.vel))

    sleep(3)

    if(mc.set_grasp_position(self.x,self.y,self.z,self.vel,self.direction) 
      == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------

#---------------------------------

class MANIPULATE_MANUAL(State):
  def __init__(self,x,y,z,vel=1.0,direction="side"):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.x_place = x
    self.y_place = y
    self.z_place = z

    # self.x = x
    # self.y = y
    # self.z = z
    self.vel = vel
    self.direction = direction

  def execute(self, userdata):
    global object_type

    if object_type == 3:
      rospy.loginfo("PLACE OBJECT PRINGLES")
      self.x = 0.7 + self.x_place 
      self.y = -0.2 + self.y_place 
      self.z = 1.15 + self.z_place 
    else:
      rospy.loginfo("PLACE OBJECT COFFEE LATTE")
      self.x = 0.7 + self.x_place 
      self.y = 0.2 + self.y_place 
      self.z = 1.15 + self.z_place 

    rospy.loginfo('Manipulate Manual Target arm at ({},{},{}) in scale velocity {}'.format(self.x,self.y,self.z,self.vel))
    if(mc.set_grasp_position(self.x,self.y,self.z,self.vel,self.direction) 
      == 'succeeded'):return 'succeeded'
    else: return 'aborted'

###############################################################################################################
class MANIPULATELEFT(State):
  def __init__(self,x,y,z,vel=1.0,direction="side"):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.x = x
    self.y = y
    self.z = z
    self.vel = vel
    self.direction = direction

  def execute(self, userdata):
    rospy.loginfo('Manipulate Left arm at ({},{},{}) in scale velocity {}'.format(self.x,self.y,self.z,self.vel))
    if(mc.set_grasp_position_left(self.x,self.y,self.z,self.vel,self.direction) 
      == 'succeeded'):return 'succeeded'
    else: return 'aborted'

class MANIPULATE_RIGH_LEFT(State):
  def __init__(self,x1,y1,z1,x2,y2,z2,vel=1.0,direction="side"):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.x1 = x1
    self.y1 = y1
    self.z1 = z1

    self.x2 = x2
    self.y2 = y2
    self.z2 = z2

    self.vel = vel
    self.direction = direction

  def execute(self, userdata):
    rospy.loginfo('Manipulate Right and Left arm at ({},{},{}) ({},{},{}) in scale velocity {}'.format(self.x1,self.y1,self.z1,self.x2,self.y2,self.z2,self.vel))
    if(mc.set_grasp_position(self.x1,self.y1,self.z1,self.vel,self.direction) and mc.set_grasp_position_left(self.x2,self.y2,self.z2,self.vel,self.direction) 
      == 'succeeded'):
      return 'succeeded'
    else: return 'aborted'

###############################################################################################################

#---------------------------------
class MOVE_LIFTER(State):
  def __init__(self,x,z,vel=1.0):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.x = x
    self.z = z
    self.vel = vel

  def execute(self, userdata):
    rospy.loginfo('Move Lifter at ({},{}) in scale velocity {}'.format(self.x,self.z,self.vel))
    if(mc.set_lifter_position(self.x,self.z,self.vel) == 'succeeded'):return 'succeeded'
    else: return 'aborted'

#---------------------------------
class INIT_POSE(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])

  def execute(self, userdata):
    rospy.loginfo('initialize wholebody')

    if(mc.set_initial_pose() == 'succeeded'):return 'succeeded'
    else: return 'aborted'

class UPDATE_OBJECTS(State):
  def __init__(self,action):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.action = action

  def execute(self, userdata):
    if(self.action == 'add'): 
      print('add objects')
      if(mc.add_objects() == 'succeeded'):return 'succeeded'
      else: return 'aborted'
    elif(self.action == 'remove'): 
      print('remove objects')
      if(mc.remove_objects() == 'succeeded'):return 'succeeded'
      else: return 'aborted'
      
class PICK(State):
  def __init__(self,object_name):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.object_name = object_name

  def execute(self, userdata):
    if(mc.attach_objects(self.object_name) == 'succeeded'):
      if(hc.grasp()): return 'succeeded'
      else: return 'aborted'
    else: return 'aborted'

class PLACE(State):
  def __init__(self,object_name):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.object_name = object_name

  def execute(self, userdata):
    if(mc.detach_objects(self.object_name) == 'succeeded'):
      if(hc.release()): return 'succeeded'
      else: return 'aborted'
    else: return 'aborted'

#############################################################################

class PICKLEFT(State):
  def __init__(self,object_name):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.object_name = object_name

  def execute(self, userdata):
    if(mc.attach_objects_left(self.object_name) == 'succeeded'):
      if(hc.grasp_left()): return 'succeeded'
      else: return 'aborted'
    else: return 'aborted'

class PLACELEFT(State):
  def __init__(self,object_name):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.object_name = object_name

  def execute(self, userdata):
    if(mc.detach_objects_left(self.object_name) == 'succeeded'):
      if(hc.release_left()): return 'succeeded'
      else: return 'aborted'
    else: return 'aborted'

#############################################################################

class MONITORING(State):
  def __init__(self, threshold):
    State.__init__(self, outcomes=['succeeded', 'aborted'])
    self.subscriber = rospy.Subscriber('counter', Int8, self.callback)
    self.threshold = threshold
    self.counter = 0

  def callback(self, data):
    self.counter = data.data
    #rospy.loginfo("Person %d %d", self.counter, self.threshold)
  
  def execute(self, userdata):
    while True:
      if self.counter > self.threshold:
        break 

    return 'succeeded'

class WAITING(State):
  #Check NFC reader, 
  # if the reader get NFC data, it will return 1
  # then, it will wait until the object is detached and return 100 
  def __init__(self,delay):
    State.__init__(self, outcomes=['succeeded', 'aborted'])
    self.delay = delay
  
  def execute(self, userdata):
    print('Delay ', self.delay)

    rospy.loginfo("Pringles Target x: %d, y: %d, z: %d", mc.pringles.pose.position.x, mc.pringles.pose.position.y, mc.pringles.pose.position.z)
    sleep(self.delay)
    return 'succeeded' 
    # else: return 'aborted'

class PAYMENT(State):
  def __init__(self):
    State.__init__(self, outcomes=['succeeded', 'aborted'])
    self.subscriber = rospy.Subscriber('nfc_checker', Int8, self.callback)
    self.flag_payment = 0

  def callback(self, data):
    self.flag_payment = data.data
    # rospy.loginfo("Flag payment: %d", self.flag_payment)

  def execute(self, userdata):
    self.flag_payment = 0
    rospy.loginfo('initialize payment')
    while not (self.flag_payment == 100):
      rospy.loginfo("NFC is not 100")
    
    self.flag_payment = 0
    return 'succeeded'


    
class GETOBJECT(State):
  #This class will get the object position,

  def __init__(self):
    State.__init__(self, outcomes=['succeeded','aborted'])
    self.subscriber = rospy.Subscriber('object_detection', PoseStamped, self.callback)
    self.subscriber_object = rospy.Subscriber('object_name', Int8, self.callback_object)
    self.i = 1


  def callback(self, data):
    global object_position_x, object_position_y, object_position_z
  
    object_position_x = data.pose.position.x
    object_position_y = data.pose.position.y
    object_position_z = data.pose.position.z
    ##rospy.loginfo(" %d Position x: %d, y: %d, z: %d", self.i, object_position[0], object_position[1], object_position[2])

  def callback_object(self, data):
    global object_flag

    object_flag = data.data

  def execute(self, userdata):
    global object_position_x, object_position_y, object_position_z
    global object_flag
    global object_type
    last_flag = object_flag

    while self.i < 20:      
      self.i += 1
    
      mc.pringles = mc.box_pose(object_position_x, object_position_y, object_position_z)

      if self.i > 0 and self.i < 20:
        if last_flag < object_flag:
          object_flag = last_flag

        object_type = object_flag

        last_flag = object_flag
      

      rospy.loginfo(" %d Pringles x: %d, y: %d, z: %d, object_type: %d, object_flag: %d", self.i, mc.pringles.pose.position.x, mc.pringles.pose.position.y, mc.pringles.pose.position.z, object_flag, object_type)

      rospy.sleep(0.5)

    self.i = 1
    if((mc.pringles.pose.position.x == 0.00) and (mc.pringles.pose.position.y == 0.00) and (mc.pringles.pose.position.z == 0.00)):
      return 'aborted'
    else:
      object_position_x = 0
      object_position_y = 0
      object_position_z = 0
      return 'succeeded'
  global object_position_x, object_position_y, object_position_z
    

#==================================
#==================================
if __name__ == '__main__':
  rospy.init_node('scenario_node')

  na = NaviAction()
  hc = HandController()
  mc = MoveitCommand()

      
  delay = StateMachine(outcomes=['succeeded', 'aborted'])
  with delay:
    StateMachine.add('DELAY', WAITING(3),\
      transitions={'succeeded':'succeeded', 'aborted':'aborted'})
  
  delay2s = StateMachine(outcomes=['succeeded', 'aborted'])
  with delay2s:
    StateMachine.add('DELAY', WAITING(2),\
      transitions={'succeeded':'succeeded', 'aborted':'aborted'})

  delay5s = StateMachine(outcomes=['succeeded', 'aborted'])
  with delay5s:
    StateMachine.add('DELAY', WAITING(5),\
      transitions={'succeeded':'succeeded', 'aborted':'aborted'})
    
  go_to_shelf = StateMachine(outcomes=['succeeded','aborted'])
  with go_to_shelf:
    StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0,0.5),\
      transitions={'succeeded':'SHELF1','aborted':'aborted'})
    StateMachine.add('SHELF1', GO_TO_PLACE(3),\
      transitions={'succeeded':'SHELF2','aborted':'aborted'})
    StateMachine.add('SHELF2', GO_TO_PLACE(4),\
      transitions={'succeeded':'SHELF3','aborted':'aborted'})
    StateMachine.add('SHELF3', GO_TO_PLACE(5),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  go_to_table = StateMachine(outcomes=['succeeded','aborted'])
  with go_to_table:
    StateMachine.add('DOWN LIFTER2', MOVE_LIFTER(0,0.5),\
      transitions={'succeeded':'MOVE4','aborted':'aborted'})
    StateMachine.add('MOVE4', GO_TO_PLACE(2),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})
    
  go_to_payment = StateMachine(outcomes=['succeeded','aborted'])
  with go_to_payment:
    StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0,0.5),\
      transitions={'succeeded':'MOVE5','aborted':'aborted'})
    StateMachine.add('MOVE5', GO_TO_PLACE(1),\
      transitions={'succeeded':'UP LIFTER','aborted':'aborted'})
    StateMachine.add('UP LIFTER', MOVE_LIFTER(0,0.9),\
      transitions={'succeeded':'PAYMENT','aborted':'aborted'})
    StateMachine.add('PAYMENT', PAYMENT(),\
      transitions={'succeeded':'DELAY5S','aborted':'aborted'})
    StateMachine.add('DELAY5S', delay5s,\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  go_to_start_point = StateMachine(outcomes=['succeeded','aborted'])
  with go_to_start_point:
    StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0,0.5),\
      transitions={'succeeded':'MOVE','aborted':'aborted'})
    StateMachine.add('MOVE', GO_TO_PLACE(0),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  lifter_up = StateMachine(outcomes=['succeeded','aborted'])
  with lifter_up:
    StateMachine.add('UP LIFTER', MOVE_LIFTER(0,0.9),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})

  lifter_down = StateMachine(outcomes=['succeeded','aborted'])
  with lifter_down:
    StateMachine.add('DOWN LIFTER', MOVE_LIFTER(0,0.55),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})
    
  lifter_up_pick = StateMachine(outcomes=['succeeded','aborted'])
  with lifter_up_pick:
    StateMachine.add('UP LIFTER PICK', MOVE_LIFTER(0,0.8),\
      transitions={'succeeded':'succeeded','aborted':'aborted'})
    
  hand_down = StateMachine(outcomes=['succeeded','aborted'])
  with hand_down:
    StateMachine.add('HAND DOWN MOTION', MANIPULATE(0, 0, 0.6, direction="top"), \
      transitions={'succeeded':'succeeded','aborted':'aborted'}) 

  go_to_wait = StateMachine(outcomes=['succeeded', 'aborted'])
  with go_to_wait:
    StateMachine.add('WAIT', WAITING(10),\
      transitions={'succeeded':'succeeded', 'aborted':'aborted'})

    
  pick_place_pringles = StateMachine(outcomes=['succeeded','aborted'])

  with pick_place_pringles:
    StateMachine.add('DELAY1', delay,\
      transitions={'succeeded':'UP LIFTER PICK','aborted':'aborted'})
    StateMachine.add('UP LIFTER PICK', lifter_up_pick,\
        transitions={'succeeded':'PICK MOTION1','aborted':'aborted'})
    StateMachine.add('PICK MOTION1', MANIPULATE(-0.04, -0.04, 0, direction="side"), \
      transitions={'succeeded':'PICK MOTION2','aborted':'aborted'})
    StateMachine.add('PICK MOTION2', MANIPULATE(0, 0, 0, direction="side"), \
      transitions={'succeeded':'PICK','aborted':'aborted'})
    StateMachine.add('PICK', PICK('pringles'),\
      transitions={'succeeded':'INITIALIZE','aborted':'aborted'})
    StateMachine.add('INITIALIZE', INIT_POSE(),\
      transitions={'succeeded':'GO TO SHELF','aborted':'aborted'})
    StateMachine.add('GO TO SHELF', go_to_shelf,\
      transitions={'succeeded':'LIFTER UP','aborted':'aborted'})
    StateMachine.add('LIFTER UP', lifter_up,\
       transitions={'succeeded':'PLACE MOTION1','aborted':'aborted'})
    StateMachine.add('PLACE MOTION1', MANIPULATE_MANUAL(0, 0, 0, direction="side"), \
      transitions={'succeeded':'PLACE','aborted':'aborted'}) 
    StateMachine.add('PLACE', PLACE('pringles'),\
      transitions={'succeeded':'PLACE MOTION2','aborted':'aborted'})
    StateMachine.add('PLACE MOTION2', MANIPULATE_MANUAL(-0.15, -0.15, 0, direction="side"), \
      transitions={'succeeded':'succeeded','aborted':'aborted'}) 
    
  scenario_play = StateMachine(outcomes=['succeeded','aborted'])
  with scenario_play:
    
    StateMachine.add('MONITORING', MONITORING(-1),\
        transitions={'succeeded':'LIFTER DOWN','aborted':'aborted'})
    StateMachine.add('LIFTER DOWN', lifter_down,\
        transitions={'succeeded':'GO TO PAYMENT','aborted':'aborted'})
    StateMachine.add('GO TO PAYMENT', go_to_payment,\
       transitions={'succeeded':'GO TO TABLE','aborted':'aborted'})
    StateMachine.add('GO TO TABLE', go_to_table,\
       transitions={'succeeded':'GET OBJECT','aborted':'aborted'})
    StateMachine.add('GET OBJECT', GETOBJECT(),\
         transitions={'succeeded':'ADD OBJECTS', 'aborted':'GO TO START POINT'})
    StateMachine.add('ADD OBJECTS', UPDATE_OBJECTS('add'),\
         transitions={'succeeded':'PICK and PLACE PRINGLES','aborted':'aborted'})
    StateMachine.add('PICK and PLACE PRINGLES', pick_place_pringles,\
       transitions={'succeeded':'INITIALIZE','aborted':'aborted'})
    StateMachine.add('INITIALIZE', INIT_POSE(),\
      transitions={'succeeded':'SHELF4','aborted':'aborted'})
    StateMachine.add('SHELF4', GO_TO_PLACE(6),\
      transitions={'succeeded':'GO TO TABLE','aborted':'aborted'})
    StateMachine.add('GO TO START POINT', go_to_start_point,\
      transitions={'succeeded':'MONITORING','aborted':'aborted'})
    
    
  sis = smach_ros.IntrospectionServer('server_name',scenario_play,'/SEED-Noid-Mover Scenario Play')
  sis.start()
  scenario_play.execute()
  sis.stop()

