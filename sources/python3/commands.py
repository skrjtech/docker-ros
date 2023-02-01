#!/usr/bin/env python
# coding:utf-8
import re
import rospy
from khi_robot_msgs.srv import *
from moveit_commander import MoveGroupCommander

replaceCompile = re.compile(r'[ "]')

service = '/khi_robot_command_service'
def cmdhandler_client(type_arg , cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service, KhiRobotCmd)
        
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        resp1 = str(resp1)
        resp1 = re.sub(replaceCompile, '', resp1)
        resp1 = resp1.split("\n")
        resp1 = {key: value for (key, value) in map(lambda x: x.split(':'), resp1)}
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)

def SolenoidOn():
    result = cmdhandler_client('driver', 'get_signal 12')
    if result['cmd_ret'] == str('0'):
        # Solenoid On
        cmdhandler_client('driver', 'set_signal 12')
        rospy.sleep(1)
        # Solenoid Off
        cmdhandler_client('driver', 'set_signal -12')
        return True
    return False

def TPSupply():
    result = cmdhandler_client('driver', 'get_signal 1005')
    if result['cmd_ret'] == str('-1'):
        return True
    return False

def OpenHand():
    # State Reset
    cmdhandler_client('driver', 'set_signal -17')
    cmdhandler_client('driver', 'set_signal -18')
    # Check Hand State // Open -> 0 Close -> -1
    result = cmdhandler_client('driver', 'get_signal 1017')
    # Close ?
    if result['cmd_ret'] == str('-1'):
        cmdhandler_client('driver', 'set_signal 17')
        cmdhandler_client('driver', 'set_signal -17')
        return True
    return False

def CloseHand():
    # State Reset
    cmdhandler_client('driver', 'set_signal -17')
    cmdhandler_client('driver', 'set_signal -18')
    # Check Hand State // Open -> 0 Close -> -1
    result = cmdhandler_client('driver', 'get_signal 1018')
    # Open ?
    if result['cmd_ret'] == str('0'):
        cmdhandler_client('driver', 'set_signal 18')
        cmdhandler_client('driver', 'set_signal -18')
        return True
    return False

def objOnColorSensor():
    result = cmdhandler_client('driver', 'get_signal 1009')
    # On ColorSensor?
    if result['cmd_ret'] == str('-1'):
        return True
    return False

def ColorSensor():
    if not objOnColorSensor(): return False
    result1 = cmdhandler_client('driver', 'get_signal 1010')
    result2 = cmdhandler_client('driver', 'get_signal 1011')
    result3 = cmdhandler_client('driver', 'get_signal 1012') # alm
    if result3['cmd_ret'] == str('-1'):
        return 3
    elif (result1['cmd_ret'] == str('0')) and (result2['cmd_ret'] == str('0')): # Blue
        return 0
    elif (result1['cmd_ret'] == str('-1')) and (result2['cmd_ret'] == str('0')): # Red
        return 1
    elif (result1['cmd_ret'] == str('-1')) and (result2['cmd_ret'] == str('-1')): # Yellow 
        return 2
    else:
        return -1

class ArmHandPlay(MoveGroupCommander):
    def __init__(self, speed=.5, sleep=0.5):
        super(ArmHandPlay, self).__init__(name="manipulator")
        assert 0 <= speed <= 1, 'min 0 ~ max 1'
        self.speed = speed
        self.sleep = sleep

        self.HOME = self.GetJoint()

    def SetJoint(self, val):
        self.set_max_velocity_scaling_factor(self.speed)
        self.set_joint_value_target(val)
        
    
    def SetPose(self, val):
        self.set_max_velocity_scaling_factor(self.speed)
        self.set_pose_target(val)
    
    def GO(self):
        ret = self.go()
        rospy.sleep(self.sleep)
        return ret 

    def GetJoint(self):
        return self.get_current_joint_values()
    
    def GetPose(self):
        return self.get_current_pose()

    def GetRPY(self):
        return self.get_current_rpy()

    def PlayJoint(self, vals):
        
        self.SetJoint(self.HOME)
        self.GO()

        for v in vals:
            self.SetJoint(v)
            self.GO()

if __name__ == '__main__':
    from IPython import start_ipython
    rospy.init_node('message', anonymous=True)
    start_ipython(user_ns=globals(), argv=[])

