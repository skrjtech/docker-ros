#!/usr/bin/python3
# coding:utf-8

import rospy
from geometry_msgs.msg import Pose
from khi_robot_msgs.srv import KhiRobotCmd
from moveit_commander import MoveGroupCommander

rospy.init_node('message', anonymous=True)
service = '/khi_robot_command_service'
def cmdhandler_client(type_arg , cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service, KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return {"driver_ret": resp1.driver_ret, "as_ret": resp1.as_ret, "cmd_ret": resp1.cmd_ret}
    
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)

def DriverRestart():
    result = cmdhandler_client('driver', 'restart')
    if result['cmd_ret'] == '':
        return True
    return False

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

def HandOpen():
    # State Reset
    cmdhandler_client('driver', 'set_signal -17')
    cmdhandler_client('driver', 'set_signal -18')
    # Check Hand State // Open -> 0 Close -> -1
    result = cmdhandler_client('driver', 'get_signal 1018')
    # Close ?
    if result['cmd_ret'] == str('0'):
        cmdhandler_client('driver', 'set_signal 17')
        cmdhandler_client('driver', 'set_signal -17')
        return True
    return False

def HandCloses():
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

import numpy as np
from Points import PosisionsData
class RS007N(MoveGroupCommander):
    def __init__(self, speed: float=.5, sleep: float=0.5):
        # マニピュレータ操作
        super(RS007N, self).__init__(name="manipulator")
        rospy.init_node('message', anonymous=True)

        assert 0 <= speed <= 1, 'min:0 < speed < max:1'
        
        self.set_max_velocity_scaling_factor(speed)

        self.speed = speed
        self.sleep = sleep
    
    def Home(self):
        HOME = PosisionsData['Home']
        self.set_joint_value_target(HOME)
        self.go()
    
    # レーン位置の設定
    def TPReturnTop(self):
        """
            収納レーンの上に移動
        """
        position = PosisionsData['TPReturnTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPReturnSet(self):
        """
            収納レーンに設置
        """
        position = PosisionsData['TPReturnSet']
        self.set_joint_value_target(position)
        self.go()
    
    def TPReturnPickTop(self):
        """
            収納レーンからピックする物体の上に移動
        """
        position = PosisionsData['TPReturnPickTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPReturnPick(self):
        """
            収納レーンから物体をピック
        """
        position = PosisionsData['TPReturnPick']
        self.set_joint_value_target(position)
        self.go()
    
    # カラーセンサ台位置の設定
    def TPColorSide(self):
        """
            カラーセンサ台脇に移動
        """
        position = PosisionsData["TPColorSide"]
        self.set_joint_value_target(position)
        self.go()
    
    def TPColorTop(self):
        """
            カラーセンサ台の上に移動
        """
        position = PosisionsData['TPColorTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPColorSet(self):
        """
            カラーセンサ台にセット
        """
        position = PosisionsData['TPColorSet']
        self.set_joint_value_target(position)
        self.go()
    
    def TPColorPick(self):
        """
            カラーセンサ台からピック
        """
        position = PosisionsData['TPColorPick']
        self.set_joint_value_target(position)
        self.go()
    
    # カラーボックス位置の設定 (RED)
    def TPBoxRedSetTop(self):
        position = PosisionsData['TPBoxRedSetTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxRedSet(self):
        position = PosisionsData['TPBoxRedSet']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxRedPickTop(self):
        position = PosisionsData["TPBoxRedPickTop"]
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxRedPick(self):
        position = PosisionsData["TPBoxRedPick"]
        self.set_joint_value_target(position)
        self.go()
    
    # カラーボックス位置の設定 (BLUE)
    def TPBoxBlueSetTop(self):
        position = PosisionsData['TPBoxBlueSetTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxBlueSet(self):
        position = PosisionsData['TPBoxBlueSet']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxBluePickTop(self):
        position = PosisionsData['TPBoxBluePickTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxBluePick(self):
        position = PosisionsData['TPBoxBluePick']
        self.set_joint_value_target(position)
        self.go()
    
    # カラーボックス位置の設定 (YELLOW)
    def TPBoxYellowSetTop(self):
        position = PosisionsData['TPBoxYellowSetTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxYellowSet(self):
        position = PosisionsData['TPBoxYellowSet']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxYellowPickTop(self):
        position = PosisionsData['TPBoxYellowPickTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxYellowPick(self):
        position = PosisionsData['TPBoxYellowPick']
        self.set_joint_value_target(position)
        self.go()
    
    # カラーボックス位置の設定 (ALM)
    def TPBoxAlmSetTop(self):
        position = PosisionsData['TPBoxAlmSetTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxAlmSet(self):
        position = PosisionsData['TPBoxAlmSet']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxAlmPickTop(self):
        position = PosisionsData['TPBoxAlmPickTop']
        self.set_joint_value_target(position)
        self.go()
    
    def TPBoxAlmPick(self):
        position = PosisionsData['TPBoxAlmPick']
        self.set_joint_value_target(position)
        self.go()

    def PositionShift(self, pose: Pose, x: float, y: float, z: float):
        pose.position.x += x
        pose.position.y += y
        pose.position.z += z
        return pose

if __name__ == '__main__':
    from IPython import start_ipython
    rospy.init_node('message', anonymous=True)
    armHand = RS007N()
    start_ipython(user_ns=globals(), argv=[])
