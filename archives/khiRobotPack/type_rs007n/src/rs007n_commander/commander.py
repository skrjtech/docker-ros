#!/usr/bin/python3
# coding:utf-8

import rospy
from geometry_msgs.msg import Pose
from khi_robot_msgs.srv import KhiRobotCmd
from moveit_commander import MoveGroupCommander

# rospy.init_node('message', anonymous=True)

service = '/khi_robot_command_service'
# コマンド実行と結果の呼び出し
def cmdhandler_client(type_arg , cmd_arg) -> dict:
    """
        cmdhandler_client
            - type_arg: str = 'driver' | 'as'
            - cmd_arg: str = 'command'
    """
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service, KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return {"driver_ret": resp1.driver_ret, "as_ret": resp1.as_ret, "cmd_ret": resp1.cmd_ret}
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)

# モーターの起動 / 状態のリセット
def DriverRestart() -> bool:
    result = cmdhandler_client('driver', 'restart')
    if result['cmd_ret'] == '':
        return True
    return False

# ソレノイド ON・OFF 作動後2秒で復帰
def SolenoidOn() -> bool:
    result = cmdhandler_client('driver', 'get_signal 12')
    if result['cmd_ret'] == str('0'):
        # Solenoid On
        cmdhandler_client('driver', 'set_signal 12')
        rospy.sleep(2)
        # Solenoid Off
        cmdhandler_client('driver', 'set_signal -12')
        return True
    return False

# 収納レーンセンサー 状態確認 
def TPSupply() -> bool:
    result = cmdhandler_client('driver', 'get_signal 1005')
    if result['cmd_ret'] == str('-1'):
        return True
    return False

# 指先 開
def HandOpen() -> bool:
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

# 指先 閉
def HandClose() -> bool:
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

# カラーセンサ台 物体存在有無
def EColorSensor() -> bool:
    result = cmdhandler_client('driver', 'get_signal 1009')
    # On ColorSensor?
    if result['cmd_ret'] == str('-1'):
        return True
    return False

# カラーセンサ台 色認識
def ColorSensor() -> int:
    if not EColorSensor(): return -1
    result1 = ~ int(cmdhandler_client('driver', 'get_signal 1010')['cmd_ret'])
    result2 = ~ int(cmdhandler_client('driver', 'get_signal 1011')['cmd_ret'])
    result3 = ~ int(cmdhandler_client('driver', 'get_signal 1012')['cmd_ret']) # alm
    
    if result3 == 0: return 3 # Alm
    elif (result1 == -1) and (result2 == -1): return 0 # Blue
    elif (result1 ==  0) and (result2 == -1): return 1 # Red
    elif (result1 ==  0) and (result2 ==  0): return 2 # Red
    else: return -1

class DriverCommander(object):
    def DriverRestart(self) -> bool: return DriverRestart()
    def SolenoidOn(self) -> bool: return SolenoidOn()
    def HandOpen(self) -> bool: return HandOpen()
    def HandCloses(self) -> bool: return HandClose()
    def TPSupply(self) -> bool: return TPSupply()
    def EColorSensor(self) -> bool: return EColorSensor()
    def ColorSensor(self) -> int: return ColorSensor

from .Points import PosisionsData
class RS007NCommander(MoveGroupCommander):
    def __init__(self, speed: float=.5, sleep: float=.5):
        """
            RS007N
                - speed: float (0 ~ 1) 移動速度 最大速度：１
                - sleep: float         停止時間 (sec)
        """

        # マニピュレータ操作
        super(RS007NCommander, self).__init__(name="manipulator")
        rospy.init_node('message', anonymous=True)

        assert 0 <= speed <= 1, 'min:0 < speed < max:1'
        self.set_max_velocity_scaling_factor(speed)

        self.speed = speed
        self.sleep = sleep
    
    # 初期位置
    def Home(self):
        position = PosisionsData['Home']
        self.set_joint_value_target(position)
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
    
    def P11Set(self):
        position = PosisionsData['P11']
        self.set_joint_value_target(position)
        self.go()

    def PositionShift(self, pose: Pose, x: float, y: float, z: float):
        pose.position.x += (x / 1000) 
        pose.position.y += (y / 1000)
        pose.position.z += (z / 1000)
        return pose
    
    def RPYShift(self, pose: Pose, r: float, p: float, y: float):
        pose.position.r += r
        pose.position.p += p
        pose.position.y += y
        return pose

if __name__ == '__main__':
    from IPython import start_ipython
    rospy.init_node('message', anonymous=True)
    armHand = RS007N()
    start_ipython(user_ns=globals(), argv=[])
