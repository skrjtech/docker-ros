#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from enum import Enum
from khi_robot_msgs.srv import KhiRobotCmd

class ColorSensorType(Enum):
    RED = 1
    BLUE = 3
    YELLOW = 4
    ALUMINUM = 5
    UNKNOWN = 0

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
def SolenoidOnOff(time=2) -> bool:
    result = cmdhandler_client('driver', 'get_signal 12')
    if result['cmd_ret'] == str('0'):
        # Solenoid On
        cmdhandler_client('driver', 'set_signal 12')
        rospy.sleep(time)
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
def EColorSensor() -> int:
    result = int(cmdhandler_client('driver', 'get_signal 1009')['cmd_ret'])
    return result

# カラーセンサ台 色認識
def ColorSensor() -> int:
    result0 = ~ EColorSensor() + 1
    result1 = ~ int(cmdhandler_client('driver', 'get_signal 1010')['cmd_ret']) + 1
    result2 = ~ int(cmdhandler_client('driver', 'get_signal 1011')['cmd_ret']) + 1
    result3 = ~ int(cmdhandler_client('driver', 'get_signal 1012')['cmd_ret']) + 1 # alm

    Color = result0 + result1 + result2 + result3

    if Color == 0: return ColorSensorType.UNKNOWN
    if Color == 1: return ColorSensorType.BLUE
    if Color == 2: return ColorSensorType.RED
    if Color == 3: return ColorSensorType.YELLOW
    if Color == 4: return ColorSensorType.ALUMINUM
    
    return ColorSensorType.UNKNOWN
    # if result3 == 0: return ColorSensorType.ALUMINUM # Alm
    # elif (result1 == -1) and (result2 == -1): return ColorSensorType.BLUE # Blue
    # elif (result1 ==  0) and (result2 == -1): return ColorSensorType.RED # Red
    # elif (result1 ==  0) and (result2 ==  0): return ColorSensorType.YELLOW # Yel
    # else: return ColorSensorType.UNKNOWN

class DriverCommander(object):
    def DriverRestart(self) -> bool: return DriverRestart()
    def SolenoidOn(self, *a, **kw) -> bool: return SolenoidOnOff(*a, **kw)
    def HandOpen(self) -> bool: return HandOpen()
    def HandCloses(self) -> bool: return HandClose()
    def TPSupply(self) -> bool: return TPSupply()
    def EColorSensor(self) -> bool: return EColorSensor()
    def ColorSensor(self) -> int: return ColorSensor

    
if __name__ == '__main__':
    pass