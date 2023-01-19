import sys
python3 = True if sys.version[0] == '3' else False

import copy
import math
import rospy
import geometry_msgs.msg
from khi_robot_msgs.srv import *

service = '/khi_robot_command_service'

def cmdhandler_client(type_arg , cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service, KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)

def get_driver_state():
    ret = cmdhandler_client('driver' , 'get_status')
    return ret

def ASCommand(TYPE: str='as', COMMAND: str):
    ret = cmdhandler_client(TYPE, COMMAND)
    if python3:
        print("Result:")
        print(f"\t\tDriver: {ret.driver_ret}")
        print(f"\t\t    AS: {ret.as_ret}")
        print(f"\t\t   CMD: {ret.cmd_ret}")

    else:
        print "Result:"
        print "\t\tDriver: %s" % ret.driver_ret
        print "\t\t    AS: %s" % ret.as_ret
        print "\t\t   CMD: %s" % ret.cmd_ret

def DriverCommand(TYPE: str='driver', COMMAND: str):
    ret = cmdhandler_client(TYPE, COMMAND)
    if python3:
        print("Result:")
        print(f"\t\tDriver: {ret.driver_ret}")
        print(f"\t\t    AS: {ret.as_ret}")
        print(f"\t\t   CMD: {ret.cmd_ret}")

    else:
        print "Result:"
        print "\t\tDriver: %s" % ret.driver_ret
        print "\t\t    AS: %s" % ret.as_ret
        print "\t\t   CMD: %s" % ret.cmd_ret

if __name__ == '__main__':
    rospy.init_node("khi_robot_control_node")
    for _ in range(3):
        DriverCommand(COMMAND="restart")
        rospy.sleep(1)
        ASCommand(COMMAND="zpow on")
        rospy.sleep(1)
        ASCommand(COMMAND="do open 1")
        rospy.sleep(1)  
        ASCommand(COMMAND="do close 1")
        rospy.sleep(1)
        ASCommand(COMMAND="ABORT")
        rospy.sleep(1)
        ASCommand(COMMAND="ERESET")