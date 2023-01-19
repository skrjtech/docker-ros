import sys
python3 = True if sys.version[0] == '3' else False

PKG = 'khi_robot_test'
import roslib; roslib.load_manifest(PKG)

import copy
import math
import unittest
import rospy
import geometry_msgs.msg
from khi_robot_msgs.srv import *

if rospy.has_param('/test_group_name'):
    gn = '/' + rospy.get_param('/test_group_name')
else:
    gn = ''
service = gn + '/khi_robot_command_service'

def cmdhandler_client(type_arg , cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service,KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return resp1
    except rospy.ServiceException, e:
        rospy.loginfo('Service call failed: %s', e)

def get_driver_state():
    ret = cmdhandler_client('driver' , 'get_status')
    return ret

class TestKhiRobotControl(unittest.TestCase):
    def test_program(self):
        # ROS -> HOLDED
        cmdhandler_client('driver', 'hold')
        rospy.sleep(3)
        ret = get_driver_state()
        self.assertEqual('HOLDED', ret.cmd_ret)

        # ROS -> INACTIVE
        cmdhandler_client('as', 'hold 1:')
        rospy.sleep(3)
        ret = cmdhandler_client('as', 'type switch(cs)')
        self.assertEqual(0, int(ret.cmd_ret))
        ret = get_driver_state()
        self.assertEqual('INACTIVE', ret.cmd_ret)

        # execute robot program by AS
        cmdhandler_client('as', 'zpow on')
        cmdhandler_client('as', 'execute 1: rb_rtc1,-1')
        ret = cmdhandler_client('as', 'type switch(cs)')
        self.assertEqual(-1, int(ret.cmd_ret))
        ret = get_driver_state()
        self.assertEqual('INACTIVE', ret.cmd_ret)

        # ROS -> ACTIVE
        cmdhandler_client('driver', 'restart')
        rospy.sleep(3)
        ret = get_driver_state()
        self.assertEqual('ACTIVE', ret.cmd_ret)

    def test_signal(self):
        size  = 512
        offset = [0, 2000, 2000]

        cmdhandler_client('as', 'ZINSIG ON')
        cmdhandler_client('as', 'RESET')

        for i in range(size):
            for j in range(3):
                rospy.loginfo('SIGNAL %d', i+1+offset[j])
                # positive
                set_cmd = 'set_signal ' + str(i+1+offset[j])
                cmdhandler_client('driver', set_cmd)
                get_cmd = 'get_signal ' + str(i+1+offset[j])
                ret = cmdhandler_client('driver', get_cmd)
                self.assertEqual('-1', ret.cmd_ret)

                # negative
                set_cmd = 'set_signal -' + str(i+1+offset[j])
                cmdhandler_client('driver', set_cmd)
                get_cmd = 'get_signal ' + str(i+1+offset[j])
                ret = cmdhandler_client('driver', get_cmd)
                self.assertEqual('0', ret.cmd_ret)

    def test_variable(self):
        cmdhandler_client('as', 'test_val = 1')
        ret = cmdhandler_client('as', 'type test_val')
        self.assertEqual(1, int(ret.cmd_ret))

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

if __name__ == '__main__':
    import rostest
    rospy.init_node("test_khi_robot_control_node")
    cmdhandler_client('driver', 'restart')
    rostest.rosrun(PKG, 'test_khi_robot_control', TestKhiRobotControl)