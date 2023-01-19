import rospy
import geometry_msgs.msg
from khi_robot_msgs.srv import *
from moveit_commander import MoveGroupCommander

service = '/khi_robot_command_service'
def cmdhandler_client(type_arg , cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service, KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)

class ArmHandPlay(MoveGroupCommander):
    def __init__(self, speed=.5, sleep=0.5):
        super(ArmHandPlay, self).__init__(name="manipulator")
        assert 0 <= speed <= 1, 'min 0 ~ max 1'
        self.speed = speed
        self.sleep = sleep
        # cmdhandler_client('driver', 'restart')
        # print("RESTART")
        # cmdhandler_client('as', 'ZPOW ON')
        # print("ZPOW ON")
        # cmdhandler_client('as', 'DO HOME 1')
        # print("DO HOME 1")

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
    armHand = ArmHandPlay()
    start_ipython(user_ns=globals(), argv=[])