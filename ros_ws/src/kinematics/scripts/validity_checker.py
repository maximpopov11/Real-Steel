import rospy
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState
from joint_names import left_arm_joint_names, right_arm_joint_names


class StateValidityChecker():
    def __init__(self):
        # prepare service for collision check
        self.service = rospy.ServiceProxy('/check_state_validity', GetStateValidity)
        # wait for service to become available
        self.service.wait_for_service()
        rospy.loginfo('service is avaiable')
        # prepare msg to interface with moveit
        self.robot_state = RobotState()
        self.robot_state.joint_state.name = left_arm_joint_names + right_arm_joint_names
        self.robot_state.joint_state.position = [0.0 for _ in range(len(self.robot_state.joint_state.name))]
        

    def checkCollision(self):
        '''
        Check if robot is in collision
        '''
        if self.getStateValidity().valid:
            rospy.loginfo('robot not in collision, all ok!')
        else:
            rospy.logwarn('robot in collision')


    def setJointStates(self, joint_angles):
        '''
        Update robot state, left arms first and then right arms as ordered in kinematics/joint_names.py
        '''
        self.robot_state.joint_state.position = joint_angles


    def getStateValidity(self, group_name, constraints=None) -> bool:
        '''
        Given a RobotState and a group name and an optional Constraints
        return the validity of the State
        '''
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self.robot_state
        gsvr.group_name = group_name
        if constraints != None:
            gsvr.constraints = constraints
        return self.service.call(gsvr).valid