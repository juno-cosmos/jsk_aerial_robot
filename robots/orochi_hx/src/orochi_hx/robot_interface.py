import rospy
from aerial_robot_base.robot_interface import RobotInterface
from sensor_msgs.msg import JointState


class OrochiHxInterface(RobotInterface):
    # servos in the "joints" group of Servo.yaml (gimbal_pitch is excluded)
    JOINT_PREFIXES = ('male_roll', 'female_roll', 'joint_yaw')
    # virtual joint of the full vectoring mode: joint_roll{i} = male_roll{i} + female_roll{i}
    JOINT_ROLL_PREFIX = 'joint_roll'

    def __init__(self, robot_ns = "orochi_hx", debug_view = False, init_node = False):
        super().__init__(robot_ns, debug_view, init_node)

        # only subscribed by the full vectoring controller
        self.joint_roll_ctrl_pub = rospy.Publisher(self.robot_ns + '/joint_roll_ctrl', JointState, queue_size = 1)

    def jointStateCallback(self, msg):
        joint_state = JointState()

        for n, j in zip(msg.name, msg.position):
            if not n.startswith(self.JOINT_PREFIXES):
                continue
            joint_state.name.append(n)
            joint_state.position.append(j)

        # add the virtual joint roll: joint_roll{i} = male_roll{i} + female_roll{i}
        joints = dict(zip(msg.name, msg.position))
        i = 1
        while 'male_roll{}'.format(i) in joints and 'female_roll{}'.format(i) in joints:
            joint_state.name.append('{}{}'.format(self.JOINT_ROLL_PREFIX, i))
            joint_state.position.append(joints['male_roll{}'.format(i)] + joints['female_roll{}'.format(i)])
            i += 1

        self.joint_state = joint_state

    def setJointAngle(self, target_joint_names, target_joint_angles, thresh = 0.05, timeout = -1):

        if self.flight_state != self.HOVER_STATE and self.flight_state != self.ARM_OFF_STATE:
            rospy.logerr("[Send Joint] the current flight state({}) does not allow joint motion ".format(self.flight_state))
            return False

        if len(target_joint_names) != len(target_joint_angles):
            rospy.logerr("[Send Joint] the size of joint names {} and angles {} are not same".format(len(target_joint_names), len(target_joint_angles)))
            return False

        # check whether the joint exists
        for name in target_joint_names:
            if name not in self.joint_state.name:
                rospy.logerr("set jont angle: cannot find {}".format(name))
                return False

        # joint_roll{i} is handled by the full vectoring controller, other joints by servo_bridge
        full_vectoring = self.joint_roll_ctrl_pub.get_num_connections() > 0
        roll_state = JointState()
        joint_state = JointState()
        for name, angle in zip(target_joint_names, target_joint_angles):
            if name.startswith(self.JOINT_ROLL_PREFIX):
                if not full_vectoring:
                    rospy.logerr("set joint angle: {} is only available in the full vectoring mode, use male_roll and female_roll instead".format(name))
                    return False
                roll_state.name.append(name)
                roll_state.position.append(angle)
            else:
                if full_vectoring and name.startswith(('male_roll', 'female_roll')):
                    rospy.logerr("set joint angle: {} is controlled by the full vectoring controller, use joint_roll instead".format(name))
                    return False
                joint_state.name.append(name)
                joint_state.position.append(angle)

        # send joint angles
        if len(roll_state.name) > 0:
            self.joint_roll_ctrl_pub.publish(roll_state)
        if len(joint_state.name) > 0:
            self.joint_ctrl_pub.publish(joint_state)

        return self.jointConvergenceCheck(timeout, target_joint_names, target_joint_angles, thresh)
