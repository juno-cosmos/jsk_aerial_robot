from aerial_robot_base.robot_interface import RobotInterface
from sensor_msgs.msg import JointState


class OrochiHxInterface(RobotInterface):
    # servos in the "joints" group of Servo.yaml (gimbal_pitch is excluded)
    JOINT_PREFIXES = ('male_roll', 'female_roll', 'joint_yaw')

    def __init__(self, robot_ns = "orochi_hx", debug_view = False, init_node = False):
        super().__init__(robot_ns, debug_view, init_node)

    def jointStateCallback(self, msg):
        joint_state = JointState()

        for n, j in zip(msg.name, msg.position):
            if not n.startswith(self.JOINT_PREFIXES):
                continue
            joint_state.name.append(n)
            joint_state.position.append(j)
        self.joint_state = joint_state
