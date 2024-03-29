#!/usr/bin/env python

import random
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

DEG_TO_RAD = 0.0174533  # PI/180


class ArmJointSync:
    JointMap = {'base_joint': {"idx": 0, 'bias': 90, 'range': [-90, 90], 'init': 0},
                'shoulder_joint': {"idx": 1, 'bias': 95, 'range': [-85, 85], 'init': 0.2834},
                'elbow_joint': {"idx": 2, 'bias': 90, 'range': [-60, 90], 'init': -0.9736},
                'wrist_flex_joint': {"idx": 3, 'bias': 90, 'range': [-90, 0], 'init': -1.4648},
                'wrist_rot_joint': {"idx": 4, 'bias': 90, 'range': [-90, 0], 'init': 0},
                'finger_joint1': {"idx": 5, 'bias': 90, 'range': [-90, 0], 'init': 0},
                'finger_joint2': {"idx": 5, 'bias': 90, 'range': [0, 90], 'init': 0}
                }

    def __init__(self):
        self.joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_flex_joint', 'wrist_rot_joint',
                            ]
        # subscriber for setting joints position
        self.joint_state = JointState()
        self.states_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

        self.pose_sub = rospy.Subscriber("/chefy_arm/joint_states", JointState, callback=self.callback_joint, queue_size=1)
        for name in self.joint_names[:5]:
            v = self.JointMap[name]
            v['range'][0] *= DEG_TO_RAD
            v['range'][1] *= DEG_TO_RAD
            print('\t', name, v['range'])

    def callback_joint(self, data):
        names, positions = [], []
        for name, pos in zip(data.name, data.position):
            if name in self.JointMap:
                names.append(name)
                positions.append(pos)
        self.publish(names, positions)

    def random_joints(self):
        joints = []
        for name in self.joint_names:
            _range = self.JointMap[name]["range"]
            r = random.random() * (_range[1] - _range[0]) + _range[0]
            joints.append(r)
        return joints

    def publish(self, names, joints):
        self.joint_state.name = names
        self.joint_state.position = joints
        self.states_pub.publish(self.joint_state)


if __name__ == '__main__':
    try:
        rospy.init_node('arm_joint_demo', anonymous=True)
        gp = ArmJointSync()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
