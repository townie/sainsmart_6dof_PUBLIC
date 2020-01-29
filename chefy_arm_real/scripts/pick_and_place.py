import sys
import signal
import copy

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler

from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32


class PickAndPlace(object):
    DEG_TO_RAD = 0.01745329251994329577 

    joint_map = {'base_joint': {"i": 0, 'bias': 90},
                 'shoulder_joint': {"i": 1, 'bias': 95},
                 'elbow_joint': {"i": 2, 'bias': 90},
                 'wrist_flex_joint': {"i": 3, 'bias': 90},
                 'wrist_rot_joint': {"i": 4, 'bias': 90},
                 'gripper_joint': {"i": 5, 'bias': 90}
                 }

    close_gripper_conf = {"finger_joint1": 0, "finger_joint2": 0}
    open_gripper_conf = {"finger_joint1": 0.015, "finger_joint2": -0.015}

    def __init__(self):
        self.joint_status = UInt16MultiArray()
        self.gripper_status = Float32()

        self.is_exit = False
        self.run_real = False

        # init moveit commander
        moveit_commander.roscpp_initialize(sys.argv)
        # specify move group
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        # init publisher
        self.arm_pub = rospy.Publisher('servo_write', UInt16MultiArray, queue_size=10)
        self.gripper_pub = rospy.Publisher('gripper_write', Float32, queue_size=10)
        # init ros node
        rospy.init_node('real_servo_driver', anonymous=True)
        # init robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # set ros publisher rate, 10hz = 10 seconds for a circle
        self.rate = rospy.Rate(50)

        rospy.sleep(2)
        # move grasper to init position
        self.init_position()
        self.init_gripper()

    def init_position(self):
        # init position of simulator
        up_right_joint_values = {"base_joint": 0, "shoulder_joint": 0, "elbow_joint": 0,
                                 "wrist_flex_joint": 0, "wrist_rot_joint": 0}
        plan_msg = self.arm_group.plan(joints=up_right_joint_values)
        self.arm_group.execute(plan_msg=plan_msg, wait=False)
        self.servo_messanger(plan_msg)
        rospy.sleep(5)

    def init_gripper(self):
        self.close_gripper()
        rospy.sleep(5)

    def open_gripper(self):
        plan_msg = self.gripper_group.plan(joints=self.open_gripper_conf)
        self.gripper_group.execute(plan_msg=plan_msg, wait=False)
        self.gripper_messanger(plan_msg)

    def close_gripper(self):
        plan_msg = self.gripper_group.plan(joints=self.close_gripper_conf)
        self.gripper_group.execute(plan_msg=plan_msg, wait=False)
        self.gripper_messanger(plan_msg)

    def pickup(self, pos):
        # open
        self.open_gripper()
        rospy.sleep(5)

        # planm move and do
        self.arm_group.set_pose_target(pos)
        plan_msg = self.arm_group.plan()
        self.arm_group.execute(plan_msg=plan_msg, wait=False)
        self.servo_messanger(plan_msg)
        rospy.sleep(10)

        # close gripper
        self.close_gripper()
        rospy.sleep(5)

    def place(self, pos):
        # move to pos
        self.arm_group.set_pose_target(pos)
        plan_msg = self.arm_group.plan()
        self.arm_group.execute(plan_msg=plan_msg, wait=False)
        self.servo_messanger(plan_msg)
        rospy.sleep(10)
        # open gripper
        self.open_gripper()
        rospy.sleep(5)

    def create_cube(self, name, pos):
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()

        p.pose.position.x = pos[0]
        p.pose.position.y = pos[1]
        p.pose.position.z = pos[2]

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = geometry_msgs.msg.Quaternion(*q)

        return p.pose

    def servo_messanger(self, plan_msg):
        if not self.run_real:
            return
        joint_names = plan_msg.joint_trajectory.joint_names
        points = plan_msg.joint_trajectory.points
        for i in xrange(len(points)):
            joint_angles = [0] * 6
            # transform angles
            for idx, angle in enumerate(points[i].positions):
                joint_param = self.joint_map[joint_names[idx]]
                joint_angles[joint_param["i"]] = int(angle / self.DEG_TO_RAD) + joint_param['bias']
            # ignore gripper_joint
            joint_angles[5] = 90
            # set joint status
            self.joint_status.data = joint_angles
            # publish joint
            self.arm_pub.publish(self.joint_status)
            # wait according to hz
            # self.rate.sleep()
            rospy.sleep(0.1)
            if self.is_exit:
                sys.exit(0)

    def gripper_messanger(self, plan_msg):
        if not self.run_real:
            return
        points = plan_msg.joint_trajectory.points
        for i in xrange(len(points)):
            gripper_pos = abs(points[i].positions[0])
            # publish joint
            self.gripper_status.data = gripper_pos
            self.gripper_pub.publish(self.gripper_status)

            rospy.sleep(0.1)
            if self.is_exit:
                sys.exit(0)

    def create_pose(self, _pos, _orient=[0, 0, 0], name="pose"):
        ppose = self.arm_group.get_random_pose()
        pose = geometry_msgs.msg.Pose()
        pose.position.x = _pos[0]
        pose.position.y = _pos[1]
        pose.position.z = _pos[2]

        pose.orientation.x = _orient[0]
        pose.orientation.y = _orient[1]
        pose.orientation.z = _orient[2]
        pose.orientation.w = _orient[3]
        ppose.pose = pose

        return pose


def pick_and_place():
    handler = PickAndPlace()
    handler.run_real = True
    source_pose = handler.create_pose([-0.01, -0.03, 0.22],
                                      [0.45, -0.48, 0.40, 0.62],
                                      name="init_pose")
    target_pose = handler.create_pose([0.12, 0.19, 0.10],
                                      [-0.52, 0.31, -0.05, 0.80],
                                      name="target_pose")
    handler.pickup(source_pose)
    handler.place(target_pose)

    handler.init_position()
    handler.init_gripper()

if __name__ == '__main__':
    try:
        pick_and_place()
    except rospy.ROSInterruptException:
        pass
