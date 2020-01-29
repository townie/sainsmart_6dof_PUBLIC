import rospy
from std_msgs.msg import UInt16MultiArray

import serv
import config

control = serv.Control(port=config.TTY_PROT)
joints = UInt16MultiArray()


def callback_servo(data):
    dic = map(int, data.data)
    control.write_joint(dic)


def listener():
    joints.data = [0] * 6
    rospy.init_node('servo_listener', anonymous=True)
    rospy.Subscriber('servo_write', UInt16MultiArray, callback_servo)

    rospy.spin()


if __name__ == '__main__':
    listener()
