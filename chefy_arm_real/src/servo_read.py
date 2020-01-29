import rospy
from std_msgs.msg import UInt16MultiArray

import serv

import config

control = serv.Control(port=config.TTY_PROT)
joints = UInt16MultiArray()


def servo_readers():
    pub = rospy.Publisher('servo_read', UInt16MultiArray, queue_size=10)
    rospy.init_node('servo_reader', anonymous=True)

    rate = rospy.Rate(1)  #
    while not rospy.is_shutdown():
        joints.data = control.read_joint()
        pub.publish(joints)
        rate.sleep()

if __name__ == '__main__':
    servo_readers()
