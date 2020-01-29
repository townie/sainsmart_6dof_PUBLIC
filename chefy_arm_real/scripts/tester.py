import rospy
from std_msgs.msg import UInt16MultiArray
from time import sleep
joints = UInt16MultiArray()


pub = rospy.Publisher('servo_write', UInt16MultiArray, queue_size=10)
rospy.init_node('servo_test', anonymous=True)
rate = rospy.Rate(1000)  # 10hz

joints.data = [90, 90, 90, 90, 90, 90]
pub.publish(joints)
rate.sleep()

for i in range(30, 150):
    joints.data = [i] * 6
    pub.publish(joints)
    rate.sleep()
    sleep(1)

joints.data = [100, 90, 90, 90, 90, 90]
pub.publish(joints)
