#!/usr/bin/python3
import time
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rospy.init_node('move_bot')
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
subscriber = rospy.Subscriber('/odom', Odometry)
rate = rospy.Rate(0.1)  # hz

stopped_position = Odometry()


def stop_bot() -> None:
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.linear.y = 0
    stop_msg.linear.z = 0
    stop_msg.angular.x = 0
    stop_msg.angular.y = 0
    stop_msg.angular.z = 0

    publisher.publish(stop_msg)


def forward_bot(distance: float) -> None:
    start_time = time.time()  # seconds

    forward_msg = Twist()
    forward_msg.angular.x = 0
    forward_msg.angular.y = 0
    forward_msg.angular.z = 0
    forward_msg.linear.x = distance/2
    forward_msg.linear.y = 0
    forward_msg.linear.z = 0
    while not rospy.is_shutdown() and (time.time() - start_time) <= 2:
        publisher.publish(forward_msg)
    stop_bot()


def callback(odom: Odometry) -> None:
    stopped_position = odom


while not rospy.is_shutdown():
    (xx, yy) = (float(num) for num in input(
        "Enter destination from current co-ordinate(x, y):").split(","))
    print('Going to ({}, {})...'.format(xx, yy))
    forward_bot(xx)
    rate.sleep()


# def rotate_z(delta_radian: float, dest_radian: float) -> None:
#     start_time = time.time()  # seconds
#     rotate_msg = Twist()
#     rotate_msg.linear.x = 0
#     rotate_msg.linear.y = 0
#     rotate_msg.angular.z = delta_radian / 2

# def rotate_bot(rad_angle: float) -> None:
#     start_time = time.time()  # seconds

#     rotate_msg = Twist()
#     rotate_msg.linear.x = 0
#     rotate_msg.linear.y = 0
#     rotate_msg.angular.z = rad_angle / 2
#     while not rospy.is_shutdown() and (time.time() - start_time) <= 2:
#         publisher.publish(rotate_msg)
#     stop_bot()
