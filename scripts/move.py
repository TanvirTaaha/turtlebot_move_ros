#!/usr/bin/python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


rospy.init_node('move_bot')
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(30)  # hz

ANGULAR_SPEED = 0.15
LINEAR_SPEED = 0.5
ANGLE_THRESHOLD = 0.1
POSITION_THRESHOLD = 0.001

x = 0.0
y = 0.0
theta = 0.0
goal_x = 0.0
goal_y = 0.0


def callback(odom: Odometry) -> None:
    global x
    global y
    global theta
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    theta = euler_from_quaternion([odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                                  odom.pose.pose.orientation.z, odom.pose.pose.orientation.w])[2]


def at_the_goal(delx, dely) -> bool:
    print('delx: {}, dely: {}'.format(delx, dely))
    return math.sqrt(delx**2 + dely**2) < POSITION_THRESHOLD


def stop_bot() -> None:
    stop_msg = Twist()
    stop_msg.linear.x = 0
    stop_msg.linear.y = 0
    stop_msg.linear.z = 0
    stop_msg.angular.x = 0
    stop_msg.angular.y = 0
    stop_msg.angular.z = 0

    publisher.publish(stop_msg)


subscriber = rospy.Subscriber('/odom', Odometry, callback)


while not rospy.is_shutdown():
    (goal_x, goal_y) = (float(num) for num in input(
        "Enter destination from current co-ordinate(x, y):").split(","))
    print('Going to ({}, {})...'.format(goal_x, goal_y))

    speed = Twist()
    while True:
        delta_x = goal_x - x
        delta_y = goal_y - y
        delta_theta = math.atan2(delta_y, delta_x) - theta

        if delta_theta > ANGLE_THRESHOLD:
            speed.linear.x = 0.0
            speed.angular.z = ANGULAR_SPEED
        elif delta_theta < - ANGLE_THRESHOLD:
            speed.linear.x = 0.0
            speed.angular.z = - ANGULAR_SPEED
        else:
            speed.linear.x = LINEAR_SPEED
            speed.angular.z = 0.0

        print("Linear vel:{}, Angular_vel:{}".format(
            speed.linear.x, speed.angular.z))

        publisher.publish(speed)

        if at_the_goal(delta_x, delta_y):
            stop_bot()
            break
        rate.sleep()
