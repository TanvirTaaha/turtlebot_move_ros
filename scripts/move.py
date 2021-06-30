#!/usr/bin/python3
import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


rospy.init_node('move_bot')
publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(35)  # hz

ANGULAR_SPEED = 0.15  # optimum angular speed
LINEAR_SPEED = 0.5  # optimum linear speed
# minimum angle difference between orientation of the bot and destination
ANGLE_THRESHOLD = 0.1
# distance between the bot and the destination which the bot will try to achive
POSITION_THRESHOLD = 0.01

x = 0.0  # current x coordinate of the bot
y = 0.0  # current y coordinate of the bot
theta = 0.0  # current angle along z axis of the bot
goal_x = 0.0
goal_y = 0.0


def callback(odom: Odometry) -> None:
    # Update the position data
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

        s = (delta_theta / 3)

        if delta_theta >= ANGLE_THRESHOLD:
            # if the destination is in left go left
            speed.linear.x = 0.0
            speed.angular.z = (s if abs(s) > ANGULAR_SPEED else ANGULAR_SPEED)
        elif delta_theta <= - ANGLE_THRESHOLD:
            # if the destination is in right go right
            speed.linear.x = 0.0
            speed.angular.z = (s if abs(s) > ANGULAR_SPEED else -ANGULAR_SPEED)
        else:
            # destination is straight ahead go straight
            speed.linear.x = LINEAR_SPEED
            speed.angular.z = 0.0

        # print verbose data
        print("Linear vel:{}, Angular_vel:{}, delta_theta:{}".format(
            speed.linear.x, speed.angular.z, delta_theta))

        publisher.publish(speed)

        if at_the_goal(delta_x, delta_y):
            # when destination is reached, stop the bot
            stop_bot()
            break
        rate.sleep()  # run the loop 30 times a sec
