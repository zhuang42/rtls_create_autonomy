#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg
import nav_msgs.msg
import numpy as np

# Global variable
current_pos = geometry_msgs.msg.Pose2D()
pose2d_pub = None
move_pub = None


def odom_callback(msg):
    """
    msg: nav_msgs
    """

    current_pos.x = msg.pose.pose.position.x
    current_pos.y = msg.pose.pose.position.y

    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

    current_pos.theta = yaw

    # rospy.loginfo("{} {} {}".format(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))


def move_forward(meter, max_vel=0.2):

    initx, inity = current_pos.x, current_pos.y

    goalx = initx + meter * math.cos(current_pos.theta)
    goaly = inity + meter * math.sin(current_pos.theta)

    #rospy.loginfo("{} {}".format(goalx, goaly))

    rate = rospy.Rate(30.0)

    rospy.loginfo("move forward")
    # https://en.wikipedia.org/wiki/Proportional_control
    # Simple P controler

    while math.sqrt((current_pos.x - initx) ** 2 + (current_pos.y - inity) ** 2) < meter:

        move = geometry_msgs.msg.Twist()

        et = math.sqrt((goalx - current_pos.x) ** 2 + (goaly - current_pos.y) ** 2)
        P0 = 0.01
        KP = 0.5

        Pout = min(max_vel - P0, KP * et) + P0
        move.linear.x = Pout
        move.angular.z = 0
        move_pub.publish(move)
        rate.sleep()

    move = geometry_msgs.msg.Twist()
    move_pub.publish(move)


def move_backward(meter, max_vel=0.2):

    initx, inity = current_pos.x, current_pos.y

    goalx = initx + -meter * math.cos(current_pos.theta)
    goaly = inity + -meter * math.sin(current_pos.theta)

    #rospy.loginfo("{} {}".format(goalx, goaly))

    rate = rospy.Rate(30.0)

    rospy.loginfo("move forward")
    # https://en.wikipedia.org/wiki/Proportional_control
    # Simple P controler

    while math.sqrt((current_pos.x - initx) ** 2 + (current_pos.y - inity) ** 2) < meter:

        move = geometry_msgs.msg.Twist()

        et = math.sqrt((goalx - current_pos.x) ** 2 + (goaly - current_pos.y) ** 2)
        P0 = 0.02
        KP = 0.5

        Pout = min(max_vel - P0, KP * et) + P0
        move.linear.x = -Pout
        move.angular.z = 0
        move_pub.publish(move)
        rate.sleep()

    move = geometry_msgs.msg.Twist()
    move_pub.publish(move)


def get_angle_diff(a1, a2):
    diff = a1 - a2
    return math.atan2(math.sin(diff), math.cos(diff))


def turn_cw(degree, max_radv=0.2):

    rad = degree * math.pi / 180.0

    init_theta = current_pos.theta

    rate = rospy.Rate(30.0)
    rospy.loginfo("Turning clockwise")
    while True:
        rad_sofar = get_angle_diff(init_theta, current_pos.theta)
        if rad_sofar < -1e-02:
            break

        if rad_sofar >= rad:
            break

        move = geometry_msgs.msg.Twist()

        et = rad - rad_sofar

        P0 = 0.1
        KP = 0.5
        Pout = min(max_radv - P0, KP * et) + P0

        move.linear.x = 0
        move.angular.z = - Pout
        move_pub.publish(move)
        rate.sleep()

    move = geometry_msgs.msg.Twist()
    move_pub.publish(move)
    rate.sleep()
    move_pub.publish(move)


def turn_ccw(degree, max_radv=0.2):
    rad = degree * math.pi / 180.0

    init_theta = current_pos.theta

    rate = rospy.Rate(30.0)
    rospy.loginfo("Turning coutner clockwise")
    while True:
        rad_sofar = get_angle_diff(current_pos.theta, init_theta)

        if rad_sofar < -1e-02:
            break
        if rad_sofar >= rad:
            break

        move = geometry_msgs.msg.Twist()

        et = rad - rad_sofar

        P0 = 0.1
        KP = 0.5
        Pout = min(max_radv - P0, KP * et) + P0

        move.linear.x = 0
        move.angular.z = Pout
        move_pub.publish(move)
        rate.sleep()

    move = geometry_msgs.msg.Twist()
    move_pub.publish(move)
    rate.sleep()
    move_pub.publish(move)


def main():
    global pose2d_pub
    global move_pub
    rospy.init_node('move_pub', log_level=rospy.INFO)
    rospy.loginfo('start')
    odom_sub = rospy.Subscriber("odom", nav_msgs.msg.Odometry, odom_callback)
    move_pub = rospy.Publisher("cmd_vel", geometry_msgs.msg.Twist)
    rospy.sleep(1)  # wait odom sub to update current odometry data

    command_dispatcher = {
        "f": move_forward,
        "b": move_backward,
        "l": turn_cw,
        "r": turn_ccw,
    }
    while True:
        print("Ready to take input: ")

        raw_input_list = raw_input().split()

        if len(raw_input_list) != 2:
            rospy.loginfo("Invalid command format {}".format(raw_input_list))
            continue

        command, arg = raw_input_list
        arg = arg.strip()
        command = command.strip()
        if command not in command_dispatcher.keys():
            rospy.loginfo("Invalid command {}".format(command))
            continue

        rospy.loginfo("Excuting command {} {}".format(command, arg))
        command_dispatcher[command](float(arg))

    rospy.spin()


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        exit()
