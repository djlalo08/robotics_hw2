#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from tf import transformations







#------------------------------------------------- callbacks -------------------------------------------------------
#returns distance from obstacle
def scan_callback(msg):
    global g_range_ahead
    g_range_ahead = min(msg.ranges)
    return g_range_ahead
'''
def odom_callback(msg):
    global botPos = msg.pose.pose.position
    return botPos

'''
def odom_callback(msg):
    global botPos_, yaw_

    # position
    botPos = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

    
#-------------------------------------------------calcualtes bot distance to line----------------------------------
def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initPos, gPosition
    p1 = initPos
    p2 = gPosition
    
    #equation for distance
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq

    return distance







#our position
botPos = Point()
initPos = Point()
initPos.x = 0
initPos.y = 0
initPos.z=0
#goal position 
gPosition = Point()
gPosition.x = 10
gPosition.y = 0 
gPosition.z = 0
#for oritentation? 
yaw = 0
g_range_ahead = 0

#what state we are in
state = 0
#0:---go to point
#1:----wall follow



#point that we will need---if impossible 
circumnavigate_starting_point_ = Point()

count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0

def main():
    global state, count_loop_, count_state_time_, g_range_ahead 
    #instantiating some stuff 
    g_range_ahead = 1 # anything to start
    scanner = rospy.Subscriber('scan', LaserScan, scan_callback)
    odom = rospy.Subscriber('/odom', Odometry, odom_callback )
    cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('bug2')


    print(botPos)
    print(yaw)

    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        distance_position_to_line = distance_to_line(botPos)

        if state == 0:# we go to the point
            twist = Twist()
            twist.linear.x = 1
            cmd_vel.publish(twist)
            print(botPos)
            print(yaw)
            print("range ahead gives: " + str( g_range_ahead))
            if g_range_ahead > 0.2 and g_range_ahead < 1:
                state = 1

        elif state == 1:# we do the wall crawling
            twist = Twist()
            twist.angular.z = 1
            cmd_vel.publish(twist)
            print(botPos)
            print("yaw is: " + str(yaw))

            if g_range_ahead > 1:
                state = 0
        
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(botPos), botPos.x, botPos.y)
        rate.sleep()


if __name__ == "__main__":
    main()