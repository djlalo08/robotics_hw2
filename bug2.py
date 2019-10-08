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
    global botPos, yaw

    # position
    botPos = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
    
#-------------------------------------------------calculates bot distance to line----------------------------------
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
initPos.z = 0

#goal position 
gPosition = Point()
gPosition.x = 10
gPosition.y = 0 
gPosition.z = 0

#robot angle 
yaw = 0

#laser-reading
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
    global state, count_loop_, count_state_time_, g_range_ahead, botPos 
    #instantiating some stuff 
    g_range_ahead = 1 # anything to start
    rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback )
    rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('bug2')

    print(botPos)
    print(yaw)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():

        distance_position_to_line = distance_to_line(botPos)
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        
        if state == 0:# we go to the point
            twist.linear.x = 1
            cmd_vel.publish(twist)
            print(botPos)
            print(yaw)
            print("range ahead gives: " + str(g_range_ahead))
            if g_range_ahead > 0.2 and g_range_ahead < .5:
                state = 1

        elif state == 1:# we do the wall crawling
            #turn to the right until object is no longer spotted
            if(g_range_ahead <1):
                while(g_range_ahead < 1):
                    print("range ahead: " + str(g_range_ahead))
                    twist.angular.z = 1
                            
            #move forward until closer
            for i in range(0,3):
                twist.linear.x = 1
                print(botPos)
                print("range ahead: " + str(g_range_ahead))
                print("yaw is: " + str(yaw))

            #to turn right to find a wall if at a corner
            while(g_range_ahead > 1):
                print("range ahead: " + str(g_range_ahead))
                twist.angular.z = -1

            #puts us back to following mline 
            if distance_to_line(botPos) > -.2 or distance_to_line(botPos) < .2:
                state = 0
#                state = 2
                
#        elif state == 2:#turn back to face x-axis
#            twist.angular.z = -yaw
            
        
        cmd_vel.publish(twist)
        
        count_loop_ = count_loop_ + 1
        if count_loop_ == 20:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0

        rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(botPos), botPos.x, botPos.y)
        rate.sleep()


if __name__ == "__main__":
    main()
