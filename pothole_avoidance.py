import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2

x_robot = 0.0
y_robot = 0.0
theta_robot = 0.0
robot_width = 0.5   #TODO: set value

def newOdom(msg):
    global x_robot
    global y_robot
    global theta_robot

    x_robot = msg.pose.pose.position.x
    y_robot = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta_robot) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node("avoidance_controller")

odom_sub = rospy.Subscriber("/odometry/filtered", Odometry, newOdom)
classifer_sub = rospy.Subscriber()
box_sub = rospy.Subscriber()
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

velocity = Twist()

r = rospy.Rate(4)

avoidance_point = Point()

velocity.linear.x = 0.5

while not rospy.is_shutdown():
     if(classifer_sub):
        if(x_robot > box_sub.x1 and x_robot < box_sub.x2):  #TODO: adjust for robot width  
            avoidance_point.x = box_sub.x2 + robot_width
            avoidance_point.y = box_sub.y2 + robot_width

            while ((x_robot + robot_width) < box_sub.x2):
                d_x = avoidance_point.x - x_robot                    
                d_y = avoidance_point.y - y_robot
                goal_angle = atan2(d_x, d_y)

                if abs(goal_angle - theta_robot) > 0.1:
                    velocity.angular.z = 0.3
                else:
                    velocity.angular.z = 0.0

                vel_pub.publish(velocity)
                r.sleep()
            while abs(theta_robot) > 0.1:
                velocity.angular.z = -0.3

                r.sleep()
                vel_pub.publish(velocity)