import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist
import math
 
roll = pitch = yaw = 0.0
target = 45
kp=0.5
 
def get_rotation (msg):
    global roll, pitch, yaw, x_robot, y_robot
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    x_robot = msg.pose.pose.position.x
    y_robot = msg.pose.pose.position.y
    #   print yaw
 
rospy.init_node('pothole_avoid')
 
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10)
command =Twist()



while not rospy.is_shutdown():
    # assume we have a pothole
    if yaw!=0:
        snapshot = yaw
        snapshot_x = x_robot
        snapshot_y = y_robot

        left = -0.255
        bottom = 1

        # add cushioning to value


        angleToBottomLeft = math.atan2(left, bottom)

        # angleToBottomLeft = 45 * math.pi / 180
        distance = math.sqrt(left**2 + bottom **2)
        
        # target_rad = angleToBottomLeft + snapshot
        target_rad = angleToBottomLeft + snapshot
        
        # if target_rad>3:
        #     target_rad -= 6
        # if target_rad < -3:
        #     target_rad -= 6
        velocity = Twist()
        # this turn the robot
        while abs(yaw - target_rad)>0.05:
            command.angular.z = kp * (target_rad-yaw)
            pub.publish(command)
            r.sleep()
            print("Left is {} bottom is {}".format(left, bottom))
            print("initial is {} target rad is {} and current is {}".format(snapshot, angleToBottomLeft, yaw))
    
        velocity.angular.z=0
        velocity.linear.x=0
        velocity.linear.y=0
        pub.publish(velocity)
        r.sleep()
        print("Stopped rotating...")

        print("Moving forward")
        while ((x_robot-snapshot_x)**2 + (y_robot-snapshot_y)**2 < distance**2):
            velocity.linear.x = 0.2
            pub.publish(velocity)
            r.sleep()


        velocity.angular.z=0
        velocity.linear.x=0
        velocity.linear.y=0
        pub.publish(velocity)
        r.sleep()
        print("Stopped")
        target_rad = snapshot - angleToBottomLeft
        while abs(yaw - target_rad)>0.05:
            command.angular.z = kp * (target_rad-yaw)
            pub.publish(command)
            r.sleep()
            print("Left is {} bottom is {}".format(left, bottom))
            print("initial is {} target rad is {} and current is {}".format(snapshot, angleToBottomLeft, yaw))
        
        snapshot_x2 = x_robot
        snapshot_y2 = y_robot


        print("Moving forward")
        while ((x_robot-snapshot_x2)**2 + (y_robot-snapshot_y2)**2 < distance**2):
            velocity.linear.x = 0.2
            pub.publish(velocity)
            r.sleep()

        velocity.angular.z=0
        velocity.linear.x=0
        velocity.linear.y=0
        pub.publish(velocity)
        r.sleep()
        print("finish")

        break
        print("This should never print")

        velocity = Twist()
        velocity.linear.x = 0.5
        pub.publish(velocity)
        r.sleep()




# plan
"""
Classifier sees pothole
then gets boudning box from localizer
robot stops after stable prediction

convert to coords relative to robot
find bottom left corner
(Here we insert hardcoded location of pothole for testing)
find angle to bottom left corner from robot - tan-1(x/y)
get current orientation -- save snapshot

add angle to orientation (turn)
use the rotate to target code
now we are facing the right direction
drive forward for sqrt(x^2+y^2)

then do the opposite -- rotate back
rotate back to: snapshot - tan-1(x/y)
drive forward sqrt(x^2+y^2)

turn back to snapshot angle
we have avoided the pothole -- continue

"""