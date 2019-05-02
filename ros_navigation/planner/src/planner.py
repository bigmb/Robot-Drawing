#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt
from turtlesim.msg import Pose
import numpy as np

# Transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def yaw_from_quaternion(quat):
    """
    Extract yaw from a geometry_msgs.msg.Quaternion.

    :param geometry_msgs.msg.Quaternion quat: the quaternion.
    :returns: yaw angle from the quaternion.
    :rtype: :py:obj:`float`
    """
    return euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]

def angle_wrap(ang):
    """
    Return the angle normalized between [-pi, pi].

    Works with numbers and numpy arrays.

    :param ang: the input angle/s.
    :type ang: float, numpy.ndarray
    :returns: angle normalized between [-pi, pi].
    :rtype: float, numpy.ndarray
    """
    ang = ang % (2 * np.pi)
    if (isinstance(ang, int) or isinstance(ang, float)) and (ang > np.pi):
        ang -= 2 * np.pi
    elif isinstance(ang, np.ndarray):
        ang[ang > np.pi] -= 2 * np.pi
    return ang

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/mobile_base_controller/odom',
                                                Odometry, self.update_pose)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal',
                                                PoseStamped, self.goal_sub)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def goal_sub(self, data):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        #goal_pose.x = input("Set your x goal: ")
        #goal_pose.y = input("Set your y goal: ")

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = input("Set your tolerance: ")
        goal_pose.x = data.pose.position.x
        goal_pose.y = data.pose.position.y
	goal_pose.theta = atan2(goal_pose.y-self.pose.y,goal_pose.x-self.pose.x)
        self.move2goal(goal_pose)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
    
        self.pose.x = round(data.pose.pose.position.x, 4)
        self.pose.y = round(data.pose.pose.position.y, 4)
        
	self.pose.theta = angle_wrap(yaw_from_quaternion(data.pose.pose.orientation))

        #print("x "+str(self.pose.x)+" y "+str(self.pose.y)+" theta"+str(self.pose.theta));

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.4):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        #return max(min(constant * self.euclidean_distance(goal_pose),2),0.5)
        return 0.3

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (angle_wrap(self.steering_angle(goal_pose) - self.pose.theta))

    def move2goal(self,goal_pose):

	print("goal received")

        vel_msg = Twist()

	distance_tolerance = 0.2
        goal_pose.theta = atan2(goal_pose.y-self.pose.y,goal_pose.x-self.pose.x)
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            ang_vel = self.angular_vel(goal_pose)
            lin_vel = self.linear_vel(goal_pose)
            print("ang: "+str(ang_vel)+" lin "+str(lin_vel))
            print("goal_pose: "+str(goal_pose.theta)+" current_pose "+str(self.pose.theta))

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control
            if abs(ang_vel) > 1000:


		    # Linear velocity in the x-axis.
		    vel_msg.linear.x = 0
		    vel_msg.linear.y = 0
		    vel_msg.linear.z = 0

		    # Angular velocity in the z-axis.
		    vel_msg.angular.x = 0
		    vel_msg.angular.y = 0
		    vel_msg.angular.z = ang_vel

		    # Publishing our vel_msg
		    self.velocity_publisher.publish(vel_msg)

            else:
                    # Linear velocity in the x-axis.
		    vel_msg.linear.x = lin_vel
		    vel_msg.linear.y = 0
		    vel_msg.linear.z = 0

		    # Angular velocity in the z-axis.
		    vel_msg.angular.x = 0
		    vel_msg.angular.y = 0
		    vel_msg.angular.z = ang_vel

		    # Publishing our vel_msg
		    self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    
    x = TurtleBot()
    rospy.spin()

