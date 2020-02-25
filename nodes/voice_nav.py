#!/usr/bin/env python

"""
  The voice_nav.py is based on Michael Ferguson's voice_cmd_vel.py script in the
  pocketsphinx ROS package and Patrick Goebel's nav_test.py script.
  
  See 
  
  http://www.ros.org/wiki/pocketsphinx
  https://github.com/pirobot/rbx1/blob/indigo-devel/rbx1_nav/nodes/nav_test.py
"""

import roslib; roslib.load_manifest('pocketnav')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from math import copysign

import os

import actionlib
from actionlib_msgs.msg import *

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import pow, sqrt, radians, pi

class voice_nav:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)        
        
        # We don't have to run the script very fast
        self.rate = rospy.get_param("~rate", 5)
        r = rospy.Rate(self.rate)

        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)

        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'kitchen': ['where is the kitchen', 'kitchen'],
                                    'living_room': ['go to the living room', 'living room'],
                                    'bathroom': ['where is the bathroom', 'bathroom'],
                                    'bedroom': ['where is the bedroom', 'bedroom'],
                                    'porch': ['where is the porch', 'porch']}
        
        rospy.loginfo("Ready to receive voice commands")

        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 10)
        
        # Initialize the Twist message we will publish.
        self.cmd_vel = Twist()

        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        

        # Create a list to hold the target quaternions (orientations)
        self.quaternions = list()
        
        # First define the corner orientations as Euler angles
        self.euler_angles = (pi/2, pi, 3*pi/2, 0)
        
        # Then convert the angles to quaternions
        for angle in self.euler_angles:
            q_angle = quaternion_from_euler(0, 0, angle, axes='sxyz')
            q = Quaternion(*q_angle)
            self.quaternions.append(q)

        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.locations = dict()

        self.locations['kitchen'] = Pose(Point(3.000, 2.500, 0.000), self.quaternions[0])
        self.locations['living_room'] = Pose(Point(0.000, 0.000, 0.000), self.quaternions[3])
        self.locations['bathroom'] = Pose(Point(1.500, 3.500, 0.000), self.quaternions[3])
        self.locations['bedroom'] = Pose(Point(-0.500, 0.500, 0.000), self.quaternions[1])
        self.locations['porch'] = Pose(Point(-0.500, -1.000, 0.000), self.quaternions[2])


        # Publish the Twist message to the cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        
        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        self.initial_pose = PoseWithCovarianceStamped()

        # Variables to keep track of success rate, running time,
        # and distance traveled
        #n_locations = len(locations)
        self.n_goals = 0
        self.n_successes = 0
        #i = n_locations
        self.distance_traveled = 0
        self.start_time = rospy.Time.now()
        self.running_time = 0
        #location = ""
        self.last_location = ""
        self.distance = 0
        
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        self.last_location = Pose()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

        # Make sure we have the initial pose
        while self.initial_pose.header.stamp == "":
            rospy.sleep(1)
            
        rospy.loginfo("Starting navigation test")

        
        # We have to keep publishing the cmd_vel message if we want the robot to keep moving.
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.cmd_vel)
            r.sleep()   

                                      
            
    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    return command
        
    def speech_callback(self, msg):
	    rospy.loginfo('Can I help you?')
            rospy.loginfo(msg.data)

            # Get the next location in the current sequence
            location = self.get_command(msg.data)

            
            # Let the user know where the robot is going next
            rospy.loginfo("Going to: " + str(location))

            if location == None:
                return

            # Keep track of the distance traveled.
            # Use updated initial pose if available.
            if self.initial_pose.header.stamp == "":
                self.distance = sqrt(pow(self.locations[location].position.x - 
                                    self.locations[self.last_location].position.x, 2) +
                                pow(self.locations[location].position.y - 
                                    self.locations[self.last_location].position.y, 2))
            else:
                rospy.loginfo("Updating current pose.")
                self.distance = sqrt(pow(self.locations[location].position.x - 
                                    self.initial_pose.pose.pose.position.x, 2) +
                                pow(self.locations[location].position.y - 
                                    self.initial_pose.pose.pose.position.y, 2))
                self.initial_pose.header.stamp = "" 
            
            # Store the last location for distance calculations
            self.last_location = location
            
            # Increment the counters
            self.n_goals += 1
        
            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = self.locations[location]
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Start the robot toward the next location
            self.move_base.send_goal(self.goal)
            
            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    self.n_successes += 1
                    self.distance_traveled += self.distance
                    rospy.loginfo("State:" + str(state))
                else:
                  rospy.loginfo("Goal failed with error code: " + str(self.goal_states[state]))
            
            # How long have we been running?
            self.running_time = rospy.Time.now() - self.start_time
            self.running_time = self.running_time.secs / 60.0
            
            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(self.n_successes) + "/" + 
                          str(self.n_goals) + " = " + 
                          str(100 * self.n_successes/self.n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(self.running_time, 1)) + 
                          " min Distance: " + str(trunc(self.distance_traveled, 1)) + " m")
            #rospy.sleep(self.rest_time)


    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def cleanup(self):
        # When shutting down be sure to stop the robot!
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1)
      
def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__=="__main__":
    rospy.init_node('voice_nav')
    try:
        voice_nav()
        # spin() simply keeps python from exiting
	# until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation terminated.")

