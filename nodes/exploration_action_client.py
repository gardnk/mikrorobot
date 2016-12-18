#!/usr/bin/env python
from __future__ import division
import rospy
import roslib
roslib.load_manifest('mikrorobot')

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
import actionlib
import numpy as np
import tf

global client, listener

def exploration_cb(OccupancyGrid):
    # calculate an unexplored position for the robot to explore
    # get the current position and pose
    current_map = OccupancyGrid.data
    current_pos = listener.lookupTransform('base_link', 'map', rospy.Time(0))
    all_unexplored = np.where(current_map == -1)
    rows = all_unexplored[0]
    cols = all_unexplored[1]
    shortest_dist = 1000000
    nearest_point = [0,0]

    for i in range(0, rows.size):
        # find the shortes euclidean distance
        print rows[i], cols[i]
        x = abs(current_pos.x - rows[i])
        y = abs(current_pos.y - cols[i])
        dist = np.sqrt(x**2 + y**2)
        if dist < shortest_dist:
            shortest_dist = dist
            nearest_point = [rows[i], cols[i]]

    print current_pos
    print nearest_point

    goal = MoveBaseGoal()
    goal.header.stamp = rospy.Time.now()
    goal.pose.position.x = nearest_point[0]
    goal.pose.position.y = nearest_point[1]
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(20.0))

if __name__ == '__main__':
  rospy.init_node("exploration")
  rospy.loginfo("Started template python node: exploration.")

  client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  listener = tf.TransformListener()


  while not rospy.is_shutdown():
      rospy.loginfo("main loop")

      client.wait_for_server()

      rospy.Subscriber("map", OccupancyGrid, exploration_cb)

      rospy.sleep(0.1)
