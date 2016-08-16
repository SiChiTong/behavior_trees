#!/usr/bin/env python
import os
import rospy
import anyjson
import argparse
from behave.core import ActionState
from behave_ros import RosBehaviorTree
from criros.utils import read_parameter, read_parameter_err


if __name__ == '__main__':
  # Parse command line arguments and initialize node
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  parser = argparse.ArgumentParser(description=node_name)
  parser.add_argument('-d', '--debug', action='store_true', help='If set, will show additional debugging messages')
  args = parser.parse_args(rospy.myargv()[1:])
  log_level= rospy.DEBUG if args.debug else rospy.INFO
  rospy.init_node(node_name,log_level=log_level)
  # Read parameters from ROS server
  has_json, json_text  = read_parameter_err('~json_tree')
  if not has_json:
    exit(1)
  # Parse JSON behavior tree
  data = anyjson.deserialize(json_text)
  btree = RosBehaviorTree()
  btree.from_dict(data)
  # Start ticking the behavior tree
  rospy.loginfo('Started ticking the behaviour tree')
  state = ActionState.RUNNING
  while state == ActionState.RUNNING:
    state = btree.tick()
  rospy.spin()
