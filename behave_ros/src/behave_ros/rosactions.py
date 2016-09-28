#! /usr/bin/env python
# ROS
import genpy
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
# Behavior trees
import behave.core
from behave.core import Action, ActionState


class RosSimpleAction(Action):
  def __init__(self, identifier, title, properties={}):
    super(RosSimpleAction, self).__init__(identifier, title, properties)
    # Action timeouts
    self.connect_timeout = self.properties['connect_timeout'] or 10.0
    self.result_timeout = self.properties['result_timeout'] or 15.0
    rate = self.properties['rate'] or 60.0
    self.rate = rospy.Rate(rate)
    self.action_name = self.properties['action_name']
    action_str = self.properties['action_class']
    invalid_action_def = True
    if action_str is not None:
      self.action_class = behave.core.get_class_from_str(action_str + 'Action')
      self.goal_class = behave.core.get_class_from_str(action_str + 'Goal')
      invalid_action_def = (self.action_class is None) or (self.goal_class is None)
    if invalid_action_def:
      rospy.logerr('Invalid action definition: {0}'.format(action_str))
      return
    if (self.action_name is None):
      rospy.logerr('{0}: Please specify properties.action_name'.format(title))
      return
    # Remove used fields
    for key in ['action_class','action_name','connect_timeout','rate','result_timeout']:
      del self.properties[key]
    # Connect to the Simple Action Server
    self.client =  actionlib.SimpleActionClient(self.action_name, self.action_class)
    # Populate the goal
    self.goal = self.goal_class()
    genpy.message.fill_message_args(self.goal, [dict(self.properties)])
    # Wait for the server
    self.server_up = self.client.wait_for_server(timeout=rospy.Duration(self.connect_timeout))
    if not self.server_up:
      rospy.logerr('{0}: Timed out waiting for action server'.format(self.action_name))
      return
    # Flags
    self.started = False
  
  def tick(self):
    # Check that the server is up
    if not self.server_up:
      self.state = ActionState.FAILURE
      return ActionState.FAILURE
    super(RosSimpleAction, self).tick()
    self.rate.sleep()
    # Send the goal
    if not self.started:
      self.client.send_goal(self.goal)
      self.start_time = rospy.get_time()
      self.state = ActionState.RUNNING
      self.started = True
    elif self.state != ActionState.RUNNING:
      return self.state
    # Check goal status
    if (rospy.get_time()-self.start_time) > self.result_timeout:
      self.client.cancel_goal()
      rospy.logerr('{0}: Timed out waiting for action result'.format(self.action_name))
      self.state = ActionState.FAILURE
    elif self.client.get_state() == GoalStatus.SUCCEEDED:
      self.state = ActionState.SUCCESS
    elif self.client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.RECALLING, GoalStatus.RECALLED]:
      self.state = ActionState.RUNNING
    else:
      self.state = ActionState.FAILURE
    return self.state
