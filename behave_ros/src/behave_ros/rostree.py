#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from behave.core import BehaviorTree


class RosBehaviorTree(BehaviorTree):
  def __init__(self, identifier=None, title='BEHAVIOR_TREE', properties={}):
    super(RosBehaviorTree, self).__init__(identifier, title, properties=properties, logger=rospy)
    dotcode_rate = rospy.get_param('~dotcode_rate', 5)
    self.dotcode_pub = rospy.Publisher('dotcode', String, queue_size=3)
    self.dotcode_timer = rospy.Timer(rospy.Duration(1.0/dotcode_rate), self.cb_publish_dotcode)
    ns = rospy.get_namespace()
    self.logger.loginfo( 'Publishing behavior tree state to: {0}dotcode at {1} Hz'.format(ns, dotcode_rate) )
    rospy.on_shutdown(self.on_shutdown)
  
  def cb_publish_dotcode(self, event):
    if rospy.is_shutdown():
      return
    self.update_graph_colors()
    dotcode = self.generate_dot()
    self.dotcode_pub.publish(data=dotcode)
  
  def on_shutdown(self):
    self.dotcode_timer.shutdown()
