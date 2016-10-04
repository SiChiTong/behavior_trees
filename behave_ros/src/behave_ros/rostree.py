#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from behave.core import BehaviorTree


class RosBehaviorTree(BehaviorTree):
  def __init__(self, identifier=None, title='BEHAVIOR_TREE', properties={}):
    super(RosBehaviorTree, self).__init__(identifier, title, properties=properties, logger=rospy)
    publish_rate = rospy.get_param('~publish_rate', 5)
    ns = rospy.get_namespace()
    topic = '{0}behave/json_tree'.format(ns)
    self.tree_pub = rospy.Publisher(topic, String, queue_size=3)
    self.timer = rospy.Timer(rospy.Duration(1.0/publish_rate), self.cb_publish_tree)
    self.logger.loginfo( 'Publishing behavior tree to: {0} at {1} Hz'.format(topic, publish_rate) )
    rospy.on_shutdown(self.on_shutdown)
  
  def cb_publish_tree(self, event):
    if rospy.is_shutdown() or not self.is_initialized():
      return
    self.update_graph_colors()
    tree_data = self.generate_tree_data()
    self.tree_pub.publish(data=tree_data)
  
  def on_shutdown(self):
    self.timer.shutdown()
