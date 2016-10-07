#! /usr/bin/env python
import rospy
from std_msgs.msg import String
from behave.core import BehaviorTree


class RosBehaviorTree(BehaviorTree):
  def __init__(self, identifier=None, title='BEHAVIOR_TREE', properties={}, publish_tree=True):
    super(RosBehaviorTree, self).__init__(identifier, title, properties=properties, logger=rospy)
    self.publish_rate = rospy.get_param('~publish_rate', 5)
    ns = rospy.get_namespace()
    topic = '{0}behave/json_tree'.format(ns)
    self.tree_pub = rospy.Publisher(topic, String, queue_size=3)
    if publish_tree:
      self.start_publishing_tree()
    rospy.on_shutdown(self.stop_publishing_tree)
  
  def cb_publish_tree(self, event):
    if rospy.is_shutdown() or not self.is_initialized():
      return
    self.update_graph_colors()
    tree_data = self.generate_tree_data()
    self.tree_pub.publish(data=tree_data)
  
  def start_publishing_tree(self):
    self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.cb_publish_tree)
    self.logger.loginfo( 'Publishing behavior tree to: {0} at {1} Hz'.format(self.tree_pub.name, self.publish_rate) )
  
  def stop_publishing_tree(self):
    if hasattr(self, 'timer'):
      if self.timer.is_alive():
        self.timer.shutdown()
      del self.timer
