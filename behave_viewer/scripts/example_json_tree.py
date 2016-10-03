#!/usr/bin/env python
import rospy
import anyjson
import itertools
import networkx as nx
from networkx.readwrite import json_graph
from std_msgs.msg import String

# Initialize ROS publisher
rospy.init_node('example_json_tree')
pub = rospy.Publisher('/behave/json_tree', String, queue_size=3)
rate = rospy.Rate(0.5)
graphs = []
graphs.append( nx.DiGraph([(1,2),(1,3),(1,4),(2,5),(2,6)]) )
graphs.append( nx.DiGraph([(1,2),(1,3),(1,4),(4,5),(4,6),(2,7),(2,8)]) )
graphs.append( nx.DiGraph([(1,2),(1,3),(2,4),(3,5),(4,6)]) )
cycle = itertools.cycle(graphs)
rospy.loginfo('Publishing DOT graph to: /behave/json_tree')
while not rospy.is_shutdown():
  G = cycle.next()
  for n in G:
    G.node[n]['name'] = n
  # Random shapes and colors
  for nodeid in G.nodes():
    shape = ['circle','diamond','square'][np.random.randint(0,3)]
    color = ['red','green','orange','white'][np.random.randint(0,4)]
    G.node[nodeid] = {'label': str(nodeid), 'shape':shape, 'color':color}
  data = json_graph.tree_data(G, root=1)
  pub.publish(data=str(anyjson.dumps(data)))
  rate.sleep()




