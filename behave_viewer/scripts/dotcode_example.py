#!/usr/bin/env python
import rospy
import anyjson
import numpy as np
import networkx as nx
import pygraphviz as pgv
from std_msgs.msg import String

# Initialize ROS publisher
rospy.init_node('playground')
pub = rospy.Publisher('/dotcode', String, queue_size=3)
rate = rospy.Rate(0.5)
rospy.loginfo('Publishing DOT graph to: /dotcode')
while not rospy.is_shutdown():
  # Create random graphs
  G = nx.gn_graph(10)
  # Random shapes and colors
  for nodeid in G.nodes():
    shape = ['circle','diamond','rectangle'][np.random.randint(0,3)]
    color = ['red','green','orange','white'][np.random.randint(0,4)]
    G.node[nodeid] = {'label': str(nodeid), 'shape':shape, 'color':color}
  for edge in G.edges():
    color = ['red','green','orange','black'][np.random.randint(0,4)]
    G.edge[edge[0]][edge[1]] = {'color':color}
  # Serialize and publish the graph
  A = nx.nx_agraph.to_agraph(G)
  pub.publish(data=A.to_string())
  rate.sleep()




