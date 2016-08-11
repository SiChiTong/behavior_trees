#!/usr/bin/env python
import networkx as nx
import pygraphviz as pgv
import matplotlib.pyplot as plt

G = nx.path_graph(8)
# Get the tree positions
pos = nx.drawing.nx_agraph.graphviz_layout(G,prog='dot')

nx.draw(G, pos=pos)
plt.show()
