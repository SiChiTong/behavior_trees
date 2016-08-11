#!/usr/bin/env python
import networkx as nx
import matplotlib.pyplot as plt

G = btree.graph
#~ G = nx.path_graph(8)
# Get the tree positions
pos = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot')
labels = {identifier:node['label'] for (identifier, node) in G.node.items()}
nx.draw(G, pos=pos, root=btree.root)
nx.draw_networkx_labels(G, pos, labels, font_size=10)
plt.show()
