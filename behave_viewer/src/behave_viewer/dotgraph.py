#!/usr/bin/env python
import networkx as nx
import matplotlib.pyplot as plt
import cStringIO


# For the matplotlib based behave_viewer we serialize using JSON
data = json_graph.node_link_data(btree.graph)
# Deserialize graph
G = json_graph.node_link_graph(data)

# For rqt we need to serialize using the 'dot' format
output = cStringIO.StringIO()
output.close()



#~ G = nx.path_graph(8)
# Get the tree positions
pos = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot')
labels = {identifier:node['label'] for (identifier, node) in G.node.items()}
nx.draw(G, pos=pos)
nx.draw_networkx_labels(G, pos, labels, font_size=10)
plt.show()
