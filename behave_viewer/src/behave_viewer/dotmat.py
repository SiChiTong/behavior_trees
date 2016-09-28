#!/usr/bin/env python
import rospy
import numpy as np
# Graph processing
import anyjson
import networkx as nx
import pygraphviz as pgv
from std_msgs.msg import String
# rqt
from rqt_gui_py.plugin import Plugin
# QT bindings
from python_qt_binding.QtCore import Qt
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
# The rqt_plot plugin checks that we have the right matplotlib version
from rqt_plot.data_plot.mat_data_plot import FigureCanvas
from matplotlib.figure import Figure


class MatPlotWidget(FigureCanvas):
  """
  Ultimately, this is a QWidget (as well as a FigureCanvasAgg, etc.)
  """
  def __init__(self, parent=None):
    super(MatPlotWidget, self).__init__(Figure())
    self.setParent(parent)
    self.axes = self.figure.add_subplot(111, axisbg='white')
    self.clear_and_hold()
    self.figure.tight_layout()
    self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
    self.updateGeometry()
  
  def clear_and_hold(self):
    self.axes.cla()
    self.axes.get_xaxis().set_visible(False)
    self.axes.get_yaxis().set_visible(False)
    self.axes.hold(True)
  
  def dummy_plot(self):
    t = np.arange(0.0, 3.0, 0.01)
    s = np.sin(2*np.pi*t)
    self.axes.plot(t, s)
  
  def refresh(self):
    self.draw()
  
  def resizeEvent(self, event):
    super(MatPlotWidget, self).resizeEvent(event)
    self.figure.tight_layout()

class DotMatViewer(Plugin):
  PLUGIN_TITLE = 'Behave Viewer'
  def __init__(self, context):
    super(DotMatViewer, self).__init__(context)
    self.context = context
    self.maxrate = 10
    self.topic = 'dotcode'
    self.arrows = False
    self.width = 2.0
    # Create the figure canvas and attach it to the context
    self.widget = MatPlotWidget()
    self.widget.setObjectName(self.PLUGIN_TITLE)
    self.widget.setWindowTitle(self.PLUGIN_TITLE)
    if context.serial_number() > 1:
      self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self.widget)
    # Set the color palette
    palette = QPalette ()
    palette.setColor(QPalette.Background, Qt.white)
    self.widget.setPalette(palette)
    # Subscribe to the specified topic
    self.last_update = rospy.get_time()
    self.sub = rospy.Subscriber(self.topic, String, self.dot_graph_cb)
  
  def get_valid_matshape(self, node, default='s'):
    if not node.has_key('shape'):
      return defautl
    else:
      pgvshape = node['shape']
    # Convert shape from pygraphviz to matplotlib
    equivalences = {'D':['diamond'], 
                    'o':['circle'],
                    's':['box', 'rect', 'rectangle', 'square']}
    matshape = default
    for key, value in equivalences.items():
      if (pgvshape == key) or (pgvshape in value):
        matshape = key
        break
    return matshape
  
  def dot_graph_cb(self, msg, alpha=0.3):
    if (rospy.get_time() - self.last_update) < (1.0/self.maxrate) or rospy.is_shutdown():
      # Throttling to 'maxrate' the update of the widget
      return
    self.last_update = rospy.get_time()
    self.widget.clear_and_hold()
    A = pgv.AGraph(msg.data)
    G = nx.nx_agraph.from_agraph(A)
    pos = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot')
    # Draw one node at the time
    for identifier in G.nodes():
      node = G.node[identifier]
      # Label
      label = {identifier:node['label']} if node.has_key('label') else {identifier:identifier}
      nx.draw_networkx_labels(G, pos, label, font_size=12, ax=self.widget.axes)
      # Shape and color
      shape = self.get_valid_matshape(node)
      color = node['color'] if node.has_key('color') else 'white'
      nx.draw_networkx_nodes(G, pos, nodelist=[identifier], ax=self.widget.axes, 
                              node_color=color, node_shape=shape, node_size=500, alpha=alpha)
    # Draw one edge at the time
    for edge in G.edges():
      edge_attrs = G.edge[edge[0]][edge[1]]
      color = edge_attrs['color'] if edge_attrs.has_key('color') else 'black'
      nx.draw_networkx_edges(G, pos, edgelist=[edge], ax=self.widget.axes, arrows=self.arrows,
                              edge_color=color, width=self.width, alpha=alpha)
    self.widget.refresh()
  
  def restore_settings(self, plugin_settings, instance_settings):
    # Nothing to be done for now
    pass
  
  def save_settings(self, plugin_settings, instance_settings):
    # Nothing to be done for now
    pass
  
  def shutdown_plugin(self):
    # Unregister the subscribers to exit gracefully
    self.sub.unregister()

