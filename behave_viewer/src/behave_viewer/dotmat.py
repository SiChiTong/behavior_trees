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

VALID_SHAPES = {'diamond':'D', 'box':'s', 'circle':'o', 'square':'s'}
DEFAULT_SHAPE = VALID_SHAPES['box']


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
  def __init__(self, context):
    super(DotMatViewer, self).__init__(context)
    self.setObjectName('DotMatViewer')
    self.context = context
    #~ self._args = self._parse_args(context.argv())
    self.maxrate = 10.
    self.topic = 'dotcode'
    # Create the figure canvas and attach it to the context
    self.widget = MatPlotWidget()
    self.widget.setObjectName('DotMat')
    if context.serial_number() > 1:
      self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self.widget)
    # Set the color palette
    palette = QPalette ()
    palette.setColor(QPalette.Background, Qt.white)
    self.widget.setPalette(palette)
    # Subscribe to the specified topic
    self.last_update = rospy.Time.now()
    self.sub = rospy.Subscriber(self.topic, String, self.dot_graph_cb)
  
  def dot_graph_cb(self, msg):
    # Throttling to 'maxrate' the update of the widget
    if (rospy.Time.now() - self.last_update) < rospy.Duration(1.0/self.maxrate):
      return
    self.last_update = rospy.Time.now()
    self.widget.clear_and_hold()
    A = pgv.AGraph(msg.data)
    G = nx.nx_agraph.from_agraph(A)
    pos = nx.drawing.nx_agraph.graphviz_layout(G, prog='dot')
    # Customize each node
    for identifier in G.nodes():
      node = G.node[identifier]
      if node.has_key('label'):
        label = {identifier:node['label']}
      else:
        label = {identifier:identifier}
      shape = DEFAULT_SHAPE
      if node.has_key('shape'):
        if node['shape'] in VALID_SHAPES.keys():
          shape = VALID_SHAPES[node['shape']]
        elif node['shape'] in VALID_SHAPES.values():
          shape = node['shape']
      nx.draw_networkx_nodes(G, pos, nodelist=[identifier], ax=self.widget.axes, 
                              node_color='w', node_shape=shape)
      nx.draw_networkx_labels(G, pos, label, font_size=11, ax=self.widget.axes)
    nx.draw_networkx_edges(G, pos, ax=self.widget.axes)
    self.widget.refresh()
  
  def save_settings(self, plugin_settings, instance_settings):
    pass
  
  def shutdown_plugin(self):
    # Unregister the subscribers to exit gracefully
    self.sub.unregister()
  
  def restore_settings(self, plugin_settings, instance_settings):
    pass

