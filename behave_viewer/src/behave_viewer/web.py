import rospkg
from qt_gui.plugin import Plugin
from rqt_web.web_widget import WebWidget


class Web(Plugin):
  """
  Plugin to interface with webtools via ros_gui
  """
  def __init__(self, context):
    super(Web, self).__init__(context)
    self.setObjectName('Behave Viewer')
    #  This method is used to specify a static url
    rospack = rospkg.RosPack()
    viewer_pkg = rospack.get_path('behave_viewer')
    self._web = WebWidget('file://{0}/html/d3js_viewer.html'.format(viewer_pkg))
    if context.serial_number() > 1:
      self._web.setWindowTitle(self._web.windowTitle() + (' (%d)' % context.serial_number()))
    context.add_widget(self._web)

  def shutdown_plugin(self):
    pass

  def save_settings(self, plugin_settings, instance_settings):
    self._web.save_settings(instance_settings)

  def restore_settings(self, plugin_settings, instance_settings):
    self._web.restore_settings(instance_settings)
