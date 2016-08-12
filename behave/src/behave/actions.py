#! /usr/bin/env python
import time
from .core import Action, ActionState


class Failer(Action):
  """
  Place-holder class that always returns FAILURE
  """
  def __init__(self, identifier, title, properties={}):
    super(Failer, self).__init__(identifier, title, properties)
  
  def tick(self):
    super(Failer, self).tick()
    return ActionState.FAILURE


class Runner(Action):
  """
  Place-holder class that always returns RUNNING
  """
  def __init__(self, identifier, title, properties={}):
    super(Runner, self).__init__(identifier, title, properties)
  
  def tick(self):
    super(Runner, self).tick()
    return ActionState.RUNNING


class Succeeder(Action):
  """
  Place-holder class that always returns SUCCESS
  """
  def __init__(self, identifier, title, properties={}):
    super(Succeeder, self).__init__(identifier, title, properties)
  
  def tick(self):
    super(Succeeder, self).tick()
    return ActionState.SUCCESS


class Wait(Action):
  """
  Place-holder class that waits the given milliseconds and returns SUCCESS
  """
  def __init__(self, identifier, title, properties={}):
    super(Wait, self).__init__(identifier, title, properties)
    self.timeout = self.properties['milliseconds'] or 10.
  
  def tick(self):
    super(Wait, self).tick()
    time.sleep(self.timeout/1000.)
    return ActionState.SUCCESS
