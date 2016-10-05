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
    self.state = ActionState.RUNNING
    super(Failer, self).tick()
    self.state = ActionState.FAILURE
    return ActionState.FAILURE


class Runner(Action):
  """
  Place-holder class that always returns RUNNING
  """
  def __init__(self, identifier, title, properties={}):
    super(Runner, self).__init__(identifier, title, properties)
  
  def tick(self):
    self.state = ActionState.RUNNING
    super(Runner, self).tick()
    return ActionState.RUNNING


class Succeeder(Action):
  """
  Place-holder class that always returns SUCCESS
  """
  def __init__(self, identifier, title, properties={}):
    super(Succeeder, self).__init__(identifier, title, properties)
  
  def tick(self):
    self.state = ActionState.RUNNING
    super(Succeeder, self).tick()
    self.state = ActionState.SUCCESS
    return ActionState.SUCCESS


class Wait(Action):
  """
  Place-holder class that waits the given milliseconds and returns SUCCESS
  """
  def __init__(self, identifier, title, properties={}):
    super(Wait, self).__init__(identifier, title, properties)
    self.milliseconds = self.properties['milliseconds'] or 10.
    self.executed = False
  
  def reset(self):
    if self.state == ActionState.RUNNING:
      return
    self.executed = False
    super(Wait, self).reset()
  
  def tick(self):
    super(Wait, self).tick()
    if not self.executed:
      self.state = ActionState.RUNNING
      time.sleep(self.milliseconds/1000.)
      self.state = ActionState.SUCCESS
      self.executed = True
      return ActionState.SUCCESS
    else:
      return self.state
