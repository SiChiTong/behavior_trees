#! /usr/bin/env python
from .core import ActionState, Decorator


class Limiter(Decorator):
  def __init__(self, identifier, title, properties={}):
    super(Limiter, self).__init__(identifier, title, properties=properties)
    self.iterations = int(self.properties['maxLoop'] or 1)
    self.ticks = 0
  
  def tick(self):
    super(Limiter, self).tick()
    if self.ticks < self.iterations:
      state = self.child.tick()
      self.ticks += 1
      return state
    return ActionState.FAILURE


class Repeater(Decorator):
  """
  Repeater decorator sends the tick signal to its child every time that its 
  child returns a SUCCESS or FAILURE value, or when this decorator receives 
  the tick. Additionally, a maximum number of repetition can be provided.
  """
  def __init__(self, identifier, title, properties={}):
    super(Repeater, self).__init__(identifier, title, properties=properties)
    self.iterations = int(self.properties['maxLoop'] or 1)
    self.ticks = 0
  
  def tick(self):
    super(Repeater, self).tick()
    while self.ticks < self.iterations:
      state = self.child.tick()
      if state == ActionState.RUNNING:
        return ActionState.RUNNING
      self.ticks += 1
    return ActionState.SUCCESS
