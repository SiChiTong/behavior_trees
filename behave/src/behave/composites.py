#! /usr/bin/env python
import time
from .core import ActionState, Composite


class Selector(Composite):
  def __init__(self, identifier, title, properties={}):
    super(Selector, self).__init__(identifier, title, properties=properties)
  
  @property
  def label(self): 
    """
    The label for a selector is '?'
    """
    return '?'
  
  def tick(self):
    super(Selector, self).tick()
    for child in self.children:
      time.sleep(self.timeout)
      state = child.tick()
      if state != ActionState.FAILURE:
        self.state = state
        return state
    self.state = ActionState.FAILURE
    return ActionState.FAILURE


class Sequence(Composite):
  def __init__(self, identifier, title, properties={}):
    super(Sequence, self).__init__(identifier, title, properties=properties)
  
  @property
  def label(self): 
    """
    The label for a selector is '->'
    """
    return '->'
  
  def tick(self):
    super(Sequence, self).tick()
    for child in self.children:
      time.sleep(self.timeout)
      state = child.tick()
      if state != ActionState.SUCCESS:
        self.state = state
        return state
    self.state = ActionState.SUCCESS
    return ActionState.SUCCESS
