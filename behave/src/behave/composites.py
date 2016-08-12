#! /usr/bin/env python
from .core import ActionState, Composite


class Selector(Composite):
  def __init__(self, identifier, title, properties={}):
    super(Selector, self).__init__(identifier, title, properties=properties)
  
  @property
  def label(self): 
    """
    The label is composed by (shape, text).
    The text of a selector's label is '?'
    """
    return ('box', '?')
  
  def tick(self):
    super(Selector, self).tick()
    for child in self.children:
      state = child.tick()
      if state != ActionState.FAILURE:
        return state
    return ActionState.FAILURE


class Sequence(Composite):
  def __init__(self, identifier, title, properties={}):
    super(Sequence, self).__init__(identifier, title, properties=properties)
  
  @property
  def label(self): 
    """
    The label is composed by (shape, text).
    The text of a sequence's label is '->'
    """
    return ('box', '->')
  
  def tick(self):
    super(Sequence, self).tick()
    for child in self.children:
      state = child.tick()
      if state != ActionState.SUCCESS:
        return state
    return ActionState.SUCCESS
