#! /usr/bin/env python
from core import ActionState, Composite


class Selector(Composite):
  def __init__(self, identifier, label):
    super(Selector, self).__init__(identifier, label)
  
  def tick(self):
    for child in children:
      state = child.tick()
      if state == ActionState.SUCCESS:
        yield ActionState.SUCCESS
        return
      else:
        yield ActionState.RUNNING
    yield ActionState.FAILURE


class Sequence(Composite):
  def __init__(self, identifier, label):
    super(Sequence, self).__init__(identifier, label)
  
  def tick(self):
    for child in children:
      state = child.tick()
      if state == ActionState.FAILURE:
        yield ActionState.FAILURE
        return
      else:
        yield ActionState.RUNNING
    yield ActionState.SUCCESS
