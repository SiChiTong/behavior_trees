#! /usr/bin/env python
from core import Action, ActionState


class Failer(Action):
  """
  Place-holder class that will always return FAILURE
  """
  def __init__(self, identifier, label):
    super(Failer, self).__init__(identifier, label)
  
  def tick(self):
    return ActionState.FAILURE


class Runner(Action):
  """
  Place-holder class that will always return RUNNING
  """
  def __init__(self, identifier, label):
    super(Runner, self).__init__(identifier, label)
  
  def tick(self):
    return ActionState.RUNNING


class Succeeder(Action):
  """
  Place-holder class that will always return SUCCESS
  """
  def __init__(self, identifier, label):
    super(Succeeder, self).__init__(identifier, label)
  
  def tick(self):
    return ActionState.SUCCESS
