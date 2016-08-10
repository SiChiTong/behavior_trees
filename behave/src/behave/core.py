#! /usr/bin/env python
import time
import criros
import networkx as nx

class ActionState(object):
  """
  A class for enumerating possible state of the Actions
  """
  FAILURE = 0
  SUCCESS = 1
  RUNNING = 2

class Action(object):
  def __init__(self, identifier, label):
    self.category = 'Action'
    self.identifier = identifier
    self.label = label
  
  @property
  def name(self): 
    return self.__class__.__name__
  
  def tick(self):
    pass

class Composite(Action):
  def __init__(self, identifier, label):
    super(Composite, self).__init__(identifier, label)
    self.category = 'Composite'
    self.children = []
  
  def get_children_identifiers(self):
    return [child.identifier for child in self.children]
  
  def add_child(self, child):
    if child.identifier in self.get_children_identifiers():
      return False
    self.children.append(child)
  
  def remove_child(self, child):
    children_ids = self.get_children_identifiers()
    if child.identifier in children_ids:
      idx = children_ids.index(child.identifier)
      self.children.pop(idx)
  
  def tick(self):
    pass

class Decorator(Action):
  def __init__(self, identifier, label):
    super(Decorator, self).__init__(identifier, label)
    self.category = 'Decorator'
    self.child = None
  
  def tick(self):
    pass
