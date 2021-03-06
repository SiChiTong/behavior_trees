#! /usr/bin/env python
import uuid
import time
import behave
import criros
import anyjson
import inspect
import importlib
import collections
import networkx as nx
from networkx.readwrite import json_graph
# Logger
from criros.utils import TextColors


def get_class_from_str(class_str):
    split = class_str.rsplit('.', 1)
    if len(split) == 1:
      module = behave
      classname = split[0]
    else:
      try:
         module = importlib.import_module(split[0])
         classname = split[1]
      except ImportError:
        return None
    if not hasattr(module, classname):
      return None
    return getattr(module, classname)


class ActionState(object):
  """
  A class for enumerating possible state of the Actions
  """
  SUCCESS = 0
  FAILURE = 1
  RUNNING = 2
  NONE    = 3


class Action(object):
  def __init__(self, identifier, title, properties):
    self.category = 'Action'
    self.identifier = identifier
    self.title = title
    self.properties = collections.defaultdict(lambda:None, properties)
    self.state = ActionState.NONE
  
  @property
  def label(self): 
    """
    The default label for actions and decorators is the title in the JSON
    specification
    """
    return self.title
  
  @property
  def name(self): 
    return self.__class__.__name__
  
  @property
  def shape(self): 
    """
    The shape for the actions is a square
    """
    return 'square'
  
  def get_state(self):
    return self.state
  
  def reset(self):
    self.state = ActionState.NONE
  
  def tick(self):
    pass


class BehaviorTree(Action):
  def __init__(self, identifier=None, title='BEHAVIOR_TREE', properties={}, logger=TextColors()):
    identifier = identifier or str(uuid.uuid1())
    super(BehaviorTree, self).__init__(identifier, title, properties=properties)
    self.category = 'BehaviorTree'
    self.title = title
    self.graph = nx.DiGraph()
    self.logger = logger
    self.nodes = dict()
    self.children = collections.defaultdict(list)
    self.initialized = False
  
  def from_dict(self, data):
    # Check that the dict has the expected keys
    if not criros.utils.has_keys(data, ['nodes','properties','root','title']):
      self.logger.logdebug('Dict does not have the expected keys')
      return False
    # Create a graph and check that it's a tree
    graph = nx.DiGraph()
    nodes = dict()
    blacklist = []
    # Process the nodes
    for identifier, spec in data['nodes'].items():
      # Get the children of the current node
      if spec.has_key('child'):
        self.children[identifier].append(spec['child'])
      elif spec.has_key('children'):
        self.children[identifier] = spec['children']
      # Check spec has the expected fields
      if not criros.utils.has_keys(spec, ['id','name','properties','title']):
        self.logger.logdebug( 'Node [{0}] does not have the expected keys'.format(identifier) )
        blacklist += self.children[identifier]
        continue
      # Check soundness
      if identifier != spec['id']:
        self.logger.logdebug( 'Identifier miss-match {0} != {1}'.format(identifier, spec['id']) )
        blacklist += self.children[identifier]
        continue
      # Skip blacklisted nodes
      if identifier in blacklist:
        self.logger.logdebug( 'Skiping blacklisted node {0}:{1}'.format(spec['title'], identifier) )
        blacklist += self.children[identifier]
        continue
      # Get node class and initialize it
      nodeclass = get_class_from_str(spec['name'])
      if nodeclass is None:
        self.logger.logdebug( 'Node {0}:{1} has unknown node class: {2}'.format(spec['title'], identifier, spec['name']) )
        blacklist += self.children[identifier]
        continue
      if behave.core.Action not in inspect.getmro(nodeclass):
        self.logger.logdebug( 'Node {0}:{1} has invalid node class: {2}'.format(spec['title'], identifier, spec['name']) )
        self.logger.logdebug( 'It must inherit from any of these: (Action, Composite, Decorator)' )
        blacklist += self.children[identifier]
        continue
      nodes[identifier] = nodeclass(identifier, title=spec['title'], properties=spec['properties'])
      # Populate the graph
      for child in self.children[identifier]:
        graph.add_edge(identifier, child)
    if len(graph) == 0:
      self.logger.logdebug( 'Failed parsing dict. Empty graph' )
      return False
    if not nx.is_tree(graph):
      self.logger.logdebug( 'Failed parsing dict. Graph must be a tree' )
      return False
    for identifier in nodes.keys():
      # Add the children for the nodes that were created
      for child_id in self.children[identifier]:
        if child_id in nodes.keys():
          # Depending on the type of class we add children or specify the child
          if    behave.core.Composite in inspect.getmro( type(nodes[identifier]) ):
            nodes[identifier].add_child( nodes[child_id] )
          elif  behave.core.Decorator in inspect.getmro( type(nodes[identifier]) ):
            nodes[identifier].set_child( nodes[child_id] )
          else:
            self.logger.logdebug( 'Node {0}:{1} has invalid node class: {2}'.format(spec['title'], identifier, spec['name']) )
            self.logger.logdebug( 'To have children it must inherit from any of these: (Composite, Decorator)' )
      # Store details in self.graph as well
      graph.node[identifier] = data['nodes'][identifier]['properties']
      graph.node[identifier]['children'] = self.children[identifier]
      graph.node[identifier]['class'] = nodes[identifier].name
      graph.node[identifier]['category'] = nodes[identifier].category
      graph.node[identifier]['title'] = nodes[identifier].title
      graph.node[identifier]['label'] = nodes[identifier].label
      graph.node[identifier]['shape'] = nodes[identifier].shape
    # Set-up the class
    if data.has_key('id'):
      self.identifier = data['id']
    self.root = data['root']
    self.graph = graph
    self.nodes = nodes
    self.initialized = True
    return True
  
  def generate_dot(self):
    A = nx.nx_agraph.to_agraph(self.graph)
    return A.to_string()
  
  def generate_tree_data(self):
    """ 
    It seems like networkx does not maintain transversal order. So, we need generate the tree manually.
    """
    ## Recursive function to generate node data
    def generate_node_data(nodeid):
      node_dict = dict()
      node_dict['label'] = self.graph.node[nodeid]['label']
      node_dict['shape'] = self.graph.node[nodeid]['shape']
      node_dict['color'] = self.graph.node[nodeid]['color']
      if nodeid in self.children.keys():
        node_dict['children'] = []
        for child in self.children[nodeid]:
          node_dict['children'].append( generate_node_data(child) )
      return node_dict
    ## End of recursive function
    json_dict = generate_node_data(self.root)
    return anyjson.dumps(json_dict)
  
  def is_initialized(self):
    return self.initialized
  
  def reset(self):
    rootid = self.root
    if rootid is None:
      return
    state = self.nodes[rootid].tick()
    if state == ActionState.RUNNING:
      self.logger.logwarn('A running tree cannot be reseted')
      return
    self.nodes[rootid].reset()
  
  def tick(self):
    super(BehaviorTree, self).tick()
    if self.root is None:
      return ActionState.FAILURE
    rootid = self.root
    if not self.nodes.has_key(rootid):
      return ActionState.FAILURE
    state = self.nodes[rootid].tick()
    return state
  
  def update_graph_colors(self):
    colors = {ActionState.SUCCESS : 'green',
              ActionState.FAILURE : 'orange',
              ActionState.RUNNING : 'red',
              ActionState.NONE    : 'bluesteel'}
    for nodeid in self.graph.nodes():
      state = self.nodes[nodeid].get_state()
      self.graph.node[nodeid]['color'] = colors[state]


class Composite(Action):
  def __init__(self, identifier, title, properties={}):
    super(Composite, self).__init__(identifier, title, properties=properties)
    self.category = 'Composite'
    self.children = []
    self.timeout = self.properties['timeout'] or 0.001
  
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
  
  def reset(self):
    super(Composite, self).reset()
    if len(self.children) == 0:
      return
    for child in self.children:
      child.reset()


class Decorator(Action):
  def __init__(self, identifier, title, properties={}):
    super(Decorator, self).__init__(identifier, title, properties=properties)
    self.category = 'Decorator'
    self.child = None
    self.timeout = self.properties['timeout'] or 0.001
  
  @property
  def shape(self): 
    """
    The shape for decorators is 'diamond'
    """
    return 'diamond'
  
  def reset(self):
    super(Decorator, self).reset()
    if self.child is None:
      return
    self.child.reset()
  
  def set_child(self, child):
    self.child = child
