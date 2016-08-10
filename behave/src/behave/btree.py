#! /usr/bin/env python
import uuid
import behave
import criros
import importlib
import networkx as nx
# Logger
from criros.utils import TextColors


class BehaviorTree(behave.core.Action):
  def __init__(self, identifier=None, label='BEHAVIOR_TREE', logger=TextColors()):
    identifier = identifier or str(uuid.uuid1())
    super(BehaviorTree, self).__init__(identifier, label)
    self.category = 'BehaviorTree'
    self.graph = nx.Graph()
    self.logger = logger
    self.nodes = dict()
  
  def _get_node_class(self, name):
    split = name.rsplit('.', 1)
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
  
  def from_dict(self, data):
    # Check that the dict has the expected keys
    if not criros.utils.has_keys(data, ['nodes','properties','root','title']):
      self.logger.logdebug('Dict does not have the expected keys')
      return False
    # Create a graph and check that it's a tree
    graph = nx.Graph()
    nodes = dict()
    # Process the nodes
    for identifier, spec in data['nodes'].items():
      if not criros.utils.has_keys(spec, ['id','name','properties','title']):
        self.logger.logdebug( 'Node [{0}] does not have the expected keys'.format(identifier) )
        continue
      # Check soundness
      if identifier != spec['id']:
        self.logger.logdebug( 'Identifier miss-match {0} != {1}'.format(identifier, spec['id']) )
        continue
      # Get node class and initialize it
      nodeclass = self._get_node_class(spec['name'])
      if nodeclass is None:
        self.logger.logdebug( 'Cannot find node class: {0}'.format(identifier, spec['name']) )
        continue
      nodes[identifier] = nodeclass(identifier, spec['title'])
      # Populate the graph
      children = []
      if spec.has_key('child'):
        children.append(spec['child'])
      elif spec.has_key('children'):
        children = spec['children']
      else:
        graph.add_node(identifier)
      for child in children:
        graph.add_edge(identifier, child)
      # Add properties and class
      graph.node[identifier] = spec['properties']
      graph.node[identifier]['children'] = list(children)
    self.root = data['root']
    if len(graph) == 0:
      return False
    if not nx.is_tree(graph):
      return False
    # All nodes created. Add the children
    for identifier in nodes.keys():
      for child_id in graph.node[identifier]['children']:
        nodes[identifier].add_child( nodes[child_id] )
    # Update identifier
    if data.has_key('id'):
      self.identifier = data['id']
    self.graph = graph
    self.nodes = nodes
    return 
  
  def tick(self):
    self.root.tick()


if __name__ == '__main__':
  import anyjson
  import rospkg
  rospack = rospkg.RosPack()
  json_path  = rospack.get_path('behave') + '/config/simple_example.json'
  with open(json_path, 'r') as f:
    data = anyjson.deserialize(f.read())
  btree = BehaviorTree()
  btree.from_dict(data)
