#! /usr/bin/env python
import criros
import networkx as nx
import uuid

class BehaviorTree(object):
  def __init__(self):
    self.id = str(uuid.uuid1())
    self.title = 'BehaviorTree'
    self.description = ''
    self.graph = nx.Graph()

  def load_json(self, data):
    self.graph.clear()
    # Check the file has the expected keys
    if not criros.utils.has_keys(data, ['nodes','properties','root','title']):
      return False
    # Process the nodes
    for key, node in data['nodes'].items():
      if not criros.utils.has_keys(node, ['id','name','properties','title']):
        continue
      # Add the composites and leafs
      children = []
      if node.has_key('child'):
        children.append(node['child'])
      elif node.has_key('children'):
        children = node['children']
      else:
        self.graph.add_node(key)
      for child in children:
        self.graph.add_edge(key, child)
      # Add attributes
      self.graph.node[key] = node['properties']
    return nx.is_tree(self.graph)

if __name__ == '__main__':
  import anyjson
  import rospkg
  rospack = rospkg.RosPack()
  json_path  = rospack.get_path('ikea_tasks') + '/config/behavior_tree.json'
  with open(json_path, 'r') as f:
    data = anyjson.deserialize(f.read())
  btree = BehaviorTree()
  btree.load_json(data)
