#!/usr/bin/env python

"""A priority queue that uses heaps and lazy deletes if required. We don't update the priority of an element
already in the queue, but keep a duplicate. Lazy deletes help in taking random actions."""
import copy
import heapq

REMOVED = '<removed-node>' #Placeholder for removed node

class PriorityQueue:
  def __init__(self):
    self.elements = []
    self.entry_finder={} #mapping nodes to entries
    self.curr_len = 0

  def empty(self):
    return self.curr_len == 0
  
  def put(self, curr_node, priority1=0, priority2=0):
    #Add task of update priority of existing task
    if curr_node in self.entry_finder:
      self.remove_task(curr_node)
    entry = [priority1, priority2, curr_node]
    self.entry_finder[curr_node] = entry
    heapq.heappush(self.elements, entry)
    self.curr_len += 1
  
  def remove_task(self, task):
    entry = self.entry_finder.pop(task)
    entry[-1] = REMOVED
    self.curr_len -= 1

  def get(self):
    #Remove and return the lowest priority task
    while self.elements:
      priority1, priority2, curr_node = heapq.heappop(self.elements)
      if curr_node is not REMOVED:
        del self.entry_finder[curr_node]
        self.curr_len -=1
        return priority1, priority2, curr_node
    raise KeyError('pop from an empty priority queue')

  def size(self):
    return self.curr_len

  def pop_idx(self, idx):
    priority1, priority2, curr_node = self.get_idx(idx)
    self.remove_task(curr_node)
    return priority1, priority2, curr_node
          
  def get_idx(self, idx):
    assert idx < self.curr_len, "idx chosen should be less than length of queue"
    priority1, priority2, curr_node = self.elements[idx]
    while curr_node is REMOVED:
      idx += 1
      priority1, priority2, curr_node = self.elements[idx]
    return priority1, priority2, curr_node

  def get_task(self, task):
    assert task in self.entry_finder, "task not in priority queue"
    priority1, priority2, curr_node= self.entry_finder[task]
    return priority1, priority2, curr_node

  def pop_task(self, task):
    assert task in self.entry_finder, "task not in priority queue"
    priority1, priority2, curr_node = self.entry_finder[task]
    self.remove_task(task)
    return priority1, priority2, curr_node

  def clear(self):
    del self.elements[:]
    self.entry_finder.clear()
    self.curr_len = 0
