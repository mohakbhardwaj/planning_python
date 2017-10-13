#!/usr/bin/env python
import os
import sys
sys.path.insert(0, os.path.abspath('..'))

import collections
class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
    	return self.elements.popleft()


