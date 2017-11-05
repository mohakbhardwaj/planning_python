#!/usr/bin/env python
class ExplicitGraph():
    def __init__(self):
        self.edges = {}
        self.ndims = 2

    def neighbors(self, id):
        return self.edges[id]
