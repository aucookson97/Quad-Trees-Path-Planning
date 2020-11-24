# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 20:27:02 2020

@author: aidan
"""

import numpy as np
from anytree import NodeMixin, RenderTree
from procedural_generation import IslandGenerator
from PIL import Image, ImageDraw
from enum import Enum

def counter():
    count = 0
    while True:
        count += 1
        yield str(count)

count = counter()

class Node(NodeMixin):
    def __init__(self, name=None, world=None, top_left=None, size=None, parent=None, children=None):
        super(Node, self).__init__()
        self.name = name
        self.nw = self.ne = self.sw = self.se = None
        self.top_left = top_left
        self.size = size
        new_size = int(size / 2)
        center = (top_left[1] + new_size, top_left[0] + new_size)

        self.pos = None

        world_slice = world[top_left[0]: top_left[0] + size, top_left[1]: top_left[1] + size]

        if np.min(world_slice) != np.max(world_slice) and size >= 8:

            nw_slice = world[top_left[0]: top_left[0] + new_size, top_left[1]: top_left[1] + new_size]
            ne_slice = world[top_left[0]: top_left[0] + new_size, top_left[1] + new_size: top_left[1] + 2 * new_size]
            sw_slice = world[top_left[0] + new_size: top_left[0] + 2 * new_size, top_left[1]: top_left[1] + new_size]
            se_slice = world[top_left[0] + new_size: top_left[0] + 2 * new_size, top_left[1] + new_size: top_left[1] + 2 * new_size]
            if not np.all(nw_slice):
                self.nw = Node(name = next(count),
                               world = world,
                               top_left = top_left,
                               size = new_size,
                               parent = self)

                self.children += (self.nw,)
            if not np.all(ne_slice):
                self.ne = Node(name = next(count),
                               world = world,
                               top_left = (top_left[0], top_left[1] + new_size),
                               size = new_size,
                               parent = self)
                self.children += (self.ne,)
            if not np.all(sw_slice):
                self.sw = Node(name = next(count),
                               world = world,
                               top_left = (top_left[0] + new_size, top_left[1]),
                               size = new_size,
                               parent = self)
                self.children += (self.sw,)
            if not np.all(se_slice):
                self.se = Node(name = next(count),
                               world = world,
                               top_left = (top_left[0] + new_size, top_left[1] + new_size),
                               size = new_size,
                               parent = self)
                self.children += (self.se,)
        elif np.min(world_slice) == np.max(world_slice):
            self.pos = center


class QuadTree:

    class Direction(Enum):
        NW = 0
        NE = 1
        SW = 2
        SE = 3
        N = 4
        S = 5
        W = 6
        E = 7

    def __init__(self, world):

        self.world = world
        self.root = Node(name=next(count),
                         world = world,
                         top_left = (0, 0),
                         size = self.world.shape[0],
                         parent=None)


    def show_tree(self, img):
        
        self._show_tree(ImageDraw.Draw(img), self.root)
        img.show()
        
        
    def _show_tree(self, img, node):
        if not node:
            return
        print(node.parent)
        rect = (node.top_left[1], node.top_left[0], node.top_left[1] + node.size, node.top_left[0] + node.size)
        img.rectangle(rect, fill=None, outline="red")
        if node.pos:
            img.point(node.pos,fill='black')
        self._show_tree(img, node.nw)
        self._show_tree(img, node.ne)
        self._show_tree(img, node.sw)
        self._show_tree(img, node.se)
            
    def get_eq_or_greater_size_neighbors(self,node,direction):
        '''
        Gets the adjacent neighbors which are of equal or greater size in certain direction
        :param direction: direction of which to search for equal or greater size neighbors
        :return: adjacent node in specified direction
        '''
        if direction == self.Direction.N:
            if self.parent is None:
                return None
            if node.parent.sw == node:
                return node.parent.nw
            if node.parent.se == node:
                return node.parent.ne

            temp_node = self.get_eq_or_greater_size_neighbors(node.parent,direction)
            if temp_node is None or temp_node.is_leaf:
                return (temp_node.sw
                        if node.parent.nw == node
                        else temp_node.se)
        #TODO: same thing for other directions

if __name__ == "__main__":
    ig = IslandGenerator(512, .1)
    ig.show_map()
    tree = QuadTree(ig.map)
    tree.show_tree(ig.img)

    for pre, fill, node in RenderTree(tree.root):
        print("%s%s" % (pre, node.pos))

    print('{} nodes in the tree'.format(next(count)))