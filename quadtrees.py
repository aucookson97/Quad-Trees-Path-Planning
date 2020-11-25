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
import pickle
import os

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

        elif not np.any(world_slice):
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
        x = ImageDraw.Draw(img)
        self._show_tree(x, self.root)
        img.show()
        return x
        
    def _show_tree(self, img, node):
        if not node:
            return

        if node.pos is not None:
            img.point(node.pos, fill='black')
            n_neighbor = self.find_neighbors(node, self.Direction.N)
            if len(n_neighbor)==0:
                img.point(node.pos,fill='pink')
            for n in n_neighbor:

                if n is not None and n.pos is not None:
                    img.line([node.pos, n.pos],fill='yellow')
        rect = (node.top_left[1], node.top_left[0], node.top_left[1] + node.size, node.top_left[0] + node.size)
        img.rectangle(rect, fill=None, outline="red")

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
        assert node != None,'SOmething wrong'
        if direction == self.Direction.N:
            if node.is_root: #node is root
                return None
            if node.parent.sw == node:
                return node.parent.nw
            if node.parent.se == node:
                return node.parent.ne

            temp = self.get_eq_or_greater_size_neighbors(node.parent, direction)

            if temp is None:
                return None
            elif temp.is_leaf:
                return temp
            elif node.parent.nw == node:
                return temp.sw
            elif node.parent.ne == node:
                return temp.se

        #TODO: same thing for other directions
    def check_smaller_neighbors(self, neighbor, direction):
        '''
        Check for the smaller neighbors in the specified direction
        :param neighbor: neighbor that is greater than or equal to the source node
        :param direction: direction that the neighbor is at
        :return: iterable of smaller neighbors that are in the specified direction
        '''

        candidates = [] if neighbor is None else [neighbor]
        neighbors = []
        if direction == self.Direction.N:
            while candidates:
                chosen = candidates[0]
                if chosen.is_leaf:
                    neighbors.append(chosen)
                else:
                    if chosen.sw is not None:
                        candidates.append(chosen.sw)
                    if chosen.se is not None:
                        candidates.append(chosen.se)
                candidates.remove(chosen)
        return neighbors
        #TODO: do it for other directions

    def find_neighbors(self, node, direction):
        '''
        Finds all the neighbors adjacent to node in specified direction
        :param node: source node of which we are trying to find its neighbors
        :param direction: direction of which to find its neighbors
        :return: iterable of neighbors in specified direction
        '''
        neighbor = self.get_eq_or_greater_size_neighbors(node,direction)

        neighbors = self.check_smaller_neighbors(neighbor,direction)
        return neighbors

if __name__ == "__main__":
    if os.path.isfile('ig.pkl'):
        ig = pickle.load(open('ig.pkl','rb'))
    else:
        ig = IslandGenerator(512 ,.1)
        pickle.dump(ig,open('ig.pkl','wb+'))
    #ig.show_map()

    tree = QuadTree(ig.map)
    img = tree.show_tree(ig.img)

    #for pre, fill, node in RenderTree(tree.root):
    #    print(node.pos)

    print('{} nodes in the tree'.format(next(count)))