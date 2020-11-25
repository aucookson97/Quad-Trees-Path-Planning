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
        E = 5
        S = 6
        W = 7


    def __init__(self, world):

        self.world = world

        self.root = Node(name=next(count),
                         world = world,
                         top_left = (0, 0),
                         size = self.world.shape[0],
                         parent=None)
        self.adj_truth_table = [[True, True, False, False],
                               [False, True, False, True],
                               [False, False, True, True],
                               [True, False, True, False]]

        self.commonside_table = [[None, self.Direction.N, self.Direction.W],
                                       [self.Direction.N, None, None, self.Direction.E],
                                       [self.Direction.W, None, None, self.Direction.S],
                                       [None, self.Direction.E, self.Direction.S, None]]

        self.reflect_table = [[self.Direction.SW, self.Direction.SE, self.Direction.NW, self.Direction.NE],
                              [self.Direction.NE, self.Direction.NW, self.Direction.SE, self.Direction.SW],
                              [self.Direction.SW, self.Direction.SE, self.Direction.NW, self.Direction.NE],
                              [self.Direction.NE, self.Direction.NW, self.Direction.SE, self.Direction.SW]]
    def sontype(self, node):
        '''
        Get the son type of node compared to its parent
        :param node: Node class
        :return: Type of the relationship the node has to its parent
        '''
        if node.parent.nw == node:
            return self.Direction.NW
        if node.parent.sw == node:
            return self.Direction.SW
        if node.parent.ne == node:
            return self.Direction.NE
        if node.parent.se == node:
            return self.Direction.SE

    def adj(self, orth_dir, corner_dir):
        '''
        :param orth_dir: direction of orthogonal (N,E,S,W)
        :param corner_dir: direction of corner
        :return: True or False depending on if the two inputs are adjacent
        '''
        return self.adj_truth_table[orth_dir-4, corner_dir]

    def opquad(self, direction):
        '''
        :param direction: direction
        :return: opposite of direction
        '''
        if direction == self.Direction.NW:
            return self.Direction.SE
        if direction == self.Direction.SW:
            return self.Direction.NE
        if direction == self.Direction.NE:
            return self.Direction.SW
        if direction == self.Direction.SE:
            return self.Direction.NW

    def commonside(self, dir1, dir2):
        '''
        :param dir1: direction 1
        :param dir2: direction 2
        :return: the cardinal direction both directions share
        '''
        return self.commonside_table[dir1, dir2]

    def reflect(self, orth_dir, corner_dir):
        '''
        :param orth_dir: direction in N,E,S,W
        :param corner_dir: direction in NW, NE, SW, SE
        :return: corner direction reflected upon orth_dir
        '''
        return self.reflect_table[orth_dir-4,corner_dir]
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
            neighbors = []

            neighbors+=self.find_neighbors(node, self.Direction.N)
            neighbors+=self.find_neighbors(node, self.Direction.W)
            #neighbors+=self.find_neighbors(node, self.Direction.NW)

            if not len(neighbors):
                img.point(node.pos,fill='pink')
            for n in neighbors:
                if n is not None and n.pos is not None:
                    img.line([node.pos, n.pos], fill='yellow')
        rect = (node.top_left[1], node.top_left[0], node.top_left[1] + node.size, node.top_left[0] + node.size)
        img.rectangle(rect, fill=None, outline="red")

        self._show_tree(img, node.nw)
        self._show_tree(img, node.ne)
        self._show_tree(img, node.sw)
        self._show_tree(img, node.se)

    def gtequal_adj_neighbor(self, node, direction):
        '''
        Uses Samet's algorithm to find the greater than or equal sized neighbors
        in specified direction
        :param node: source node
        :param direction: direction to find neighbors in
        :return: Neighbor in specified direction which is greater or equal in size
        '''
        if node.parent is not None and self.adj(direction, self.sontype(node)):
            q = self.gtequal_adj_neighbor(node.parent, direction)
        else:
            q = node.parent

        return (q.children[self.reflect(direction, self.sontype(node))]
                if q is not None and not q.is_leaf
                else q)
    def get_eq_or_greater_size_neighbors(self,node,direction):
        '''
        MARKED FOR DELETE going to replaced by self.gtequal_adj_neighbor
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

            if temp is None or temp.is_leaf:
                return temp
            elif node.parent.nw == node:
                return temp.sw
            elif node.parent.ne == node:
                return temp.se

        elif direction == self.Direction.W:
            if node.is_root: #node is root
                return None
            if node.parent.ne == node:
                return node.parent.nw
            if node.parent.se == node:
                return node.parent.sw

            temp = self.get_eq_or_greater_size_neighbors(node.parent, direction)

            if temp is None or temp.is_leaf:
                return temp
            elif node.parent.nw == node:
                return temp.ne
            elif node.parent.sw == node:
                return temp.se

        elif direction == self.Direction.NW:
            if node.is_root:
                return None
            if node.parent.se == node:
                return node.parent

            if node.parent.nw == node:
                temp = self.get_eq_or_greater_size_neighbors(node.parent, direction)
            else:
                common_side = self.Direction.N if node.parent.ne == self else self.Direction.S
                temp = self.get_eq_or_greater_size_neighbors(node.parent, common_side)

            if temp is None:
                return None
            elif temp.is_leaf:
                return temp
            elif node.parent.se == node:
                return temp.nw

    def check_smaller_neighbors(self, neighbor, direction):
        '''
        MARKED FOR REWORK going to use the helper functions adj, reflect, opquad, commonside
        to make code look cleaner
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

        if direction == self.Direction.W:
            while candidates:
                chosen = candidates[0]
                if chosen.is_leaf:
                    neighbors.append(chosen)
                else:
                    if chosen.ne is not None:
                        candidates.append(chosen.ne)
                    if chosen.se is not None:
                        candidates.append(chosen.se)
                candidates.remove(chosen)

        if direction == self.Direction.NW:
            while candidates:
                chosen = candidates[0]
                if chosen.is_leaf:
                    neighbors.append(chosen)
                else:
                    if chosen.se is not None:
                        candidates.append(chosen.se)
        return neighbors


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
    ig.show_map()

    tree = QuadTree(ig.map)
    img = tree.show_tree(ig.img)



    print('{} nodes in the tree'.format(next(count)))