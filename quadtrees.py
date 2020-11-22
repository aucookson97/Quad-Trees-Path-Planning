# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 20:27:02 2020

@author: aidan
"""

import numpy as np
from procedural_generation import IslandGenerator
from PIL import Image, ImageDraw

class Node():
    
    def __init__(self, world, top_left, size):
        self.nw = self.ne = self.sw = self.se = None
        self.top_left = top_left
        self.size = size
        
        world_slice = world[top_left[0]: top_left[0] + size, top_left[1]: top_left[1] + size]
        
        if np.min(world_slice) != np.max(world_slice):
            new_size = int(size / 2)
            
            nw_slice = world_slice = world[top_left[0]: top_left[0] + new_size, top_left[1]: top_left[1] + new_size]
            ne_slice = world_slice = world[top_left[0]: top_left[0] + new_size, top_left[1] + new_size: top_left[1] + 2 * new_size]
            sw_slice = world_slice = world[top_left[0] + new_size: top_left[0] + 2 * new_size, top_left[1]: top_left[1] + new_size]
            se_slice = world_slice = world[top_left[0] + new_size: top_left[0] + 2 * new_size, top_left[1] + new_size: top_left[1] + 2 * new_size]
            if not np.all(nw_slice):
                self.nw = Node(world, top_left, new_size)
            if not np.all(ne_slice):
                self.ne = Node(world, (top_left[0], top_left[1] + new_size), new_size)
            if not np.all(sw_slice):
                self.sw = Node(world, (top_left[0] + new_size, top_left[1]), new_size)
            if not np.all(se_slice):
                self.se = Node(world, (top_left[0] + new_size, top_left[1] + new_size), new_size)
            
    
        
class QuadTree():
    
    def __init__(self, world):
        self.world = world
        self.root = Node(world, (0, 0), self.world.shape[0])
        
    def show_tree(self, img):
        
        self._show_tree(ImageDraw.Draw(img), self.root)
        img.show()
        
        
    def _show_tree(self, img, node):
        if not node:
            return
        
        rect = (node.top_left[1], node.top_left[0], node.top_left[1] + node.size, node.top_left[0] + node.size)
        img.rectangle(rect, fill=None, outline="red")
        
        self._show_tree(img, node.nw)
        self._show_tree(img, node.ne)
        self._show_tree(img, node.sw)
        self._show_tree(img, node.se)
            
    
if __name__ == "__main__":
    ig = IslandGenerator(512, .1)
    ig.show_map()
    tree = QuadTree(ig.map)
    tree.show_tree(ig.img)
        