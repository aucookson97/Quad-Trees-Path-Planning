# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 20:27:02 2020

@author: aidan
"""
import os
# os.chdir(r'C:\Users\chang\OneDrive\Desktop\209 AS robos\Project\Quad-Trees-Path-Planning')

import numpy as np
from anytree import NodeMixin, RenderTree
from minimum_resolution_quadtree import QuadTreeOptimizer
from procedural_generation import IslandGenerator
from PIL import Image, ImageDraw, ImageFont
from enum import Enum
import pickle

MIN_RESOLUTION = 8    # Smallest Node allowed

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

        self.is_valid = True # Keeps track of obstacles
        
        self.Astar_parent = None
        # costs for A* algorithm
        self.f = np.Inf
        self.g = np.Inf
        new_size = int(size / 2)
        center = (top_left[1] + new_size, top_left[0] + new_size)

        self.pos = None

        world_slice = world[top_left[0]: top_left[0] + size, top_left[1]: top_left[1] + size]

        if np.min(world_slice) != np.max(world_slice) and size >= MIN_RESOLUTION:
            # (NW, NE, SE, SW)
            for new_top_left in [top_left, (top_left[0], center[0]), (center[1], top_left[1]), (center[1], center[0])]:
                child = Node(name=next(count),
                            world=world,
                            top_left=new_top_left,
                            size=new_size,
                            parent=self)

                # Add a frame of nodes
                # frame = [child]

                # if new_size > MIN_RESOLUTION:
                #     for x in range(new_top_left[1], new_top_left[1] + new_size, MIN_RESOLUTION):
                #         top_left_top = (new_top_left[0], x)
                #         top_left_bottom = (new_top_left[0] + new_size - MIN_RESOLUTION, x)
                #         frame.append(Node(name=next(count), world=world, top_left=top_left_top, size=MIN_RESOLUTION))
                #         frame.append(Node(name=next(count), world=world, top_left=top_left_bottom, size=MIN_RESOLUTION))
                #     for y in range(new_top_left[0] + MIN_RESOLUTION, new_top_left[0] + new_size - MIN_RESOLUTION, MIN_RESOLUTION):
                #         top_left_left = (y, new_top_left[1])
                #         top_left_right = (y, new_top_left[1] + new_size)
                #         frame.append(Node(name=next(count), world=world, top_left=top_left_left, size=MIN_RESOLUTION))
                #         frame.append(Node(name=next(count), world=world, top_left=top_left_right, size=MIN_RESOLUTION))
                # self.children += tuple(frame)
                self.children += (child, )


        elif not np.any(world_slice):
            #heuristic = np.linalg.norm()
            self.pos = np.array(center)

    def reset(self):
        self.Astar_parent = None
        # costs for A* algorithm
        self.f = np.Inf
        self.g = np.Inf


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


    def __init__(self, world, start, goal):
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

        self.commonside_table =[[None, self.Direction.N, self.Direction.W],
                                [self.Direction.N, None, None, self.Direction.E],
                                [self.Direction.W, None, None, self.Direction.S],
                                [None, self.Direction.E, self.Direction.S, None]]

        self.reflect_table = [[self.Direction.SW, self.Direction.SE, self.Direction.NW, self.Direction.NE],
                              [self.Direction.NE, self.Direction.NW, self.Direction.SE, self.Direction.SW],
                              [self.Direction.SW, self.Direction.SE, self.Direction.NW, self.Direction.NE],
                              [self.Direction.NE, self.Direction.NW, self.Direction.SE, self.Direction.SW]]
        
        self.start_node = self.get_closest_node(start)
        self.goal_node = self.get_closest_node(goal)
        # self.prune_heuristic()

    def prune_heuristic(self):
        '''
        Prunes quadtree nodes based on heuristic
        Distance from start and goal nodes
        :return: None
        '''
        cost_threshold = np.sum(np.square(self.goal_node.pos-self.start_node.pos))+np.square(310)
        for _, _, node in RenderTree(self.root):
            if node.pos is not None:
                node_cost = np.sum(np.square(node.pos - self.start_node.pos)) + np.sum(np.square(node.pos - self.goal_node.pos))
                if node_cost >= cost_threshold:
                    node.pos = None

    def get_child_type(self, node):
        '''
        Get the son type of node compared to its parent
        :param node: Node class
        :return: Type of the relationship the node has to its parent
        '''
        direction_num = node.parent.children.index(node)
        return self.Direction(direction_num)

    def adj(self, orth_dir, corner_dir):
        '''
        :param orth_dir: direction of orthogonal (N,E,S,W)
        :param corner_dir: direction of corner
        :return: True or False depending on if the two inputs are adjacent
        '''
        return self.adj_truth_table[orth_dir.value-4][corner_dir.value]

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
        :param dir1: corner direction 1
        :param dir2: corner direction 2
        :return: the cardinal direction both directions share
        '''
        return self.commonside_table[dir1.value][dir2.value]

    def reflect(self, orth_dir, corner_dir):
        '''
        :param orth_dir: direction in N,E,S,W
        :param corner_dir: direction in NW, NE, SW, SE
        :return: corner direction reflected upon orth_dir
        '''
        return self.reflect_table[orth_dir.value-4][corner_dir.value]

    def draw_tree(self, img):
        img_draw = ImageDraw.Draw(img)
        self._draw_tree(img_draw, self.root)
        self._draw_start_and_goal(img, img_draw)
        return img_draw

    def _draw_circle(self, draw, center, radius, color):
        x, y = center
        top_left = (x-radius, y-radius)
        bottom_right = (x+radius, y+radius)
        draw.ellipse((top_left, bottom_right), fill=color)

    def _draw_start_and_goal(self, img, draw):
        # Draw start and goal

        font = ImageFont.truetype("arial.ttf", 14, encoding="unic")
        node_size = int(img.size[0] / 75) # Scale node circle size based on size of image

        self._draw_circle(draw, self.start_node.pos, node_size, '#db09b5')
        w, h = draw.textsize('Start', font=font)
        start_text_pos = (int(self.start_node.pos[0] - w / 2), int(self.start_node.pos[1] - h - 5))
        draw.text(start_text_pos, 'Start', fill='black', font=font) # Anchor is not working, so had to use fontmetrics

        self._draw_circle(draw, self.goal_node.pos, node_size, '#db09b5')
        w, h = draw.textsize('Goal', font=font)
        goal_text_pos = (int(self.goal_node.pos[0] - w / 2), int(self.goal_node.pos[1] - h - 5))
        draw.text(goal_text_pos, 'Goal', fill='black', font=font)
        
    def _draw_tree(self, img, node):
        if not node:
            return
        if node.pos is not None:
            # img.point(node.pos, fill='black')
            # color = 'green' if node.is_valid else 'red'
            # self._draw_circle(img, node.pos, 5, color=color)
            neighbors = []
            neighbors += self.find_neighbors(node, self.Direction.N)
            neighbors += self.find_neighbors(node, self.Direction.W)
            neighbors += self.find_neighbors(node, self.Direction.NW)
            neighbors += self.find_neighbors(node, self.Direction.NE)
            if not len(neighbors):
                img.point(node.pos,fill='pink')
            for n in neighbors:
                if n.pos is not None:
                    img.line((tuple(node.pos), tuple(n.pos)), fill='yellow')
        rect = (node.top_left[1], node.top_left[0], node.top_left[1] + node.size, node.top_left[0] + node.size)
        img.rectangle(rect, fill=None, outline="red")

        for child in node.children:
            self._draw_tree(img, child)
        '''
        self._show_tree(img, node.ne)
        self._show_tree(img, node.sw)
        self._show_tree(img, node.se)'''

    def get_equal_adj_neighbor(self, node, direction):
        '''
        Uses Samet's algorithm to find the greater than or equal sized neighbors
        in specified direction
        :param node: source node
        :param direction: direction to find neighbors in
        :return: Neighbor in specified direction which is greater or equal in size
        '''
        if node.parent is not None and self.adj(direction, self.get_child_type(node)):
            q = self.get_equal_adj_neighbor(node.parent, direction)
        else:
            q = node.parent

        return (q
                if q is None or q.is_leaf
                else q.children[self.reflect(direction, self.get_child_type(node)).value])

    def get_corner_neighbor(self, node, direction):
        '''
        Uses Samet's algorithm to find the greater than or equal sized neighbors
        in specified direction
        :param node: source node
        :param direction: direction to find neighbors in
        :return: Neighbor in specified direction which is greater or equal in size
        '''
        if node.parent is not None and self.get_child_type(node) != self.opquad(direction):
            if self.get_child_type(node) == direction:
                q = self.get_corner_neighbor(node.parent, direction)
            else:
                q = self.get_equal_adj_neighbor(node.parent, self.commonside(self.get_child_type(node), direction))
        else:
            q = node.parent

        return (q
                if q is None or q.is_leaf
                else q.children[self.opquad(self.get_child_type(node)).value])

    def check_smaller_neighbors(self, neighbor, direction):
        '''
        Check for the smaller neighbors in the specified direction
        :param neighbor: neighbor that is greater than or equal to the source node
        :param direction: direction that the neighbor is at
        :return: iterable of smaller neighbors that are in the specified direction
        '''

        candidates = [] if neighbor is None else [neighbor]
        neighbors = []
        while candidates:
            chosen = candidates[0]
            if chosen.is_leaf:
                neighbors.append(chosen)
            else:
                if direction.value >= 4:  # direction is (N, E, S, W)
                    adj_sides = self.adj_truth_table[direction.value-4]
                    op_sides = [self.reflect_table[direction.value-4][i] for i in range(len(adj_sides)) if adj_sides[i]]
                else:  # direction is (NW, NE, SE, SW)
                    op_sides = [self.opquad(direction)]
                for side in op_sides:
                    candidates.append(chosen.children[side.value])
            candidates.remove(chosen)
        return neighbors


    def find_neighbors(self, node, direction):
        '''
        Finds all the neighbors adjacent to node in specified direction
        :param node: source node of which we are trying to find its neighbors
        :param direction: direction of which to find its neighbors
        :return: iterable of neighbors in specified direction
        '''
        if direction.value >= 4:
            neighbor = self.get_equal_adj_neighbor(node, direction)
        else:
            neighbor = self.get_corner_neighbor(node, direction)
        neighbors = self.check_smaller_neighbors(neighbor, direction)
        return neighbors

    def get_closest_node(self, node_position):
    
        lowest_distance = np.Inf
        closest_node = None
        
        for _, _, node in RenderTree(self.root):
            if node.pos is None:
                continue
            
            dist = np.linalg.norm(np.array(node_position) - np.array(node.pos))
            if lowest_distance > dist:
                
                closest_node = node
                lowest_distance = dist
                
        return closest_node
    

if __name__ == "__main__":
    if os.path.isfile('ig.pkl'):
        ig = pickle.load(open('ig.pkl','rb'))
    else:
        ig = IslandGenerator(512 ,.07)
        pickle.dump(ig, open('ig.pkl','wb+'))
    # ig.show_map()

    qto = QuadTreeOptimizer(ig.map)
    
    num_islands, min_distance, closest_locations, which_island = qto.get_min_distance()
    valid_numbers = (1, 2)
    MIN_RESOLUTION = qto._closest_power_of_2(int(min_distance))
    
    tree = QuadTree(ig.map, [33, 95], [496, 81])
    img = tree.draw_tree(ig.img)
    # ig.img.save('map_pic.jpg')
    
    tree.get_closest_node([33, 95])
    
    tree.get_closest_node([496, 81])

    ig.img.show()
    

    print('{} nodes in the tree'.format(next(count)))