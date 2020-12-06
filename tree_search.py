import os
# os.chdir(r'C:\Users\chang\OneDrive\Desktop\209 AS robos\Project\Quad-Trees-Path-Planning')
from PIL import Image, ImageDraw
import pickle
import numpy as np
from procedural_generation import IslandGenerator
from quadtrees import QuadTree
from collections import deque

class A_star:
    def __init__(self, quadtree):
        '''
        Initializes the A* algorithm to find the optimal path from start node to goal node in graph
        :param quadtree: QuadTree class instance
        '''
        
        self.open_list = []
        self.closed_list = set()
        self.quadtree = quadtree

        self.start_node = quadtree.start_node
        self.goal_node = quadtree.goal_node

        self.start_node.f = 0
        self.start_node.g = 0
        self.open_list.append(self.start_node)
        

    def _dist_h(self, node):
        '''
        In defining the cost of a node f = g + h
        This function finds h
        :param node: node class from quadtrees.py
        :return: h, the heuristic cost of getting from node to goal node
        '''
        #return abs(node.pos[0] - self.goal_node.pos[0]) + abs(node.pos[1] - self.goal_node.pos[1])
        return np.sqrt(np.square(node.pos[0] - self.goal_node.pos[0]) + np.square(node.pos[1] - self.goal_node.pos[1]))

    def _get_cost(self, node):
        '''
        Returns cost of node
        :param node: node class from quadtrees.py
        :return: cost
        '''
        
        return node.f

    def run(self):
        '''
        Runs the A* algorithm to find optimal path from start to end node
        :return: list of nodes defining path from start to goal node
        '''

        while self.open_list:
            self.open_list.sort(key=lambda node: self._get_cost(node), reverse=True)
            current_node = self.open_list.pop()
            self.closed_list.add(current_node)

            if current_node == self.goal_node:
                print('Path found')
                return self._reconstruct_path(current_node)

            for direction in self.quadtree.Direction:
                neighbors = self.quadtree.find_neighbors(current_node, direction)
                for neigh in neighbors:
    
                    if neigh in self.closed_list or neigh.pos is None:
                        continue

                    #neigh.g = current_node.g + abs(current_node.pos[0] - neigh.pos[0]) + abs(current_node.pos[1] - neigh.pos[1])
                    tentative_g = current_node.g + np.sqrt(np.square(current_node.pos[0] - neigh.pos[0]) + \
                                                           np.square(current_node.pos[1] - neigh.pos[1]))

                    if tentative_g < neigh.g:
                        neigh.Astar_parent = current_node
                        neigh.g = tentative_g
                        neigh.f = neigh.g + self._dist_h(neigh)
                        if neigh not in self.open_list:
                            self.open_list.append(neigh)

                    # if self._add_to_open_list(neigh):
                    #     neigh.Astar_parent = current_node
                    #     self.open_list.append(neigh)

                    # if neigh in self.open_list:
                    #     index = self.open_list.index(neigh)
                    #     if self.open_list[index].f >= neigh.f:
                    #         self.open_list[index] = neigh
                    # else:
                    #     self.open_list.append(neigh)

        return None

    def _reconstruct_path(self, current_node):
        # Reconstruct path backwards
        path = []
        while current_node != self.start_node:
            path.append(current_node)
            current_node = current_node.Astar_parent #quadtree parent is different from A* parent
        path.append(self.start_node)
        return path[::-1]

    def _add_to_open_list(self, node_to_add):
        for node in self.open_list:
            if node_to_add == node and node_to_add.f >= node.f:
                return False
        return True

    def motion_plan(self, A_map, path):
        
        for i in range(len(path)-1):
            A_map.line((tuple(path[i].pos), tuple(path[i+1].pos)), fill="black")
        return A_map


if __name__ == "__main__":
    if os.path.isfile('ig.pkl'):
        ig = pickle.load(open('ig.pkl','rb'))
    else:
       ig = IslandGenerator(512 ,.07)
       pickle.dump(ig,open('ig.pkl','wb+'))
    # ig.show_map()
    
    start = [33, 95]

    # Several good goals for this map
    goal1 = [496, 81]
    goal2 = [480, 225]
    goal3 = [336, 400]
    goal4 = [0,512] #impossible node

    tree = QuadTree(ig.map, start, goal3)

    Astar = A_star(tree)
    path = Astar.run()

    img_draw = tree.draw_tree(ig.img)

    if path is not None:
        Astar.motion_plan(img_draw, path)
    
    ig.img.show()
    # ig.img.save('map1.png')


    
    

