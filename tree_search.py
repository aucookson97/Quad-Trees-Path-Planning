import os
os.chdir(r'C:\Users\chang\OneDrive\Desktop\209 AS robos\Project\Quad-Trees-Path-Planning')
from PIL import Image, ImageDraw
import pickle
import numpy as np
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
        self.open_list.append(self.start_node)
        
        
        
    def _dist_h(self, node):
        '''
        In defining the cost of a node f = g + h
        This function finds h
        :param node:
        :return: h, the heuristic cost of getting from node to goal node
        '''
        return np.linalg.norm((np.array(node.pos)-np.array(self.goal_node.pos)))

    def _get_cost(self, node):
        '''
        Returns cost of node
        :param node: Tuple of (node, cost)
        :return: cost
        '''
        
        return node.f

    def run(self):
        '''
        Runs the A* algorithm to find optimal path from start to end node
        :return: list of nodes defining path from start to goal node
        '''

        while self.open_list:
            self.open_list.sort(key=self._get_cost, reverse = True)
            current_node = self.open_list.pop()

            self.closed_list.add(current_node)

            if current_node == self.goal_node:
                break
            
            for direction in self.quadtree.Direction:
                neighbors = self.quadtree.find_neighbors(current_node, direction)
                for neigh in neighbors:
    
                    if neigh in self.closed_list or neigh.pos is None:
                        continue
                   
                    if neigh.Astar_parent is None:
                       neigh.Astar_parent = current_node   
                    
                    neigh.g = current_node.f + np.linalg.norm(np.array(current_node.pos) - np.array(neigh.pos))
                    neigh.f = neigh.g + self._dist_h(neigh)
                    
                    if neigh in self.open_list:
                        index = self.open_list.index(neigh)
                        self.open_list[index] = neigh
                        
                    else:
                        self.open_list.append(neigh)
                    
             #       print(len(self.open_list))
                    
        path = []
        
        while current_node != self.start_node:
            #print(current_node.pos)
            path.append(current_node)
            current_node = current_node.Astar_parent #quadtree is different from A* parent... 

        path.append(self.start_node)
        return path[::-1]
                    
    
    def motion_plan(self, A_map, path):
        
        for i in range(len(path)-1):
            A_map.line((path[i].pos, path[i+1].pos), fill = "black")
        
            
        return A_map
            
        
if __name__ == "__main__":
    if os.path.isfile('ig.pkl'):
        ig = pickle.load(open('ig.pkl','rb'))
    #else:
       # ig = IslandGenerator(512 ,.07)
       # pickle.dump(ig,open('ig.pkl','wb+'))
    # ig.show_map()
    
    tree = QuadTree(ig.map)
    img = tree.show_tree(ig.img)
    print(ig.img)

    
    Astar = A_star(tree)
    path = Astar.run()
    
    Astar.motion_plan(img, path)
    
    ig.img.show()


    
    

