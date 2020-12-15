from PIL import Image, ImageDraw
import pickle
import numpy as np
from procedural_generation import IslandGenerator
from quadtrees import QuadTree
from collections import deque
import os
import time
from matplotlib import pyplot as plt
from matplotlib import animation
from D_star_lite import Dstarlite

RANGE = 100 # How far the boat can "see"
TIME_STEP_LENGTH = 100 # In milliseconds

FOG_COLOR = (197, 207, 209, 0)
BOAT_COLOR = (255, 255, 255)# (158, 0, 142)

class AStarDynamic:
    def __init__(self, quadtree):
        '''
        Initializes the A* algorithm to find the optimal path from start node to goal node in graph
        :param quadtree: QuadTree class instance
        '''
        
        self.open_list = []
        self.closed_list = set()
        self.quadtree = quadtree

        self.reset_tree()

        self.start_node = quadtree.start_node
        self.goal_node = quadtree.goal_node

        self.start_node.f = 0
        self.start_node.g = 0
        self.open_list.append(self.start_node)

    def reset_tree(self):
        
        def _iterate_tree(node):
            node.reset()
            for child in node.children:
                _iterate_tree(child)
        
        _iterate_tree(self.quadtree.root)

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

                    if neigh in self.closed_list or neigh.pos is None or not neigh.is_valid: # Ignore non-valid obstacle nodes
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

class Obstacle:

    CHANCE_TO_STAY_STILL = 0 # 0-1

    def __init__(self, tree, start_node):
        self.tree = tree
        self.current_node = start_node
        self.current_node.is_valid = False
        self.seen = False
        self.prev_node = None

    def move(self, player_node, goal_node):
        """ Move to one of the neighboring nodes
        """
        if np.random.sample() < self.CHANCE_TO_STAY_STILL:
            return
        all_neighbors = []
        for direction in self.tree.Direction:
            neighbors = self.tree.find_neighbors(self.current_node, direction)
            for neigh in neighbors:
                if neigh.pos is not None and neigh != player_node and neigh != goal_node and neigh.is_valid:
                    all_neighbors.append(neigh)
        if len(all_neighbors) > 0:
            next_node = np.random.choice(all_neighbors)
            self.current_node.is_valid = True
            next_node.is_valid = False
            self.prev_node = self.current_node
            self.current_node = next_node

    def get_pos(self):
        return self.current_node.pos    


class SimulateTravel:

    def __init__(self, ig, start, goal, num_obstacles, fog=False):
        self.fog = fog
        self.tree = QuadTree(ig.map, start, goal)
        self.start_node = self.tree.start_node
        self.goal = goal
        self.goal_node = self.tree.goal_node

        self.fig = plt.figure()

        self.obstacles = []
        for _ in range(num_obstacles):
            valid_pos = False
            while not valid_pos:
                random_pos = np.random.randint(0, ig.img.size[0], 2)
                node = self.tree.get_closest_node(random_pos)
                if node != self.start_node and node != self.goal_node and node.is_valid:
                    valid_pos = True
            self.obstacles.append(Obstacle(self.tree, node))
        self.Dstarlite = Dstarlite(self.tree)
        print('initialized')
        # self.obstacles.append(Obstacle(self.tree, self.tree.get_closest_node((432, 240))))

        # for i in range(len(self.obstacles)):
        #     for j in range(len(self.obstacles)):
        #         if i != j and self.obstacles[i].current_node == self.obstacles[j].current_node:
        #             print ('match')
        self.tree.draw_tree(ig.img)
        self.img = ig.img

        # Fog
        self.mask = Image.new('L', self.img.size, color=0) 
        self.img_draw = ImageDraw.Draw(self.img)
        self.mask_draw = ImageDraw.Draw(self.mask)

        # Initial Frame
        self.window = plt.imshow(self.img)
        img_new = self.img.copy()
        self.draw_boat(img_new, self.start_node.pos, 16, 'white')

        # Draw Obstacles (Cant be combined with above loop)
        for obstacle in self.obstacles:
            if not self.fog or (obstacle.seen or self.fog and (np.square(obstacle.current_node.pos[0] - self.start_node.pos[0]) + \
                                                                np.square(obstacle.current_node.pos[1] - self.start_node.pos[1])) < RANGE**2):
                self.draw_boat(img_new, obstacle.get_pos())

        # Update MATLAB Plot 
        self.window.set_data(np.array(img_new))
        self.fig.canvas.draw_idle()
        plt.pause(10)
        # _ = input('Press Enter to Start.')
        #_ = input('Press Enter to Close.')

        # for i in range(len(path)-1):
        #     A_map.line((tuple(path[i].pos), tuple(path[i+1].pos)), fill="black")
        # return A_map

    def simulate(self):
        step = 1
        current_node = self.start_node
        
        found_goal = False

        full_path = [current_node]
        while not found_goal: 
            print ('Step {}'.format(step))  

            # Move Obstacles
            #self.tree = QuadTree(ig.map, current_node.pos, self.goal)
            for obstacle in self.obstacles:
                obstacle.move(current_node, self.goal_node)

            # Re-run A star with moved obstacles and from the current position
            self.tree.start_node = current_node
            #Astar = AStarDynamic(self.tree)
            print('move')
            self.Dstarlite.next_move(current_node, self.obstacles)
            path = [current_node, self.Dstarlite.next_position]
            #print(self.Dstarlite.start_node)
            #path = Astar.run()

            self.update_mask(current_node.pos)

            # self.is_not_valid_count = 0
            # def _iterate_tree(node):

            #     if not node.is_valid:
            #         self.is_not_valid_count += 1
            #     for child in node.children:
            #         _iterate_tree(child)

            # _iterate_tree(self.tree.root)
            # print (self.is_not_valid_count)

            if path:

                next_node = path[1]
                # Draw path taken and boat
                self.img_draw.line((tuple(next_node.pos), tuple(current_node.pos)), fill="black", width=4)
                img_new = self.img.copy()

                # Draw current path
                for i in range(len(path)-1):
                    ImageDraw.Draw(img_new).line((tuple(path[i].pos), tuple(path[i+1].pos)), fill="black")
                # Update current position and check if we are at the goal state
                current_node = next_node
                full_path.append(current_node)
            else:
                img_new = self.img.copy()
                print('No Path Found')

            self.draw_boat(img_new, current_node.pos, 16, 'white')

            # Draw Obstacles (Cant be combined with above loop)
            for obstacle in self.obstacles:
                if not self.fog or (obstacle.seen or self.fog and (np.square(obstacle.current_node.pos[0] - next_node.pos[0]) + \
                                                                        np.square(obstacle.current_node.pos[1] - next_node.pos[1])) < RANGE**2):
                    obstacle.seen = True
                    self.draw_boat(img_new, obstacle.get_pos())

            # Update MATLAB Plot 
            self.window.set_data(np.array(img_new))
            self.fig.canvas.draw_idle()
            plt.pause(TIME_STEP_LENGTH / 1000)
            found_goal = (current_node == self.goal_node)
            # _= input("press enter")

            step += 1

        return full_path

    def draw_boat(self, img, pos, size=8, color='red'):
        img_draw = ImageDraw.Draw(img)
        top = (pos[0], pos[1] - size * (0.866 / 2))
        left = (pos[0] - size / 2, pos[1] + size * (0.866 / 2))
        right = (pos[0] + size / 2, pos[1] + size * (0.866 / 2))
        img_draw.polygon([top, left, right], fill = color)

    def update_mask(self, center):
        # img.putalpha(255)
        # mask = Image.new('L', self.img.size, color=0)  

        x, y = center
        top_left = (x-RANGE, y-RANGE)
        bottom_right = (x+RANGE, y+RANGE)
        self.mask_draw.ellipse((top_left, bottom_right), fill = 255)

        if self.fog:
            self.img.putalpha(self.mask)

if __name__ == "__main__":
    if os.path.isfile('ig.pkl'):
        ig = pickle.load(open('ig.pkl','rb'))
    else:
       ig = IslandGenerator(512 ,.07)
       pickle.dump(ig,open('ig.pkl','wb+'))
    
    start = [33, 95]

    # Several good goals for this map
    goal1 = [496, 81]
    goal2 = [480, 225]
    goal3 = [336, 400]
    goal4 = [0,512] #impossible node

    dynamic_obs = SimulateTravel(ig, start, goal2, 50, fog=False)
    dynamic_obs.simulate()


    ig.img.save('map1.png')