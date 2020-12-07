import os, pickle
from PIL import ImageDraw, ImageFont
from procedural_generation import IslandGenerator
# from quadtrees import QuadTree
from collections import deque
import numpy as np
import math

class QuadTreeOptimizer:


    def __init__(self, map_):
        self.map = map_
        self.size = len(self.map)

        # Finds neighboring nodes
        self.x_number = (-1, 1, 0, 0)
        self.y_number = (0, 0, -1, 1)

        # self.x_number = [-1, -1, -1, 0, 0, 1, 1, 1]
        # self.y_number = [-1, 0, 1, -1, 1, -1, 0, 1]

    def _closest_power_of_2(self, x):
        return 1<<(x-1).bit_length()

    def _assign_identifiers_to_islands(self):

        # Assign a number to each unique island and ostore perimeter
        which_island = {} # Keeps track of which island an (x, y) pair is part of
        visited = set()
        queue = deque()

        num_islands = 0
        for y in range(self.size):
            for x in range(self.size):
                if (x, y) not in visited and self.map[y][x]:
                    queue.append((x, y))
                    self._BFS(queue, visited, which_island, num_islands)
                    num_islands += 1

        return which_island, num_islands

    def get_choke_points(self, max_distance=np.inf):
        """
        """
        max_distance = math.pow(max_distance, 2)
        which_island, num_islands = self._assign_identifiers_to_islands()

        choke_points = []

        for loc_1, identifier_1 in which_island.items():
            for loc_2, identifier_2 in which_island.items():
                if identifier_1 == identifier_2:
                    continue
                distance = self._get_distance_squared(loc_1, loc_2)
                if distance > max_distance:
                    continue
                choke_points.append((loc_1, loc_2, distance))

        # Sort choke points by distance
        choke_points.sort(key = lambda pairs: pairs[2])
        return num_islands, choke_points, which_island

    def get_min_distance(self):

        # Now find the smallest distance between two locations in 
        # which_islands that have different identifiers

        which_island, num_islands = self._assign_identifiers_to_islands()

        min_distance = np.inf
        closest_locations = None

        for loc_1, identifier_1 in which_island.items():
            for loc_2, identifier_2 in which_island.items():
                if identifier_1 == identifier_2:
                    continue
                distance = self._get_distance_squared(loc_1, loc_2)
                if distance < min_distance:
                    min_distance = distance
                    closest_locations = (loc_1, loc_2) 

        return num_islands, math.sqrt(min_distance), closest_locations, which_island

    def _get_distance_squared(self, p1, p2):
        """ Returns squared euclidean distance between two points
        """
        x1, y1 = p1
        x2, y2 = p2
        return math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2)

    def _BFS(self, queue, visited, which_island, identifier):
        while queue:
            x, y = queue.popleft()

            # if x < 0 or y < 0 or x >= self.size or y >= self.size or (x, y) in visited or not self.map[y][x]:
                # continue
            if (x, y) in visited or not self.map[y][x]:
                continue

            neighbors = self._get_neighbors(x, y)
            if self._is_perimeter(x, y):
                which_island[(x, y)] = identifier # Assign identifier to location if its a perimeter node

            visited.add((x, y))
            queue.extend(neighbors)

    def _is_perimeter(self, x, y):
        """ Returns true if any neighbor is water
        """
        neighbors = self._get_neighbors(x, y)
        for x, y in neighbors:
            if not self.map[y][x]:
                return True
        return False

    def _get_neighbors(self, x, y):
        neighbors = []
        for i in range(4):
            new_x, new_y = x + self.x_number[i], y + self.y_number[i]

            if new_x >= 0 and new_x < self.size and new_y >= 0 and new_y < self.size:
                neighbors.append((new_x, new_y))
        return neighbors

if __name__=="__main__":
    if os.path.isfile('ig.pkl'):
        ig = pickle.load(open('ig.pkl','rb'))
    else:
       ig = IslandGenerator(512 ,.07)
       pickle.dump(ig,open('ig.pkl','wb+'))

    qto = QuadTreeOptimizer(ig.map)

    process = 'min' # choke or min

    if process == 'min':
        num_islands, min_distance, closest_locations, which_island = qto.get_min_distance()
        x1, y1 = closest_locations[0]
        x2, y2 = closest_locations[1]

        print ('Num Islands: {}'.format(num_islands))
        print ('Min Distance: {}'.format(min_distance))
        print ('Num Perimeter Locations: {}'.format(len(which_island)))
        print ('Closest Points: ')
        print ('\t {}, {}: {}'.format(x1, y1, which_island[x1, y1]))
        print ('\t {}, {}: {}'.format(x2, y2, which_island[x2, y2]))
        
        # Draw Boarders and closest distance
        pixels = ig.img.load()
        for x, y in which_island.keys():
            pixels[x, y] = (0, 0, 0)

        img_draw = ImageDraw.Draw(ig.img)
        img_draw.line(closest_locations, fill='black', width=4)

        font = ImageFont.truetype("arial.ttf", 14, encoding="unic")
        text_loc = ((x1 + x2) / 2 - 30, (y1 + y2) / 2 - 30)
        img_draw.text(text_loc, '{} Pixels'.format(round(min_distance, 2)))
        ig.img.show()

    elif process == 'choke':
        points_to_show = 10
        num_islands, choke_points, which_island = qto.get_choke_points(max_distance=50)

        # Draw Boarders and closest distance
        pixels = ig.img.load()
        for x, y in which_island.keys():
            pixels[x, y] = (0, 0, 0)

        img_draw = ImageDraw.Draw(ig.img)

        font = ImageFont.truetype("arial.ttf", 14, encoding="unic")
        print ('Num Islands: {}'.format(num_islands))
        print ('Num Perimeter Locations: {}'.format(len(which_island)))
        print ('Top {} Choke Points:'.format(points_to_show))
        for i in range(min(points_to_show, len(choke_points))):
            (x1, y1), (x2, y2), distance = choke_points[i]
            distance = math.sqrt(distance)
            print ('\t ({}, {}) -> ({}, {}), {} pixels'.format(x1, y1, x2, y2, distance))
            img_draw.line(((x1, y1), (x2, y2)), fill='black', width=4)

            text_loc = ((x1 + x2) / 2 - 30, (y1 + y2) / 2 - 30)
            img_draw.text(text_loc, '{} Pixels'.format(round(distance, 2)))

        ig.img.show()