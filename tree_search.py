import numpy as np

class A_star:
    def __init__(self,start_node,goal_node):
        self.open_list = []
        self.closed_list = []

        self.start_node = start_node
        self.goal_node = goal_node

        self.open_list.append(self.start_node)

    def _run(self):
        '''
        Runs the A* algorithm to find optimal path from start to end node
        :return: list of nodes defining path from start to goal node
        '''

        while self.open_list:
            self.open_list.sort()
            current_node = self.open_list.pop()

            self.closed_list.append(current_node)

            if current_node == self.goal_node:
                path = []

                while current_node != self.start_node:
                    path.append(current_node)
                    current_node = current_node.parent

                path.append(self.start_node)
                return path[::-1]

            neighbors = current_node.get_neighbors()
            for neigh in neighbors:
                if neigh in self.closed_list:
                    continue

                neigh =