import numpy as np

class A_star:
    def __init__(self, quadtree):
        '''
        Initializes the A* algorithm to find the optimal path from start node to goal node in graph
        :param quadtree: QuadTree class instance
        '''
        self.open_list = []
        self.closed_list = []
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
        return np.linalg.norm((node.pos-self.goal_node.pos))

    def _get_cost(self, node):
        '''
        Returns cost of node
        :param node: Tuple of (node, cost)
        :return: cost
        '''
        return node.f

    def _run(self):
        '''
        Runs the A* algorithm to find optimal path from start to end node
        :return: list of nodes defining path from start to goal node
        '''

        while self.open_list:
            self.open_list.sort(key=self._get_cost)
            current_node = self.open_list.pop()

            self.closed_list.append(current_node)

            if current_node == self.goal_node:
                path = []

                while current_node != self.start_node:
                    path.append(current_node)
                    current_node = current_node.parent

                path.append(self.start_node)
                return path[::-1]

            for direction in self.quadtree.Direction:
                neighbors = self.quadtree.find_neighbors(current_node, direction)
                for neigh in neighbors:
                    if neigh in self.closed_list:
                        continue

                    neigh =