import numpy as np
import heapq


class Dstarlite:

    def __init__(self, tree):
        self.quadtree = tree
        self.goal_node = tree.goal_node
        self.goal_node.rhs = 0
        self.next_position = tree.start_node
        self.s_last = self.next_position
        self.k_m = 0

        self.queue = [(*self.calculate_key(self.goal_node), self.goal_node)]
        print(self.queue)
        heapq.heapify(self.queue)
        self.compute_shortest_path()

    def calculate_rhs(self, node):
        min_neigh = None
        neigh_cost = np.Inf
        for neigh in self.quadtree.find_all_direction_neighbors(node):
            if neigh.pos is not None:
                temp_cost = neigh.g + self.calc_cost(node, neigh)
                if temp_cost < neigh_cost:
                    neigh_cost = temp_cost
        return neigh_cost

    def calculate_min_cost_neigh(self, node):
        min_neigh = None
        neigh_cost = np.Inf
        for neigh in self.quadtree.find_all_direction_neighbors(node):
            if neigh.g + self.calc_cost(node, neigh) < neigh_cost:
                neigh_cost = neigh.g + self.calc_cost(node, neigh)
                min_neigh = neigh
        return min_neigh

    def calculate_key(self, node):
        return (min([node.g, node.rhs]) + self.calc_dist(self.next_position, node) + self.k_m, min([node.g, node.rhs]))

    def calc_dist(self, node1, node2):
        dist = np.sqrt(np.square(node1.pos[0]-node2.pos[0]) + np.square(node1.pos[1]-node2.pos[1]))
        return dist

    def calc_cost(self, node, neigh):
        if not neigh.is_valid:
            return np.Inf
        else:
            return self.calc_dist(node, neigh)

    def top_key(self):
        #returns key of highest priority entry in queue
        if len(self.queue) == 0:
            return np.Inf
        else:
            return heapq.nsmallest(1, self.queue)[0][:2]

    def update_vertex(self, node):
        if node != self.goal_node:
            node.rhs = self.calculate_rhs(node)

        self.queue = [x for x in self.queue if x[2] != node]

        if node.g != node.rhs:
            heapq.heappush(self.queue, (*self.calculate_key(node), node))

    def compute_shortest_path(self):

        while self.top_key() < self.calculate_key(self.next_position) or self.next_position.rhs != self.next_position.g:
            priority_node = heapq.heappop(self.queue)

            k_old = priority_node[:2]
            u = priority_node[-1]
            k_new = self.calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.queue, (*k_new, u))
            elif u.g > u.rhs:
                u.g = u.rhs
                for neigh in self.quadtree.find_all_direction_neighbors(u):
                    self.update_vertex(neigh)
            else:
                u.g = np.Inf
                check_nodes = self.quadtree.find_all_direction_neighbors(u) + [u]
                for v in check_nodes:
                    self.update_vertex(v)

    def next_move(self, curr_node, obstacles):
        if curr_node.g == np.Inf:
            return None
        next_node = self.calculate_min_cost_neigh(curr_node)
        if next_node is None:
            self.next_position = curr_node
        else:
            self.next_position = next_node

        self.k_m += self.calc_dist(self.s_last, self.next_position)
        self.s_last = self.next_position

        for obstacle in obstacles:
            changed_nodes = self.quadtree.find_all_direction_neighbors(obstacle.current_node)
            if obstacle.prev_node is not None:
                changed_nodes += self.quadtree.find_all_direction_neighbors(obstacle.prev_node)
            for node in changed_nodes:
                self.update_vertex(node)



















