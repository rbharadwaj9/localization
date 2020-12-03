from Queue import PriorityQueue
import numpy as np
import heapq
import Queue

class Node:
    def __init__(self, pos, cost, prev):
        self.x = pos[0]
        self.y = pos[1]
        self.t = pos[2]
        self.cost = cost
        self.prev = prev

    def __hash__(self):
        return hash((self.x, self.y, self.t))

    def __eq__(self, other):
        # return (abs(self.x - other.x) <= 0.05 and abs(self.y - other.y) <= 0.05 and abs(self.t - other.t) <= np.pi/4)
        return (round(self.x, 2) == round(other.x, 2)) and (round(self.y, 2) == round(other.y, 2)) and (round(self.t, 2) == round(other.t, 2))

    def _check_t_bounded(self):
        if (self.t > np.pi or self.t < -np.pi):
            return False
        return True

    @property
    def pos(self):
        return [self.x, self.y, self.t]


class AStarSearch:

    def __init__(self, start, goal, mode, step):
        self.start = Node(start, 0, None)
        self.goal = Node(goal, None, None)
        self.open_list = PriorityQueue()
        self.closed_list = set() 
        self.fourconn_not_eightconn = mode
        self.step = step

        self.solution = []
        self.open_list.put((0, self.start))

    def _action_cost(self, p1, p2):
        return np.sqrt(((p1[0]-p2[0]) ** 2) + ((p1[1] - p2[1]) ** 2) + \
                min(abs(p1[2] - p2[2]), 2*np.pi - abs(p1[2] - p2[2])) ** 2)

    def _goal_cost(self, p1):
        return self._action_cost(p1, self.goal.pos)

    def _heuristic_function(self, current_cost, next):
        return current_cost + self._goal_cost(next)

    def _find_next_nodes(self, node):
        node = node.pos
        n1 = np.array([node[0] + self.step, node[1], node[2]])
        n2 = np.array([node[0], node[1] + self.step, node[2]])
        n3 = np.array([node[0] - self.step, node[1], node[2]])
        n4 = np.array([node[0], node[1] - self.step, node[2]])
        n5 = np.array([node[0], node[1], node[2] + np.pi/2])
        n6 = np.array([node[0], node[1], node[2] - np.pi/2])

        if n5[2] < -np.pi:
            n5[2] += 2*np.pi
        elif n5[2] > np.pi:
            n5[2] -= 2*np.pi

        if n6[2] < -np.pi:
            n6[2] += 2*np.pi
        elif n6[2] > np.pi:
            n6[2] -= 2*np.pi

        if self.fourconn_not_eightconn:
            return [n1, n2, n3, n4, n5, n6]

        n7 = np.array([node[0] + self.step, node[1] + self.step, node[2]])
        n8 = np.array([node[0] + self.step, node[1] - self.step, node[2]])
        n9 = np.array([node[0] - self.step, node[1] + self.step, node[2]])
        n10 = np.array([node[0] - self.step, node[1] - self.step, node[2]])
        n11 = np.array([node[0], node[1], node[2] + np.pi/4])
        n12 = np.array([node[0], node[1], node[2] + 3*np.pi/4])
        n13 = np.array([node[0], node[1], node[2] - np.pi/4])
        n14 = np.array([node[0], node[1], node[2] - 3*np.pi/4])


        for n in [n11, n12, n13, n14]:
            if n[2] < -np.pi:
                n[2] += 2*np.pi
            elif n[2] > np.pi:
                n[2] -= 2*np.pi

        return [n1, n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14]

    def backtrace(self):
        curr = self.goal
        while curr is not self.start:
            self.solution.append(curr.pos)
            curr = curr.prev
        self.solution.reverse()

    def search(self, env, robot):
        globalctr = 0
        counter = 0
        while not self.open_list.empty():
            current = self.open_list.get()[1]

            if counter == 1000:
                print "Distance to Goal: ", self._goal_cost(current.pos)
                print "Size: ", self.open_list.qsize()
                counter = 0
            if current == self.goal:
                print "Solution Found"
                self.goal.prev = current
                self.goal.cost = current.cost + self._action_cost(current.pos, self.goal.pos)
                return self.goal

            for node in self._find_next_nodes(current):
                node_object = Node(node, current.cost + self._action_cost(current.pos, node), current)
                if not node_object._check_t_bounded():
                    import pdb; pdb.set_trace()
                robot.SetActiveDOFValues(node)
                if env.CheckCollision(robot):
                    self.closed_list.add(node_object)
                    continue

                if node_object in self.closed_list:
                    continue

                hcost = self._heuristic_function(current.cost + self._action_cost(current.pos, node), node)
                self.open_list.put((hcost, node_object))

                self.closed_list.add(node_object)
            counter += 1
            globalctr += 1
        raise Exception("No Solution Found")

