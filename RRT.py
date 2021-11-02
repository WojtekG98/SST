#!/usr/bin/env python

import matplotlib.pyplot as plt
import Plan
import math
import random
from enum import Enum

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    from os.path import abspath, dirname, join
    import sys

    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), "py-bindings"))
    from ompl import base as ob
    from ompl import geometric as og


class Node:
    """
    RRT_Connect Node
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.path = []

    def __eq__(self, other):
        return self.position.getX() == other.position.getX() and self.position.getY() == other.position.getY()

    def __str__(self):
        return str(self.position.getX()) + ", " + str(self.position.getY()) + ", " + \
               str(self.position.getYaw() * 180 / math.pi)


def distance(pos1, pos2):
    return math.sqrt((pos1.getX() - pos2.getX()) ** 2 + (pos1.getY() - pos2.getY()) ** 2)


class GrowState(Enum):
    Trapped = 0
    Advanced = 1
    Reached = 2


# noinspection PyPep8Naming
class RRT(ob.Planner):

    def __init__(self, si):
        super(RRT, self).__init__(si, "RRT_Connect")
        self.tree = []  # tree starting from start node
        self.dmax = 5
        self.states_ = []
        self.sampler_ = si.allocStateSampler()

    def expand(self, Tree, goal):
        si = self.getSpaceInformation()
        # new random node
        random_state = si.allocState()
        self.sampler_.sampleUniform(random_state)
        # find nearest node
        nearest_node = self.near(random_state, Tree)
        # find new node based on step size
        new_node_xy = self.step(nearest_node, random_state)
        if new_node_xy is None:
            return GrowState.Trapped
        new_node_position = si.allocState()
        new_node_position.setXY(new_node_xy[0], new_node_xy[1])
        new_node_position.setYaw(new_node_xy[2])
        # connect the random node with its nearest node
        new_node = Node(nearest_node, new_node_position)
        if si.checkMotion(nearest_node.position, new_node.position):
            Tree.append(new_node)
            if si.distance(goal, Tree[-1].position) < self.dmax and si.checkMotion(Tree[-1].position, goal):
                Tree.append(Node(Tree[-1], goal))
                return GrowState.Reached
            else:
                return GrowState.Advanced
        else:
            return GrowState.Trapped

    def near(self, random_state, Tree):
        si = self.getSpaceInformation()
        dlist = [si.distance(node.position, random_state) for node in Tree]
        return Tree[dlist.index(min(dlist))]

    def step(self, nearest_node, random_state):
        si = self.getSpaceInformation()
        d = si.distance(nearest_node.position, random_state)
        if d > self.dmax:
            (xnear, ynear) = (nearest_node.position.getX(), nearest_node.position.getY())
            (xrand, yrand) = (random_state.getX(), random_state.getY())
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (xnear + self.dmax * math.cos(theta), ynear + self.dmax * math.sin(theta))
            return x, y, theta

    def solve(self, ptc):
        pdef = self.getProblemDefinition()
        si = self.getSpaceInformation()
        pi = self.getPlannerInputStates()
        goal = pdef.getGoal()
        st = pi.nextStart()
        while st:
            self.states_.append(st)
            st = pi.nextStart()
        start_state = pdef.getStartState(0)
        goal_state = goal.getState()
        self.tree.append(Node(None, start_state))
        solution = None
        approxsol = 0
        approxdif = 1e6
        while not ptc():
            if self.expand(self.tree, goal_state) == GrowState.Reached:
                current = self.tree[-1]
                path = []
                while current is not None:
                    path.append(current.position)
                    current = current.parent
                for i in range(1, len(path)):
                    self.states_.append(path[len(path) - i - 1])
                solution = len(self.states_)
                break
        solved = False
        approximate = False
        if not solution:
            solution = approxsol
            approximate = True
        if solution:
            path = og.PathGeometric(si)
            for s in self.states_[:solution]:
                path.append(s)
            pdef.addSolutionPath(path)
            solved = True
        return ob.PlannerStatus(solved, approximate)

    def clear(self):
        super(RRT, self).clear()
        self.states_ = []


def isStateValid(state):
    return True


def plan():
    N = 50
    # create an ReedsShepp State space
    space = ob.ReedsSheppStateSpace(2)
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    space.setBounds(bounds)
    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    start = ob.State(space)
    start[0], start[1] = random.randint(0, N), random.randint(0, N)
    goal = ob.State(space)
    goal[0], goal[1] = random.randint(0, N), random.randint(0, N)
    ss.setStartAndGoalStates(start, goal)
    # set the planner
    planner = RRT(ss.getSpaceInformation())
    ss.setPlanner(planner)

    result = ss.solve(600.0)
    if result:
        if result.getStatus() == ob.PlannerStatus.APPROXIMATE_SOLUTION:
            print("Solution is approximate")
        # try to shorten the path
        # ss.simplifySolution()
        # print the simplified path
        path = ss.getSolutionPath()
        path.interpolate(100)
        # print(path.printAsMatrix())
        path = path.printAsMatrix()
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        Plan.plot_path(path, 'b-', -1, N + 1)
        plt.show()


if __name__ == "__main__":
    plan()
