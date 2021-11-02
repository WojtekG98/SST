from ompl import base as ob
from ompl import geometric as og
import matplotlib.pyplot as plt
from math import sqrt
from math import pi
import sys
import Astar
import RRT
import random
import ciesnina
import prostokat as prostokatfile

cies = 0
prostokat = 1
inna = 0

N = 100.0
dgoal = 50
c_best = 100
def plan(space, planner, runTime, start, goal):
    ss = og.SimpleSetup(space)
    if cies == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(ciesnina.isStateValid))
    if prostokat == 1:
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(prostokatfile.isStateValid))

    ss.setStartAndGoalStates(start, goal)
    if planner == 'RRT':
        ss.setPlanner(RRT.RRT(ss.getSpaceInformation()))
    elif planner.lower() == 'astar':
        ss.setPlanner(Astar.Astar(ss.getSpaceInformation()))
    elif planner.lower() == "rrtconnect":
        ss.setPlanner(og.RRTConnect(ss.getSpaceInformation()))
    elif planner.lower() == "est":
        ss.setPlanner(og.EST(ss.getSpaceInformation()))
    elif planner.lower() == "rrtstar":
        ss.setPlanner(og.RRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "informedrrtstar":
        ss.setPlanner(og.InformedRRTstar(ss.getSpaceInformation()))
    elif planner.lower() == "sst":
        ss.setPlanner(og.SST(ss.getSpaceInformation()))
    elif planner.lower() == "sststar":
        ss.setPlanner(og.SSTstar(ss.getSpaceInformation()))
    else:
        print('Bad planner')
    print(planner, ":")
    if planner.lower() == "informedrrtstar" or planner.lower() == "rrtstar":
        ss.setup()
        OptObj = ss.getOptimizationObjective()
        OptObj.setCostThreshold(c_best)
        ss.setOptimizationObjective(OptObj)
    solved = ss.solve(runTime)
    print(ss.getLastPlanComputationTime())
    if solved:
        ss.simplifySolution()
        path = ss.getSolutionPath()
        #print("Info:    Path length:", path.length())
        # print(path.printAsMatrix())
        path.interpolate(1000)
        if planner.lower() == 'astar':
            return path.length()
        else:
            return path.printAsMatrix()

    else:
        print("No solution found.")
        return None


def print_path_txt(path):
    plt.axis([0, N, 0, N])
    matrix = []
    for line in path.split("\n"):
        tmp = []
        for item in line.split():
            tmp.append(float(item))
        if len(tmp) is not 0:
            matrix.append(list(tmp))
    for item in matrix:
        item[2] = item[2] * 180 / pi
    return matrix


def plot_path(path, style, LowB, HighB):
    plt.axis([LowB, HighB, LowB, HighB])
    matrix = []
    for line in path.split("\n"):
        tmp = []
        for item in line.split():
            tmp.append(float(item))
        if len(tmp) is not 0:
            matrix.append(list(tmp))
    x = []
    y = []
    for item in matrix:
        x.append(item[0])
        y.append(item[1])
    plt.plot(x, y, style)

def paintobs():
    if cies == 1:
        ciesnina.paint_obs(0, N)
    if prostokat == 1:
        prostokatfile.paint_obs(0, N)

def plot_path_to_png(path, style, LowB, HighB, fignum, legend, pathtofile):
    plt.figure(fignum)
    if path:
        plot_path(path, style, LowB, HighB)
        plt.plot(start[0], start[1], 'g*')
        plt.plot(goal[0], goal[1], 'y*')
        paintobs()
        plt.legend(legend)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.savefig(pathtofile)

def set_start_and_goal(start, goal):
    if cies == 1 or prostokat == 1:
        start[0], start[1] = (N-dgoal)/2, N/2
        goal[0], goal[1] = (N+dgoal)/2, N/2
    if inna == 1:
        start[0], start[1] = random.randint(0, N), random.randint(0, N)
        goal[0], goal[1] = random.randint(0, N), random.randint(0, N)
        if cies == 1:
            while not ciesnina.isStateValid2(start):
                start[0], start[1] = random.randint(0, N), random.randint(0, N / 2)
            while not ciesnina.isStateValid2(goal):
                goal[0], goal[1] = random.randint(0, N), random.randint(N / 2, N)
        if prostokat == 1:
            while not prostokatfile.isStateValid2(start):
                start[0], start[1] = random.randint(0, N), random.randint(0, N / 2)
            while not prostokatfile.isStateValid2(goal):
                goal[0], goal[1] = random.randint(0, N), random.randint(N / 2, N)

def badanie(nr, space, start, goal):
    nazwa_pliku = "1.txt"
    if cies == 1:
        nazwa_pliku = "badania/gap/gap5" + str(nr) + ".txt"
    elif prostokat == 1:
        nazwa_pliku = "badania/obs/kwadrat_N" + str(N) + "_dgoal_" + str(dgoal) + "_" + str(nr) + ".txt"
    f = open(nazwa_pliku, 'w')
    sys.stdout = f
    if cies == 1:
        print("GAP\n")
    elif prostokat == 1:
        print("OBSTACLE\n")
        print("l/dgoal =")
        print(N / dgoal)
    rrtstar_path = plan(space, 'rrtstar', 1000, start, goal)
    plot_path_to_png(rrtstar_path, 'g-', 0, N, 1, ('RRT*', 'start', 'goal'), 'figures/path_RRTStar.png')

    informedrrtstar_path = plan(space, 'informedrrtstar', 1000, start, goal)
    plot_path_to_png(informedrrtstar_path, 'm-', 0, N, 2, ('Informed RRT*', 'start', 'goal'),'figures/path_InformedRRTStar.png')

    plt.figure(3)
    paintobs()
    plot_path(rrtstar_path, 'g-', 0, N)
    plot_path(informedrrtstar_path, 'm-', 0, N)
    plt.legend(('RRT*', 'Informed RRT*'))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.plot(start[0], start[1], 'g*')
    plt.plot(goal[0], goal[1], 'y*')
    plt.savefig('figures/paths.png')


if __name__ == '__main__':
    space = ob.ReedsSheppStateSpace(2)
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0)
    bounds.setHigh(N)
    space.setBounds(bounds)
    # Set our robot's starting and goal states to be random
    start, goal = ob.State(space), ob.State(space)
    set_start_and_goal(start, goal)
    print("start: ", start[0], start[1])
    print("goal: ", goal[0], goal[1])
    if prostokat == 1:
        astar_path_length = plan(space, 'astar', 1000, start, goal)
        print(astar_path_length)
        if astar_path_length:
            c_best = 1.02 * astar_path_length
        else:
            c_best = dgoal*dgoal
    else:
        c_best = 150
    for numer in range(10):
        badanie(numer, space, start, goal)
