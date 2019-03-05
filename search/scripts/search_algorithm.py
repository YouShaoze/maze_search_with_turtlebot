#!/usr/bin/env python

import heapq
import problem
import rospy
from std_msgs.msg import String
import argparse
import time

rospy.init_node("search_algo")
publisher = rospy.Publisher("/actions",String,queue_size = 10)
#subscriber = rospy.Subscriber("/status",String,self.callback)

parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)



def bfs():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    goal_reached = False
    queue = [[(init_state,'')]]
    visited = list()

    while queue:
        path = queue.pop(0)
        (vertex,prev_action) = path[-1]
        if problem.is_goal_state(vertex):
            print("Goal Reached!!")
            path = path[1:]
            goal_reached = True
            break
        elif vertex not in visited:
            for next_action in possible_actions:
                (current_neighbour,cost_for_action)=problem.get_successor(vertex, next_action)
                new_path = list(path)
                new_path.append((current_neighbour,next_action))
                queue.append(new_path)
            visited.append(vertex)

    action_list = [actions for nodes,actions in path]
    if len(action_list) == 0 or not goal_reached:
        print("No path found!!")
        return ['']
    end = time.time()
    print("Time for execution of BFS: ",end-start);
    return action_list

def ucs():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    queue = [[(0,init_state,'')]]
    visited = list()
    goal_reached = False

    while queue:
        queue.sort()
        path = queue.pop(0)
        (curr_cost, vertex, prev_action) = path[0]

        if problem.is_goal_state(vertex):
            print("Goal Reached!!")
            path = path[:0:-1]
            goal_reached = True
            break
        elif vertex not in visited:
            for next_action in possible_actions:
                (current_neighbour,cost_for_action)=problem.get_successor(vertex, next_action)
                new_path = list(path)
                new_path.append((curr_cost+cost_for_action,current_neighbour,next_action))
                new_path.sort(reverse=True)
                queue.append(new_path)
            visited.append(vertex)

    action_list = [actions for cost,nodes,actions in path]
    if len(action_list) == 0 or not goal_reached:
        print("No path found!!")
        return ['']
    end = time.time()
    print("Time for execution of UCS: ",end-start);
    return action_list


def gbfs():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    goal_reached = False

    def manhatten_heuristic(state):
        return abs(state .x - goal_state.x) + abs(state.y - goal_state.y)

    def euclidian_heuristic(state):
        return ((state .x - goal_state.x)**2 + (state.y - goal_state.y)**2)**0.5

    stack = [(euclidian_heuristic(init_state),init_state, [(init_state,'')])]

    visited = list()
    while stack:
        stack.sort()
        (heuristic,vertex, path) = stack.pop(0)
        if vertex not in visited:
            if problem.is_goal_state(vertex):
                print("Goal Reached!!")
                goal_reached = True
                path = path[1:]
                break
            visited.append(vertex)
            for next_action in possible_actions:
                (current_neighbour,cost_for_action)=problem.get_successor(vertex, next_action)
                f_of_n = manhatten_heuristic(current_neighbour)
                stack.append((f_of_n,current_neighbour, path + [(current_neighbour,next_action)]))

    action_list = [actions for nodes,actions in path]
    if len(action_list) == 0 or not goal_reached:
        print("No path found!!")
        return ['']
    end = time.time()
    print("Time for execution of GBFS: ",end-start);
    return action_list

def astar():
    start = time.time()
    init_state = problem.get_initial_state()
    goal_state = problem.get_goal_state()
    possible_actions = problem.get_actions()
    action_list = []
    goal_reached = False

    def manhatten_heuristic(state):
        return abs(state .x - goal_state.x) + abs(state.y - goal_state.y)

    def euclidian_heuristic(state):
        return ((state .x - goal_state.x)**2 + (state.y - goal_state.y)**2)**0.5

    stack = [(manhatten_heuristic(init_state)+0,init_state, [(init_state,'')])]

    visited = list()
    while stack:
        stack.sort()
        (heuristic,vertex, path) = stack.pop(0)
        if vertex not in visited:
            if problem.is_goal_state(vertex):
                print("Goal Reached!!")
                path = path[1:]
                goal_reached = True
                break
            visited.append(vertex)
            for next_action in possible_actions:
                (current_neighbour,cost_for_action)=problem.get_successor(vertex, next_action)
                f_of_n = cost_for_action + heuristic - euclidian_heuristic(vertex) + euclidian_heuristic(current_neighbour)
                stack.append((f_of_n,current_neighbour, path + [(current_neighbour,next_action)]))

    action_list = [actions for nodes,actions in path]
    if len(action_list) == 0 or not goal_reached:
        print("No path found!!")
        return ['']
    end = time.time()
    print("Time for execution of A*: ",end-start);
    return action_list

 # to execute a plan action_list = <list of actions>, use:
def exec_action_list(action_list):
    plan_str = '_'.join(action for action in action_list)
    publisher.publish(String(data = plan_str))

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    actions = algorithm()
    exec_action_list(actions)
