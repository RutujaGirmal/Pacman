# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""


import util


class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """


    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    # initialize initial position, stack and the visited dictionary
    currentPosition = problem.getStartState()
    myQueue = util.Stack()
    myQueue.push((currentPosition, None))
    visitedDict = {}
    while not myQueue.isEmpty():
        currentPosition, parent = myQueue.pop()
        visitedDict[currentPosition] = parent
    # if the position is the goal state position then terminate
        if problem.isGoalState(currentPosition):
            break
        for successor in problem.getSuccessors(currentPosition):
            if successor[0] not in visitedDict:
                myQueue.push((successor[0], (currentPosition, successor[1])))
    path = []
    currentTracingPosition = currentPosition
    # trace the path
    while visitedDict[currentTracingPosition]:
        path.insert(0, visitedDict[currentTracingPosition][1])
        currentTracingPosition = visitedDict[currentTracingPosition][0]
    return path


def breadthFirstSearch(problem):
    # initialize initial position in queue and the explored set

    myQueue = util.Queue()
    myQueue.push((problem.getStartState(), []))
    explored = []
    while not myQueue.isEmpty():
        node = myQueue.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        if node[0] not in explored:
            explored.append(node[0])
            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in explored:
                    # path is the sum of previous action plus the current action
                    path = node[1] + [successor[1]]
                    next_node = (successor[0], path)
                    # add the latest node
                    myQueue.push(next_node)
    return node[1]



def uniformCostSearch(problem):
    # initialize initial position, priority queue and the explored set
    initial_node = (problem.getStartState(), [])
    myPriorityQueue = util.PriorityQueue()
    myPriorityQueue.push(initial_node, 0)
    explored = set()
    emptyList = []
    while not myPriorityQueue.isEmpty():
        node = myPriorityQueue.pop()
        if problem.isGoalState(node[0]):
            return node[1]
        if node[0] not in explored:
            # add an entry to the visited/ explored set
            explored.add(node[0])
            for successor in problem.getSuccessors(node[0]):
                if successor[0] not in explored:
                    cost = problem.getCostOfActions(node[1] + [successor[1]])
                    if node[1] is emptyList:
                        next_node = (successor[0], [successor[1]])
                    else:
                        next_node = (successor[0], node[1] + [successor[1]])
                    # add the latest node
                    myPriorityQueue.push(next_node, cost)
    return node[1]

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    # initialize initial position, priority queue and the explored list
    myPriorityQueue = util.PriorityQueue()
    myPriorityQueue.push((problem.getStartState(), [], 0), 0)
    explored = []

    while not myPriorityQueue.isEmpty():
        currentTuple = myPriorityQueue.pop()
        # add the position and its cost to the explored list
        explored.append((currentTuple[0], currentTuple[2]))
        # if the position is goal then return its path
        if problem.isGoalState(currentTuple[0]):
            path = currentTuple[1]
            return path
        if not problem.isGoalState(currentTuple[0]):
            # if the goal is not yet reached then compute the successor nodes
            for successor in problem.getSuccessors(currentTuple[0]):
                nextAction = currentTuple[1] + [successor[1]]
                cost = problem.getCostOfActions(nextAction)
                next_node = (successor[0], nextAction, cost)
                # create a flag to test the condition
                flag = False
                for visited in explored:
                    if successor[0] == visited[0] and next_node[2] >= visited[1]:
                        flag = True
                # if the above condition is satisfied,
                # the flag becomes true,
                # we ignore the successor as it has more cost
                if not flag:
                    # if the flag remians false then
                    # this condition becomes true and
                    # the latest node is pushed on the priority queue
                    myPriorityQueue.push(next_node, cost+heuristic(successor[0], problem))
                    # add the entry as the explored node
                    explored.append((successor[0], cost))

    return path



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch