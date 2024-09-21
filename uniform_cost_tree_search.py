import heapq
import itertools
import time
import numpy as np

#problem space dicts
ACTIONS = {'Left': 1.0, 'Right': 0.9, 'Up': 0.8, 'Down': 0.7, 'Suck': 0.6}
DIRS = {'Left': (0, -1), 'Right': (0, 1), 'Up': (-1, 0), 'Down': (1, 0)}


    
#movemnet logic
def move(position, direction):
    x, y = position
    move_x, move_y = DIRS[direction]
    new_position = (x + move_x, y + move_y)
    
    #check to ensure we're bounded in the grid
    if 0 < new_position[0] <= 4 and 0 < new_position[1] <= 5:
        return new_position
    #if the move would be invalid, don't do it 
    return position  

#state class to keep track of the world state
class State:
    #initialize state
    def __init__(self, position, dirty_rooms):
        self.position = position
        self.dirty_rooms = frozenset(dirty_rooms)

    #check to see if two state objects are the same
    def __eq__(self, other):
        return self.position == other.position and self.dirty_rooms == other.dirty_rooms

    #generates a unique hash for each state
    def __hash__(self):
        return hash((self.position, self.dirty_rooms))

    #"toString" function 
    def __repr__(self):
        return f"State(pos={self.position}, dirty={self.dirty_rooms})"

    #check if the goal is 
    def is_goal(self):
        return len(self.dirty_rooms) == 0

#uniform cost search, takes a starting state and returns the path to the solution, the cost, the 
def uniform_cost_search(start_state):
    #priority queue, stores state, cost
    frontier = []
    
    #sequence counter
    counter = itertools.count() 
    heapq.heappush(frontier, (0, next(counter), start_state, []))

    #keep track of what we've explored
    explored = set()
    nodes_expanded = 0  
    nodes_generated = 1  

    #store first five expanded nodes to display later
    first_5_nodes = []  

    #setup is done track the time of the actual algorithm
    start_time = time.time()

    #while our queue is unempty
    while frontier:
        #expand the node
        cost, _, current_state, path = heapq.heappop(frontier)
        nodes_expanded += 1 

        #grab the first five nodes
        if nodes_expanded <= 5:
            first_5_nodes.append(current_state)

        #we found a solution, we can go ahead and return 
        if current_state.is_goal():
            end_time = time.time() 
            execution_time = end_time - start_time
            return path, cost, first_5_nodes, nodes_expanded, nodes_generated, execution_time

        #we already explored this state, skip
        if current_state in explored:
            continue
        #keeps track of already explored nodes
        explored.add(current_state)

        #generate sucsessor states
        for action, action_cost in ACTIONS.items():
            new_position = current_state.position
            new_dirty_rooms = set(current_state.dirty_rooms)

            if action == 'Suck'and current_state.position in new_dirty_rooms:
                #if we want to move 
                new_dirty_rooms.remove(current_state.position)
            elif action in DIRS:
                new_position = move(current_state.position, action)
                
            new_state = State(new_position, new_dirty_rooms)
            new_cost = cost + action_cost

            #generate a new node
            if new_state not in explored:
                nodes_generated += 1 
                heapq.heappush(frontier, (new_cost, next(counter), new_state, path + [(action, new_position)]))

    #no solution was found
    end_time = time.time() 
    execution_time = end_time - start_time
    return None, float('inf'), first_5_nodes, nodes_expanded, nodes_generated, execution_time

#instances
instance_1 = State((2, 2), [(1, 2), (2, 4), (3, 5)])
instance_2 = State((3, 2), [(1, 2), (2, 1), (2, 4), (3, 3)])

#results
print("Instance 1:")
path_1, cost_1, first_5_1, expanded_1, generated_1, time_1 = uniform_cost_search(instance_1)

print("First 5 Expanded States (Instance 1):", first_5_1)
print("Total Nodes Expanded (Instance 1):", expanded_1)
print("Total Nodes Generated (Instance 1):", generated_1)
print("Execution Time (Instance 1):", time_1, "seconds")
print("Path (Instance 1):", path_1, "\nCost:", np.round(cost_1,decimals=1), "\nMoves:", len(path_1))

print("\n\nInstance 2...")
path_2, cost_2, first_5_2, expanded_2, generated_2, time_2 = uniform_cost_search(instance_2)

print("First 5 Expanded States (Instance 2):", first_5_2)
print("Total Nodes Expanded (Instance 2):", expanded_2)
print("Total Nodes Generated (Instance 2):", generated_2)
print("Execution Time (Instance 2):", time_2, "seconds")
print("Path (Instance 2):", path_2, "\nCost:", np.round(cost_2,decimals=1), "\nMoves:", len(path_2))