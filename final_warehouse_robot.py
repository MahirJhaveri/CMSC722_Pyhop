import pyhop
import copy

""" 
Planner for warehouse robot capable of picking up and dropping off multiple packages
one at a time in the most optimal order.

Planner is capable of:
- Plan optimal order for picking and delivering packages.
- Plan appropriate trips to charging stations with goal to minimize number of recharges
- Plan a route which minimizes the total length of the plan using branch and bound search
- Replan when encoutered with a blocked path during execution of generated plan.
"""

################ Rigid Relations ###################

rigid_relations = pyhop.State("rigid relations")

rigid_relations.types = {
    'robot': ['R1'],
    'object': ['c1', 'c2', 'c3'],
    'loc': ['room1', 'room2', 'room3', 'room4', 'room5', 'room6', 'room7', 'room8', 'room9'],
    'charging_stations': ['room3', 'room6', 'room9']
}

rigid_relations.adjacent = {
    ('room1', 'room2'): True,
    ('room2', 'room3'): True,
    ('room2', 'room4'): True,
    ('room2', 'room5'): True,
    ('room5', 'room6'): True,
    ('room3', 'room4'): True,
    ('room3', 'room7'): True,
    ('room7', 'room8'): True,
    ('room8', 'room9'): True,
    ('room6', 'room9'): True
}

# cost in terms of battery charge per move operation
COST_PER_MOVE = 20

# max capacity of a battery
FULL_BATTERY = 100

# LIMIT of number of refuels in a given solution
LIMIT = len(rigid_relations.types['charging_stations'])

################ State #############################

state0 = pyhop.State("initial state")

state0.loc = {
    'R1': 'room1',
    'c1': 'room4',
    'c2': 'room6',
    'c3': 'room8'
}

state0.cargo = {
    'R1': []
}

state0.battery = {
    'R1': 100
}

################ Helpers ###########################

def is_adjacent(room_x, room_y):
    val1 = (room_x, room_y) in rigid_relations.adjacent
    val2 = (room_y, room_x) in  rigid_relations.adjacent
    return val1 or val2

def is_a(variable, type): 
    return variable in rigid_relations.types[type]

def is_at(state, robot, room):
    return room == state.loc[robot]

# check if loc is a charging station
def is_charging_station(loc):
    return loc in rigid_relations.types["charging_stations"]

def get_shortest_path(start, end):

    ## Generate a graph from the rigid relations
    G = {}
    for (x,y) in rigid_relations.adjacent:
        if x not in G: G[x] = {}
        if y not in G: G[y] = {}
        G[x][y] = True
        G[y][x] = True
    
    visited = {start:None}
    queue = [start]
    while len(queue) > 0:
        node = queue.pop(0)
        if node == end: break
        for neighbor in G[node]:
            if neighbor not in visited:
                visited[neighbor] = node
                queue.append(neighbor)
    
    # disconnected graph
    if end not in visited: return False
    
    path = []
    prev = end
    while visited[prev] != None:
        path.append(prev)
        prev = visited[prev]
    
    path.reverse()
    return path

# Finds the all charging station to the current loc, SORTED in increasing order of distance from loc
def get_all_charging_station(loc):
    stations = []

    ## Generate a graph from the rigid relations
    G = {}
    for (x,y) in rigid_relations.adjacent:
        if x not in G: G[x] = {}
        if y not in G: G[y] = {}
        G[x][y] = True
        G[y][x] = True
    
    start = loc

    visited = {start:True}
    queue = [start]
    while len(queue) > 0:
        node = queue.pop(0)
        for neighbor in G[node]:
            if neighbor not in visited:
                visited[neighbor] = True
                queue.append(neighbor)
                if neighbor in rigid_relations.types["charging_stations"]:
                    stations.append(neighbor)
    return stations


# Finds the nearest charging station to the current loc. CAN BE MADE MORE EFFICIENTs
def get_closest_charging_station(loc):
    if is_charging_station(loc): return loc
    else:
        stations = get_all_charging_station(loc)
        return stations[0] if len(stations) > 0 else None

# computes the distance between loc1 and loc2. CAN BE MADE MORE EFFICIENT
def distance(loc1, loc2):
    return len(get_shortest_path(loc1, loc2))

################ Operators #########################

def move(state, R, start_loc, end_loc):
    type_check = is_a(R, 'robot') and is_a(start_loc, 'loc') \
        and is_a(end_loc, 'loc')
    if type_check and is_adjacent(start_loc, end_loc) and is_at(state, R, start_loc) and state.battery[R] > COST_PER_MOVE:
        state.loc[R] = end_loc
        state.battery[R] -= COST_PER_MOVE
        return state
    return False

# Max capacity of robot is 1
def pickup(state, R, c):
    type_check = is_a(R, 'robot') and is_a(c, 'object')
    if type_check and len(state.cargo[R]) == 0:
        state.cargo[R].append(c)
        state.loc[c] = R
        return state
    return False

def drop(state, R, c):
    type_check = is_a(R, 'robot') and is_a(c, 'object')
    if type_check and state.loc[c] == R:
        state.loc[c] = state.loc[R]
        state.cargo[R].remove(c)
        return state
    return False

# recharge only at a charging station
def recharge(state, R):
    type_check = is_a(R, 'robot')
    if type_check and is_charging_station(state.loc[R]):
        state.battery[R] = FULL_BATTERY
        return state
    return False

pyhop.declare_operators(move, pickup, drop, recharge)

################ Task Methods ######################

# An object is already present on the robot, then transport it first
def transport_all1(state, R, container_destination_map):
    if len(state.cargo[R]) > 0 and state.cargo[R][0] in container_destination_map:
        new_map = copy.deepcopy(container_destination_map)
        new_map.pop(state.cargo[R][0])
        return [('transport', R, state.cargo[R][0], container_destination_map[state.cargo[R][0]]),
        ('transport_all', R, new_map)]
    return False

# First drop off the container which is closest to the robots current location
def transport_all2(state, R, container_destination_map):
    if len(state.cargo[R]) == 0 and len(container_destination_map) > 0:
        containers = list(container_destination_map.keys())
        containers.sort(key=lambda c: distance(state.loc[R], state.loc[c]))
        return [('transport_all_order', R, containers, container_destination_map, 0)]
    return False

# nothing left to drop off
def transport_all3(state, R, container_destination_map):
    if len(container_destination_map) == 0:
        return []
    return False

"""
The main task which deals with the delivery of all containers in container_destination_map
to the appropriate locations.
"""
pyhop.declare_methods('transport_all', transport_all1, transport_all2, transport_all3)

# option 1 is to transport the container at containers[i] first and then deal with the others
def transport_all_order1(state, R, containers, container_destination_map, i):
    if len(state.cargo[R]) == 0 and len(container_destination_map) > 0 and i < len(container_destination_map):
        c = containers[i]
        new_map = copy.deepcopy(container_destination_map)
        new_map.pop(c)
        return [('transport', R, c, container_destination_map[c]),
        ('transport_all', R, new_map)]
    return False

# option 2 is to call transport_all_order on i+1
def transport_all_order2(state, R, containers, container_destination_map, i):
    if len(state.cargo[R]) == 0 and len(container_destination_map) > 0 and i < len(container_destination_map):
        return [('transport_all_order', R, containers, container_destination_map, i+1)]
    return False

"""
The whole point of transport_all_order task is so that we dont have to enforce any order of 
delivery of containers manually instead we only "recommend" an order which is the closest neighbor first.

The reason this is done is that sometimes the nearest neigbor might not be the most optimal delivery strategy
and in this case we want the branch-and-bound search to try out other possibilites of ordering which wouldn't 
be possible if we strictly enforced ordering.
"""
pyhop.declare_methods('transport_all_order', transport_all_order1, transport_all_order2)


# case 1: c has not been picked 
def transport1(state, R, c, dest):
    if state.loc[c] != dest and is_a(state.loc[c], 'loc'):
        return [('travel', R, state.loc[c], [state.loc[c]]), ('pickup', R, c), 
        ('travel', R, dest, [dest]), ('drop', R, c)]
    return False

# case 2: c is already loaded onto R
def transport2(state, R, c, dest):
    if state.loc[c] != dest and state.loc[c] == R:
        return [('travel', R, dest, [dest]), ('drop', R, c)]
    return False

# case 3: c is already at dest
def transport3(state, R, c, dest):
    if state.loc[c] == dest: return []
    return False

"""
transport task deals with delivering a particular container to its destination.
"""
pyhop.declare_methods('transport', transport1, transport2, transport3)

# case 1: handles the default case when R is already at dest
def travel_default(state, R, dest, excluded_stations):
    if state.loc[R] == dest: return []
    return False

# otherwise simply return the travel_with_wrapper task
def travel1(state, R, dest, excluded_stations):
    return [('travel_with_wrapper', R, dest, 0, LIMIT)]

"""
This task deals with traveling the robot to a certain destination.

Here excluded_stations is the set of charging stations which the robot is not allowed to visit
in case it needs to recharge. The reason for this is to avoid infinite loops in the code.
Basically, it doesn't make sense to visit a station twice while travelling from one point to another.
"""
pyhop.declare_methods('travel', travel_default, travel1)

# First try to solve with the available limit
def travel_with_wrapper1(state, R, dest, i, limit):
    if i <= limit:
        return [('travel_with', R, dest, i, [])]
    return False

# else try to stretch the number of recharges by 1
def travel_with_wrapper2(state, R, dest, i, limit):
    if i < limit:
        return [('travel_with_wrapper', R, dest, i+1, limit)]
    return False

"""
This is just a wrapper task around the travel_with task. It is used to "recommend" a certain order
of finding a plan to the seek_plan function.

Essentially, what it does is that it first tries to find a plan to travel from the current loc to dest
by using exactly i recharges on the way. If nothing is possible then it increments i by one and then calls 
travel_with_wrapper task again. Fails when i exceeds limit it doesn't find a valid plan.

Designed this way to allow the planner to find a plan with minimum number of recharges.
"""
pyhop.declare_methods('travel_with_wrapper', travel_with_wrapper1, travel_with_wrapper2)

# case 1: i == 0, so travel directly to dest
def travel_with1(state, R, dest, i, excluded_stations):
    if i == 0: 
        enough_battery = state.battery[R] > COST_PER_MOVE*(distance(state.loc[R], dest) + distance(dest, get_closest_charging_station(dest)))
        if state.loc[R] != dest and is_a(dest, 'loc') and enough_battery:
            path = get_shortest_path(state.loc[R], dest)
            prev = state.loc[R]
            res = []
            for loc in path:
                res.append(('move', R, prev, loc))
                prev = loc
            return res
    return False

# case 2: i > 0, so travel to dest via exactly i stations 
def travel_with2(state, R, dest, i, excluded_stations):
    if i > 0 : 
        # stations in a increasing order of distance from dest ie. stations[0] is the closest station to dest
        stations = get_all_charging_station(dest) 
        # remove all excluded stations from considerations (coz they have been already considered)
        filtered_stations = list(filter(lambda s: s not in excluded_stations, stations)) 
        if state.loc[R] != dest and is_a(dest, 'loc') and len(filtered_stations)>0:
            return [('travel_via', R, dest, filtered_stations[0], list(filtered_stations[1:]), list(excluded_stations), i)] # NOTE: NO NEED TO COPY LIST HERE
    return False

"""
travel_with task attempts to make robot reach dest via exactly i stations on the way.

Also, in case i > 0 travel_with imposes an order to check which station is suitable as the last one.
It's is easy to reason that we prefer a plan where the last charging station is as close to dest as possible.
This way we will have more charge left when we reach dest and will require less charging later. So, no need to 
"recommend" here simply force the order of preference. 
"""
pyhop.declare_methods('travel_with', travel_with1, travel_with2)

# travel to dest via station as the last station
def travel_via1(state, R, dest, station, other_stations, excluded_stations, i):
    enough_battery = FULL_BATTERY > COST_PER_MOVE*(distance(station, dest) + distance(dest, get_closest_charging_station(dest)))
    if i > 0 and is_a(station, 'loc') and enough_battery:
        new_excluded_stations = list(excluded_stations)
        new_excluded_stations.append(station)
        return [('travel_with', R, station, i-1, new_excluded_stations), ('recharge', R), ('travel_with', R, dest, 0, new_excluded_stations)]
    return False

# makes another call to travel_via
def travel_via2(state, R, dest, station, other_stations, excluded_stations, i):
    if i > 0 and len(other_stations) > 0:
        new_excluded_stations = list(excluded_stations)
        new_excluded_stations.append(station)
        return [('travel_via', R, dest, other_stations[0], list(other_stations[1:]), new_excluded_stations, i)]
    return False

"""
travel_via task attemps to travel robot to dest via station as the last charging 
station and exactly i recharges. If this doesn't work then it tries another station
from the list other_stations. Here other_stations is already sorted in the order of
increasing distance from dest. travel_via2 is basically tries the second closest 
station to dest if the closest doesnt work and so on.
"""
pyhop.declare_methods('travel_via', travel_via1, travel_via2)

########################## Compute Plan #######################################
print("Solution using pyhop()")

pyhop.pyhop(state0, [('transport_all', 'R1', {
    'c2': 'room7',
    'c3': 'room5',
    'c1': 'room2'
})], verbose=1)

########################## Compute + Exec Plan ################################

# move operator for execution in run-lazy-lookahead
# same as the move operator but updated to fail when robot attempts to move from 
# room2 to room4 to indicate a broken path / obstacle
def move_exec(state, R, start_loc, end_loc):
    type_check = is_a(R, 'robot') and is_a(start_loc, 'loc') \
        and is_a(end_loc, 'loc')
    if type_check and is_adjacent(start_loc, end_loc) and is_at(state, R, start_loc) and state.battery[R] > COST_PER_MOVE:
        if (start_loc=="room4" and end_loc=="room2") or (start_loc=="room2" and end_loc=="room4"):
            rigid_relations.adjacent.pop(('room2', 'room4')) # basically robot learns that this path doesnt exist
            return False
        state.loc[R] = end_loc
        state.battery[R] -= COST_PER_MOVE
        return state
    return False

print("run-lazy-lookahead")

# This function is used to add the move operator to be used for execution
# in run-lazy-lookahead
pyhop.update_exec_operator(move_exec, "move")

exec_trace = []

new_state = pyhop.run_lazy_lookahead(state0, [('transport_all', 'R1', {
    'c2': 'room7',
    'c3': 'room5',
    'c1': 'room2'
})], history=exec_trace, verbose=1)

print("\n Final State: ")
print(new_state.loc)