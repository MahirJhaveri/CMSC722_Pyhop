import pyhop

""" 
Simple task of robot picking up a package and droping it off at dest. With GPS 
module for path finding and recharge only possible at specific locations.

This module tries to find the solution with fewest refuels first.
"""

################ Rigid Relations ###################

rigid_relations = pyhop.State("rigid relations")

rigid_relations.types = {
    'robot': ['R1'],
    'object': ['c1'],
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
COST_PER_MOVE = 26

# max capacity of a battery
FULL_BATTERY = 100

# LIMIT of number of refuels in a given solution
LIMIT = len(rigid_relations.types['charging_stations'])

################ State #############################

state0 = pyhop.State("initial state")

state0.loc = {
    'R1': 'room1',
    'c1': 'room4'
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

def transport1(state, R, c, dest):
    if state.loc[c] != dest and is_a(state.loc[c], 'loc'):
        return [('travel', R, state.loc[c], [state.loc[c]]), ('pickup', R, c), 
        ('travel', R, dest, [dest]), ('drop', R, c)]
    return False

# c is already loaded onto R
def transport2(state, R, c, dest):
    if state.loc[c] != dest and state.loc[c] == R:
        return [('travel', R, dest, [dest]), ('drop', R, c)]
    return False

# c is already at dest
def transport3(state, R, c, dest):
    if state.loc[c] == dest: return []
    return False

pyhop.declare_methods('transport', transport1, transport2, transport3)

# travel directly if robot has sufficient charge
def travel1(state, R, dest, excluded_stations):
    return [('travel_with_wrapper', R, dest, 0, LIMIT)]

# handles the default case when R is already at dest
def travel_default(state, R, dest, excluded_stations):
    if state.loc[R] == dest: return []
    return False

pyhop.declare_methods('travel', travel1, travel_default)

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

pyhop.declare_methods('travel_with_wrapper', travel_with_wrapper1, travel_with_wrapper2)

# travel directly ...
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

# travel via a refueling station
def travel_with2(state, R, dest, i, excluded_stations):
    if i > 0 : 
        stations = get_all_charging_station(dest)
        filtered_stations = list(filter(lambda s: s not in excluded_stations, stations)) # remove all excluded stations from considerations (coz they have been already considered)
        if state.loc[R] != dest and is_a(dest, 'loc') and len(filtered_stations)>0:
            return [('travel_via', R, dest, filtered_stations[0], list(filtered_stations[1:]), list(excluded_stations), i)] # NOTE: NO NEED TO COPY LIST HERE
    return False

pyhop.declare_methods('travel_with', travel_with1, travel_with2)

# travel to dest via station
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

pyhop.declare_methods('travel_via', travel_via1, travel_via2)

##########################################################

pyhop.pyhop(state0, [('transport', 'R1', 'c1', 'room6')], verbose=1)

