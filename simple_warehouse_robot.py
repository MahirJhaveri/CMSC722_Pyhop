
import pyhop

""" 
Simple task of robot picking up a package and droping it off at dest. With GPS 
module for path finding and recharge available at all locations.
"""

################ Rigid Relations ###################

rigid_relations = pyhop.State("rigid relations")

rigid_relations.types = {
    'robot': ['R1'],
    'object': ['c1'],
    'loc': ['room1', 'room2', 'room3', 'room4']
}

rigid_relations.adjacent = {
    ('room1', 'room2'): True,
    ('room1', 'room3'): True,
    ('room2', 'room4'): True
}

################ State #############################

state0 = pyhop.State("initial state")

state0.loc = {
    'R1': 'room1',
    'c1': 'room3'
}

state0.cargo = {
    'R1': []
}

state0.battery = {
    'R1': 50
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

################ Operators #########################

def move(state, R, start_loc, end_loc):
    type_check = is_a(R, 'robot') and is_a(start_loc, 'loc') \
        and is_a(end_loc, 'loc')
    if type_check and is_adjacent(start_loc, end_loc) and is_at(state, R, start_loc) and state.battery[R] > 25:
        state.loc[R] = end_loc
        state.battery[R] -= 25
        return state
    return False

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

def recharge(state, R):
    type_check = is_a(R, 'robot')
    if type_check:
        state.battery[R] = 100
        return state
    return False

pyhop.declare_operators(move, pickup, drop, recharge)

################ Task Methods ######################

def transport1(state, R, c, dest):
    if state.loc[c] != dest and is_a(state.loc[c], 'loc'):
        return [('travel', R, state.loc[c]), ('pickup', R, c), 
        ('travel', R, dest), ('drop', R, c)]
    return False

# c is already loaded onto R
def transport2(state, R, c, dest):
    if state.loc[c] != dest and state.loc[c] == R:
        return [('travel', R, dest), ('drop', R, c)]
    return False

# c is already at dest
def transport3(state, R, c, dest):
    if state.loc[c] == dest: return []
    return False

pyhop.declare_methods('transport', transport1, transport2, transport3)


def travel1(state, R, dest):
    if state.loc[R] != dest and is_a(dest, 'loc'):
        path = get_shortest_path(state.loc[R], dest)
        prev = state.loc[R]
        res = []
        for loc in path:
            res.append(('move_next', R, prev, loc))
            prev = loc
        return res
    return False

def travel2(state, R, dest):
    if state.loc[R] == dest: return []
    return False

pyhop.declare_methods('travel', travel1, travel2)

# move to adjacent node with no need to recharge
def move_next1(state, R, loc, next_loc):
    if is_adjacent(loc, next_loc) and state.battery[R] > 25:
        return [('move', R, loc, next_loc)]
    return False

# insufficient battery, so move to adjacent node after recharging
def move_next2(state, R, loc, next_loc):
    if is_adjacent(loc, next_loc) and state.battery[R] <= 25:
        return [('recharge', R), ('move', R, loc, next_loc)]
    return False

pyhop.declare_methods('move_next', move_next1, move_next2)

##########################################################

pyhop.pyhop(state0, [('transport', 'R1', 'c1', 'room4')], verbose=1)

