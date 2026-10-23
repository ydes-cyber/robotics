from math import floor
from autorob.heap_sort_node import heap_sort 
map_data = None    
def get_map_data(message):
    global map_data
    map_data = message


def get_map_cell(x, y):
    if map_data is None:
        return None
    data = map_data["data"]
    width = map_data["info"]["width"] 
    height = map_data["info"]["height"]
    if 0 <= x < width and 0 <= y < height:
     index = y * width + x
     return data[index]
    return None 
 

def cell_free(x, y):
    state = get_map_cell(x,y)
    if state is not None and 0 <= state < 50:
        return True
    else:
        return False 
    
def alter_to_map_coord(x,y):
    global map_data
    x_coord = map_data["info"]["origin"]["position"]["x"]
    y_coord = map_data["info"]["origin"]["position"]['y']
    resolution = map_data["info"]["resolution"]
    floor_x = floor(( x - x_coord) / resolution)
    floor_y = floor((y - y_coord)/ resolution)
    return floor_x, floor_y

def cell_to_world_coord(x, y):
    origin = map_data["info"]["origin"]["position"]
    resolution = map_data["info"]["resolution"]

    world_x = origin["x"] + (x + 0.5) * resolution
    world_y = origin["y"] + (y + 0.5) * resolution

    return world_x, world_y


def existance_check(x,y): 
    state = get_map_cell(x,y)
    if state is None:
        return False 
    cell = cell_free(x,y)
    return cell 

def filter(x,y): 
    
    next_cell = [
    (x + 1, y),
    (x - 1, y),
    (x, y + 1),
    (x, y - 1)
]
    valid = []
    for x, y in next_cell: 
        if existance_check(x,y):
            valid.append((x,y))
    return valid 
#Calculates how many horizontal and vertical moves it takes to get from one cell to another. 
# This is used by A* to estimate the distance to the goal
def distance(cell, target):
    return abs(cell[0] - target[0]) + abs(cell[1] - target[1])

def astar(start, target):
    if not existance_check(start[0], start[1]):
        return None
    
    if not existance_check(target[0], target[1]):
        return None

    list_unchecked = [start]
    origin = {start: None}
    spaces = {start: 0}

    while len(list_unchecked) > 0:
        focuses = []

        for cell in list_unchecked:
            focus = spaces[cell] + distance(cell, target)
            focuses.append(focus)

        lowest = heap_sort(focuses)[0]

        choices = []
        for cell in list_unchecked:
            focus = spaces[cell] + distance(cell, target)
            if focus == lowest:
                choices.append(cell)

        current = min(choices)
        list_unchecked.remove(current)

        if current == target:
            path = []

            while current is not None:
                path.append(current)
                current = origin[current]

            path.reverse()
            return path

        cousins = filter(current[0], current[1])

        for cousin in cousins:
            new_distance = spaces[current] + 1

            if cousin not in spaces or new_distance < spaces[cousin]:
                spaces[cousin] = new_distance
                origin[cousin] = current

                if cousin not in list_unchecked:
                    list_unchecked.append(cousin)

    return None
def map_is_valid(message):
    try:
        header = message["header"]
        info = message["info"]
        width = info["width"]
        height = info["height"]
        data = message["data"]
        resolution = info["resolution"]
        origin = info["origin"]
        position = origin["position"]
        orientation = origin["orientation"]
        if not isinstance (header, dict):
                return False
        if not isinstance(height, int) or height <= 0:
            return False
        if not isinstance(width, int ) or width <= 0:
            return False
        if not isinstance(resolution, (int, float)) or resolution <= 0:
            return False
        if not isinstance(data, list) or len(data) != width *height:
            return False
        if  not all(isinstance(item, int) for item in data):
            return False
        position_values = [position["x"], position["y"], position["z"]]
        orientation_values = [orientation["x"], orientation["y"], orientation["z"], orientation["w"]]
        
        if  not all(isinstance(value,(int,float))
            for value in position_values + orientation_values):
            return False 
        else:
            return True 
    except(KeyError, TypeError):
        return False
  
  
  
