
from SubwayMap import *
from utils import *
import os
import math
import copy
import copy
import timeit


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information need to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
            connections=map.connections

    """

    path_list = []

    for i in map.connections[path.last]:
        new = copy.deepcopy(path)
        new.add_route(i)
        path_list.append(new)


    return path_list
def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.

"""


    copy = path_list.copy()
    for path in path_list:

        for i, j in enumerate(path.route):
            if j in path.route[i + 1:]:
                copy.remove(path)

        path_list = copy

    return path_list


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted


    """

    return expand_paths[:] + list_of_path

def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id


     """


    list_of_path =  [Path(origin_id)]
    bool = False
    while bool != True:
        if list_of_path[0] and list_of_path[0].last == destination_id:
            bool = True
        else:
            l= list_of_path.pop(0)
            E =expand(l,map)

            E= remove_cycles(E)
            list_of_path = insert_depth_first_search(E,list_of_path)
    if list_of_path:
        return list_of_path[0]
    else:

        None



def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    #aixo s'ha de canviar no reverse
    return  list_of_path + expand_paths[:] # you do the copy of the list (invers) and then concatenate with the list of path that have been visited

def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    list_of_path =  [Path(origin_id)]
    bool = False  # TRUE OR FALSe
    while bool != True:
        if list_of_path[0] and list_of_path[0].last == destination_id:
            bool = True
        else:
            l= list_of_path.pop(0)
            E =expand(l,map)
            E= remove_cycles(E)
            list_of_path = insert_breadth_first_search(E,list_of_path)
    if list_of_path:
        return list_of_path[0]
    else:
        print("no existeix solucio")
        None
    #return list_of_path.pop() if list_of_path  else None




def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded pºath with updated cost

    """

    if type_preference == 0:

        for i in expand_paths:
            i.update_g(1)
        return expand_paths



    else :
        if type_preference == 1:
            for i in expand_paths:
                t = map.connections[i.penultimate][i.last]

                i.update_g(t)

            return expand_paths


        else:

            if type_preference == 2:
                for i in expand_paths:

                    # if line is the same no update else si
                    t = map.connections[i.penultimate][i.last]
                    v = map.stations[i.penultimate]['velocity']
                    #print("ESTO ES V",v)#last or penultimate??
                    d = v*t
                    #print("ESTO ES D", d)
                    if map.stations[i.penultimate]['name'] == map.stations[i.last]['name']:
                        i.update_g(0)
                    else:
                        i.update_g(d)



                return expand_paths

            else:
                if type_preference == 3:
                    cont = 0
                    for i in expand_paths:
                        #t = map.connections[i.penultimate][i.last]
                        if map.stations[i.penultimate]['line'] != map.stations[i.last]['line']:
                          cont +=1
                          i.update_g(cont)
                return expand_paths








def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """

    list_of_path+=expand_paths[:]
    list = sorted(list_of_path, key=lambda path: path.g)



    return list



def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """


    list_of_path = [Path(origin_id)]
    bool = False #TRUE OR FALSe
    while bool != True:
        if list_of_path[0] and list_of_path[0].last == destination_id:
            bool =True
        else:
            l = list_of_path.pop(0)
            E = expand(l,map)
            E = remove_cycles(E)
            E = calculate_cost(E,map,type_preference)
            list_of_path = insert_cost(E,list_of_path)


    if list_of_path:

        return list_of_path[0]
    else:

        None


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """

    if type_preference == 0:
        for paths in expand_paths:


            if map.stations[paths.last]['name'] == map.stations[destination_id]['name']:
                 paths.h = 0
            else:


                paths.h = 1

        return expand_paths

    if type_preference == 1:
        list = []
        for paths in expand_paths:
            coordx =map.stations[paths.last]['x']
            coordy =map.stations[paths.last]['y']
            destix = map.stations[destination_id]['x']
            destiy= map.stations[destination_id]['y']
            dista_heucladiana = square(coordx,destix,coordy,destiy) #tot be

            for i in map.stations:
                list.append(map.stations[i]['velocity'])
            maximo_vel = max(list)
            temps = dista_heucladiana/maximo_vel
            paths.h = temps


        return expand_paths
    if type_preference == 2:
        for paths in expand_paths:
            coordx = map.stations[paths.last]['x']
            coordy = map.stations[paths.last]['y']
            destix = map.stations[destination_id]['x']
            destiy = map.stations[destination_id]['y']
            dista_heucladiana = square(coordx, destix, coordy, destiy)
            paths.h = dista_heucladiana
        return expand_paths
    if type_preference == 3:
        for paths in expand_paths:
            if map.stations[paths.last]['line'] == map.stations[destination_id]['line']:
                paths.h = 0
            else:
                paths.h = 1
        return expand_paths
def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """

    for path in expand_paths:

        path.update_f()
    return expand_paths






def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g in this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths PATHS FILLS
             list_of_path (LIST of Path Class): All the paths to be expanded LLISTA GLOBAL DE PATHS DE L'ARBRE EXPLORAT
             visited_stations_cost (dict): All visited stations cost COSTOS PARCIALS
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
    """




    for i in expand_paths:

        if i.last in visited_stations_cost:

            if i.f >= visited_stations_cost[i.last]:

                expand_paths.remove(i)


            else:

                visited_stations_cost[i.last] = i.f


                for j in list_of_path:


                    if i.last == j.route[-1]:

                            list_of_path.pop(-1)

            expand_paths.pop(-1)




    return expand_paths,list_of_path, visited_stations_cost




def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """

    list_of_path+= expand_paths[:]
    list = sorted(list_of_path, key=lambda path: path.f)


    return list

def Astar(origin_coor, dest_coor, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (list): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """

    visited_stations_cost = {}
    origin = coord2station(origin_coor,map)[0]
    desti = coord2station(dest_coor,map)[0]
    list_of_path = [Path(origin)]
    bool =False
    while bool != True:
        if list_of_path[0] and list_of_path[0].last == desti:
            bool =True
        else:
            l = list_of_path.pop(0)


            E = expand(l,map)

            E = remove_cycles(E)

            calculate_cost(E,map,type_preference)
            calculate_heuristics(E, map, desti, type_preference)
            E = update_f(E)

            E,l,visited_stations_cost =remove_redundant_paths(E,l,visited_stations_cost)
            list_of_path = insert_cost_f(E,list_of_path)


    if list_of_path:

        return list_of_path[0]
    else:
        None

def square(x1, x0,y1, y0):

    hello = math.sqrt(((x1-x0)**2)+((y1-y0)**2))
    return hello

def coord2station(coord, map):
    """
        From coordinates, it searches the closest station.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """

    possible_or=[]
    possible_origins = []

    for i,j in map.stations.items():

        x =  map.stations[i]['x']
        y =map.stations[i]['y']
        s = square(coord[0],x,coord[1],y)
        possible_origins.append(s)

    for k in map.stations.keys():
        x = map.stations[k]['x']
        y = map.stations[k]['y']
        if square(coord[0],x,coord[1],y) == min(possible_origins):
            possible_or.append(k)


    return possible_or
