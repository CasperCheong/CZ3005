import json
from queue import PriorityQueue
import heapq as heap
import math
import time
import utils
import logging

#Creating and Configuring Logger

Log_Format = "%(levelname)s %(asctime)s - %(message)s"

logging.basicConfig(filename = "logfile.log",
                    filemode = "w",
                    format = Log_Format, 
                    level = logging.DEBUG)

logger = logging.getLogger()

# Default parameters
start = 1
goal = 50
budget = 287932

# Loading files into dictionary
with open('Dist.json', 'r') as data:
    dist = json.load(data)

with open('Cost.json', 'r') as data:
    cost = json.load(data)

with open('G.json', 'r') as data:
    graph = json.load(data) # adjacency list 

with open('Coord.json', 'r') as data:
    coord = json.load(data)

def UCS(G, startingNode, endingNode, Dist):
        parents = {}
        visited = set()
        distance = defaultdict(lambda: float('inf'))
        pq = []
        # priority queue
        heap.heappush(pq, (0, startingNode))
        distance[startingNode] = 0

        # while priority queue not empty
        while pq:
            _, curr = heap.heappop(pq)

            if curr == endingNode:
                break

            adjacent_nodes = G[curr]

            for nodes in adjacent_nodes:
                # if nodes in visited: continue
                if nodes in visited:
                    continue

                cost = float(distance[curr] + float(Dist[curr + ',' + nodes]))

                if distance[nodes] > cost:
                    parents[nodes] = curr
                    distance[nodes] = cost
                    heap.heappush(pq, (cost, nodes))

            visited.add(curr)

        return parents, distance

print (" ============================ TASK 1 =======================================")
def uniform_cost_search(graph, start, goal):
    visited=set()
    queue = PriorityQueue()
    queue.put((0, [start])) #((Distance, (Cost,Node[]))

    
    while queue:
        
        # logger.debug(queue.queue)
        distance, node = queue.get()
        # print(node)
        cur_node = int(node[-1]) #Check cur_node as its always at the end

        if cur_node == goal: 
            #Perform print 
            utils.print_path(node)
            print("Shortest distance: " + str(distance))
            return distance, node
        if cur_node not in visited:
            visited.add(cur_node)
            for i in graph[str(cur_node)]:
                if i not in visited:
                    temp= str(cur_node)+","+str(i)
                    new_distance = distance + dist[temp] 
                    node_path = node.copy()
                    node_path.append(i)
                    queue.put((new_distance,node_path))

st = time.perf_counter()
uniform_cost_search(graph, start, goal)
end = time.perf_counter()
print("Time taken: " , (end-st))

print ("\n ============================ TASK 2 =======================================")
def uniform_cost_search_constrained(g, start, goal, budget):
    visited=set()
    queue = PriorityQueue()
    queue.put((0, (0,[str(start)]))) #((Distance, (Cost,Node[]))
    
    while queue:
        distance, energy_node = queue.get()
        energy = energy_node[0]
        node = energy_node[1]
        cur_node = int(node[-1]) #Check cur_node as its always at the end
        if cur_node == goal: 
            #Perform print 
            utils.print_path(node)
            print("Shortest distance: " + str(distance))
            print("Total energy cost: "+ str(energy))
            return distance, energy, node
        if cur_node not in visited:
            visited.add(cur_node)
            for i in g[str(cur_node)]:
                if i not in visited:
                    temp= str(cur_node)+","+str(i)
                    new_energy = energy + cost[temp]
                    if new_energy < budget:
                        new_distance = distance + dist[temp] 
                        node_path = node.copy()
                        node_path.append(i)
                        queue.put((new_distance,(new_energy,node_path)))

st = time.perf_counter()
uniform_cost_search_constrained(graph, start, goal, budget)
end = time.perf_counter()
print("Time taken: " , (end-st))

print (" \n ============================ TASK 3 =======================================")
def heuristic(nodeA, nodeB):
    (xA, yA) = coord[str(nodeA)]
    (xB, yB) = coord[str(nodeB)]
    distance = math.sqrt((xA - xB)**2 + (yA - yB)**2)
    return distance

def a_star_search(graph, start, goal):
    pq = PriorityQueue()
    pq.put((0, start, 0)) # initial dist, start node, initial energy
    
    path = {} # Child: Parent

    distance_travelled = {} # : Node: Cost

    path[start] = None
    distance_travelled[start] = 0
    
    while not pq.empty():
        current_dist, current_node, current_energy = pq.get()
        # print(current_cost, current_node, current_energy)
        
        if int(current_node) == goal:
            # print("Found")
            path = utils.reconstruct_path(path, start, goal)
            utils.print_path(path)
            print("Shortest distance: ", current_dist)
            print("Total energy cost: ", current_energy)
            # print(came_from)
            return path, current_dist, current_energy
        else: 
            for next in graph[str(current_node)]:
                new_dist = distance_travelled[current_node] + float(dist[str(current_node)+","+str(next)])
                new_energy = current_energy + float(cost[str(current_node)+","+str(next)])
                if next not in distance_travelled or new_dist < distance_travelled[next]:
                    if new_energy < budget:
                        distance_travelled[next] = new_dist
                        priority = new_dist + heuristic(next, goal)
                        pq.put((priority, next, new_energy))
                        path[next] = current_node
    return None, None

st = time.perf_counter()

a_star_search(graph, start, goal)
end = time.perf_counter()
print("Time taken: " , (end-st))

