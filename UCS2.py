
import json
from queue import PriorityQueue
from collections import defaultdict
import heapq as heap
import math
from re import I
import time
import utils
import logging

#DEBUG LOGs

Log_Format = "%(levelname)s %(asctime)s - %(message)s"

logging.basicConfig(filename = "logfile6.log",
                    filemode = "w",
                    format = Log_Format, 
                    level = logging.DEBUG)

logger = logging.getLogger()

#Setting Parameter
start = 1
goal = 50
budget = 287932

#Load Files
with open("G.json") as gData:
    G = json.load(gData)


with open("Dist.json") as distData:
    Dist = json.load(distData)


with open("Coord.json") as coordData:
    Coord = json.load(coordData)

with open ("Cost.json") as costData:
    Cost = json.load(costData)

print ("\n ============================ TASK 2 =======================================")
def uniform_cost_search_constrained(g, start, goal, budget,cost,dist):
    visited=set()
    queue = PriorityQueue()
    queue.put((0, (0,[str(start)]))) #((Distance, (Cost,Node[]))
    
    while queue:
        
        distance, energy_node = queue.get()
        
        energy = energy_node[0]
        node = energy_node[1]
        cur_node = int(node[-1]) #Check cur_node as its always at the end
        logger.debug(cur_node)
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
uniform_cost_search_constrained(G, start, goal, budget,Cost,Dist)
end = time.perf_counter()
print("Time taken: " , (end-st))