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

logging.basicConfig(filename = "logfile5.log",
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


class Helpers:
    def path_from_parents(self,parentDict, startNode, endNode):
        currentNode = endNode
        path = []

        while currentNode != startNode: 
            path.append(currentNode)
            currentNode = parentDict[str(currentNode)]  #Trace to parent in parent Dictionary

        path.append(startNode) 
        path.reverse()  #reverse order

        return path

    def print_path(self,path):
        solution_str = "Shortest path: {}" .format(path[0])
        for i in range(1,len(path)):
            solution_str = solution_str + "->" + path[i]
        print (solution_str)
    
    def print_results(self,path,cost,dist):
        totalDistance = self.calculateDistance(path,dist)
        totalEnergy = self.calculateEnergyCost(path,cost)
        solution_str = "Shortest path: {}" .format(path[0])
        for i in range(1,len(path)):
            solution_str = solution_str + "->" + path[i]
        print (solution_str)
        print("Shortest Distance:", totalDistance)
        print("Total Energy Cost:",totalEnergy)
        

    def calculateDistance(self,path,dist):
        totalDistance = 0
        for i in range(1,(len(path))):
            totalDistance += dist[path[i-1]+","+path[i]]
        return totalDistance

    def calculateEnergyCost(self,path, cost):
            totalEnergy = 0
            for i in range(1, (len(path))):
                totalEnergy += cost[path[i - 1] + "," + path[i]]
            return totalEnergy


class Solution :
    def UCSWithoutConstraint(self,G, startingNode, endingNode, dist):
            visited = set()
            parentDict = {}  #use Dictionary to track parents
            distance = defaultdict(lambda: float('inf')) #keeps track of shortest distance to nodes
         
            priorityQ = []


            heap.heappush(priorityQ, (0, startingNode))
            distance[startingNode] = 0
            
      
            while priorityQ:
                _, current_node = heap.heappop(priorityQ)


                if current_node == endingNode:
                    break

                neighbour_nodes = G[current_node]

                for nodes in neighbour_nodes:
                
                    if nodes in visited:
                        continue

                    pathCost = float(distance[current_node] + float(dist[current_node + ',' + nodes]))

                    if distance[nodes] > pathCost:  #if new Path is shorter than current shortest path update
                        parentDict[nodes] = current_node
                        distance[nodes] = pathCost
                        heap.heappush(priorityQ, (pathCost, nodes))

                visited.add(current_node)

            return parentDict

    def UCSWithConstraint(self,G,startingNode,endingNode,cost,dist,budget):
        visited = set()
        priorityQ = []
        
        heap.heappush(priorityQ,(0,(0,[str(startingNode)]))) #((Distance, (EnergyCost,Array of path))
        
        while priorityQ:
            
            distEnergyNode = heap.heappop(priorityQ)
            distance , energyNode = distEnergyNode[0],distEnergyNode[1]
            energyCost =energyNode[0]
            nodeArr = energyNode[1]
            
            current_node = int(nodeArr[-1]) #last node of arr
            
            end = int(endingNode)
            if current_node == end :
                
                return nodeArr
            
            if current_node not in visited :
                visited.add(current_node)
                
                for neighbour in G[str(current_node)]:
                    if neighbour not in visited:
                        path_str = str(current_node)+ ","+ str(neighbour)
                        newEnergyCost = energyCost + cost[path_str]
                        if newEnergyCost < budget :
                            newDistance = distance + dist[path_str]
                            newNodeArr = nodeArr.copy()
                            newNodeArr.append(neighbour)
                            heap.heappush(priorityQ,(newDistance,(newEnergyCost,newNodeArr)))






x= Solution()
y = Helpers()

print (" ============================ TASK 1 =======================================")
st = time.perf_counter()
pr  =x.UCSWithoutConstraint(G,"1","50",Dist)
end = time.perf_counter()
path = y.path_from_parents(pr,"1","50")
y.print_results(path,Cost,Dist)
print("Time taken: " , (end-st))
print (" ============================ TASK 2 =======================================")
st = time.perf_counter()
pr = x.UCSWithConstraint(G,"1","50",Cost,Dist,budget)
y.print_results(pr,Cost,Dist)
end = time.perf_counter()

print("Time taken: " , (end-st))

# print (" ============================ TASK 1 =======================================")
# def uniform_cost_search(graph, start, goal,dist):
#     visited=set()
#     queue = PriorityQueue()
#     queue.put((0, [start])) #((Distance, (Cost,Node[]))

    
#     while queue:
        
#         # logger.debug(queue.queue)
#         distance, node = queue.get()
#         # print(node)
#         cur_node = int(node[-1]) #Check cur_node as its always at the end

#         if cur_node == goal: 
#             #Perform print 
#             Solution = Helpers()
#             Solution.print_path(node)
#             # utils.print_path(node)
#             print("Shortest distance: " + str(distance))
#             return distance, node
#         if cur_node not in visited:
#             visited.add(cur_node)
#             for i in graph[str(cur_node)]:
#                 if i not in visited:
#                     temp= str(cur_node)+","+str(i)
#                     new_distance = distance + dist[temp] 
#                     node_path = node.copy()
#                     node_path.append(i)
#                     queue.put((new_distance,node_path))

# st = time.perf_counter()
# uniform_cost_search(G, start, goal,Dist)
# end = time.perf_counter()
# print("Time taken: " , (end-st))

# print ("\n ============================ TASK 2 =======================================")
# def uniform_cost_search_constrained(g, start, goal, budget,cost,dist):
#     visited=set()
#     queue = PriorityQueue()
#     queue.put((0, (0,[str(start)]))) #((Distance, (Cost,Node[]))
    
#     while queue:
#         distance, energy_node = queue.get()
#         energy = energy_node[0]
#         node = energy_node[1]
#         cur_node = int(node[-1]) #Check cur_node as its always at the end
#         if cur_node == goal: 
#             #Perform print 
#             utils.print_path(node)
#             print("Shortest distance: " + str(distance))
#             print("Total energy cost: "+ str(energy))
#             return distance, energy, node
#         if cur_node not in visited:
#             visited.add(cur_node)
#             for i in g[str(cur_node)]:
#                 if i not in visited:
#                     temp= str(cur_node)+","+str(i)
#                     new_energy = energy + cost[temp]
#                     if new_energy < budget:
#                         new_distance = distance + dist[temp] 
#                         node_path = node.copy()
#                         node_path.append(i)
#                         queue.put((new_distance,(new_energy,node_path)))

# st = time.perf_counter()
# uniform_cost_search_constrained(G, start, goal, budget,Cost,Dist)
# end = time.perf_counter()
# print("Time taken: " , (end-st))

# print (" \n ============================ TASK 3 =======================================")
# def heuristic(nodeA, nodeB,coord):
#     (xA, yA) = coord[str(nodeA)]
#     (xB, yB) = coord[str(nodeB)]
#     distance = math.sqrt((xA - xB)**2 + (yA - yB)**2)
#     return distance

# def a_star_search(graph, start, goal,dist,cost,coord):
#     pq = PriorityQueue()
#     pq.put((0, start, 0)) # initial dist, start node, initial energy
    
#     path = {} # Child: Parent

#     distance_travelled = {} # : Node: Cost

#     path[start] = None
#     distance_travelled[start] = 0
    
#     while not pq.empty():
#         current_dist, current_node, current_energy = pq.get()
#         # print(current_cost, current_node, current_energy)
        
#         if int(current_node) == goal:
#             # print("Found")
#             path = utils.reconstruct_path(path, start, goal)
#             utils.print_path(path)
#             print("Shortest distance: ", current_dist)
#             print("Total energy cost: ", current_energy)
#             # print(came_from)
#             return path, current_dist, current_energy
#         else: 
#             for next in graph[str(current_node)]:
#                 new_dist = distance_travelled[current_node] + float(dist[str(current_node)+","+str(next)])
#                 new_energy = current_energy + float(cost[str(current_node)+","+str(next)])
#                 if next not in distance_travelled or new_dist < distance_travelled[next]:
#                     if new_energy < budget:
#                         distance_travelled[next] = new_dist
#                         priority = new_dist + heuristic(next, goal,coord)
#                         pq.put((priority, next, new_energy))
#                         path[next] = current_node
#     return None, None

# st = time.perf_counter()

# a_star_search(G, start, goal,Dist,Cost,Coord)
# end = time.perf_counter()
# print("Time taken: " , (end-st))

