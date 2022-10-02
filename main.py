import json
from queue import PriorityQueue
from collections import defaultdict
from heapq import heapify, heappush, heappop
import math
import string
import time
import logging

#DEBUG LOGs

# Log_Format = "%(message)s"

# logging.basicConfig(filename = "logfile5.log",
#                     filemode = "w",
#                     format = Log_Format, 
#                     level = logging.DEBUG)
# logger = logging.getLogger()


#Load Files
with open("G.json") as gData:
    G = json.load(gData)


with open("Dist.json") as distData:
    Dist = json.load(distData)


with open("Coord.json") as coordData:
    Coord = json.load(coordData)

with open ("Cost.json") as costData:
    Cost = json.load(costData)

#Helper Class
class Helpers:
    # To get the full path from the parent dictionary
    def path_from_parents(self,parentDict, startNode, endNode):
        currentNode = endNode
        path = []

        while currentNode != startNode: 
            path.append(currentNode)
            currentNode = parentDict[str(currentNode)]  #Trace to parent in parent Dictionary

        path.append(startNode) 
        path.reverse()  #reverse order

        return path

    # Print Path
    def print_path(self,path): 
        solution_str = "Shortest path: {}" .format(path[0])
        for i in range(1,len(path)):
            if path[i] is string :
                solution_str = solution_str + "->" + path[i]
            else :
                solution_str = solution_str + "->" + str(path[i])
        print (solution_str)
    
    # Print results of the shortest distance and total energy cost
    def print_results(self,path,cost,dist):
        totalDistance = self.calculateDistance(path,dist)
        totalEnergy = self.calculateEnergyCost(path,cost)
        self.print_path(path)
        print("Shortest Distance:", totalDistance)
        print("Total Energy Cost:",totalEnergy)
        
    # Calculate total distance of the path
    def calculateDistance(self,path,dist): 
        totalDistance = 0
        for i in range(1,(len(path))):
             if path[i] is string :
                totalDistance += dist[path[i-1]+","+path[i]]
             else:
                totalDistance += dist[str(path[i-1])+","+str(path[i])]
        return totalDistance

    # Calculate total energy cost of the path
    def calculateEnergyCost(self,path, cost): 
            totalEnergy = 0
            for i in range(1, (len(path))):
                if path[i] is string :
                    totalEnergy += cost[path[i - 1] + "," + path[i]]
                else :
                    totalEnergy += cost[str(path[i - 1]) + "," + str(path[i])]
            return totalEnergy

    def calculateHeuristicDistance(self,current,end,coord): 
        current_coord = coord[str(current)]
        end_coord = coord[str(end)]
        return math.dist(current_coord,end_coord)


class Solution :
    def UCSWithoutConstraint(self,G, startingNode, endingNode, dist):
            visited = set()
            parent = {}  # Dictionary to track parents
            pathCost = defaultdict(lambda: float('inf')) # Keeps track of shortest distance to nodes
         
            min_heap = []
            
            # Add starting node to min heap
            heappush(min_heap, (0, startingNode))

            # Set distance of starting node to 0
            pathCost[startingNode] = 0
            
            # While min heap is not empty
            while min_heap:
                # Pop node with smallest distance from min heap
                _, current_node = heappop(min_heap)

                # If current node is the ending node, return path
                if current_node == endingNode:
                    break
                
                # Initialise neighbours of current node
                #neighbour_nodes = G[current_node]

                # For each neighbour of current node
                for neighbour in G[current_node]:
                    # If neighbour node has been visited, we do not add it to our path
                    if neighbour in visited:
                        continue
                    
                    # Calculate distance of neighbour node
                    newPathCost = float(pathCost[current_node] + float(dist[current_node + ',' + neighbour]))

                    # If new path cost of current neighbour node is less than current distance of neighbour node, 
                    # Update distance, parent and add to min heap
                    if pathCost[neighbour] > newPathCost: 
                        parent[neighbour] = current_node
                        pathCost[neighbour] = newPathCost
                        heappush(min_heap, (newPathCost, neighbour))

                # Add current node to visited set
                visited.add(current_node)

            return parent

    # Need to consider the energy cost constraint
    def UCSWithConstraint(self,G,startingNode,endingNode,cost,dist,budget):
        visited = set()
        min_heap = []
        
        # Same as what we did in UCSWithoutConstraint, put the starting node in the min heap, with distance 0 and energy cost 0
        #((Distance, (EnergyCost,Array of path))
        heappush(min_heap,(0,(0,[startingNode]))) 

        # Keeps track of shortest distance to nodes
        pathCost = defaultdict(lambda: float('inf'))
        pathCost[startingNode] = 0

        # While min heap is not empty
        while min_heap:
            
            # Pop node with smallest distance from min heap
            distEnergyNode = heappop(min_heap)
            
            # Seperate the distance and energy cost from the node
            distance, energyNode = distEnergyNode[0], distEnergyNode[1]
            energyCost = energyNode[0]
            nodeArr = energyNode[1]
            
            # Last node in the node array is the current node
            current_node = int(nodeArr[-1]) #last node of arr
            
            end = int(endingNode)

            # If current node is the ending node, return path
            if current_node == end :
                return nodeArr
            
            # If current node is not in the visited set
            if current_node not in visited:
                visited.add(current_node)
                
                # Initialise neighbours of current node
                for neighbour in G[str(current_node)]:
                    # If neighbour node has been visited, we do not add it to our path
                    if neighbour in visited:
                        continue

                    path_str = str(current_node)+ ","+ str(neighbour)

                    # Calculate total energy cost of the current path
                    newEnergyCost = energyCost + cost[path_str]

                    # Calculate total distance of the current path
                    newPathCost = distance + dist[path_str]

                    # If current total energy cost is less than the budget,  
                    if newEnergyCost < budget and pathCost[neighbour] > newPathCost:
                        # If new path cost of current neighbour node is less than current distance of neighbour node,
                        # add the neighbour node to the newNodeArr and push the current path into the min heap
                            pathCost[neighbour] = newPathCost
                            newNodeArr = nodeArr.copy()
                            newNodeArr.append(neighbour)
                            heappush(min_heap,(newPathCost,(newEnergyCost,newNodeArr)))

    def aStarSearch (self,G,startingNode,endingNode,dist,cost,coord,budget):
        
        # We will be using a different data structure for the A star search, which is a priority queue
        # The priority queue will be sorted by the total cost of the path, which is the distance + heuristic budget
        priorityQ = PriorityQueue()

        # Add starting node to min heap
        priorityQ.put((0,startingNode,0))
        visited = set()
        parent = {}
        pathCost = defaultdict(lambda: float('inf'))

        # Set parent of starting node to None
        parent[startingNode] = None
        # Set distance of starting node to 0
        pathCost[startingNode] = 0

        # While priority queue is not empty
        while priorityQ:
            # Pop node with smallest distance from priority queue
            _ , currentNode, currentEnergy = priorityQ.get()
           
            # If current node is the ending node, return path
            if currentNode == endingNode:
                return parent

            # If current node is not in the visited set
            if currentNode not in visited:
                visited.add(currentNode)
                # Initialise neighbours of current node
                for neighbour in G[currentNode]:
                    currNeighbourStr = currentNode+","+str(neighbour)

                    # Calculate total distance of the current path
                    newPathCost = pathCost[currentNode] + float(dist[currNeighbourStr])
                    # Calculate total energy cost of the current path
                    newEnergyCost = currentEnergy + float(cost[currNeighbourStr])

                    # If current total energy cost is less than the budget,
                    # and if new path cost of current neighbour node is less than current distance of neighbour node,
                    if pathCost[neighbour] > newPathCost and newEnergyCost < budget:
                        # we calculate the heuristic distance of the neighbour node and push the neighbour node into the priority queue
                        # We also update parent and pathCost
                        pathCost[neighbour] = newPathCost
                        f_function = newPathCost + Helpers().calculateHeuristicDistance(neighbour,endingNode,coord)
                        priorityQ.put((f_function, neighbour,newEnergyCost))
                        parent[neighbour] = currentNode
                    
        return None, None

   


#Init

budget = 287932
Sol= Solution()
helper = Helpers()


print (" ============================ TASK 1 UCS Without Constraint =======================================")
start = time.perf_counter()
pr  =Sol.UCSWithoutConstraint(G,"1","50",Dist)
end = time.perf_counter()
path = helper.path_from_parents(pr,"1","50")
helper.print_results(path,Cost,Dist)
print("Time Taken: " , (end-start))



print (" ============================ TASK 2  UCS With Constraint =======================================")
start = time.perf_counter()
pr = Sol.UCSWithConstraint(G,"1","50",Cost,Dist,budget)
helper.print_results(pr,Cost,Dist)
end = time.perf_counter()
print("Time taken: " , (end-start))



print (" ============================ TASK 3  A* Search W Budget =======================================")
start = time.perf_counter()
pr = Sol.aStarSearch(G,"1","50",Dist,Cost,Coord,budget)
path = helper.path_from_parents(pr,"1","50")
helper.print_results(path,Cost,Dist)
end = time.perf_counter()
print("Time taken: " , (end-start))

