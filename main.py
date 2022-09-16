import json
from queue import PriorityQueue
from collections import defaultdict
import heapq as heap
import math
from re import I
import string
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
            if path[i] is string :
                solution_str = solution_str + "->" + path[i]
            else :
                solution_str = solution_str + "->" + str(path[i])
        print (solution_str)
    
    def print_results(self,path,cost,dist):
        totalDistance = self.calculateDistance(path,dist)
        totalEnergy = self.calculateEnergyCost(path,cost)
        self.print_path(path)
        print("Shortest Distance:", totalDistance)
        print("Total Energy Cost:",totalEnergy)
        

    def calculateDistance(self,path,dist):
        totalDistance = 0
        for i in range(1,(len(path))):
             if path[i] is string :
                totalDistance += dist[path[i-1]+","+path[i]]
             else:
                totalDistance += dist[str(path[i-1])+","+str(path[i])]
        return totalDistance

    def calculateEnergyCost(self,path, cost):
            totalEnergy = 0
            for i in range(1, (len(path))):
                if path[i] is string :
                    totalEnergy += cost[path[i - 1] + "," + path[i]]
                else :
                    totalEnergy += cost[str(path[i - 1]) + "," + str(path[i])]
            return totalEnergy

    def calculateHeuristicDistance(self,current,end,coord): #cuurent end input wiht int
        current_coord = coord[str(current)]
        end_coord = coord[str(end)]
        return math.dist(current_coord,end_coord)


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
        
        heap.heappush(priorityQ,(0,(0,[startingNode]))) #((Distance, (EnergyCost,Array of path))
        
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

    def aStarSearch (self,G,startingNode,endingNode , dist,cost,coord,budget):
    
        priorityQ = PriorityQueue()
        priorityQ.put((0,startingNode,0))

        parentDict = {}
        distanceDict = {}

        parentDict[startingNode] = None
        distanceDict[startingNode] = 0

        while not priorityQ.empty():
            _ , currentNode, currentEnergy = priorityQ.get()
           
            if currentNode == endingNode:
                return parentDict

            else:
                for neighbours in G[currentNode]:
                    currNeighbourStr = currentNode+","+str(neighbours)
                    newDist = distanceDict[currentNode] + float(dist[currNeighbourStr])
                    newEnergy = currentEnergy + float(cost[currNeighbourStr])
                    if neighbours not in distanceDict or newDist < distanceDict[neighbours]:
                        if newEnergy < budget:
                            distanceDict[neighbours] = newDist
                            f_function = newDist + Helpers().calculateHeuristicDistance(neighbours,endingNode,coord)
                            priorityQ.put((f_function, neighbours,newEnergy))
                            parentDict[neighbours] = currentNode
        return None, None

   


#Setting Parameter
start = "1"
end = "50"
budget = 287932
Sol= Solution()
helper = Helpers()


print (" ============================ TASK 1 UCS Without Constraint =======================================")
st = time.perf_counter()
pr  =Sol.UCSWithoutConstraint(G,"1","50",Dist)
end = time.perf_counter()
path = helper.path_from_parents(pr,"1","50")
helper.print_results(path,Cost,Dist)
print("Time taken: " , (end-st))
print (" ============================ TASK 2  UCS Without Constraint =======================================")
st = time.perf_counter()
pr = Sol.UCSWithConstraint(G,"1","50",Cost,Dist,budget)
helper.print_results(pr,Cost,Dist)
end = time.perf_counter()

print("Time taken: " , (end-st))
print (" ============================ TASK 3  A* Search W Budget =======================================")
st = time.perf_counter()
pr = Sol.aStarSearch(G,"1","50",Dist,Cost,Coord,budget)
path = helper.path_from_parents(pr,"1","50")
helper.print_results(path,Cost,Dist)
end = time.perf_counter()
print("Time taken: " , (end-st))

