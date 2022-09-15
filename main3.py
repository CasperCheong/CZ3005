import json
from collections import defaultdict
import heapq as heap
import copy
import math
import time

with open('G.json') as g:
    dataG = json.load(g)

with open('Cost.json') as cost:
    dataCost = json.load(cost)

with open('Dist.json') as dist:
    dataDist = json.load(dist)

with open('Coord.json') as coord:
    dataCoord = json.load(coord)


# Using UCS
def task1():
    def genPath(parent, startNode, endNode):
        path = []
        node = endNode
        while (True):
            path.insert(0, node)
            if node == startNode:
                break
            node = parent[node]
        return path

    def genEnergy(path, dataCost):
        totalEnergy = 0

        for i in range(1, (len(path))):
            totalEnergy = totalEnergy + dataCost[path[i - 1] + "," + path[i]]
        return totalEnergy

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

    parents, dist = UCS(dataG, "1", "50", dataDist)
    path = genPath(parents, "1", "50")
    energy = genEnergy(path, dataCost)
    nodePrint = "50"
    toPrint = "50"
    while (True):
        if nodePrint == "1":
            break
        toPrint = parents[nodePrint] + "->" + toPrint
        nodePrint = parents[nodePrint]

    print("Task 1 results:")
    print("Shortest Path:", toPrint)
    print("Shortest distance:", str(dist["50"]))
    print("Total energy cost:", energy)
    print()


# Using UCS
def task2():
    def genPath(parent, startNode, endNode):
        path = []
        node = endNode
        while (True):
            path.insert(0, node)
            if node == startNode:
                break
            node = parent[node]
        return path

    def genDistance(path, dataDist):
        totalDist = 0

        for i in range(1, (len(path))):
            totalDist = totalDist + dataDist[path[i - 1] + "," + path[i]]

        return totalDist

    def genEnergy(path, dataCost):
        totalEnergy = 0

        for i in range(1, (len(path))):
            totalEnergy = totalEnergy + dataCost[path[i - 1] + "," + path[i]]
        return totalEnergy

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

    def SPP_Energy(dataG, startNode, endNode, maxEnergy, dataDist, dataCost):
        parent, distMap = UCS(dataG, startNode, endNode, dataDist)
        path = genPath(parent, startNode, endNode)
        distance = genDistance(path, dataDist)
        energy = genEnergy(path, dataCost)

        if energy <= maxEnergy:
            return path

        tempDataG = copy.deepcopy(dataG)
        alternativePath = []

        for i in range(1, len(path)):
            spareNode0 = path[i - 1]  # node 1 for 1st iteration
            spareNode1 = path[i]  # node 1363 for 1st iteration

            tempDataG[spareNode0].remove(spareNode1)  # delete 1363 from node1 vertex list for 1st iteration
            tempDataG[spareNode1].remove(spareNode0)  # delete 1 from 1363 vertex list for 1st iteration

            tempParent, tempDistMap = UCS(tempDataG, spareNode0, endNode, dataDist)
            frontPath = []
            for j in range(0, i - 1):
                frontPath.append(path[j])

            if endNode in tempDistMap:
                tempPath = genPath(tempParent, spareNode0, endNode)
                for i in tempPath:
                    frontPath.append(i)
                tempEnergy = genEnergy(frontPath, dataCost)
                if tempEnergy <= maxEnergy:
                    tempDistance = genDistance(frontPath, dataDist)
                    disPath = [tempDistance, frontPath]
                    alternativePath.append(disPath)

            tempDataG[spareNode0].append(spareNode1)  # add back
            tempDataG[spareNode1].append(spareNode0)  # add back

        alternativePath.sort()
        return (alternativePath[0])

    maxEnergy = 287932
    shortestPath = SPP_Energy(dataG, "1", "50", maxEnergy, dataDist, dataCost)
    newPath = shortestPath[1]

    toPrint = ""
    for node in newPath:
        if (node == "50"):
            toPrint += str(node)
        else:
            toPrint += (str(node) + "->")
    print("Task 2 results: ")
    print("Shortest path:", toPrint)
    newDistance = genDistance(newPath, dataDist)
    print("Shortest distance:", str(newDistance))
    print("Total energy cost:", str(genEnergy(newPath, dataCost)))
    print()

# Using A Star
def task3():
    def euclidean_distance(x, y):
        coord_50_x, coord_50_y = dataCoord["50"]
        return math.sqrt((x - coord_50_x) ** 2 + (y - coord_50_y) ** 2)

    def AStar(G, startingNode, endingNode, Dist, Coord):
        parents = {}
        visited = set()
        distance = defaultdict(lambda: float('inf'))
        pq = []

        heap.heappush(pq, (0, startingNode))
        distance[startingNode] = 0

        while pq:
            _, curr = heap.heappop(pq)

            if curr == endingNode:
                break

            next = G[curr]

            for nodes in next:
                # if nodes in visited: continue
                if nodes in visited:
                    continue

                cost = float(distance[curr] + float(Dist[curr + ',' + nodes]))
                heuristic = euclidean_distance(Coord[nodes][0], Coord[nodes][1])
                f = cost + heuristic

                if distance[nodes] > cost:
                    parents[nodes] = curr
                    distance[nodes] = cost
                    heap.heappush(pq, (f, nodes))

            visited.add(curr)
        return parents, distance

    def genPath(parent, startNode, endNode):
        path = []
        node = endNode
        while (True):
            path.insert(0, node)
            if node == startNode:
                break
            node = parent[node]
        return path

    def genDistance(path, dataDist):
        totalDist = 0

        for i in range(1, (len(path))):
            totalDist = totalDist + dataDist[path[i - 1] + "," + path[i]]

        return totalDist

    def genEnergy(path, dataCost):
        totalEnergy = 0

        for i in range(1, (len(path))):
            totalEnergy = totalEnergy + dataCost[path[i - 1] + "," + path[i]]
        return totalEnergy

    def SPP_Energy(dataG, startNode, endNode, maxEnergy, dataDist, dataCost, dataCoord):
        parent, distMap = AStar(dataG, startNode, endNode, dataDist, dataCoord)
        path = genPath(parent, startNode, endNode)
        distance = genDistance(path, dataDist)
        energy = genEnergy(path, dataCost)

        if energy <= maxEnergy:
            return path

        tempDataG = copy.deepcopy(dataG)
        alternativePath = []

        for i in range(1, len(path)):
            spareNode0 = path[i - 1]  # node 1 for 1st iteration
            spareNode1 = path[i]  # node 1363 for 1st iteration

            tempDataG[spareNode0].remove(spareNode1)  # delete 1363 from node1 vertex list for 1st iteration
            tempDataG[spareNode1].remove(spareNode0)  # delete 1 from 1363 vertex list for 1st iteration

            tempParent, tempDistMap = AStar(tempDataG, spareNode0, endNode, dataDist, dataCoord)
            frontPath = []
            for j in range(0, i - 1):
                frontPath.append(path[j])

            if endNode in tempDistMap:
                tempPath = genPath(tempParent, spareNode0, endNode)
                for i in tempPath:
                    frontPath.append(i)
                tempEnergy = genEnergy(frontPath, dataCost)
                if tempEnergy <= maxEnergy:
                    tempDistance = genDistance(frontPath, dataDist)
                    disPath = [tempDistance, frontPath]
                    alternativePath.append(disPath)

            tempDataG[spareNode0].append(spareNode1)  # add back
            tempDataG[spareNode1].append(spareNode0)  # add back

        alternativePath.sort()
        return (alternativePath[0])

    maxEnergy = 287932
    shortestPath = SPP_Energy(dataG, "1", "50", maxEnergy, dataDist, dataCost, dataCoord)
    newPath = shortestPath[1]

    toPrint = ""
    for node in newPath:
        if (node == "50"):
            toPrint += str(node)
        else:
            toPrint += (str(node) + "->")
    print("Task 3 results: ")
    print("Shortest path:", toPrint)
    newDistance = genDistance(newPath, dataDist)
    print("Shortest distance:", str(newDistance))
    print("Total energy cost:", str(genEnergy(newPath, dataCost)))
    print()

st = time.perf_counter()
task1()
end = time.perf_counter()
print("Time taken: " , (end-st))
st = time.perf_counter()
task2()
end = time.perf_counter()
print("Time taken: " , (end-st))
st = time.perf_counter()
task3()
end = time.perf_counter()
print("Time taken: " , (end-st))