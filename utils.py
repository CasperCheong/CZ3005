# UTILITY FUNCTIONS 
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []

    while current != start: # note: this will fail if no path found
        path.append(current)
        current = came_from[str(current)]

    path.append(start) # optional
    path.reverse() # optional

    return path

def print_path(path):
    print(f"Shortest path: {path[0]}", end="")
    for i in range(1, len(path)):
        print("->", end="")
        print(path[i], end="")
    print()