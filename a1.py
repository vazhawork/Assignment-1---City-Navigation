import heapq
import time
import math

# Mendefinisi kota dan jalan
cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

roads = {
    "A": ["B", "E"],
    "B": ["A", "C"],
    "C": ["B", "D"],
    "D": ["C"],
    "E": ["A", "D"]
}

# Fungsi untuk menghitung jarak Euclidean
def euclidean_distance(a, b):
    (x1, y1), (x2, y2) = cities[a], cities[b]
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

# Mendefinisikan A* 
def a_star_search(start, goal):
    open_set = []
    heapq.heappush(open_set, (0 + euclidean_distance(start, goal), 0, start, [start]))
    visited = set()
    node_count = 0

    while open_set:
        f, g, current, path = heapq.heappop(open_set)
        node_count += 1
        if current == goal:
            return path, g, node_count
        if current in visited:
            continue
        visited.add(current)
        for neighbor in roads[current]:
            if neighbor not in visited:
                new_g = g + euclidean_distance(current, neighbor)
                new_f = new_g + euclidean_distance(neighbor, goal)
                heapq.heappush(open_set, (new_f, new_g, neighbor, path + [neighbor]))
    return None, float('inf'), node_count

# Mendefinisikan GBFS
def gbfs_search(start, goal):
    open_set = []
    heapq.heappush(open_set, (euclidean_distance(start, goal), start, [start]))
    visited = set()
    node_count = 0

    while open_set:
        h, current, path = heapq.heappop(open_set)
        node_count += 1
        if current == goal:
            return path, node_count
        if current in visited:
            continue
        visited.add(current)
        for neighbor in roads[current]:
            if neighbor not in visited:
                heapq.heappush(open_set, (euclidean_distance(neighbor, goal), neighbor, path + [neighbor]))
    return None, node_count

# Untuk Running A*
start_time = time.time()
path_a_star, cost_a_star, nodes_a_star = a_star_search("A", "D")
end_time = time.time()
elapsed_time_a_star = (end_time - start_time) * 1000

# Untuk Running GBFS
start_time = time.time()
path_gbfs, nodes_gbfs = gbfs_search("A", "D")
end_time = time.time()
elapsed_time_gbfs = (end_time - start_time) * 1000

# Menampilkan hasil A*
print("A*")
print(f"Path: {path_a_star}")
print(f"Cost: {cost_a_star}")
print(f"Nodes explored: {nodes_a_star}")
print(f"Elapsed time: {elapsed_time_a_star:.2f} ms")

# Menampilkan hasil GBFS
print("\nGBFS")
print(f"Path: {path_gbfs}")
print(f"Nodes explored: {nodes_gbfs}")
print(f"Elapsed time: {elapsed_time_gbfs:.2f} ms")