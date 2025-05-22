import heapq
import math

def neighbors(current):
    x, y = current
    movements = [(0, 1), (0, -1), (-1, 0), (1, 0)]
    neighbor_list = []
    for dx, dy in movements:
        neighbor_list.append((x + dx, y + dy))
    return neighbor_list

def heuristic_distance(candidate, goal):
    return abs(candidate[0] - goal[0]) + abs(candidate[1] - goal[1])

def get_path_from_A_star(start, goal, obstacles):
    obstacle_set = set(obstacles)

    open_set = []
    heapq.heappush(open_set, (0 + heuristic_distance(start, goal), start))

    g_scores = {start: 0}

    came_from = {}

    while open_set:
        current_f_score, current_node = heapq.heappop(open_set)

        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.reverse()
            return path

        for neighbor in neighbors(current_node):
            if neighbor in obstacle_set:
                continue

            tentative_g_score = g_scores[current_node] + 1

            if neighbor not in g_scores or tentative_g_score < g_scores[neighbor]:
                g_scores[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic_distance(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
                came_from[neighbor] = current_node

    return []

if __name__ == '__main__':
    start = (0, 0)
    goal = (-5, -2)
    obstacles = [(-2, 1), (-2, 0), (-2, -1), (-2, -2), (-4, -2), (-4, -3)]
    path = get_path_from_A_star(start, goal, obstacles)
    print(f"start: {start}, goal: {goal}")
    print(f"obstacles: {obstacles}")
    print(f"path: {path}")

    start2 = (0, 0)
    goal2 = (2, 2)
    obstacles2 = [(1, 0), (1, 1)]
    path2 = get_path_from_A_star(start2, goal2, obstacles2)
    print(f"\nstart: {start2}, goal: {goal2}")
    print(f"obstacles: {obstacles2}")
    print(f"path: {path2}")