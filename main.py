import streamlit as st
import heapq
import random

# Constants
GRID_SIZE = 10
OBSTACLES = 15
WEATHER_HAZARDS = 10

# Initialize grid
grid = [['.' for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
start = (0, 0)
end = (GRID_SIZE - 1, GRID_SIZE - 1)

# Generate grid with obstacles and weather
def generate_grid():
    global grid
    grid = [['.' for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    grid[start[0]][start[1]] = 'S'
    grid[end[0]][end[1]] = 'D'

    for _ in range(OBSTACLES):
        x, y = random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1)
        if grid[x][y] == '.':
            grid[x][y] = '#'

    for _ in range(WEATHER_HAZARDS):
        x, y = random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1)
        if grid[x][y] == '.':
            grid[x][y] = 'W'

# Heuristic function for A* (Manhattan Distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# A* pathfinding algorithm
def a_star(start, goal):
    heap = []
    heapq.heappush(heap, (0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while heap:
        _, current = heapq.heappop(heap)

        if current == goal:
            break

        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            next_node = (current[0] + dx, current[1] + dy)
            if 0 <= next_node[0] < GRID_SIZE and 0 <= next_node[1] < GRID_SIZE:
                terrain = grid[next_node[0]][next_node[1]]
                if terrain == '#':
                    continue
                cost = 1
                if terrain == 'W':
                    cost = 5  # Weather makes it slower
                new_cost = cost_so_far[current] + cost
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)
                    heapq.heappush(heap, (priority, next_node))
                    came_from[next_node] = current

    # Reconstruct path
    path = []
    current = goal
    while current != start:
        path.append(current)
        current = came_from.get(current)
        if current is None:
            return []
    path.append(start)
    path.reverse()
    return path

# Streamlit UI
st.title("🚨 Emergency Drone Route Planner (A* Algorithm)")
st.sidebar.header("🛰️ Drone Input")

# Destination input
x = st.sidebar.slider("Destination X", 0, GRID_SIZE - 1, GRID_SIZE - 1)
y = st.sidebar.slider("Destination Y", 0, GRID_SIZE - 1, GRID_SIZE - 1)
end = (x, y)

# Generate map and path
if st.button("Generate Grid & Find Route"):
    generate_grid()
    path = a_star(start, end)

    # Show grid
    for i in range(GRID_SIZE):
        row = ""
        for j in range(GRID_SIZE):
            cell = grid[i][j]
            if (i, j) == start:
                row += "🟢 "  # Start
            elif (i, j) == end:
                row += "🔴 "  # Destination
            elif (i, j) in path:
                row += "🟡 "  # Path
            elif cell == '#':
                row += "⬛ "  # Obstacle
            elif cell == 'W':
                row += "🌧️ "  # Weather
            else:
                row += "⬜ "  # Empty
        st.markdown(row)

    # Result
    if path:
        st.success("✅ Route Found!")
        st.code(f"Path: {path}")
    else:
        st.error("❌ No Route Available!")
