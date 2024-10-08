import heapq

# Node class to represent each cell in the grid
class Node:
    def __init__(self, x, y, cost=0, h=0, parent=None):
        self.x = x  # X-coordinate
        self.y = y  # Y-coordinate
        self.cost = cost  # G-cost (distance from the start node)
        self.h = h  # H-cost (heuristic distance to the goal)
        self.parent = parent  # Reference to the parent node

    def __lt__(self, other):
        # Less than operator for priority queue comparison
        return (self.cost + self.h) < (other.cost + other.h)

def heuristic(a, b):
    # Manhattan distance heuristic
    return abs(a.x - b.x) + abs(a.y - b.y)

def a_star(start, goal, grid):
    # Priority queue to store the open list of nodes to be evaluated
    open_list = []
    heapq.heappush(open_list, start)

    # Set to store visited nodes (closed list)
    visited = set()

    while open_list:
        # Get the node with the lowest f-cost (g + h)
        current = heapq.heappop(open_list)
        
        # If we have reached the goal, return the path
        if current.x == goal.x and current.y == goal.y:
            path = []
            while current:
                path.append((current.x, current.y))
                current = current.parent
            return path[::-1]  # Return reversed path (from start to goal)

        visited.add((current.x, current.y))

        # Explore the neighbors (up, down, left, right)
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor_x, neighbor_y = current.x + dx, current.y + dy

            # Skip if out of bounds or blocked by an obstacle
            if not (0 <= neighbor_x < len(grid)) or not (0 <= neighbor_y < len(grid[0])):
                continue
            if grid[neighbor_x][neighbor_y] == 1:
                continue

            # If the neighbor has already been visited, skip it
            if (neighbor_x, neighbor_y) in visited:
                continue

            # Calculate new G-cost (distance from start to neighbor)
            new_cost = current.cost + 1
            neighbor = Node(neighbor_x, neighbor_y, new_cost, heuristic(Node(neighbor_x, neighbor_y), goal), current)

            # Add the neighbor to the open list for evaluation
            heapq.heappush(open_list, neighbor)

    # If no path was found, return None
    return None

def print_grid(grid, path):
    # Create a visual representation of the grid and path
    for x in range(len(grid)):
        for y in range(len(grid[0])):
            if (x, y) in path:
                print("P", end=" ")  # Path
            elif grid[x][y] == 1:
                print("X", end=" ")  # Obstacle
            else:
                print(".", end=" ")  # Empty space
        print()

# Example usage
if __name__ == "__main__":
    # Define the grid (0 = empty, 1 = obstacle)
    grid = [
        [0, 0, 0, 0, 0, 0],
        [1, 1, 0, 1, 1, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 1, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 0],
    ]

    start_node = Node(0, 0)  # Starting position (top-left corner)
    goal_node = Node(4, 5)   # Goal position (bottom-right corner)

    path = a_star(start_node, goal_node, grid)

    if path:
        print("Path found:")
        print(path)
        print("\nGrid with path:")
        print_grid(grid, path)
    else:
        print("No path found!")
