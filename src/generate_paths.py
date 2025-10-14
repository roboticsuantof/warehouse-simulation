
import xml.etree.ElementTree as ET
import random
import math
import heapq

# --- CONFIGURATION ---
SDF_FILE_PATH = "final_world/industrial-warehouse.sdf"
ACTOR_NAMES = ["actor_walking", "actor_walking_2", "actor_walking_3"]

# Pathfinding & Trajectory configuration
PATH_SEGMENTS = 20 # Create a longer, more interesting path
PATH_SIMPLIFICATION_TOLERANCE = 0.3
ACTOR_SPEED = 1.0 # meters per second

# World and Grid configuration
X_MIN, X_MAX = -8, 8
Y_MIN, Y_MAX = -12, 12
GRID_RESOLUTION = 0.2
OBSTACLE_INFLATION_RADIUS = 5 # Increased from 3

# Heuristics for obstacle dimensions
OBSTACLE_APPROXIMATIONS = {
    "Shelf": (4.0, 0.8), "Bucket": (0.6, 0.6),
    "Cluttering": (1.2, 1.2), "PalletJack": (1.5, 1.0), "TrashCan": (0.6, 0.6),
    "HospitalRobot": (0.7, 0.7),
    "Wall": (24.0, 0.2) # Added Wall approximation
}

# --- HELPER CLASSES & FUNCTIONS ---

class OccupancyGrid:
    def __init__(self, x_min, x_max, y_min, y_max, resolution):
        self.x_min, self.y_min, self.resolution = x_min, y_min, resolution
        self.width = int((x_max - x_min) / resolution)
        self.height = int((y_max - y_min) / resolution)
        self.grid = [[0] * self.height for _ in range(self.width)]

    def world_to_grid(self, wx, wy):
        return int((wx - self.x_min) / self.resolution), int((wy - self.y_min) / self.resolution)

    def grid_to_world(self, gx, gy):
        return gx * self.resolution + self.x_min, gy * self.resolution + self.y_min

    def is_valid(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied(self, gx, gy):
        return not self.is_valid(gx, gy) or self.grid[gx][gy] == 1

    def add_rotated_obstacle(self, wx, wy, w, h, yaw):
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        hw, hh = w / 2.0, h / 2.0
        corners = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        rotated_corners = [(wx + x*cos_y - y*sin_y, wy + x*sin_y + y*cos_y) for x, y in corners]
        grid_corners = [self.world_to_grid(rx, ry) for rx, ry in rotated_corners]
        min_gx, max_gx = max(0, min(c[0] for c in grid_corners)), min(self.width - 1, max(c[0] for c in grid_corners))
        min_gy, max_gy = max(0, min(c[1] for c in grid_corners)), min(self.height - 1, max(c[1] for c in grid_corners))

        for gx in range(min_gx, max_gx + 1):
            for gy in range(min_gy, max_gy + 1):
                self.grid[gx][gy] = 1

    def inflate_obstacles(self, radius):
        new_grid = [row[:] for row in self.grid]
        for gx in range(self.width):
            for gy in range(self.height):
                if self.grid[gx][gy] == 1:
                    for dx in range(-radius, radius + 1):
                        for dy in range(-radius, radius + 1):
                            nx, ny = gx + dx, gy + dy
                            if self.is_valid(nx, ny): new_grid[nx][ny] = 1
        self.grid = new_grid

    def get_random_free_cell(self):
        while True:
            gx, gy = random.randint(0, self.width - 1), random.randint(0, self.height - 1)
            if not self.is_occupied(gx, gy): return gx, gy

def a_star_pathfinding(grid, start, end):
    open_set = [(0, start)]
    came_from, g_score = {}, {start: 0}
    f_score = {start: math.hypot(end[0] - start[0], end[1] - start[1])}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == end:
            path = []
            while current in came_from: path.append(current); current = came_from[current]
            return path[::-1]

        for dx, dy in [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if not grid.is_valid(*neighbor) or grid.is_occupied(*neighbor): continue
            tentative_g = g_score[current] + math.hypot(dx, dy)
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current; g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + math.hypot(end[0]-neighbor[0], end[1]-neighbor[1])
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

def perpendicular_distance(point, line_start, line_end):
    x0, y0 = point
    x1, y1 = line_start
    x2, y2 = line_end
    if (x1, y1) == (x2, y2): return math.hypot(x0 - x1, y0 - y1)
    return abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / math.hypot(y2 - y1, x2 - x1)

def simplify_path_rdp(path, epsilon):
    if len(path) < 3: return path
    dmax, index = 0, 0
    for i in range(1, len(path) - 1):
        d = perpendicular_distance(path[i], path[0], path[-1])
        if d > dmax: index, dmax = i, d
    
    if dmax > epsilon:
        rec_results1 = simplify_path_rdp(path[:index+1], epsilon)
        rec_results2 = simplify_path_rdp(path[index:], epsilon)
        return rec_results1[:-1] + rec_results2
    else:
        return [path[0], path[-1]]

def main():
    print("1. Initializing grid and parsing SDF...")
    occ_grid = OccupancyGrid(X_MIN, X_MAX, Y_MIN, Y_MAX, GRID_RESOLUTION)

    # Add boundary walls manually
    wall_thickness = 0.2
    occ_grid.add_rotated_obstacle((X_MIN + X_MAX) / 2, Y_MAX - wall_thickness / 2, (X_MAX - X_MIN), wall_thickness, 0)  # Top
    occ_grid.add_rotated_obstacle((X_MIN + X_MAX) / 2, Y_MIN + wall_thickness / 2, (X_MAX - X_MIN), wall_thickness, 0)  # Bottom
    occ_grid.add_rotated_obstacle(X_MIN + wall_thickness / 2, (Y_MIN + Y_MAX) / 2, wall_thickness, (Y_MAX - Y_MIN), 0)  # Left
    occ_grid.add_rotated_obstacle(X_MAX - wall_thickness / 2, (Y_MIN + Y_MAX) / 2, wall_thickness, (Y_MAX - Y_MIN), 0)  # Right

    try:
        tree = ET.parse(SDF_FILE_PATH)
        world = tree.getroot().find("world")
    except (FileNotFoundError, ET.ParseError) as e:
        print(f"Error: Could not read or parse SDF file '{SDF_FILE_PATH}'. {e}")
        return

    print("2. Building obstacle map from SDF...")
    for model in world.findall("model"):
        model_name = model.get("name", "")
        if model_name in ACTOR_NAMES: continue
        
        dims = next((v for k, v in OBSTACLE_APPROXIMATIONS.items() if k in model_name), None)
        pose_tag = model.find("pose")
        if dims and pose_tag is not None and pose_tag.text:
            pose = [float(v) for v in pose_tag.text.split()]
            # Use 0 for yaw if pose is only x, y, z
            yaw = pose[5] if len(pose) == 6 else 0
            occ_grid.add_rotated_obstacle(pose[0], pose[1], dims[0], dims[1], yaw)

    print("3. Inflating obstacles...")
    occ_grid.inflate_obstacles(OBSTACLE_INFLATION_RADIUS)

    print("4. Generating and updating trajectories in SDF...")
    for name in ACTOR_NAMES:
        actor = world.find(f".//actor[@name='{name}']")
        if actor is None:
            print(f"  - Could not find actor '{name}' in SDF file.")
            continue

        trajectory = actor.find(".//trajectory")
        if trajectory is None:
            print(f"  - Could not find trajectory for actor '{name}'.")
            continue

        # Generate a new path
        start_node = occ_grid.get_random_free_cell()
        full_grid_path = [start_node]
        last_point = start_node

        for i in range(PATH_SEGMENTS):
            is_last_segment = (i == PATH_SEGMENTS - 1)
            end_point = start_node if is_last_segment else occ_grid.get_random_free_cell()
            
            segment = a_star_pathfinding(occ_grid, last_point, end_point)
            if segment: 
                full_grid_path.extend(segment[1:])
                last_point = end_point
        
        if not full_grid_path: 
            print(f"  - Could not generate path for {name}")
            continue

        simplified_grid_path = simplify_path_rdp(full_grid_path, PATH_SIMPLIFICATION_TOLERANCE)
        
        # Clear existing waypoints
        for waypoint in trajectory.findall("waypoint"):
            trajectory.remove(waypoint)

        # Add new waypoints
        current_time = 0.0
        last_wp_world = None
        for gx, gy in simplified_grid_path:
            wx, wy = occ_grid.grid_to_world(gx, gy)
            
            if last_wp_world is not None:
                dist = math.hypot(wx - last_wp_world[0], wy - last_wp_world[1])
                current_time += dist / ACTOR_SPEED

            # For simplicity, calculate yaw based on the next point
            # This is a rough approximation
            yaw = 0
            current_index = simplified_grid_path.index((gx,gy))
            if current_index + 1 < len(simplified_grid_path):
                next_gx, next_gy = simplified_grid_path[current_index+1]
                next_wx, next_wy = occ_grid.grid_to_world(next_gx, next_gy)
                yaw = math.atan2(next_wy - wy, next_wx - wx)

            waypoint_elem = ET.SubElement(trajectory, "waypoint")
            time_elem = ET.SubElement(waypoint_elem, "time")
            time_elem.text = f"{current_time:.2f}"
            pose_elem = ET.SubElement(waypoint_elem, "pose")
            pose_elem.text = f"{wx:.4f} {wy:.4f} 1.05 0 0 {yaw:.4f}"
            
            last_wp_world = (wx, wy)

        print(f"  - {name}: Updated trajectory with {len(simplified_grid_path)} waypoints.")

    # Write the modified SDF back to the file
    tree.write(SDF_FILE_PATH, encoding="utf-8", xml_declaration=True)
    print(f"\nSuccessfully updated trajectories in '{SDF_FILE_PATH}'")


if __name__ == "__main__":
    main()
