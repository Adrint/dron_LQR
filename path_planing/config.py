"""
Configuration file for path planning system
"""

# ============================================================================
# DRONE PARAMETERS (should match your LQR drone parameters)
# ============================================================================
DRONE_VELOCITY = 2.0  # m/s - default velocity (can be changed)
SAFETY_DISTANCE = 1.5  # m - minimum distance from obstacles
DRONE_SIZE = 0.5  # m - approximate drone size (radius)

# ============================================================================
# PATH PLANNING PARAMETERS
# ============================================================================
GRID_RESOLUTION = 0.5  # m - grid size for discretization
DEFAULT_ALTITUDE = 2.0  # m - default flight altitude above terrain

# Algorithm selection
AVAILABLE_ALGORITHMS = ['astar', 'rrt', 'rrt_star', 'dijkstra']
DEFAULT_ALGORITHM = 'astar'

# RRT parameters
RRT_MAX_ITERATIONS = 5000
RRT_STEP_SIZE = 1.0  # m
RRT_GOAL_SAMPLE_RATE = 0.1  # 10% chance to sample goal

# RRT* parameters
RRT_STAR_SEARCH_RADIUS = 3.0  # m

# ============================================================================
# MAP PARAMETERS
# ============================================================================
DEFAULT_MAP_SIZE = (100, 100, 30)  # X, Y, Z in meters
MAP_RESOLUTION = 0.5  # m per grid cell

# ============================================================================
# TRAJECTORY EXPORT PARAMETERS
# ============================================================================
EXPORT_TIME_STEP = 0.01  # s - must match LQR controller dt
EXPORT_FORMAT = 'numpy'  # 'numpy' or 'json'

# File paths (relative to path_planning folder)
DATA_DIR = 'data'
MAPS_DIR = f'{DATA_DIR}/maps'
OBSTACLES_DIR = f'{DATA_DIR}/obstacles'
GENERATED_DIR = f'{DATA_DIR}/generated'

# Default output file
DEFAULT_OUTPUT_FILE = f'{GENERATED_DIR}/trajectory_output.npz'

# ============================================================================
# VISUALIZATION PARAMETERS
# ============================================================================
SHOW_PLOT = True
PLOT_STYLE = 'default'  # 'default', 'seaborn', 'ggplot'
FIGURE_SIZE = (16, 10)

# Colors
COLOR_START = 'green'
COLOR_GOAL = 'red'
COLOR_PATH = 'blue'
COLOR_OBSTACLE = 'gray'
COLOR_TERRAIN = 'brown'

# ============================================================================
# NED COORDINATE SYSTEM SETTINGS
# ============================================================================
# Following the convention from dron_lqr_with_orientation.py:
# - X: Forward (North)
# - Y: Right (East)  
# - Z: Down (positive downward, negative = altitude)
# - Ground at Z = 0
USE_NED_COORDINATES = True
