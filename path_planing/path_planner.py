"""
Path planning algorithms for drone navigation
"""

import numpy as np
import heapq
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
import random

from obstacles import ObstacleManager


@dataclass
class Node:
    """Node for graph-based path planning"""
    position: np.ndarray
    g_cost: float = float('inf')  # Cost from start
    h_cost: float = 0.0  # Heuristic cost to goal
    parent: Optional['Node'] = None
    
    @property
    def f_cost(self):
        """Total cost (for A*)"""
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost
    
    def __hash__(self):
        return hash(tuple(self.position))
    
    def __eq__(self, other):
        return np.allclose(self.position, other.position)


class PathPlanner:
    """Base class for path planning algorithms"""
    
    def __init__(self, obstacle_manager: ObstacleManager, 
                 grid_resolution: float = 0.5,
                 safety_distance: float = 1.0):
        """
        Initialize path planner
        
        Args:
            obstacle_manager: Manager containing all obstacles
            grid_resolution: Grid size for discretization (meters)
            safety_distance: Minimum distance from obstacles (meters)
        """
        self.obstacle_manager = obstacle_manager
        self.grid_resolution = grid_resolution
        self.safety_distance = safety_distance
    
    def plan(self, start: Tuple[float, float, float], 
             goal: Tuple[float, float, float]) -> Optional[List[np.ndarray]]:
        """
        Plan a path from start to goal
        
        Args:
            start: Start position (x, y, z)
            goal: Goal position (x, y, z)
            
        Returns:
            List of waypoints from start to goal, or None if no path found
        """
        raise NotImplementedError("Must be implemented by subclass")
    
    def heuristic(self, pos1: np.ndarray, pos2: np.ndarray) -> float:
        """Euclidean distance heuristic"""
        return np.linalg.norm(pos1 - pos2)
    
    def is_valid_point(self, point: np.ndarray) -> bool:
        """Check if a point is valid (collision-free)"""
        return self.obstacle_manager.is_collision_free(point, self.safety_distance)


class AStarPlanner(PathPlanner):
    """A* path planning algorithm"""
    
    def get_neighbors(self, position: np.ndarray) -> List[np.ndarray]:
        """
        Get valid neighbors for a position (26-connected grid in 3D)
        
        Args:
            position: Current position
            
        Returns:
            List of valid neighbor positions
        """
        neighbors = []
        
        # 26 directions in 3D (including diagonals)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    if dx == dy == dz == 0:
                        continue
                    
                    new_pos = position + np.array([dx, dy, dz]) * self.grid_resolution
                    
                    if self.is_valid_point(new_pos):
                        neighbors.append(new_pos)
        
        return neighbors
    
    def plan(self, start: Tuple[float, float, float], 
             goal: Tuple[float, float, float]) -> Optional[List[np.ndarray]]:
        """
        Plan path using A* algorithm
        
        Args:
            start: Start position (x, y, z)
            goal: Goal position (x, y, z)
            
        Returns:
            List of waypoints or None if no path found
        """
        start_pos = np.array(start, dtype=float)
        goal_pos = np.array(goal, dtype=float)
        
        # Check if start and goal are valid
        if not self.is_valid_point(start_pos):
            print("ERROR: Start position is in collision!")
            return None
        if not self.is_valid_point(goal_pos):
            print("ERROR: Goal position is in collision!")
            return None
        
        # Initialize
        start_node = Node(start_pos, g_cost=0, h_cost=self.heuristic(start_pos, goal_pos))
        
        open_list = [start_node]
        closed_set = set()
        nodes_dict = {tuple(start_pos): start_node}
        
        iterations = 0
        max_iterations = 100000
        
        while open_list and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_cost
            current = heapq.heappop(open_list)
            
            # Check if we reached goal
            if self.heuristic(current.position, goal_pos) < self.grid_resolution:
                # Reconstruct path
                path = []
                node = current
                while node is not None:
                    path.append(node.position)
                    node = node.parent
                print(f"A* found path with {len(path)} waypoints in {iterations} iterations")
                return path[::-1]  # Reverse to get start -> goal
            
            closed_set.add(tuple(current.position))
            
            # Check neighbors
            for neighbor_pos in self.get_neighbors(current.position):
                neighbor_key = tuple(neighbor_pos)
                
                if neighbor_key in closed_set:
                    continue
                
                # Calculate tentative g_cost
                tentative_g = current.g_cost + self.heuristic(current.position, neighbor_pos)
                
                if neighbor_key not in nodes_dict:
                    # New node
                    neighbor = Node(
                        neighbor_pos,
                        g_cost=tentative_g,
                        h_cost=self.heuristic(neighbor_pos, goal_pos),
                        parent=current
                    )
                    nodes_dict[neighbor_key] = neighbor
                    heapq.heappush(open_list, neighbor)
                    
                elif tentative_g < nodes_dict[neighbor_key].g_cost:
                    # Better path found
                    neighbor = nodes_dict[neighbor_key]
                    neighbor.g_cost = tentative_g
                    neighbor.parent = current
                    heapq.heappush(open_list, neighbor)
        
        print(f"A* failed to find path after {iterations} iterations")
        return None


class RRTNode:
    """Node for RRT tree"""
    
    def __init__(self, position: np.ndarray, parent: Optional['RRTNode'] = None):
        self.position = position
        self.parent = parent
        self.cost = 0.0  # Cost from root (for RRT*)


class RRTPlanner(PathPlanner):
    """RRT (Rapidly-exploring Random Tree) path planning"""
    
    def __init__(self, obstacle_manager: ObstacleManager,
                 grid_resolution: float = 0.5,
                 safety_distance: float = 1.0,
                 max_iterations: int = 5000,
                 step_size: float = 1.0,
                 goal_sample_rate: float = 0.1):
        """
        Args:
            max_iterations: Maximum number of iterations
            step_size: Maximum distance to extend tree
            goal_sample_rate: Probability of sampling goal
        """
        super().__init__(obstacle_manager, grid_resolution, safety_distance)
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
    
    def sample_free(self, start: np.ndarray, goal: np.ndarray, 
                    bounds: Tuple[np.ndarray, np.ndarray]) -> np.ndarray:
        """
        Sample a random free configuration
        
        Args:
            start: Start position
            goal: Goal position
            bounds: (min_point, max_point) defining sampling space
            
        Returns:
            Random point in free space
        """
        # Sample goal with certain probability
        if random.random() < self.goal_sample_rate:
            return goal.copy()
        
        # Random sample in bounds
        min_pt, max_pt = bounds
        return np.random.uniform(min_pt, max_pt)
    
    def nearest(self, tree: List[RRTNode], point: np.ndarray) -> RRTNode:
        """Find nearest node in tree to given point"""
        min_dist = float('inf')
        nearest_node = tree[0]
        
        for node in tree:
            dist = np.linalg.norm(node.position - point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        
        return nearest_node
    
    def steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """
        Steer from one position towards another (limited by step_size)
        
        Args:
            from_pos: Starting position
            to_pos: Target position
            
        Returns:
            New position (at most step_size away from from_pos)
        """
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)
        
        if distance <= self.step_size:
            return to_pos
        
        return from_pos + (direction / distance) * self.step_size
    
    def is_path_valid(self, from_pos: np.ndarray, to_pos: np.ndarray) -> bool:
        """Check if straight line path is collision-free"""
        return self.obstacle_manager.is_path_collision_free(
            from_pos, to_pos, num_checks=20, safety_distance=self.safety_distance
        )
    
    def plan(self, start: Tuple[float, float, float],
             goal: Tuple[float, float, float]) -> Optional[List[np.ndarray]]:
        """
        Plan path using RRT
        
        Args:
            start: Start position
            goal: Goal position
            
        Returns:
            List of waypoints or None if no path found
        """
        start_pos = np.array(start, dtype=float)
        goal_pos = np.array(goal, dtype=float)
        
        # Check validity
        if not self.is_valid_point(start_pos):
            print("ERROR: Start position is in collision!")
            return None
        if not self.is_valid_point(goal_pos):
            print("ERROR: Goal position is in collision!")
            return None
        
        # Define sampling bounds
        obstacle_bounds = self.obstacle_manager.get_all_bounds()
        min_bound = np.minimum(start_pos, goal_pos) - 10
        max_bound = np.maximum(start_pos, goal_pos) + 10
        
        if len(self.obstacle_manager) > 0:
            obs_min, obs_max = obstacle_bounds
            min_bound = np.minimum(min_bound, obs_min - 5)
            max_bound = np.maximum(max_bound, obs_max + 5)
        
        bounds = (min_bound, max_bound)
        
        # Initialize tree
        root = RRTNode(start_pos)
        tree = [root]
        
        print(f"RRT: Starting with bounds {min_bound} to {max_bound}")
        
        for i in range(self.max_iterations):
            # Sample random point
            rand_point = self.sample_free(start_pos, goal_pos, bounds)
            
            # Find nearest node
            nearest_node = self.nearest(tree, rand_point)
            
            # Steer towards random point
            new_pos = self.steer(nearest_node.position, rand_point)
            
            # Check if path is valid
            if self.is_path_valid(nearest_node.position, new_pos):
                new_node = RRTNode(new_pos, parent=nearest_node)
                tree.append(new_node)
                
                # Check if we can connect to goal
                if np.linalg.norm(new_pos - goal_pos) < self.step_size:
                    if self.is_path_valid(new_pos, goal_pos):
                        # Path found!
                        goal_node = RRTNode(goal_pos, parent=new_node)
                        
                        # Reconstruct path
                        path = []
                        node = goal_node
                        while node is not None:
                            path.append(node.position)
                            node = node.parent
                        
                        print(f"RRT found path with {len(path)} waypoints in {i+1} iterations")
                        return path[::-1]  # Reverse
            
            if (i + 1) % 500 == 0:
                print(f"RRT: {i+1} iterations, tree size: {len(tree)}")
        
        print(f"RRT failed to find path after {self.max_iterations} iterations")
        return None


class RRTStarPlanner(RRTPlanner):
    """RRT* (optimal variant of RRT)"""
    
    def __init__(self, obstacle_manager: ObstacleManager,
                 grid_resolution: float = 0.5,
                 safety_distance: float = 1.0,
                 max_iterations: int = 5000,
                 step_size: float = 1.0,
                 goal_sample_rate: float = 0.1,
                 search_radius: float = 3.0):
        """
        Args:
            search_radius: Radius for rewiring neighbors
        """
        super().__init__(obstacle_manager, grid_resolution, safety_distance,
                        max_iterations, step_size, goal_sample_rate)
        self.search_radius = search_radius
    
    def near(self, tree: List[RRTNode], point: np.ndarray, radius: float) -> List[RRTNode]:
        """Find all nodes within radius of point"""
        near_nodes = []
        for node in tree:
            if np.linalg.norm(node.position - point) <= radius:
                near_nodes.append(node)
        return near_nodes
    
    def cost(self, node: RRTNode) -> float:
        """Calculate cost from root to node"""
        cost = 0.0
        current = node
        while current.parent is not None:
            cost += np.linalg.norm(current.position - current.parent.position)
            current = current.parent
        return cost
    
    def plan(self, start: Tuple[float, float, float],
             goal: Tuple[float, float, float]) -> Optional[List[np.ndarray]]:
        """Plan path using RRT*"""
        start_pos = np.array(start, dtype=float)
        goal_pos = np.array(goal, dtype=float)
        
        if not self.is_valid_point(start_pos):
            print("ERROR: Start position is in collision!")
            return None
        if not self.is_valid_point(goal_pos):
            print("ERROR: Goal position is in collision!")
            return None
        
        # Define bounds
        obstacle_bounds = self.obstacle_manager.get_all_bounds()
        min_bound = np.minimum(start_pos, goal_pos) - 10
        max_bound = np.maximum(start_pos, goal_pos) + 10
        
        if len(self.obstacle_manager) > 0:
            obs_min, obs_max = obstacle_bounds
            min_bound = np.minimum(min_bound, obs_min - 5)
            max_bound = np.maximum(max_bound, obs_max + 5)
        
        bounds = (min_bound, max_bound)
        
        # Initialize
        root = RRTNode(start_pos)
        root.cost = 0.0
        tree = [root]
        
        best_goal_node = None
        best_cost = float('inf')
        
        print(f"RRT*: Starting optimization")
        
        for i in range(self.max_iterations):
            # Sample
            rand_point = self.sample_free(start_pos, goal_pos, bounds)
            nearest_node = self.nearest(tree, rand_point)
            new_pos = self.steer(nearest_node.position, rand_point)
            
            if not self.is_path_valid(nearest_node.position, new_pos):
                continue
            
            # Find nearby nodes
            near_nodes = self.near(tree, new_pos, self.search_radius)
            
            # Choose best parent
            min_cost = nearest_node.cost + np.linalg.norm(new_pos - nearest_node.position)
            best_parent = nearest_node
            
            for near_node in near_nodes:
                if self.is_path_valid(near_node.position, new_pos):
                    cost = near_node.cost + np.linalg.norm(new_pos - near_node.position)
                    if cost < min_cost:
                        min_cost = cost
                        best_parent = near_node
            
            # Add new node
            new_node = RRTNode(new_pos, parent=best_parent)
            new_node.cost = min_cost
            tree.append(new_node)
            
            # Rewire tree
            for near_node in near_nodes:
                if near_node == best_parent:
                    continue
                
                new_cost = new_node.cost + np.linalg.norm(near_node.position - new_pos)
                if new_cost < near_node.cost:
                    if self.is_path_valid(new_pos, near_node.position):
                        near_node.parent = new_node
                        near_node.cost = new_cost
            
            # Check goal connection
            if np.linalg.norm(new_pos - goal_pos) < self.step_size:
                if self.is_path_valid(new_pos, goal_pos):
                    goal_cost = new_node.cost + np.linalg.norm(goal_pos - new_pos)
                    if goal_cost < best_cost:
                        best_goal_node = RRTNode(goal_pos, parent=new_node)
                        best_goal_node.cost = goal_cost
                        best_cost = goal_cost
                        print(f"RRT*: Found better path at iteration {i+1}, cost: {best_cost:.2f}")
            
            if (i + 1) % 500 == 0:
                print(f"RRT*: {i+1} iterations, tree size: {len(tree)}, best cost: {best_cost:.2f}")
        
        if best_goal_node is not None:
            # Reconstruct path
            path = []
            node = best_goal_node
            while node is not None:
                path.append(node.position)
                node = node.parent
            print(f"RRT* final path: {len(path)} waypoints, cost: {best_cost:.2f}")
            return path[::-1]
        
        print("RRT* failed to find path")
        return None


def create_planner(algorithm: str, obstacle_manager: ObstacleManager,
                   **kwargs) -> PathPlanner:
    """
    Factory function to create path planner
    
    Args:
        algorithm: 'astar', 'rrt', or 'rrt_star'
        obstacle_manager: Obstacle manager
        **kwargs: Additional parameters for specific planners
        
    Returns:
        PathPlanner instance
    """
    if algorithm == 'astar':
        return AStarPlanner(obstacle_manager, **kwargs)
    elif algorithm == 'rrt':
        return RRTPlanner(obstacle_manager, **kwargs)
    elif algorithm == 'rrt_star':
        return RRTStarPlanner(obstacle_manager, **kwargs)
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}")
