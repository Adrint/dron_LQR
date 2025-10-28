"""
Obstacle definitions for path planning
"""

import numpy as np
from typing import Tuple, List


class Obstacle:
    """Base class for obstacles"""
    
    def __init__(self, position: Tuple[float, float, float], name: str = ""):
        """
        Initialize obstacle
        
        Args:
            position: (x, y, z) position in NED coordinates
            name: Optional name for the obstacle
        """
        self.position = np.array(position, dtype=float)
        self.name = name
    
    def is_collision(self, point: Tuple[float, float, float], safety_distance: float = 0.0) -> bool:
        """
        Check if a point collides with this obstacle
        
        Args:
            point: (x, y, z) to check
            safety_distance: Additional safety margin
            
        Returns:
            True if collision detected
        """
        raise NotImplementedError("Must be implemented by subclass")
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get bounding box of obstacle
        
        Returns:
            (min_point, max_point) as (x, y, z) arrays
        """
        raise NotImplementedError("Must be implemented by subclass")


class BoxObstacle(Obstacle):
    """Rectangular box obstacle (e.g., building)"""
    
    def __init__(self, position: Tuple[float, float, float], 
                 size: Tuple[float, float, float], name: str = ""):
        """
        Args:
            position: Center position (x, y, z) in NED
            size: (width_x, width_y, height_z) dimensions
            name: Optional name
        """
        super().__init__(position, name)
        self.size = np.array(size, dtype=float)
        self.half_size = self.size / 2.0
    
    def is_collision(self, point: Tuple[float, float, float], safety_distance: float = 0.0) -> bool:
        """Check if point is inside expanded box"""
        point = np.array(point)
        expanded_half_size = self.half_size + safety_distance
        
        # Check if point is within box bounds
        diff = np.abs(point - self.position)
        return np.all(diff <= expanded_half_size)
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        min_point = self.position - self.half_size
        max_point = self.position + self.half_size
        return min_point, max_point
    
    def get_corners(self) -> np.ndarray:
        """Get all 8 corners of the box"""
        corners = []
        for dx in [-1, 1]:
            for dy in [-1, 1]:
                for dz in [-1, 1]:
                    corner = self.position + self.half_size * np.array([dx, dy, dz])
                    corners.append(corner)
        return np.array(corners)


class CylinderObstacle(Obstacle):
    """Cylindrical obstacle (e.g., tower, tree)"""
    
    def __init__(self, position: Tuple[float, float, float], 
                 radius: float, height: float, name: str = ""):
        """
        Args:
            position: Base center position (x, y, z_base) in NED
            radius: Radius of cylinder
            height: Height of cylinder (extends downward in NED: more negative Z)
            name: Optional name
        """
        super().__init__(position, name)
        self.radius = radius
        self.height = height
    
    def is_collision(self, point: Tuple[float, float, float], safety_distance: float = 0.0) -> bool:
        """Check if point is inside expanded cylinder"""
        point = np.array(point)
        
        # Check Z range (height)
        z_min = self.position[2] - self.height  # More negative (higher up)
        z_max = self.position[2]  # Base
        if not (z_min <= point[2] <= z_max):
            return False
        
        # Check XY distance (radius)
        xy_distance = np.linalg.norm(point[:2] - self.position[:2])
        return xy_distance <= (self.radius + safety_distance)
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        min_point = np.array([
            self.position[0] - self.radius,
            self.position[1] - self.radius,
            self.position[2] - self.height
        ])
        max_point = np.array([
            self.position[0] + self.radius,
            self.position[1] + self.radius,
            self.position[2]
        ])
        return min_point, max_point


class SphereObstacle(Obstacle):
    """Spherical obstacle (e.g., balloon, dome)"""
    
    def __init__(self, position: Tuple[float, float, float], 
                 radius: float, name: str = ""):
        """
        Args:
            position: Center position (x, y, z) in NED
            radius: Radius of sphere
            name: Optional name
        """
        super().__init__(position, name)
        self.radius = radius
    
    def is_collision(self, point: Tuple[float, float, float], safety_distance: float = 0.0) -> bool:
        """Check if point is inside expanded sphere"""
        point = np.array(point)
        distance = np.linalg.norm(point - self.position)
        return distance <= (self.radius + safety_distance)
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        min_point = self.position - self.radius
        max_point = self.position + self.radius
        return min_point, max_point


class PolygonObstacle(Obstacle):
    """Polygon extruded vertically (e.g., irregular building)"""
    
    def __init__(self, vertices_xy: List[Tuple[float, float]], 
                 z_base: float, height: float, name: str = ""):
        """
        Args:
            vertices_xy: List of (x, y) vertices defining polygon base
            z_base: Base Z coordinate (NED)
            height: Height of extrusion (extends downward: more negative Z)
            name: Optional name
        """
        # Calculate center as mean of vertices
        vertices = np.array(vertices_xy)
        center_xy = np.mean(vertices, axis=0)
        center = np.array([center_xy[0], center_xy[1], z_base - height/2])
        
        super().__init__(center, name)
        self.vertices_xy = np.array(vertices_xy)
        self.z_base = z_base
        self.height = height
        self.z_min = z_base - height
    
    def is_collision(self, point: Tuple[float, float, float], safety_distance: float = 0.0) -> bool:
        """Check if point is inside extruded polygon"""
        point = np.array(point)
        
        # Check Z range
        if not (self.z_min <= point[2] <= self.z_base):
            return False
        
        # Check if point is inside polygon (2D)
        return self._point_in_polygon(point[:2], self.vertices_xy, safety_distance)
    
    @staticmethod
    def _point_in_polygon(point: np.ndarray, vertices: np.ndarray, margin: float = 0.0) -> bool:
        """Ray casting algorithm to check if point is inside polygon"""
        x, y = point
        n = len(vertices)
        inside = False
        
        p1x, p1y = vertices[0]
        for i in range(1, n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        # TODO: Implement proper margin checking
        return inside
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        min_xy = np.min(self.vertices_xy, axis=0)
        max_xy = np.max(self.vertices_xy, axis=0)
        
        min_point = np.array([min_xy[0], min_xy[1], self.z_min])
        max_point = np.array([max_xy[0], max_xy[1], self.z_base])
        return min_point, max_point


class ObstacleManager:
    """Manager for handling multiple obstacles"""
    
    def __init__(self):
        self.obstacles: List[Obstacle] = []
    
    def add_obstacle(self, obstacle: Obstacle):
        """Add an obstacle to the manager"""
        self.obstacles.append(obstacle)
    
    def add_box(self, position: Tuple[float, float, float], 
                size: Tuple[float, float, float], name: str = ""):
        """Convenience method to add a box obstacle"""
        self.add_obstacle(BoxObstacle(position, size, name))
    
    def add_cylinder(self, position: Tuple[float, float, float], 
                     radius: float, height: float, name: str = ""):
        """Convenience method to add a cylinder obstacle"""
        self.add_obstacle(CylinderObstacle(position, radius, height, name))
    
    def add_sphere(self, position: Tuple[float, float, float], 
                   radius: float, name: str = ""):
        """Convenience method to add a sphere obstacle"""
        self.add_obstacle(SphereObstacle(position, radius, name))
    
    def is_collision_free(self, point: Tuple[float, float, float], 
                         safety_distance: float = 0.0) -> bool:
        """
        Check if a point is free from all obstacles
        
        Args:
            point: (x, y, z) to check
            safety_distance: Additional safety margin
            
        Returns:
            True if point is collision-free
        """
        for obstacle in self.obstacles:
            if obstacle.is_collision(point, safety_distance):
                return False
        return True
    
    def is_path_collision_free(self, point1: np.ndarray, point2: np.ndarray,
                               num_checks: int = 10, safety_distance: float = 0.0) -> bool:
        """
        Check if a straight line path between two points is collision-free
        
        Args:
            point1: Start point (x, y, z)
            point2: End point (x, y, z)
            num_checks: Number of points to check along the path
            safety_distance: Additional safety margin
            
        Returns:
            True if entire path is collision-free
        """
        for i in range(num_checks + 1):
            t = i / num_checks
            point = point1 + t * (point2 - point1)
            if not self.is_collision_free(point, safety_distance):
                return False
        return True
    
    def get_all_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get bounding box containing all obstacles"""
        if not self.obstacles:
            return np.array([0, 0, 0]), np.array([0, 0, 0])
        
        all_mins = []
        all_maxs = []
        for obs in self.obstacles:
            min_pt, max_pt = obs.get_bounds()
            all_mins.append(min_pt)
            all_maxs.append(max_pt)
        
        overall_min = np.min(all_mins, axis=0)
        overall_max = np.max(all_maxs, axis=0)
        return overall_min, overall_max
    
    def clear(self):
        """Remove all obstacles"""
        self.obstacles.clear()
    
    def __len__(self):
        return len(self.obstacles)
    
    def __iter__(self):
        return iter(self.obstacles)
