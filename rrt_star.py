import numpy as np
import math
from typing import List, Tuple, Dict

class ObstacleAvoidance:
    def __init__(self, bounds: List[Tuple[float, float]], 
                 obstacle_list: List[List[float]], 
                 max_iter: int = 10000, 
                 step_size: float = 30.0,
                 goal_sample_rate: float = 0.3):
        
        self.bounds = np.array(bounds, dtype=np.float32)
        self.obstacle_list = [tuple(obs) for obs in obstacle_list]  
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate

    def find_path(self, start: List[float], goal: List[float]) -> List[np.ndarray]:
        """Main method compatible with UDP server"""
        try:
            # Конвертируем точки в tuple сразу
            start_tuple = tuple(self._validate_point(start))
            goal_tuple = tuple(self._validate_point(goal))
            
            tree = {start_tuple: {'parent': None, 'cost': 0.0}}
            
            for _ in range(self.max_iter):
                # Random sampling with goal bias
                if np.random.random() < self.goal_sample_rate:
                    rand_point = goal_tuple
                else:
                    rand_point = tuple(self._random_point())
                
                nearest, _ = self._nearest_neighbor(tree, rand_point)
                new_point = self._steer(np.array(nearest), np.array(rand_point))
                new_point_tuple = tuple(new_point)
                
                if not self._check_collision(np.array(nearest), new_point):
                    neighbors = self._find_neighbors(tree, new_point)
                    best_parent, min_cost = self._choose_parent(tree, neighbors, nearest, new_point_tuple)
                    
                    tree[new_point_tuple] = {'parent': best_parent, 'cost': min_cost}
                    self._rewire(tree, neighbors, new_point_tuple, min_cost)
                    
                    if np.linalg.norm(new_point - np.array(goal_tuple)) < self.step_size:
                        tree[goal_tuple] = {'parent': new_point_tuple, 
                                          'cost': min_cost + np.linalg.norm(new_point - np.array(goal_tuple))}
                        return self._build_path(tree, goal_tuple)
            
            last_node, _ = self._nearest_neighbor(tree, goal_tuple)
            return self._build_path(tree, last_node)
            
        except Exception as e:
            print(f"Path finding error: {str(e)}")
            return [np.array(start), np.array(goal)]

    def _validate_point(self, point: List[float]) -> np.ndarray:
        point = np.array(point, dtype=np.float32).flatten()
        return np.clip(point, self.bounds[:,0], self.bounds[:,1])

    def _random_point(self) -> np.ndarray:
        return np.array([np.random.uniform(b[0], b[1]) for b in self.bounds], dtype=np.float32)

    def _nearest_neighbor(self, tree: Dict, point: Tuple) -> Tuple[Tuple, float]:
        min_dist = float('inf')
        nearest = None
        point_arr = np.array(point)
        for node in tree:
            dist = np.linalg.norm(np.array(node) - point_arr)
            if dist < min_dist:
                min_dist = dist
                nearest = node
        return nearest, min_dist

    def _steer(self, from_point: np.ndarray, to_point: np.ndarray) -> np.ndarray:
        direction = to_point - from_point
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            return from_point.copy()
        step = min(self.step_size, dist)
        return from_point + (direction / dist) * step

    def _check_collision(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        for obs in self.obstacle_list:
            if self._line_sphere_intersection(p1, p2, np.array(obs[:3]), obs[3]):
                return True
        return False

    def _line_sphere_intersection(self, p1: np.ndarray, p2: np.ndarray, 
                            center: np.ndarray, radius: float) -> bool:
        """
        Checks if the line segment p1-p2 intersects with a sphere (center, radius).
        Returns True if there is an intersection.
        """
        # Direction vector of the segment
        d = p2 - p1
    
        # Check for degenerate segment (zero length)
        a = np.dot(d, d)
        if a < 1e-12:  # Practically zero length
            return np.linalg.norm(p1 - center) <= radius  # Check if point is inside sphere
    
        # Vector from sphere center to segment start
        f = p1 - center
    
        b = 2.0 * np.dot(f, d)
        c = np.dot(f, f) - radius**2
    
        discriminant = b**2 - 4.0*a*c
    
        # No intersection if discriminant is negative
        if discriminant < 0:
            return False
    
        # Calculate intersection parameters
        sqrt_discriminant = np.sqrt(discriminant)
        try:
            t1 = (-b - sqrt_discriminant) / (2.0*a)
            t2 = (-b + sqrt_discriminant) / (2.0*a)
        except ZeroDivisionError:
            return False
    
        # Check if intersection occurs within segment [0, 1]
        return (0 <= t1 <= 1) or (0 <= t2 <= 1)

    def _find_neighbors(self, tree: Dict, point: np.ndarray, radius: float = 5.0) -> List[Tuple]:
        return [node for node in tree if np.linalg.norm(np.array(node) - point) <= radius]

    def _choose_parent(self, tree: Dict, neighbors: List[Tuple], 
                      nearest: Tuple, new_point: Tuple) -> Tuple[Tuple, float]:
        best = nearest
        min_cost = tree[nearest]['cost'] + np.linalg.norm(np.array(nearest) - np.array(new_point))
        for node in neighbors:
            cost = tree[node]['cost'] + np.linalg.norm(np.array(node) - np.array(new_point))
            if cost < min_cost and not self._check_collision(np.array(node), np.array(new_point)):
                min_cost = cost
                best = node
        return best, min_cost

    def _rewire(self, tree: Dict, neighbors: List[Tuple], new_point: Tuple, new_cost: float):
        for node in neighbors:
            cost = tree[node]['cost']
            potential_cost = new_cost + np.linalg.norm(np.array(node) - np.array(new_point))
            if potential_cost < cost and not self._check_collision(np.array(new_point), np.array(node)):
                tree[node]['parent'] = new_point
                tree[node]['cost'] = potential_cost

    def _build_path(self, tree: Dict, end_node: Tuple) -> List[np.ndarray]:
        path = []
        current = end_node
        while current is not None:
            path.append(np.array(current, dtype=np.float32))
            current = tree[current]['parent']
        return path[::-1] or [np.zeros(3, dtype=np.float32), np.ones(3, dtype=np.float32)]

    def optimize_path(self, path: List[np.ndarray]) -> List[np.ndarray]:
        """Path smoothing method"""
        if len(path) < 4:
            return path
            
        smoothed = [path[0]]
        for i in range(1, len(path)-1):
            smoothed.append(0.3*path[i-1] + 0.4*path[i] + 0.3*path[i+1])
        smoothed.append(path[-1])
        
        step = max(2, len(smoothed)//5)
        simplified = smoothed[::step]
        
        if not np.allclose(simplified[0], path[0]):
            simplified.insert(0, path[0])
        if not np.allclose(simplified[-1], path[-1]):
            simplified.append(path[-1])
            
        return simplified