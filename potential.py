# -*- coding: utf-8 -*-
import numpy as np 

class PotentialFieldController:
    def __init__(self, k_att=0.5, k_rep=1.0, rep_threshold=5.0):
        """
        Potential Field Controller implementation
        
        Args:
            k_att: Attraction coefficient (how strongly to pull toward target)
            k_rep: Repulsion coefficient (how strongly to push away from obstacles)
            rep_threshold: Influence radius of obstacles (distance at which obstacles affect movement)
        """
        self.k_att = k_att  # Attraction coefficient
        self.k_rep = k_rep  # Repulsion coefficient
        self.rep_threshold = rep_threshold  # Obstacle influence radius

    def compute_force(self, position, target, obstacles):
        """
        Compute the resultant force vector
        
        Args:
            position: Current position [x, y, z]
            target: Target position [x, y, z]
            obstacles: List of obstacles (each as [x, y, z, radius])
            
        Returns:
            Resultant force vector [Fx, Fy, Fz]
        """
        # Attractive force toward target
        att_force = self._compute_attractive(position, target)
        
        # Repulsive force from obstacles
        rep_force = np.zeros_like(position)
        for obs in obstacles:
            rep_force += self._compute_repulsive(position, obs)
        
        return att_force + rep_force

    def _compute_attractive(self, position, target):
        """
        Compute attractive force toward target
        
        Args:
            position: Current position
            target: Target position
            
        Returns:
            Attractive force vector
        """
        direction = target - position
        distance = np.linalg.norm(direction)
        # Add small epsilon to avoid division by zero
        return self.k_att * direction / (distance + 1e-6)

    def _compute_repulsive(self, position, obstacle):
        """
        Compute repulsive force from obstacle
        
        Args:
            position: Current position
            obstacle: Obstacle position [x, y, z, radius]
            
        Returns:
            Repulsive force vector (zero if outside influence radius)
        """
        direction = position - obstacle[:3]  # Only use x,y,z coordinates
        distance = np.linalg.norm(direction)
        
        if distance < self.rep_threshold:
            # Inverse square law repulsion
            return self.k_rep * (1/distance - 1/self.rep_threshold) * direction / (distance**3 + 1e-6)
        return np.zeros_like(position)