# -*- coding: utf-8 -*-
import numpy as np
from typing import List, Optional, Tuple
from dataclasses import dataclass

# Import required modules (assuming they exist in the core)
from .dynamics import NPAParameters
from .Controller.pid import PIDController
from .Controller.potential import PotentialFieldController
from .Controller.rrt_star import ObstacleAvoidance

@dataclass
class ControllerConfig:
    """Configuration parameters for the controller"""
    pid_kp: float = 1.5
    pid_ki: float = 0.2
    pid_kd: float = 0.5
    pf_k_att: float = 0.5  # Potential field attraction coefficient
    pf_k_rep: float = 1.0  # Potential field repulsion coefficient
    safety_distance: float = 3.0  # Minimum safe distance from obstacles
    bounds: Tuple[Tuple[float, float]] = ((-500, 500), (-500, 500), (-500, 500))  # Workspace boundaries

class IntegratedController:
    MAX_SPEED = 7.0  # Maximum allowed speed

    def __init__(self, params: NPAParameters, config: ControllerConfig, obstacle_list = None):
        """
        Initialize the integrated controller
        
        Args:
            params: NPA physical parameters
            config: Controller configuration
        """
        self.config = config if config else ControllerConfig()
        
        # Initialize sub-controllers
        self.pid = PIDController(
            Kp=self.config.pid_kp,
            Ki=self.config.pid_ki,
            Kd=self.config.pid_kd
        )
        
        self.pf = PotentialFieldController(
            k_att=self.config.pf_k_att,
            k_rep=self.config.pf_k_rep
        )
        
        self.oa = ObstacleAvoidance(
            bounds=self.config.bounds,
            obstacle_list=obstacle_list or [],
            max_iter=500,
            step_size=2.0
        )
        
        # Store parameters
        self.max_thrust = params.max_thrust
        self.current_path = None
        self.last_target = None

    def compute_control(self, state: np.ndarray, target: np.ndarray, obstacles: List[np.ndarray]) -> np.ndarray:
        """Compute control forces and moments based on current state, target and obstacles
    
        Args:
            state: Current state vector [u, v, w, p, q, r, x, y, z, ...]
            target: Target position [x, y, z]
            obstacles: List of obstacles (each as [x, y, z, radius])
        
        Returns:
            Control vector [X, Y, Z, K, M, N] (forces and moments)
        """
        if self._need_replan(state[6:9], target, obstacles):
            self.current_path = self.oa.find_path(state[6:9], target)  # RRT*
        current_target = self._get_current_target(state[6:9], target)

        # 1. Debug information
        self.debug = False  # Flag for debug output
        if self.debug:
            print(f"\nCurrent position: {state[6:9]}, Target: {target}")
            print(f"Current velocity: {state[:3]}")

        # 2. Path planning 
        if self._need_replan(state[6:9], target, obstacles):
            self.current_path = self.oa.find_path(state[6:9], target)
    
        # 3. Get current target point from path
        current_target = self._get_current_target(state[6:9], target)
    
        # 4. Calculate desired velocity using potential field
        desired_velocity = self.pf.compute_force(
            position=state[6:9],
            target=current_target,
            obstacles=obstacles
        )
    
        # 5. Debug output for desired velocity
        if self.debug:
            print(f"Desired velocity from potential field: {desired_velocity}")

        # 6. Velocity limiting
        for i in range(3):
            if abs(desired_velocity[i]) > self.MAX_SPEED:
                desired_velocity[i] = np.sign(desired_velocity[i]) * self.MAX_SPEED
    
        # 7. Control calculation
        control = np.zeros(6)
        for i in range(3):
            # PID control for each axis
            control[i] = self.pid.update(
                current_value=state[i],  # Current velocity
                desired_value=desired_velocity[i],  # Desired velocity
                dt=0.1
            )
        
            # Debug output
            if self.debug and i == 0:  # Example for X axis
                print(f"PID X: current={state[i]:.2f}, target={desired_velocity[i]:.2f}")
                print(f"Control output: {control[i]:.2f}")

        # 8. Debug control output
        if self.debug:
            print(f"Final control output: {control[:3]}")
            if (target[0] > state[6] and control[0] > 0) or (target[0] < state[6] and control[0] < 0):
                print("Correct control direction")
            else:
                print("Warning: Potential control inversion!")

        return np.clip(control, -self.max_thrust, self.max_thrust)

    def _get_current_target(self,
                          position: np.ndarray,
                          final_target: np.ndarray) -> np.ndarray:
        """Get the current target point from the path"""
        if self.current_path is None or len(self.current_path) < 2:
            return final_target
            
        next_point = self.current_path[1]
        if np.linalg.norm(position - next_point) < 1.0:
            self.current_path.pop(0)
            return self._get_current_target(position, final_target)
            
        return next_point

    def _need_replan(self,
                    position: np.ndarray,
                    target: np.ndarray,
                    obstacles: List[np.ndarray]) -> bool:
        """Determine if path replanning is needed"""
        target_changed = not np.array_equal(target, self.last_target)
        obstacle_near = any(
            np.linalg.norm(position - obs[:3]) < obs[3] + self.config.safety_distance
            for obs in obstacles
        )
        return target_changed or obstacle_near or self.current_path is None