import numpy as np
import socket
import struct
import time
import sys
import json
from core.Controller.rrt_star import ObstacleAvoidance
from core.integratedController import IntegratedController
from core.integratedController import ControllerConfig
from core.dynamics import NPAParameters
from typing import Optional, List

class UDPServer:
    def __init__(self, host="127.0.0.1", port=1234, unity_port=1235):
        self.host = host
        self.port = port
        self.unity_port = unity_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        print(f"[Server] Initialized on {self.host}:{self.port}")

        self.obstacle_list = []
        
        # 1. Initialize NPA parameters
        self.params = NPAParameters(
            mass=100.0,
            max_thrust=300.0,
            inertia=np.diag([50.0, 60.0, 70.0]),
            damping=np.diag([30.0, 30.0, 30.0, 10.0, 10.0, 5.0])
        )

        self.config = ControllerConfig(
            pid_kp=1.5,
            pid_ki=0.1,
            pid_kd=0.5,
            pf_k_att=0.5,
            pf_k_rep=1.0,
            safety_distance=3.0,
            bounds=((-500, 500), (-500, 500), (-500, 500))
        )

        self.controller = IntegratedController(
            params=self.params,
            config=self.config,
            obstacle_list=self.obstacle_list  
        )
        
        # 2. Create integrated controller
        self.controller = IntegratedController(
            params=self.params,
            config=self.config

        )
        
        print("[Server] Full simulation activated (RRT* + PID + Dynamics)")



    def parse_message(self, data: bytes):
        try:
            decoded = data.decode('utf-8').strip()
        
            if ';' not in decoded:
                print("Invalid message format: missing separator ';'")
                return None, None
            
            target_part, obstacles_part = decoded.split(';', 1)
        
            try:
                target_coords = [float(x) for x in target_part.split(',')[:3]]
                target = np.array(target_coords)
            except ValueError as e:
                print(f"Target parsing error: {str(e)}")
                return None, None
        
            self.obstacle_list = []
        
            if obstacles_part and obstacles_part != "{}":  
                try:
                    obstacles_data = json.loads(obstacles_part)
                
                    if not isinstance(obstacles_data, list):
                        print("Obstacles data is not a list")
                        return target, []
                
                    for obs in obstacles_data:
                        try:
                            obstacle = [
                                float(obs.get("x", 0)),
                                float(obs.get("y", 0)),
                                float(obs.get("z", 0)),
                                max(0.1, float(obs.get("radius", 0.5)))
                            ]
                            self.obstacle_list.append(obstacle)
                        except (ValueError, TypeError) as e:
                            print(f"Skipping invalid obstacle: {obs}. Error: {str(e)}")
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {str(e)}")
        
            print(f"Parsed - Target: {target}, Obstacles: {len(self.obstacle_list)}")
            return target, self.obstacle_list

        except Exception as e:
            print(f"Parse error: {str(e)}")
            return None, None
        
    def optimize_path(self, path):
        """Path smoothing and simplification"""
        if len(path) < 4:
            return path
    
        # Smoothing
        smoothed = [path[0]]
        for i in range(1, len(path)-1):
            smoothed.append(0.3*path[i-1] + 0.4*path[i] + 0.3*path[i+1])
        smoothed.append(path[-1])
        
        # Simplification
        step = max(2, len(smoothed)//5)
        simplified = smoothed[::step]
        
        # Ensure first/last points are preserved
        if not np.allclose(simplified[0], path[0]):
            simplified.insert(0, path[0])
        if not np.allclose(simplified[-1], path[-1]):
            simplified.append(path[-1])
            
        return simplified

    def send_path(self, path, client_addr):
        """Send path to Unity in binary format"""
        try:
            # Prepare data
            flat_data = []
            for point in path:
                flat_data.extend([float(point[0]), float(point[1]), float(point[2])])
            
            # Pack data
            header = struct.pack("<I", len(path))  # Number of points (uint32)
            body = struct.pack("<%df" % len(flat_data), *flat_data)  # Coordinates (float32)
            
            # Send
            corrected_addr = (client_addr[0], self.unity_port)
            self.sock.sendto(header + body, corrected_addr)
            print(f"[Sent] {len(path)} points to {corrected_addr}")
            
        except Exception as e:
            print(f"[Sending error] {str(e)}")
            # Fallback: straight line
            fallback = struct.pack("<I6f", 2, 0.0, 0.0, 0.0, 30.0, 0.0, 0.0)
            self.sock.sendto(fallback, client_addr)

    def compute_full_simulation(self, target, obstacles):
        """Full simulation cycle with physics"""
        print("\n=== SIMULATION START ===")
        print(f"Target: {target}, Obstacles: {obstacles}")

        try:
            # Validate input
            if target is None:
                print("Error: No target provided", file=sys.stderr)
                return [np.zeros(3), np.zeros(3)]
        
            # Initialize state
            current_state = np.zeros(12)
            if hasattr(self.controller, 'model'):
                current_state = self.controller.model.state.copy()
        
            print(f"Initial position: {current_state[6:9]}")

            # Validate and normalize obstacles
            valid_obstacles = []
            for obs in obstacles:
                if len(obs) >= 4:
                    radius = max(0.5, float(obs[3]))  # Minimum radius 0.5
                    valid_obstacles.append([obs[0], obs[1], obs[2], radius])
                else:
                    print(f"Warning: Skipping invalid obstacle {obs}", file=sys.stderr)

            if not valid_obstacles:
                print("Warning: No valid obstacles found", file=sys.stderr)
                return [current_state[6:9], target]

            # 1. Path Planning
            print("\n[1] Running RRT* path planning...")
            self.controller.oa.obstacle_list = valid_obstacles
            global_path = self.controller.oa.find_path(current_state[6:9], target)
        
            if not global_path or len(global_path) < 2:
                print("Warning: Path planning failed", file=sys.stderr)
                return [current_state[6:9], target]

            print(f"Planned path points: {len(global_path)}")
            print(f"First 3 points: {global_path[:3]}")

            # 2. Path optimization
            optimized_path = self.optimize_path(global_path)
            print(f"Optimized path points: {len(optimized_path)}")
        
            return optimized_path

        except Exception as e:
            print(f"Simulation error: {str(e)}", file=sys.stderr)
            return [current_state[6:9] if 'current_state' in locals() else np.zeros(3), target]

    def run(self):
        print(f"[Server] Ready on {self.host}:{self.port}")
        print("[Server] Waiting for Unity data...")
        try:
            while True:
                data, addr = self.sock.recvfrom(1024)
                print(f"[Received] Raw data: {data.decode('utf-8')} from {addr}")
                target, obstacles = self.parse_message(data)
                
                if target is None:
                    continue

                print(f"Target: {target}, Obstacles: {obstacles}")

                # Full simulation 
                path = self.compute_full_simulation(target, obstacles)
                optimized_path = self.optimize_path(path)
                self.send_path(optimized_path, addr)
                
        except KeyboardInterrupt:
            print("\n[Server] Stopping")

if __name__ == "__main__":
    try:
        server = UDPServer()
        server.run()
    except Exception as e:
        print(f"[Critical error] {str(e)}")