#!/usr/bin/env python3

"""
Autonomous Vehicle implementation for Isaac Sim.
"""

import numpy as np

try:
    from omni.isaac.core import World
    from omni.isaac.core.objects import DynamicCuboid
    from omni.isaac.core.utils.prims import create_prim
    from omni.isaac.core.utils.stage import add_reference_to_stage
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    # Mock classes for testing
    class World:
        def __init__(self):
            self.scene = MockScene()

    class MockScene:
        def add(self, obj):
            return obj

    class DynamicCuboid:
        def __init__(self, prim_path, name, position, size, color):
            self.position = position
            self.size = size
            self.color = color

        def set_world_pose(self, position):
            self.position = position

        def get_world_pose(self):
            return (self.position, None)

try:
    import rust_engine
    RUST_AVAILABLE = True
except ImportError:
    RUST_AVAILABLE = False

class AutonomousVehicle:
    def __init__(self, config=None):
        self.world = World()
        self.vehicle_prim = None
        self.config = config or {}
        
        # Load parameters from config
        self.wheelbase = self.config.get('vehicle', {}).get('wheelbase', 2.5)
        initial_pose = self.config.get('vehicle', {}).get('initial_pose', [0.0, 0.0, 0.0])
        
        self.position = np.array([initial_pose[0], initial_pose[1], 0.0])
        self.yaw = initial_pose[2]
        self.v = 0.0
        self.goal = np.array([10.0, 0.0, 0.0])

        if RUST_AVAILABLE:
            self.engine = rust_engine.RustSimulationEngine(
                self.position[0], self.position[1], self.yaw, self.v, self.wheelbase
            )
        else:
            print("Warning: Rust engine not found. Falling back to Python physics.")
            self.engine = None

    def spawn_vehicle(self, position=None):
        """Spawn the vehicle in the simulation world."""
        if position is None:
            position = self.position

        # Create a simple vehicle representation (cuboid)
        self.vehicle_prim = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/Vehicle",
                name="vehicle",
                position=position,
                size=np.array([2.0, 1.0, 1.5]),
                color=np.array([0.0, 0.0, 1.0])
            )
        )

    def step(self, acceleration, steering_angle, dt):
        """Perform one simulation step using the Rust engine."""
        if self.engine:
            self.engine.step(acceleration, steering_angle, dt)
            x, y, yaw, v = self.engine.get_vehicle_state()
            self.position = np.array([x, y, 0.0])
            self.yaw = yaw
            self.v = v
        else:
            # Simple Python fallback
            self.position[0] += self.v * np.cos(self.yaw) * dt
            self.position[1] += self.v * np.sin(self.yaw) * dt
            self.yaw += self.v / self.wheelbase * np.tan(steering_angle) * dt
            self.v += acceleration * dt
        
        self.update_position()

    def update_position(self):
        """Update the vehicle's position in the simulation."""
        if self.vehicle_prim:
            self.vehicle_prim.set_world_pose(position=self.position)

    def get_lidar_data(self, obstacles):
        """Get lidar data from Rust engine."""
        if self.engine:
            obs_data = []
            for obs in obstacles:
                pos, _ = obs.get_world_pose()
                # Assuming obstacles are cuboids for now
                obs_data.append((pos[0], pos[1], 1.0, 1.0))
            
            num_rays = self.config.get('sensors', {}).get('lidar', {}).get('num_rays', 32)
            max_range = self.config.get('sensors', {}).get('lidar', {}).get('max_range', 20.0)
            return self.engine.cast_rays(num_rays, max_range, obs_data)
        return []

    def set_goal(self, goal):
        """Set a new goal for the vehicle."""
        self.goal = np.array(goal)
