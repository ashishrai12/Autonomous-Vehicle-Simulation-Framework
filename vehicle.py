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

class AutonomousVehicle:
    def __init__(self):
        self.world = World()
        self.vehicle_prim = None
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.goal = np.array([10.0, 0.0, 0.0])  # Default goal

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
                size=np.array([2.0, 1.0, 1.5]),  # Length, width, height
                color=np.array([0.0, 0.0, 1.0])  # Blue color
            )
        )

    def navigate_to_goal(self, goal=None):
        """Basic navigation to a goal position."""
        if goal is not None:
            self.goal = np.array(goal)

        # Simple proportional control towards goal
        direction = self.goal - self.position
        distance = np.linalg.norm(direction)

        if distance > 0.1:  # If not at goal
            direction_normalized = direction / distance
            self.velocity = direction_normalized * 2.0  # Speed
            self.position += self.velocity * 0.1  # Time step
            self.update_position()

    def avoid_obstacles(self, obstacles):
        """Implement obstacle avoidance."""
        avoidance_force = np.array([0.0, 0.0, 0.0])

        for obstacle in obstacles:
            obstacle_pos = np.array(obstacle.get_world_pose()[0])
            distance = np.linalg.norm(obstacle_pos - self.position)

            if distance < 5.0:  # Avoidance radius
                # Repulsive force
                force_direction = self.position - obstacle_pos
                force_direction /= distance
                avoidance_force += force_direction * (1.0 / distance)

        # Apply avoidance
        self.velocity += avoidance_force * 0.5
        self.velocity = self.velocity / np.linalg.norm(self.velocity) * 2.0  # Normalize speed
        self.position += self.velocity * 0.1
        self.update_position()

    def plan_path(self, start, goal, obstacles):
        """Simple path planning using A* or similar."""
        # For simplicity, implement a basic A* path planner
        # This is a placeholder - in practice, use a proper path planning library
        path = [start, goal]  # Direct path
        return path

    def update_position(self):
        """Update the vehicle's position in the simulation."""
        if self.vehicle_prim:
            self.vehicle_prim.set_world_pose(position=self.position)

    def get_sensor_data(self):
        """Get sensor data (lidar, camera, etc.) - placeholder."""
        # In Isaac Sim, this would interface with actual sensors
        return {"lidar": [], "camera": None}

    def set_goal(self, goal):
        """Set a new goal for the vehicle."""
        self.goal = np.array(goal)
