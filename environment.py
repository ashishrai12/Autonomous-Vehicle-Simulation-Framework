#!/usr/bin/env python3

"""
Simulation Environment setup for Isaac Sim.
"""

import numpy as np

try:
    from omni.isaac.core import World
    from omni.isaac.core.objects import GroundPlane, FixedCuboid
    from omni.isaac.core.utils.prims import create_prim
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    # Mock classes for testing
    class World:
        def __init__(self):
            self.scene = MockScene()

        def clear(self):
            pass

    class MockScene:
        def add_default_ground_plane(self):
            pass

        def add(self, obj):
            return obj

    class FixedCuboid:
        def __init__(self, prim_path, name, position, size, color):
            self.position = position
            self.size = size
            self.color = color

        def get_world_pose(self):
            return (self.position, None)

    def create_prim(path, prim_type, attributes=None):
        pass

class SimulationEnvironment:
    def __init__(self):
        self.world = World()
        self.ground_plane = None
        self.obstacles = []
        self.pedestrians = []
        self.weather_conditions = "clear"

    def setup_world(self):
        """Set up the basic world with ground plane."""
        self.world.scene.add_default_ground_plane()

        # Add some basic lighting
        create_prim("/World/Light", "DistantLight", attributes={"intensity": 3000})

    def add_obstacle(self, position, size=None):
        """Add an obstacle to the environment."""
        if size is None:
            size = np.array([1.0, 1.0, 2.0])

        obstacle = self.world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Obstacle_{len(self.obstacles)}",
                name=f"obstacle_{len(self.obstacles)}",
                position=position,
                size=size,
                color=np.array([1.0, 0.0, 0.0])  # Red color
            )
        )
        self.obstacles.append(obstacle)
        return obstacle

    def add_pedestrian(self, position):
        """Add a pedestrian to the environment."""
        pedestrian = self.world.scene.add(
            FixedCuboid(
                prim_path=f"/World/Pedestrian_{len(self.pedestrians)}",
                name=f"pedestrian_{len(self.pedestrians)}",
                position=position,
                size=np.array([0.5, 0.5, 1.8]),  # Human-like size
                color=np.array([0.0, 1.0, 0.0])  # Green color
            )
        )
        self.pedestrians.append(pedestrian)
        return pedestrian

    def set_weather(self, condition):
        """Set weather conditions (low visibility, etc.)."""
        self.weather_conditions = condition
        # In Isaac Sim, this would adjust fog, lighting, etc.
        if condition == "low_visibility":
            # Add fog effect
            create_prim("/World/Fog", "Fog", attributes={"density": 0.1})
        elif condition == "clear":
            # Remove fog
            pass

    def create_traffic(self, num_vehicles):
        """Create traffic with multiple vehicles."""
        traffic_vehicles = []
        for i in range(num_vehicles):
            pos = np.array([np.random.uniform(-20, 20), np.random.uniform(-20, 20), 0])
            vehicle = self.world.scene.add(
                FixedCuboid(
                    prim_path=f"/World/Traffic_{i}",
                    name=f"traffic_{i}",
                    position=pos,
                    size=np.array([2.0, 1.0, 1.5]),
                    color=np.array([1.0, 1.0, 0.0])  # Yellow color
                )
            )
            traffic_vehicles.append(vehicle)
        return traffic_vehicles

    def reset(self):
        """Reset the environment."""
        self.obstacles = []
        self.pedestrians = []
        self.weather_conditions = "clear"
        # Clear the world and recreate
        self.world.clear()
        self.setup_world()
