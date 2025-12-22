#!/usr/bin/env python3

"""
Scenario Manager and Test Scenarios for Autonomous Vehicle Simulation.
"""

import numpy as np
import time

class ScenarioManager:
    def __init__(self, environment, vehicle):
        self.env = environment
        self.vehicle = vehicle

    def run_scenario(self, scenario_name):
        """Run a specific scenario."""
        if scenario_name == "normal_navigation":
            self.normal_navigation_scenario()
        elif scenario_name == "obstacle_avoidance":
            self.obstacle_avoidance_scenario()
        elif scenario_name == "low_visibility":
            self.low_visibility_scenario()
        elif scenario_name == "high_traffic":
            self.high_traffic_scenario()
        elif scenario_name == "pedestrians":
            self.pedestrians_scenario()
        else:
            print(f"Unknown scenario: {scenario_name}")

    def normal_navigation_scenario(self):
        """Scenario: Normal navigation to a goal."""
        self.env.setup_world()
        self.vehicle.spawn_vehicle()
        self.vehicle.set_goal([10.0, 0.0, 0.0])

        # Simulate for some steps
        for _ in range(100):
            self.vehicle.navigate_to_goal()
            time.sleep(0.1)  # Simulation step

        print("Normal navigation scenario completed.")

    def obstacle_avoidance_scenario(self):
        """Scenario: Navigate with obstacles."""
        self.env.setup_world()
        self.vehicle.spawn_vehicle()

        # Add obstacles
        obstacles = [
            self.env.add_obstacle([5.0, 0.0, 0.0]),
            self.env.add_obstacle([7.0, 2.0, 0.0]),
            self.env.add_obstacle([3.0, -1.0, 0.0])
        ]

        self.vehicle.set_goal([10.0, 0.0, 0.0])

        # Simulate with obstacle avoidance
        for _ in range(100):
            self.vehicle.avoid_obstacles(obstacles)
            time.sleep(0.1)

        print("Obstacle avoidance scenario completed.")

    def low_visibility_scenario(self):
        """Scenario: Navigation in low visibility conditions."""
        self.env.setup_world()
        self.env.set_weather("low_visibility")
        self.vehicle.spawn_vehicle()

        # Add some obstacles
        obstacles = [
            self.env.add_obstacle([5.0, 0.0, 0.0]),
            self.env.add_obstacle([8.0, 1.0, 0.0])
        ]

        self.vehicle.set_goal([10.0, 0.0, 0.0])

        # Simulate with reduced sensor accuracy (simulated)
        for _ in range(100):
            # In low visibility, sensor data is noisier
            self.vehicle.avoid_obstacles(obstacles)
            time.sleep(0.1)

        print("Low visibility scenario completed.")

    def high_traffic_scenario(self):
        """Scenario: Navigation in high traffic."""
        self.env.setup_world()
        self.vehicle.spawn_vehicle()

        # Create traffic
        traffic = self.env.create_traffic(5)

        # Treat traffic as dynamic obstacles
        self.vehicle.set_goal([10.0, 0.0, 0.0])

        for _ in range(100):
            # Dynamic obstacle avoidance
            self.vehicle.avoid_obstacles(traffic)
            time.sleep(0.1)

        print("High traffic scenario completed.")

    def pedestrians_scenario(self):
        """Scenario: Navigation with pedestrians."""
        self.env.setup_world()
        self.vehicle.spawn_vehicle()

        # Add pedestrians
        pedestrians = [
            self.env.add_pedestrian([6.0, 1.0, 0.0]),
            self.env.add_pedestrian([4.0, -1.0, 0.0]),
            self.env.add_pedestrian([8.0, 0.5, 0.0])
        ]

        self.vehicle.set_goal([10.0, 0.0, 0.0])

        for _ in range(100):
            self.vehicle.avoid_obstacles(pedestrians)
            time.sleep(0.1)

        print("Pedestrians scenario completed.")
