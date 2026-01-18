#!/usr/bin/env python3

"""
Scenario Manager and Test Scenarios for Autonomous Vehicle Simulation.
"""

import numpy as np
import time

class ScenarioManager:
    def __init__(self, environment, vehicle, config=None):
        self.env = environment
        self.vehicle = vehicle
        self.config = config or {}
        self.dt = self.config.get('simulation', {}).get('dt', 0.1)

    def run_scenario(self, scenario_name):
        """Run a specific scenario."""
        print(f"\n--- Starting Scenario: {scenario_name} ---")
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
        print(f"--- Scenario {scenario_name} Completed ---\n")

    def _get_control(self, goal, obstacles):
        """Simple proportional controller for steering and speed."""
        # Calculate distance and angle to goal
        dx = goal[0] - self.vehicle.position[0]
        dy = goal[1] - self.vehicle.position[1]
        dist = np.sqrt(dx**2 + dy**2)
        target_yaw = np.arctan2(dy, dx)
        
        # Yaw error
        yaw_err = target_yaw - self.vehicle.yaw
        yaw_err = (yaw_err + np.pi) % (2 * np.pi) - np.pi # Normalize
        
        # Steering: P-control
        steer = 0.5 * yaw_err
        steer = np.clip(steer, -0.5, 0.5)
        
        # Speed: P-control
        speed_err = 2.0 - self.vehicle.v # Target speed 2.0
        accel = 1.0 * speed_err
        
        # Basic obstacle avoidance logic (repulsive force affecting steering)
        avoidance_yaw = 0.0
        for obs in obstacles:
            obs_pos, _ = obs.get_world_pose()
            d_obs = np.linalg.norm(obs_pos[:2] - self.vehicle.position[:2])
            if d_obs < 5.0:
                repulse_yaw = np.arctan2(self.vehicle.position[1] - obs_pos[1], self.vehicle.position[0] - obs_pos[0])
                avoidance_yaw += (repulse_yaw - self.vehicle.yaw) * (1.0 / d_obs)
        
        steer += 0.3 * avoidance_yaw
        steer = np.clip(steer, -0.5, 0.5)
        
        return accel, steer

    def normal_navigation_scenario(self):
        """Scenario: Normal navigation to a goal."""
        self.env.reset()
        self.vehicle.spawn_vehicle()
        goal = [10.0, 5.0, 0.0]
        self.vehicle.set_goal(goal)

        for i in range(100):
            accel, steer = self._get_control(goal, [])
            self.vehicle.step(accel, steer, self.dt)
            if i % 20 == 0:
                print(f"Step {i}: Pos={self.vehicle.position[:2]}, V={self.vehicle.v:.2f}")

    def obstacle_avoidance_scenario(self):
        """Scenario: Navigate with obstacles."""
        self.env.reset()
        self.vehicle.spawn_vehicle()

        obstacles = [
            self.env.add_obstacle([5.0, 0.5, 0.0]),
            self.env.add_obstacle([7.0, 2.0, 0.0])
        ]
        goal = [12.0, 0.0, 0.0]

        for i in range(100):
            accel, steer = self._get_control(goal, obstacles)
            self.vehicle.step(accel, steer, self.dt)
            # Simulate lidar
            lidar_data = self.vehicle.get_lidar_data(obstacles)
            if i % 20 == 0:
                print(f"Step {i}: Pos={self.vehicle.position[:2]}, Lidar Min={min(lidar_data) if lidar_data else 'N/A'}")

    def low_visibility_scenario(self):
        self.env.reset()
        self.env.set_weather("low_visibility")
        self.vehicle.spawn_vehicle()
        goal = [10.0, 0.0, 0.0]
        for i in range(50):
            accel, steer = self._get_control(goal, [])
            self.vehicle.step(accel, steer, self.dt)

    def high_traffic_scenario(self):
        self.env.reset()
        self.vehicle.spawn_vehicle()
        traffic = self.env.create_traffic(5)
        goal = [15.0, 0.0, 0.0]
        for i in range(100):
            accel, steer = self._get_control(goal, traffic)
            self.vehicle.step(accel, steer, self.dt)

    def pedestrians_scenario(self):
        self.env.reset()
        self.vehicle.spawn_vehicle()
        pedestrians = [self.env.add_pedestrian([6.0, 1.0, 0.0])]
        goal = [10.0, 0.0, 0.0]
        for i in range(50):
            accel, steer = self._get_control(goal, pedestrians)
            self.vehicle.step(accel, steer, self.dt)
