#!/usr/bin/env python3

"""
Unit tests for the AutonomousVehicle class.
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch
from vehicle import AutonomousVehicle

class TestAutonomousVehicle:
    def setup_method(self):
        self.vehicle = AutonomousVehicle()

    def test_initialization(self):
        """Test vehicle initialization."""
        assert np.array_equal(self.vehicle.position, np.array([0.0, 0.0, 0.0]))
        assert np.array_equal(self.vehicle.velocity, np.array([0.0, 0.0, 0.0]))
        assert np.array_equal(self.vehicle.goal, np.array([10.0, 0.0, 0.0]))

    def test_set_goal(self):
        """Test setting a new goal."""
        new_goal = [5.0, 3.0, 0.0]
        self.vehicle.set_goal(new_goal)
        assert np.array_equal(self.vehicle.goal, np.array(new_goal))

    def test_navigate_to_goal(self):
        """Test navigation towards goal."""
        initial_pos = self.vehicle.position.copy()
        self.vehicle.navigate_to_goal()
        # Position should have changed
        assert not np.array_equal(self.vehicle.position, initial_pos)

    def test_avoid_obstacles(self):
        """Test obstacle avoidance."""
        # Mock obstacles
        obstacle1 = Mock()
        obstacle1.get_world_pose.return_value = ([5.0, 0.0, 0.0], None)
        obstacle2 = Mock()
        obstacle2.get_world_pose.return_value = ([0.0, 5.0, 0.0], None)

        obstacles = [obstacle1, obstacle2]
        initial_pos = self.vehicle.position.copy()
        self.vehicle.avoid_obstacles(obstacles)
        # Position should have changed due to avoidance
        assert not np.array_equal(self.vehicle.position, initial_pos)

    def test_plan_path(self):
        """Test path planning."""
        start = [0.0, 0.0, 0.0]
        goal = [10.0, 0.0, 0.0]
        obstacles = []
        path = self.vehicle.plan_path(start, goal, obstacles)
        assert len(path) >= 2
        assert np.array_equal(path[0], start)
        assert np.array_equal(path[-1], goal)

    def test_get_sensor_data(self):
        """Test sensor data retrieval."""
        data = self.vehicle.get_sensor_data()
        assert "lidar" in data
        assert "camera" in data
