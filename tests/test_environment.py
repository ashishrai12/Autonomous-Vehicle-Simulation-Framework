#!/usr/bin/env python3

"""
Unit tests for the SimulationEnvironment class.
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch
from environment import SimulationEnvironment

class TestSimulationEnvironment:
    def setup_method(self):
        self.env = SimulationEnvironment()

    def test_initialization(self):
        """Test environment initialization."""
        assert self.env.weather_conditions == "clear"
        assert len(self.env.obstacles) == 0
        assert len(self.env.pedestrians) == 0

    @patch('environment.create_prim')
    def test_setup_world(self, mock_create_prim):
        """Test world setup."""
        self.env.setup_world()
        # Check that lighting was added
        mock_create_prim.assert_called_with("/World/Light", "DistantLight", attributes={"intensity": 3000})

    @patch('environment.FixedCuboid')
    def test_add_obstacle(self, mock_cuboid):
        """Test adding obstacles."""
        position = [1.0, 2.0, 0.0]
        obstacle = self.env.add_obstacle(position)
        assert len(self.env.obstacles) == 1
        mock_cuboid.assert_called()

    @patch('environment.FixedCuboid')
    def test_add_pedestrian(self, mock_cuboid):
        """Test adding pedestrians."""
        position = [3.0, 4.0, 0.0]
        pedestrian = self.env.add_pedestrian(position)
        assert len(self.env.pedestrians) == 1
        mock_cuboid.assert_called()

    def test_set_weather(self):
        """Test setting weather conditions."""
        self.env.set_weather("low_visibility")
        assert self.env.weather_conditions == "low_visibility"

        self.env.set_weather("clear")
        assert self.env.weather_conditions == "clear"

    @patch('environment.FixedCuboid')
    def test_create_traffic(self, mock_cuboid):
        """Test creating traffic."""
        traffic = self.env.create_traffic(3)
        assert len(traffic) == 3
        assert mock_cuboid.call_count == 3

    @patch('environment.SimulationEnvironment.setup_world')
    def test_reset(self, mock_setup):
        """Test environment reset."""
        self.env.obstacles = [Mock()]
        self.env.pedestrians = [Mock()]
        self.env.weather_conditions = "low_visibility"

        self.env.reset()

        assert len(self.env.obstacles) == 0
        assert len(self.env.pedestrians) == 0
        assert self.env.weather_conditions == "clear"
        mock_setup.assert_called_once()
