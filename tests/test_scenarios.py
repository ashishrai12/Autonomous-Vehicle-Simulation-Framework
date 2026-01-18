#!/usr/bin/env python3

"""
Unit tests for the ScenarioManager class.
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
from src.scenarios import ScenarioManager

class TestScenarioManager:
    def setup_method(self):
        self.env = Mock()
        self.vehicle = MagicMock()
        # Initialize attributes to avoid errors in numeric operations and formatting
        self.vehicle.position = np.array([0.0, 0.0, 0.0])
        self.vehicle.v = 0.0
        self.vehicle.yaw = 0.0
        self.vehicle.get_lidar_data.return_value = [10.0]
        self.manager = ScenarioManager(self.env, self.vehicle)

    def test_run_scenario_normal_navigation(self):
        """Test running normal navigation scenario."""
        with patch.object(self.manager, 'normal_navigation_scenario') as mock_scenario:
            self.manager.run_scenario("normal_navigation")
            mock_scenario.assert_called_once()

    def test_run_scenario_obstacle_avoidance(self):
        """Test running obstacle avoidance scenario."""
        with patch.object(self.manager, 'obstacle_avoidance_scenario') as mock_scenario:
            self.manager.run_scenario("obstacle_avoidance")
            mock_scenario.assert_called_once()

    def test_run_scenario_low_visibility(self):
        """Test running low visibility scenario."""
        with patch.object(self.manager, 'low_visibility_scenario') as mock_scenario:
            self.manager.run_scenario("low_visibility")
            mock_scenario.assert_called_once()

    def test_run_scenario_high_traffic(self):
        """Test running high traffic scenario."""
        with patch.object(self.manager, 'high_traffic_scenario') as mock_scenario:
            self.manager.run_scenario("high_traffic")
            mock_scenario.assert_called_once()

    def test_run_scenario_pedestrians(self):
        """Test running pedestrians scenario."""
        with patch.object(self.manager, 'pedestrians_scenario') as mock_scenario:
            self.manager.run_scenario("pedestrians")
            mock_scenario.assert_called_once()

    def test_run_scenario_unknown(self):
        """Test running unknown scenario."""
        with patch('builtins.print') as mock_print:
            self.manager.run_scenario("unknown_scenario")
            mock_print.assert_any_call("Unknown scenario: unknown_scenario")

    @patch('time.sleep')
    def test_normal_navigation_scenario(self, mock_sleep):
        """Test normal navigation scenario execution."""
        self.manager.normal_navigation_scenario()
        self.env.reset.assert_called_once()
        self.vehicle.spawn_vehicle.assert_called_once()
        self.vehicle.set_goal.assert_called_with([10.0, 5.0, 0.0]) # Matches code
        # Should call step 100 times
        assert self.vehicle.step.call_count == 100
    
    @patch('time.sleep')
    def test_low_visibility_scenario(self, mock_sleep):
        """Test low visibility scenario execution."""
        self.manager.low_visibility_scenario()
        self.env.reset.assert_called_once()
        self.env.set_weather.assert_called_with("low_visibility")
        assert self.vehicle.step.call_count == 50

    @patch('time.sleep')
    def test_obstacle_avoidance_scenario(self, mock_sleep):
        """Test obstacle avoidance scenario execution."""
        mock_obstacle1 = MagicMock()
        mock_obstacle1.get_world_pose.return_value = ([5.0, 0.5, 0.0], None)
        mock_obstacle2 = MagicMock()
        mock_obstacle2.get_world_pose.return_value = ([7.0, 2.0, 0.0], None)
        mock_obstacle3 = MagicMock()
        mock_obstacle3.get_world_pose.return_value = ([0.0, 0.0, 0.0], None)
        self.env.add_obstacle.side_effect = [mock_obstacle1, mock_obstacle2, mock_obstacle3]

        self.manager.obstacle_avoidance_scenario()

        self.env.reset.assert_called_once()
        self.vehicle.spawn_vehicle.assert_called_once()
        assert self.env.add_obstacle.call_count == 2 # In the actual code it adds 2 obstacles
        # Should call step 100 times
        assert self.vehicle.step.call_count == 100
