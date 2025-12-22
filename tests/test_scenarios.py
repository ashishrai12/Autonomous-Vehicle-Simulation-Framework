#!/usr/bin/env python3

"""
Unit tests for the ScenarioManager class.
"""

import pytest
from unittest.mock import Mock, patch
from scenarios import ScenarioManager

class TestScenarioManager:
    def setup_method(self):
        self.env = Mock()
        self.vehicle = Mock()
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
            mock_print.assert_called_with("Unknown scenario: unknown_scenario")

    @patch('time.sleep')
    def test_normal_navigation_scenario(self, mock_sleep):
        """Test normal navigation scenario execution."""
        self.manager.normal_navigation_scenario()
        self.env.setup_world.assert_called_once()
        self.vehicle.spawn_vehicle.assert_called_once()
        self.vehicle.set_goal.assert_called_with([10.0, 0.0, 0.0])
        # Should call navigate_to_goal 100 times
        assert self.vehicle.navigate_to_goal.call_count == 100

    @patch('time.sleep')
    def test_obstacle_avoidance_scenario(self, mock_sleep):
        """Test obstacle avoidance scenario execution."""
        mock_obstacle1 = Mock()
        mock_obstacle2 = Mock()
        mock_obstacle3 = Mock()
        self.env.add_obstacle.side_effect = [mock_obstacle1, mock_obstacle2, mock_obstacle3]

        self.manager.obstacle_avoidance_scenario()

        self.env.setup_world.assert_called_once()
        self.vehicle.spawn_vehicle.assert_called_once()
        assert self.env.add_obstacle.call_count == 3
        self.vehicle.set_goal.assert_called_with([10.0, 0.0, 0.0])
        # Should call avoid_obstacles 100 times
        assert self.vehicle.avoid_obstacles.call_count == 100
