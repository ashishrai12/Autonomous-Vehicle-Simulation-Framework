#!/usr/bin/env python3

"""
Main entry point for the Autonomous Vehicle Simulation Framework in Isaac Sim.
"""

from isaacsim import SimulationApp
import omni
from vehicle import AutonomousVehicle
from environment import SimulationEnvironment
from scenarios import ScenarioManager

def main():
    # Launch Isaac Sim
    simulation_app = SimulationApp({"headless": False})

    # Create simulation environment
    env = SimulationEnvironment()

    # Create autonomous vehicle
    vehicle = AutonomousVehicle()

    # Create scenario manager
    scenario_manager = ScenarioManager(env, vehicle)

    # Run different scenarios
    scenarios = [
        "normal_navigation",
        "obstacle_avoidance",
        "low_visibility",
        "high_traffic",
        "pedestrians"
    ]

    for scenario_name in scenarios:
        print(f"Running scenario: {scenario_name}")
        scenario_manager.run_scenario(scenario_name)

    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
