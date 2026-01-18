import yaml
import os
from vehicle import AutonomousVehicle
from environment import SimulationEnvironment
from scenarios import ScenarioManager

def load_config(config_path="config/settings.yaml"):
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            return yaml.safe_load(f)
    return {}

def main():
    config = load_config()
    
    # Try to launch Isaac Sim SimulationApp
    try:
        from isaacsim import SimulationApp
        simulation_app = SimulationApp({"headless": False})
    except ImportError:
        print("Isaac Sim not found. Running in headless/mock mode.")
        simulation_app = None

    # Create simulation environment
    env = SimulationEnvironment()

    # Create autonomous vehicle with config
    vehicle = AutonomousVehicle(config=config)

    # Create scenario manager with config
    scenario_manager = ScenarioManager(env, vehicle, config=config)

    # Run different scenarios
    scenarios = [
        "normal_navigation",
        "obstacle_avoidance",
        "low_visibility",
        "high_traffic",
        "pedestrians"
    ]

    for scenario_name in scenarios:
        scenario_manager.run_scenario(scenario_name)

    # Cleanup
    if simulation_app:
        simulation_app.close()

if __name__ == "__main__":
    main()
