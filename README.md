# Autonomous Vehicle Simulation Framework

This project provides a basic framework for simulating autonomous vehicles in Isaac Sim. It includes modules for vehicle control, environment setup, and various test scenarios including edge cases like low visibility, high traffic, and pedestrians.

## Features

- Autonomous vehicle with navigation, obstacle avoidance, and path planning
- Modular scenario system for testing
- Edge case scenarios: low visibility, high traffic, pedestrians
- Integration with Isaac Sim for realistic simulation

## Requirements

- Isaac Sim (NVIDIA Omniverse)
- Python 3.7+
- Required Python packages (see requirements.txt)

## Installation

1. Install Isaac Sim from NVIDIA's website.
2. Clone this repository.
3. Install dependencies: `pip install -r requirements.txt`

## Usage

Run the main simulation:

```bash
python main.py
```

## Project Structure

- `main.py`: Entry point for the simulation
- `vehicle.py`: Autonomous vehicle implementation
- `environment.py`: Simulation environment setup
- `scenarios.py`: Test scenarios and edge cases
- `tests/`: Unit tests
- `requirements.txt`: Python dependencies
