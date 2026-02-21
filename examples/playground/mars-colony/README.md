# 🚀 Mars Colony Command

Multi-robot habitat management simulation. Commander view with fleet coordination and resource management.

## Concept

You are the Commander of Mars Habitat Alpha. Manage a fleet of specialized robots:
- **Excavators**: Mine ice and minerals
- **Solar Drones**: Deploy and maintain energy farms
- **Builder Bots**: Construct hab modules
- **Rover Scouts**: Explore and map terrain
- **Med Bots**: Monitor crew health (simulated)

## Game Mechanics

```
Resources: Ice ⚪ | Power ⚡ | Minerals ⬡ | Oxygen ◎
Mission: Survive 30 sols, expand habitat, keep crew alive
```

## Files

- `mars_command.py` - Main command interface
- `colony_fleet.py` - Fleet management
- `resource_sim.py` - Mars resource model
- `habitat_builder.py` - Construction system

## Run

```bash
docker-compose up
# Terminal UI for colony management
```

## Features

- Real-time fleet coordination
- Resource scarcity challenges
- Emergency scenarios (dust storms, equipment failure)
- Construction queue management
- Exploration discovery system