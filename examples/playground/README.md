# 🎮 Playground Examples

Four complete, Dockerized ROS2 examples demonstrating real-world AI + robotics integration.

---

## 🌱 Talking Garden

**Concept:** LLM monitors and converses with 6 IoT-enabled plants

**Features:**
- 6 simulated plants with unique personalities
- Real-time moisture/temperature sensors
- AI-generated plant dialogue
- Web dashboard for monitoring
- Automatic watering control

**Run:**
```bash
cd talking-garden
docker-compose -f docker-compose.ros2.yml up
# Open http://localhost:8080
```

**Topics:**
- `/garden/plants/{name}/moisture`
- `/garden/plants/{name}/health`
- `/garden/dialogue`

---

## 🚀 Mars Colony

**Concept:** Multi-robot task coordination on Mars

**Features:**
- 4 robot types: Excavator, Solar Drone, Builder, Scout
- Task assignment and coordination
- Resource management
- Real-time position tracking
- Mission control dashboard

**Run:**
```bash
cd mars-colony
docker-compose -f docker-compose.ros2.yml up
# Open http://localhost:8080
```

**Actions:**
- `excavate_area`
- `deploy_solar`
- `construct_base`
- `scout_terrain`

---

## 🎭 Theater Bots

**Concept:** AI director controls robot actors on stage

**Features:**
- AI director node
- 3 robot actors with roles
- Scripted performance
- Real-time dialogue
- Stage position tracking

**Run:**
```bash
cd theater-bots
docker-compose -f docker-compose.ros2.yml up
# Open http://localhost:8080/stage.html
```

**Topics:**
- `/theater/cues`
- `/theater/dialogue`
- `/theater/positions`

---

## 🎨 Art Studio

**Concept:** Human and robot collaborative painting

**Features:**
- Human painter interface
- Robot painter with AI vision
- Shared canvas state
- Color mixing logic
- Collaborative artwork generation

**Run:**
```bash
cd art-studio
docker-compose -f docker-compose.ros2.yml up
# Open http://localhost:8080/canvas.html
```

**Services:**
- `paint_stroke`
- `mix_color`
- `analyze_composition`

---

## Requirements

- Docker + Docker Compose
- 4GB RAM minimum per example
- Ports 8080, 8765-8767 available

## Common Commands

```bash
# Start any example
cd <example-name>
docker-compose -f docker-compose.ros2.yml up

# View logs
docker-compose logs -f

# Stop
docker-compose down

# Rebuild after changes
docker-compose build --no-cache
```

---

*These examples showcase the full capabilities of Agent ROS Bridge in realistic scenarios.*
