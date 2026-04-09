"""Multi-robot coordination and swarm behaviors."""

import math
from typing import Any


class SwarmCoordinator:
    """Coordinates multiple robots in swarm formations."""

    def calculate_formation(
        self,
        robots: list[str],
        formation_type: str,
        spacing: float,
    ) -> list[dict[str, Any]]:
        """Calculate positions for swarm formation.

        Args:
            robots: List of robot IDs
            formation_type: "line", "circle", "triangle"
            spacing: Distance between robots

        Returns:
            List of position assignments
        """
        positions = []
        n = len(robots)

        if formation_type == "line":
            # Line formation along x-axis
            for i, robot in enumerate(robots):
                positions.append(
                    {
                        "robot_id": robot,
                        "position": (i * spacing, 0),
                    }
                )

        elif formation_type == "circle":
            # Circle formation
            radius = spacing * n / (2 * math.pi)
            for i, robot in enumerate(robots):
                angle = 2 * math.pi * i / n
                positions.append(
                    {
                        "robot_id": robot,
                        "position": (
                            radius * math.cos(angle),
                            radius * math.sin(angle),
                        ),
                    }
                )

        elif formation_type == "triangle":
            # Triangle formation
            for i, robot in enumerate(robots):
                row = int(math.sqrt(2 * i + 0.25) - 0.5)
                col = i - row * (row + 1) // 2
                positions.append(
                    {
                        "robot_id": robot,
                        "position": (
                            col * spacing - row * spacing / 2,
                            row * spacing * math.sqrt(3) / 2,
                        ),
                    }
                )

        return positions

    def should_share_task(
        self,
        task: dict[str, Any],
        current_robot: str,
        available_robots: list[str],
    ) -> bool:
        """Decide if task should be shared with other robots."""
        # Heavy tasks should be shared
        if task.get("type") == "lift":
            weight = task.get("weight", 0)
            if weight > 30:  # Heavy threshold
                return True

        # Urgent tasks with available robots
        return task.get("priority", 5) <= 2 and bool(available_robots)

    def check_collision_risk(
        self,
        robot1_pos: dict[str, float],
        robot2_pos: dict[str, float],
    ) -> float:
        """Check collision risk between two robots.

        Returns:
            Risk score (0-1, higher is more risky)
        """
        # Calculate distance
        dx = robot1_pos["x"] - robot2_pos["x"]
        dy = robot1_pos["y"] - robot2_pos["y"]
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate relative velocity
        dvx = robot1_pos.get("vx", 0) - robot2_pos.get("vx", 0)
        dvy = robot1_pos.get("vy", 0) - robot2_pos.get("vy", 0)
        rel_speed = math.sqrt(dvx**2 + dvy**2)

        # Time to collision estimate
        if rel_speed < 0.01:
            return 0.0  # Not moving toward each other

        time_to_collision = distance / rel_speed

        # Risk increases as time to collision decreases
        if time_to_collision < 2.0:
            return 1.0  # High risk
        elif time_to_collision < 5.0:
            return 0.5  # Medium risk

        return 0.0  # Low risk

    def suggest_avoidance(
        self,
        robot1_pos: dict[str, float],
        robot2_pos: dict[str, float],
    ) -> dict[str, Any]:
        """Suggest avoidance maneuver."""
        # Calculate direction away from other robot
        dx = robot1_pos["x"] - robot2_pos["x"]
        dy = robot1_pos["y"] - robot2_pos["y"]
        distance = math.sqrt(dx**2 + dy**2)

        if distance < 0.01:
            return {"action": "stop"}

        # Normalize and scale
        avoid_x = dx / distance * 1.0  # Move 1m away
        avoid_y = dy / distance * 1.0

        return {
            "action": "move_relative",
            "delta_x": avoid_x,
            "delta_y": avoid_y,
        }

    def assign_roles(
        self,
        robots: dict[str, dict[str, Any]],
        mission: str,
    ) -> dict[str, str]:
        """Assign roles to robots based on capabilities and mission."""
        roles = {}

        # Sort robots by battery (highest first)
        sorted_robots = sorted(robots.items(), key=lambda r: r[1].get("battery", 0), reverse=True)

        if mission == "surveillance":
            # Assign camera-equipped robots to surveillance
            for robot_id, robot_data in sorted_robots:
                if "camera" in robot_data.get("capabilities", []):
                    roles[robot_id] = "surveillance"
                    break

        # Assign remaining as navigators
        for robot_id, _ in sorted_robots:
            if robot_id not in roles:
                roles[robot_id] = "navigator"

        return roles
