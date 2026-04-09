"""Safety validation for robot commands."""

import hashlib
import time
import uuid
from dataclasses import dataclass
from typing import Any


@dataclass
class ValidationResult:
    """Result of safety validation."""

    is_safe: bool
    reason: str
    severity: str = "info"  # info, warning, error


class SafetyValidator:
    """Validates robot commands for safety."""

    # Safety limits
    MAX_LINEAR_VELOCITY = 2.0  # m/s
    MAX_ANGULAR_VELOCITY = 1.0  # rad/s
    CERTIFICATE_VALIDITY = 30.0  # seconds
    CACHE_TTL_SEC = 300.0  # 5 minutes cache TTL

    def __init__(self, enable_cache: bool = True):
        self.violations = []
        self.enable_cache = enable_cache
        self._enable_cache = enable_cache
        self._validation_count = 0
        self._rejection_count = 0
        self._cache: dict[str, dict[str, Any]] = {}
        self._cache_hits = 0
        self._cache_misses = 0
        self._validation_times: list[float] = []

    def validate_command(self, command: Any) -> ValidationResult:
        """Validate a command for safety.

        Args:
            command: Command to validate

        Returns:
            ValidationResult with safety status
        """
        if command is None:
            return ValidationResult(
                is_safe=False, reason="Command cannot be None", severity="error"
            )

        if command == "":
            return ValidationResult(
                is_safe=False, reason="Command cannot be empty", severity="error"
            )

        # Command is valid
        return ValidationResult(
            is_safe=True, reason="Command passed safety checks", severity="info"
        )

    def validate_velocity(self, cmd: dict[str, Any]) -> ValidationResult:
        """Validate velocity command.

        Args:
            cmd: Velocity command with linear and angular components

        Returns:
            ValidationResult with safety status
        """
        # Check linear velocity
        linear = cmd.get("linear", {})
        linear_x = linear.get("x", 0.0)

        if abs(linear_x) > self.MAX_LINEAR_VELOCITY:
            return ValidationResult(
                is_safe=False,
                reason=f"Linear velocity {linear_x} exceeds limit {self.MAX_LINEAR_VELOCITY}",
                severity="error",
            )

        # Check angular velocity
        angular = cmd.get("angular", {})
        angular_z = angular.get("z", 0.0)

        if abs(angular_z) > self.MAX_ANGULAR_VELOCITY:
            return ValidationResult(
                is_safe=False,
                reason=f"Angular velocity {angular_z} exceeds limit {self.MAX_ANGULAR_VELOCITY}",
                severity="error",
            )

        return ValidationResult(is_safe=True, reason="Velocity within safe limits", severity="info")

    def _compute_trajectory_hash(self, trajectory: dict[str, Any], limits: dict[str, Any]) -> str:
        """Compute a hash for the trajectory and limits.

        Args:
            trajectory: Trajectory data
            limits: Safety limits

        Returns:
            Hash string
        """
        # Create a consistent string representation
        traj_str = str(sorted(trajectory.items())) if trajectory else ""
        limits_str = str(sorted(limits.items())) if limits else ""
        combined = traj_str + limits_str
        return hashlib.md5(combined.encode()).hexdigest()  # nosec B324

    def _hash_trajectory(self, trajectory: dict[str, Any]) -> str:
        """Generate a simple hash for a trajectory.

        Args:
            trajectory: Trajectory data

        Returns:
            Hash string describing the trajectory
        """
        waypoints = trajectory.get("waypoints", [])
        if not waypoints:
            return "empty"
        return f"{len(waypoints)}_waypoints"

    def _is_point_in_polygon(
        self, point: dict[str, float], polygon: list[dict[str, float]]
    ) -> bool:
        """Check if a point is inside a polygon using ray casting.

        Args:
            point: Point with x, y coordinates
            polygon: List of polygon vertices

        Returns:
            True if point is inside polygon
        """
        x, y = point.get("x", 0.0), point.get("y", 0.0)
        n = len(polygon)
        inside = False

        for i in range(n):
            j = (i + 1) % n
            xi, yi = polygon[i].get("x", 0.0), polygon[i].get("y", 0.0)
            xj, yj = polygon[j].get("x", 0.0), polygon[j].get("y", 0.0)

            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi) + xi):
                inside = not inside

        return inside

    def validate_trajectory(
        self, trajectory: dict[str, Any], limits: dict[str, Any]
    ) -> dict[str, Any]:
        """Validate a trajectory for safety.

        Args:
            trajectory: Trajectory with type and parameters
            limits: Device limits

        Returns:
            Dict with 'approved' key and optional 'rejection_reason'
        """
        start_time = time.time()
        self._validation_count += 1

        # Check cache if enabled
        cached = False
        cache_key = ""
        if self._enable_cache:
            cache_key = self._compute_trajectory_hash(trajectory, limits)
            if cache_key in self._cache:
                cached_result, cached_time = self._cache[cache_key]
                # Check if cache entry is still valid (TTL)
                if time.time() - cached_time < self.CACHE_TTL_SEC:
                    self._cache_hits += 1
                    cached = True
                    result = cached_result.copy()
                    result["cached"] = True
                    return result
                else:
                    # Expired, remove from cache
                    del self._cache[cache_key]
            self._cache_misses += 1

        # Initialize result
        result: dict[str, Any] = {"approved": True}
        rejection_reason = None

        # Check for velocities array (direct velocity trajectory)
        velocities = trajectory.get("velocities", [])
        if velocities:
            max_vel = limits.get("max_linear_velocity", float("inf"))
            for v in velocities:
                if v > max_vel:
                    rejection_reason = f"velocity_exceeded: {v} > {max_vel}"
                    break

        # Check angular velocities
        if not rejection_reason:
            angular_velocities = trajectory.get("angular_velocities", [])
            if angular_velocities:
                max_angular = limits.get("max_angular_velocity", float("inf"))
                for v in angular_velocities:
                    if v > max_angular:
                        rejection_reason = f"angular velocity_exceeded: {v} > {max_angular}"
                        break

        # Check joint velocities
        if not rejection_reason:
            joint_velocities = trajectory.get("joint_velocities", [])
            if joint_velocities:
                max_joint_vel = limits.get("max_joint_velocity", float("inf"))
                for v in joint_velocities:
                    if v > max_joint_vel:
                        rejection_reason = f"joint velocity_exceeded: {v} > {max_joint_vel}"
                        break

        # Check forces
        if not rejection_reason:
            forces = trajectory.get("forces", [])
            if forces:
                max_force = limits.get("max_force", float("inf"))
                for f in forces:
                    if f > max_force:
                        rejection_reason = f"force_exceeded: {f} > {max_force}"
                        break

        # Check workspace bounds
        if not rejection_reason:
            waypoints = trajectory.get("waypoints", [])
            workspace_bounds = limits.get("workspace_bounds")
            if waypoints and workspace_bounds:
                # Handle different workspace bounds formats
                if isinstance(workspace_bounds, dict):
                    x_min = workspace_bounds.get("x_min", float("-inf"))
                    x_max = workspace_bounds.get("x_max", float("inf"))
                    y_min = workspace_bounds.get("y_min", float("-inf"))
                    y_max = workspace_bounds.get("y_max", float("inf"))
                elif (
                    isinstance(workspace_bounds, list)
                    and len(workspace_bounds) == 4
                    and all(isinstance(x, (int, float)) for x in workspace_bounds)
                ):
                    # Simple list format: [x_min, x_max, y_min, y_max]
                    x_min, x_max, y_min, y_max = workspace_bounds
                elif isinstance(workspace_bounds, list) and len(workspace_bounds) >= 4:
                    # Polygon format - compute bounding box
                    xs = [p.get("x", 0.0) for p in workspace_bounds if isinstance(p, dict)]
                    ys = [p.get("y", 0.0) for p in workspace_bounds if isinstance(p, dict)]
                    x_min, x_max = min(xs) if xs else float("-inf"), max(xs) if xs else float("inf")
                    y_min, y_max = min(ys) if ys else float("-inf"), max(ys) if ys else float("inf")
                else:
                    x_min, x_max, y_min, y_max = (
                        float("-inf"),
                        float("inf"),
                        float("-inf"),
                        float("inf"),
                    )

                for wp in waypoints:
                    x = wp.get("x", 0.0)
                    y = wp.get("y", 0.0)
                    if x < x_min or x > x_max or y < y_min or y > y_max:
                        rejection_reason = f"workspace bounds violated: ({x}, {y}) outside bounds"
                        break

        # Check restricted zones
        if not rejection_reason:
            restricted_zones = limits.get("restricted_zones", [])
            if waypoints and restricted_zones:
                for zone in restricted_zones:
                    zone_bounds = zone.get("bounds", [])
                    if zone_bounds:
                        for wp in waypoints:
                            if self._is_point_in_polygon(wp, zone_bounds):
                                rejection_reason = f"restricted zone violation: waypoint in {zone.get('name', 'unknown')}"
                                break
                    if rejection_reason:
                        break

        # Check limits from parameters (legacy)
        if not rejection_reason:
            params = trajectory.get("parameters", {})
            velocity = params.get("velocity")
            if velocity is not None:
                max_vel = limits.get("max_velocity", float("inf"))
                if velocity > max_vel:
                    rejection_reason = f"velocity_exceeded: {velocity} > {max_vel}"

        # Set final result
        if rejection_reason:
            result["approved"] = False
            result["rejection_reason"] = rejection_reason
            self._rejection_count += 1

        # Generate certificate
        issued_at = time.time()
        certificate = {
            "validation_id": str(uuid.uuid4()),
            "issued_at": issued_at,
            "expires_at": issued_at + self.CERTIFICATE_VALIDITY,
            "trajectory_hash": self._hash_trajectory(trajectory),
        }
        result["certificate"] = certificate

        # Track timing
        elapsed = (time.time() - start_time) * 1000  # Convert to ms
        self._validation_times.append(elapsed)

        # Cache result if enabled (store as tuple with timestamp for TTL)
        if self._enable_cache and cache_key:
            result_copy = result.copy()
            result_copy["cached"] = False
            self._cache[cache_key] = (result_copy, time.time())

        result["cached"] = cached
        return result

    def get_stats(self) -> dict[str, Any]:
        """Get validation statistics.

        Returns:
            Dict with validation statistics
        """
        avg_time = (
            sum(self._validation_times) / len(self._validation_times)
            if self._validation_times
            else 0.0
        )
        max_time = max(self._validation_times) if self._validation_times else 0.0

        return {
            "validation_count": self._validation_count,
            "rejection_count": self._rejection_count,
            "average_validation_time_ms": avg_time,
            "max_validation_time_ms": max_time,
        }

    def get_cache_statistics(self) -> dict[str, Any]:
        """Get cache statistics.

        Returns:
            Dict with cache statistics
        """
        total = self._cache_hits + self._cache_misses
        hit_rate = self._cache_hits / total if total > 0 else 0.0

        return {
            "cache_hits": self._cache_hits,
            "cache_misses": self._cache_misses,
            "hit_rate": hit_rate,
        }

    def clear_cache(self) -> None:
        """Clear the validation cache."""
        self._cache.clear()
        self._cache_hits = 0
        self._cache_misses = 0


# Alias for backwards compatibility
SafetyValidatorNode = SafetyValidator
