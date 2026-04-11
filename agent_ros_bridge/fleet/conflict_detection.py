"""
Path Conflict Detection for Multi-Robot Coordination

Detects and resolves path conflicts between multiple robots.
Implements various resolution strategies including priority-based,
time-based, and auction-based conflict resolution.

Usage:
    from agent_ros_bridge.fleet.conflict_detection import ConflictDetector
    
    detector = ConflictDetector()
    conflicts = detector.detect_conflicts(robot_paths)
    resolved = detector.resolve_conflicts(conflicts)
"""

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class ConflictType(Enum):
    """Types of path conflicts."""
    HEAD_ON = "head_on"  # Robots moving toward each other
    CROSSING = "crossing"  # Paths cross at intersection
    FOLLOWING = "following"  # Same path, different speeds
    MERGING = "merging"  # Paths merge
    INTERSECTION = "intersection"  # General intersection conflict
    STATIC = "static"  # Robot blocks path (e.g., stopped)


class ResolutionStrategy(Enum):
    """Strategies for resolving conflicts."""
    PRIORITY = "priority"  # Higher priority robot goes first
    WAIT = "wait"  # Lower priority robot waits
    REROUTE = "reroute"  # Reroute one robot
    SLOW_DOWN = "slow_down"  # Slow down to avoid collision
    AUCTION = "auction"  # Auction for right-of-way


@dataclass
class PathPoint:
    """Point along a robot's path with timing."""
    x: float
    y: float
    z: float = 0.0
    timestamp: float = 0.0  # When robot will be at this point
    robot_id: str = ""
    speed: float = 0.0
    
    def distance_to(self, other: "PathPoint") -> float:
        """Calculate distance to another point."""
        return math.sqrt(
            (self.x - other.x)**2 +
            (self.y - other.y)**2 +
            (self.z - other.z)**2
        )


@dataclass
class RobotPath:
    """Path for a robot with timing information."""
    robot_id: str
    path: list[PathPoint] = field(default_factory=list)
    start_time: float = 0.0
    priority: int = 5  # 1-10, higher = more important
    max_speed: float = 1.0  # m/s
    robot_radius: float = 0.3  # meters
    
    def get_position_at_time(self, t: float) -> PathPoint | None:
        """Get robot position at specific time."""
        for point in self.path:
            if abs(point.timestamp - t) < 0.5:  # Within 0.5s
                return point
        return None
    
    def get_bounding_box(self) -> tuple[float, float, float, float]:
        """Get bounding box of path (min_x, min_y, max_x, max_y)."""
        if not self.path:
            return (0, 0, 0, 0)
        
        xs = [p.x for p in self.path]
        ys = [p.y for p in self.path]
        return (min(xs), min(ys), max(xs), max(ys))


@dataclass
class Conflict:
    """Detected conflict between robots."""
    conflict_id: str
    conflict_type: ConflictType
    robot_a: str
    robot_b: str
    location: PathPoint  # Where conflict occurs
    time_a: float  # When robot A reaches conflict point
    time_b: float  # When robot B reaches conflict point
    time_window: float  # Duration of overlap
    severity: float  # 0.0 - 1.0, higher = more severe
    
    def __hash__(self):
        return hash(self.conflict_id)


@dataclass
class Resolution:
    """Resolution for a conflict."""
    conflict_id: str
    strategy: ResolutionStrategy
    robot_to_wait: str | None = None
    robot_to_reroute: str | None = None
    wait_duration: float = 0.0  # seconds
    new_path: list[PathPoint] | None = None
    speed_reduction: float = 1.0  # Multiplier
    reason: str = ""


class ConflictDetector:
    """Detects path conflicts between robots."""
    
    def __init__(self, safety_margin: float = 0.5, time_tolerance: float = 2.0):
        """
        Initialize conflict detector.
        
        Args:
            safety_margin: Minimum distance between robots (meters)
            time_tolerance: Time window for conflict detection (seconds)
        """
        self.safety_margin = safety_margin
        self.time_tolerance = time_tolerance
        self.conflicts: list[Conflict] = []
    
    def detect_conflicts(self, paths: list[RobotPath]) -> list[Conflict]:
        """
        Detect all conflicts between robot paths.
        
        Args:
            paths: List of robot paths
        
        Returns:
            List of detected conflicts
        """
        self.conflicts = []
        
        # Compare each pair of paths
        for i, path_a in enumerate(paths):
            for path_b in paths[i+1:]:
                conflicts = self._detect_pair_conflicts(path_a, path_b)
                self.conflicts.extend(conflicts)
        
        # Sort by severity (highest first)
        self.conflicts.sort(key=lambda c: -c.severity)
        
        return self.conflicts
    
    def _detect_pair_conflicts(
        self, path_a: RobotPath, path_b: RobotPath
    ) -> list[Conflict]:
        """Detect conflicts between two paths."""
        conflicts = []
        
        # Check for spatial-temporal conflicts
        for point_a in path_a.path:
            for point_b in path_b.path:
                # Check if points are close in space
                distance = point_a.distance_to(point_b)
                
                if distance < (path_a.robot_radius + path_b.robot_radius + self.safety_margin):
                    # Points are close in space, check time
                    time_diff = abs(point_a.timestamp - point_b.timestamp)
                    
                    if time_diff < self.time_tolerance:
                        # Conflict detected!
                        conflict = self._create_conflict(
                            path_a, path_b, point_a, point_b, distance, time_diff
                        )
                        conflicts.append(conflict)
        
        # Remove duplicate conflicts (same location/time)
        unique_conflicts = self._deduplicate_conflicts(conflicts)
        
        return unique_conflicts
    
    def _create_conflict(
        self,
        path_a: RobotPath,
        path_b: RobotPath,
        point_a: PathPoint,
        point_b: PathPoint,
        distance: float,
        time_diff: float,
    ) -> Conflict:
        """Create a Conflict object."""
        # Determine conflict type
        conflict_type = self._classify_conflict(path_a, path_b, point_a, point_b)
        
        # Calculate severity
        severity = self._calculate_severity(distance, time_diff)
        
        # Create unique ID
        conflict_id = f"{path_a.robot_id}_{path_b.robot_id}_{int(time.time() * 1000)}"
        
        return Conflict(
            conflict_id=conflict_id,
            conflict_type=conflict_type,
            robot_a=path_a.robot_id,
            robot_b=path_b.robot_id,
            location=PathPoint(
                x=(point_a.x + point_b.x) / 2,
                y=(point_a.y + point_b.y) / 2,
                z=(point_a.z + point_b.z) / 2,
            ),
            time_a=point_a.timestamp,
            time_b=point_b.timestamp,
            time_window=self.time_tolerance - time_diff,
            severity=severity,
        )
    
    def _classify_conflict(
        self,
        path_a: RobotPath,
        path_b: RobotPath,
        point_a: PathPoint,
        point_b: PathPoint,
    ) -> ConflictType:
        """Classify the type of conflict."""
        # Get direction vectors
        if len(path_a.path) > 1 and len(path_b.path) > 1:
            idx_a = path_a.path.index(point_a) if point_a in path_a.path else 0
            idx_b = path_b.path.index(point_b) if point_b in path_b.path else 0
            
            if idx_a < len(path_a.path) - 1 and idx_b < len(path_b.path) - 1:
                dir_a = (
                    path_a.path[idx_a + 1].x - point_a.x,
                    path_a.path[idx_a + 1].y - point_a.y,
                )
                dir_b = (
                    path_b.path[idx_b + 1].x - point_b.x,
                    path_b.path[idx_b + 1].y - point_b.y,
                )
                
                # Check if head-on (dot product < 0)
                dot_product = dir_a[0] * dir_b[0] + dir_a[1] * dir_b[1]
                if dot_product < -0.5:
                    return ConflictType.HEAD_ON
                
                # Check if following (similar direction)
                if dot_product > 0.8:
                    return ConflictType.FOLLOWING
                
                # Check if crossing (perpendicular)
                if abs(dot_product) < 0.3:
                    return ConflictType.CROSSING
        
        return ConflictType.INTERSECTION
    
    def _calculate_severity(self, distance: float, time_diff: float) -> float:
        """Calculate conflict severity (0.0 - 1.0)."""
        # Closer distance = higher severity
        distance_severity = max(0, 1.0 - (distance / self.safety_margin))
        
        # Smaller time diff = higher severity
        time_severity = max(0, 1.0 - (time_diff / self.time_tolerance))
        
        # Combined severity
        return min(1.0, (distance_severity + time_severity) / 2)
    
    def _deduplicate_conflicts(self, conflicts: list[Conflict]) -> list[Conflict]:
        """Remove duplicate conflicts."""
        seen = set()
        unique = []
        
        for conflict in conflicts:
            # Create key based on robots and approximate location/time
            key = (
                tuple(sorted([conflict.robot_a, conflict.robot_b])),
                round(conflict.location.x, 1),
                round(conflict.location.y, 1),
                round(conflict.time_a, 0),
            )
            
            if key not in seen:
                seen.add(key)
                unique.append(conflict)
        
        return unique
    
    def check_imminent_collision(
        self, paths: list[RobotPath], horizon_sec: float = 5.0
    ) -> list[Conflict]:
        """Check for collisions within time horizon."""
        current_time = time.time()
        imminent = []
        
        for conflict in self.conflicts:
            time_to_conflict = min(conflict.time_a, conflict.time_b) - current_time
            if 0 < time_to_conflict < horizon_sec:
                imminent.append(conflict)
        
        return imminent


class ConflictResolver:
    """Resolves detected conflicts."""
    
    def __init__(self, default_strategy: ResolutionStrategy = ResolutionStrategy.PRIORITY):
        self.default_strategy = default_strategy
        self.resolutions: dict[str, Resolution] = {}
    
    def resolve_conflicts(
        self,
        conflicts: list[Conflict],
        paths: dict[str, RobotPath],
        strategy: ResolutionStrategy | None = None,
    ) -> list[Resolution]:
        """
        Resolve all conflicts.
        
        Args:
            conflicts: List of conflicts to resolve
            paths: Dict of robot_id -> RobotPath
            strategy: Resolution strategy (uses default if None)
        
        Returns:
            List of resolutions
        """
        strategy = strategy or self.default_strategy
        resolutions = []
        
        for conflict in conflicts:
            resolution = self._resolve_single_conflict(conflict, paths, strategy)
            if resolution:
                resolutions.append(resolution)
                self.resolutions[conflict.conflict_id] = resolution
        
        return resolutions
    
    def _resolve_single_conflict(
        self,
        conflict: Conflict,
        paths: dict[str, RobotPath],
        strategy: ResolutionStrategy,
    ) -> Resolution | None:
        """Resolve a single conflict."""
        path_a = paths.get(conflict.robot_a)
        path_b = paths.get(conflict.robot_b)
        
        if not path_a or not path_b:
            return None
        
        if strategy == ResolutionStrategy.PRIORITY:
            return self._resolve_by_priority(conflict, path_a, path_b)
        
        elif strategy == ResolutionStrategy.WAIT:
            return self._resolve_by_waiting(conflict, path_a, path_b)
        
        elif strategy == ResolutionStrategy.SLOW_DOWN:
            return self._resolve_by_slow_down(conflict, path_a, path_b)
        
        elif strategy == ResolutionStrategy.REROUTE:
            return self._resolve_by_reroute(conflict, path_a, path_b)
        
        else:
            # Default to priority
            return self._resolve_by_priority(conflict, path_a, path_b)
    
    def _resolve_by_priority(
        self, conflict: Conflict, path_a: RobotPath, path_b: RobotPath
    ) -> Resolution:
        """Resolve by priority - lower priority robot waits."""
        if path_a.priority > path_b.priority:
            robot_to_wait = path_b.robot_id
            wait_duration = conflict.time_window + 1.0
        elif path_b.priority > path_a.priority:
            robot_to_wait = path_a.robot_id
            wait_duration = conflict.time_window + 1.0
        else:
            # Equal priority - robot that arrives second waits
            if conflict.time_a < conflict.time_b:
                robot_to_wait = path_b.robot_id
                wait_duration = conflict.time_a - conflict.time_b + conflict.time_window + 1.0
            else:
                robot_to_wait = path_a.robot_id
                wait_duration = conflict.time_b - conflict.time_a + conflict.time_window + 1.0
        
        return Resolution(
            conflict_id=conflict.conflict_id,
            strategy=ResolutionStrategy.PRIORITY,
            robot_to_wait=robot_to_wait,
            wait_duration=max(0, wait_duration),
            reason=f"Priority-based: {robot_to_wait} has lower priority or arrives second",
        )
    
    def _resolve_by_waiting(
        self, conflict: Conflict, path_a: RobotPath, path_b: RobotPath
    ) -> Resolution:
        """Resolve by having robot A wait."""
        return Resolution(
            conflict_id=conflict.conflict_id,
            strategy=ResolutionStrategy.WAIT,
            robot_to_wait=path_a.robot_id,
            wait_duration=conflict.time_window + 2.0,
            reason=f"{path_a.robot_id} waits for {path_b.robot_id}",
        )
    
    def _resolve_by_slow_down(
        self, conflict: Conflict, path_a: RobotPath, path_b: RobotPath
    ) -> Resolution:
        """Resolve by slowing down one robot."""
        # Slow down robot with lower priority
        if path_a.priority <= path_b.priority:
            return Resolution(
                conflict_id=conflict.conflict_id,
                strategy=ResolutionStrategy.SLOW_DOWN,
                robot_to_wait=None,
                speed_reduction=0.5,
                reason=f"{path_a.robot_id} slows down to avoid collision",
            )
        else:
            return Resolution(
                conflict_id=conflict.conflict_id,
                strategy=ResolutionStrategy.SLOW_DOWN,
                robot_to_wait=None,
                speed_reduction=0.5,
                reason=f"{path_b.robot_id} slows down to avoid collision",
            )
    
    def _resolve_by_reroute(
        self, conflict: Conflict, path_a: RobotPath, path_b: RobotPath
    ) -> Resolution:
        """Resolve by rerouting (placeholder - would need path planning)."""
        # For now, just wait instead of reroute
        return self._resolve_by_priority(conflict, path_a, path_b)
    
    def get_resolution(self, conflict_id: str) -> Resolution | None:
        """Get resolution for a conflict."""
        return self.resolutions.get(conflict_id)


class FleetCoordinator:
    """Coordinates fleet with conflict detection and resolution."""
    
    def __init__(self):
        self.detector = ConflictDetector()
        self.resolver = ConflictResolver()
        self.paths: dict[str, RobotPath] = {}
        self.resolutions: list[Resolution] = []
    
    def update_paths(self, paths: list[RobotPath]):
        """Update robot paths."""
        self.paths = {p.robot_id: p for p in paths}
    
    async def check_and_resolve(self) -> dict[str, Any]:
        """Check for conflicts and resolve them."""
        # Detect conflicts
        paths_list = list(self.paths.values())
        conflicts = self.detector.detect_conflicts(paths_list)
        
        # Check for imminent collisions
        imminent = self.detector.check_imminent_collision(paths_list, horizon_sec=5.0)
        
        # Resolve conflicts
        if conflicts:
            self.resolutions = self.resolver.resolve_conflicts(
                conflicts, self.paths, ResolutionStrategy.PRIORITY
            )
        
        return {
            "conflicts_detected": len(conflicts),
            "imminent_collisions": len(imminent),
            "resolutions": len(self.resolutions),
            "safe_to_proceed": len(imminent) == 0,
        }
    
    def get_robot_instructions(self, robot_id: str) -> dict[str, Any] | None:
        """Get movement instructions for a robot based on resolutions."""
        for resolution in self.resolutions:
            if resolution.robot_to_wait == robot_id:
                return {
                    "action": "wait",
                    "duration": resolution.wait_duration,
                    "reason": resolution.reason,
                }
            
            if resolution.strategy == ResolutionStrategy.SLOW_DOWN:
                # Check if this robot needs to slow down
                return {
                    "action": "slow_down",
                    "speed_multiplier": resolution.speed_reduction,
                    "reason": resolution.reason,
                }
        
        return None
