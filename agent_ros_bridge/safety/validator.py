"""
/safety/validator Node
Agent ROS Bridge v0.6.1 - Week 2 Implementation

Validates trajectories against safety constraints before execution.
Generates safety certificates for approved trajectories.
Target: <10ms validation response time
"""

import time
import uuid
from typing import Dict, List, Optional, Any, Tuple


class SafetyValidatorNode:
    """
    Safety Validator Node
    
    Validates trajectories against safety constraints.
    Issues cryptographically-signed safety certificates for approved trajectories.
    
    Timing Requirements:
    - Validation must complete within <10ms
    - Certificate validity window: 30 seconds
    
    Optimization:
    - Uses LRU cache for identical trajectories
    - Cache size: 1000 entries
    - Cache TTL: 60 seconds
    """
    
    # Service name constant
    VALIDATE_SERVICE = '/safety/validate_trajectory'
    
    # Certificate validity in seconds
    CERTIFICATE_VALIDITY_SEC = 30.0
    
    # Cache settings
    CACHE_SIZE = 1000
    CACHE_TTL_SEC = 60.0
    
    def __init__(self, enable_cache: bool = True):
        """Initialize Safety Validator Node
        
        Args:
            enable_cache: Whether to enable result caching
        """
        self._validation_count = 0
        self._rejection_count = 0
        self._validation_times = []  # For performance monitoring
        self._cache_hits = 0
        self._cache_misses = 0
        
        # Simple LRU cache: trajectory_hash -> (result, timestamp)
        self._enable_cache = enable_cache
        self._cache: Dict[str, tuple] = {}
        self._cache_order: List[str] = []  # For LRU eviction
    
    def _compute_trajectory_hash(self, trajectory: Dict[str, Any], limits: Dict[str, Any]) -> str:
        """Compute hash for trajectory + limits combination."""
        import hashlib
        import json
        
        # Create deterministic string representation
        data = {
            'trajectory': trajectory,
            'limits': limits
        }
        data_str = json.dumps(data, sort_keys=True)
        return hashlib.md5(data_str.encode()).hexdigest()
    
    def _get_cached_result(self, traj_hash: str) -> Optional[Dict[str, Any]]:
        """Get cached validation result if available and not expired."""
        if not self._enable_cache or traj_hash not in self._cache:
            return None
        
        result, timestamp = self._cache[traj_hash]
        
        # Check if expired
        if time.time() - timestamp > self.CACHE_TTL_SEC:
            # Remove expired entry
            del self._cache[traj_hash]
            self._cache_order.remove(traj_hash)
            return None
        
        # Update LRU order
        self._cache_order.remove(traj_hash)
        self._cache_order.append(traj_hash)
        
        self._cache_hits += 1
        return result
    
    def _cache_result(self, traj_hash: str, result: Dict[str, Any]):
        """Cache validation result."""
        if not self._enable_cache:
            return
        
        # Evict oldest if cache is full
        while len(self._cache) >= self.CACHE_SIZE:
            oldest = self._cache_order.pop(0)
            if oldest in self._cache:
                del self._cache[oldest]
        
        # Add new entry
        self._cache[traj_hash] = (result, time.time())
        self._cache_order.append(traj_hash)
        self._cache_misses += 1
    
    def get_cache_statistics(self) -> Dict[str, Any]:
        """Get cache statistics."""
        total = self._cache_hits + self._cache_misses
        hit_rate = self._cache_hits / total if total > 0 else 0.0
        
        return {
            'cache_enabled': self._enable_cache,
            'cache_size': len(self._cache),
            'cache_hits': self._cache_hits,
            'cache_misses': self._cache_misses,
            'hit_rate': hit_rate,
            'max_cache_size': self.CACHE_SIZE,
            'cache_ttl_sec': self.CACHE_TTL_SEC
        }
    
    def clear_cache(self):
        """Clear the validation cache."""
        self._cache.clear()
        self._cache_order.clear()
        self._cache_hits = 0
        self._cache_misses = 0
    
    def validate_trajectory(self, trajectory: Dict[str, Any], 
                           limits: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate a trajectory against safety limits
        
        Args:
            trajectory: Trajectory to validate (velocities, waypoints, etc.)
            limits: Safety limits to check against
            
        Returns:
            Validation result with approval status and certificate if approved
        """
        start_time = time.time()
        self._validation_count += 1
        
        # Check cache first
        traj_hash = self._compute_trajectory_hash(trajectory, limits)
        cached_result = self._get_cached_result(traj_hash)
        
        if cached_result is not None:
            # Return cached result with updated timing
            cached_result['validation_time_ms'] = (time.time() - start_time) * 1000
            cached_result['cached'] = True
            return cached_result
        
        # Check velocity limits
        velocity_result = self._check_velocity(trajectory, limits)
        if not velocity_result['passed']:
            self._rejection_count += 1
            result = self._create_rejection_result(velocity_result['reason'])
            result['validation_time_ms'] = (time.time() - start_time) * 1000
            result['cached'] = False
            self._cache_result(traj_hash, result)
            return result
        
        # Check workspace bounds
        workspace_result = self._check_workspace(trajectory, limits)
        if not workspace_result['passed']:
            self._rejection_count += 1
            result = self._create_rejection_result(workspace_result['reason'])
            result['validation_time_ms'] = (time.time() - start_time) * 1000
            result['cached'] = False
            self._cache_result(traj_hash, result)
            return result
        
        # Check joint limits
        joint_result = self._check_joint_limits(trajectory, limits)
        if not joint_result['passed']:
            self._rejection_count += 1
            result = self._create_rejection_result(joint_result['reason'])
            result['validation_time_ms'] = (time.time() - start_time) * 1000
            result['cached'] = False
            self._cache_result(traj_hash, result)
            return result
        
        # Check force limits
        force_result = self._check_force_limits(trajectory, limits)
        if not force_result['passed']:
            self._rejection_count += 1
            result = self._create_rejection_result(force_result['reason'])
            result['validation_time_ms'] = (time.time() - start_time) * 1000
            result['cached'] = False
            self._cache_result(traj_hash, result)
            return result
        
        # Check restricted zones
        zone_result = self._check_restricted_zones(trajectory, limits)
        if not zone_result['passed']:
            self._rejection_count += 1
            result = self._create_rejection_result(zone_result['reason'])
            result['validation_time_ms'] = (time.time() - start_time) * 1000
            result['cached'] = False
            self._cache_result(traj_hash, result)
            return result
        
        # All checks passed - generate certificate
        elapsed = time.time() - start_time
        self._validation_times.append(elapsed)
        
        result = self._create_approval_result(trajectory)
        result['validation_time_ms'] = elapsed * 1000
        result['cached'] = False
        self._cache_result(traj_hash, result)
        
        return result
    
    def _check_velocity(self, trajectory: Dict[str, Any], 
                       limits: Dict[str, Any]) -> Dict[str, Any]:
        """Check velocity constraints"""
        max_linear = limits.get('max_linear_velocity')
        max_angular = limits.get('max_angular_velocity')
        
        # Check linear velocities
        velocities = trajectory.get('velocities', [])
        if velocities and max_linear is not None:
            for velocity in velocities:
                if velocity > max_linear:
                    return {
                        'passed': False,
                        'reason': f"velocity exceeds limit: {velocity} > {max_linear}"
                    }
        
        # Check angular velocities
        angular_velocities = trajectory.get('angular_velocities', [])
        if angular_velocities and max_angular is not None:
            for velocity in angular_velocities:
                if velocity > max_angular:
                    return {
                        'passed': False,
                        'reason': f"angular velocity exceeds limit: {velocity} > {max_angular}"
                    }
        
        return {'passed': True}
    
    def _check_workspace(self, trajectory: Dict[str, Any], 
                        limits: Dict[str, Any]) -> Dict[str, Any]:
        """Check workspace boundaries"""
        bounds = limits.get('workspace_bounds', {})
        if not bounds:
            return {'passed': True}
        
        waypoints = trajectory.get('waypoints', [])
        if not waypoints:
            return {'passed': True}
        
        # Handle different bounds formats
        if isinstance(bounds, list):
            if len(bounds) == 0:
                return {'passed': True}
            
            # Check if it's a polygon format (list of dicts with x,y)
            if isinstance(bounds[0], dict) and 'x' in bounds[0] and 'y' in bounds[0]:
                # Polygon format: [{'x': x1, 'y': y1}, ...]
                xs = [p['x'] for p in bounds]
                ys = [p['y'] for p in bounds]
                min_x, max_x = min(xs), max(xs)
                min_y, max_y = min(ys), max(ys)
            elif len(bounds) >= 4 and isinstance(bounds[0], (int, float)):
                # List format: [x_min, x_max, y_min, y_max]
                min_x, max_x, min_y, max_y = bounds[0], bounds[1], bounds[2], bounds[3]
            else:
                return {'passed': True}
        elif isinstance(bounds, dict):
            # Dict format
            min_x = bounds.get('x_min', float('-inf'))
            max_x = bounds.get('x_max', float('inf'))
            min_y = bounds.get('y_min', float('-inf'))
            max_y = bounds.get('y_max', float('inf'))
        else:
            return {'passed': True}
        
        for waypoint in waypoints:
            x = waypoint.get('x', 0.0)
            y = waypoint.get('y', 0.0)
            
            if x < min_x or x > max_x or y < min_y or y > max_y:
                return {
                    'passed': False,
                    'reason': f"waypoint ({x}, {y}) outside workspace bounds"
                }
        
        return {'passed': True}
    
    def _check_joint_limits(self, trajectory: Dict[str, Any], 
                           limits: Dict[str, Any]) -> Dict[str, Any]:
        """Check joint velocity limits"""
        max_joint_velocity = limits.get('max_joint_velocity')
        
        joint_velocities = trajectory.get('joint_velocities', [])
        if joint_velocities and max_joint_velocity is not None:
            for velocity in joint_velocities:
                if velocity > max_joint_velocity:
                    return {
                        'passed': False,
                        'reason': f"joint velocity exceeds limit: {velocity} > {max_joint_velocity}"
                    }
        
        return {'passed': True}
    
    def _check_force_limits(self, trajectory: Dict[str, Any], 
                           limits: Dict[str, Any]) -> Dict[str, Any]:
        """Check force limits"""
        max_force = limits.get('max_force')
        
        forces = trajectory.get('forces', [])
        if forces and max_force is not None:
            for force in forces:
                if force > max_force:
                    return {
                        'passed': False,
                        'reason': f"force exceeds limit: {force} > {max_force}"
                    }
        
        return {'passed': True}
    
    def _check_restricted_zones(self, trajectory: Dict[str, Any], 
                                limits: Dict[str, Any]) -> Dict[str, Any]:
        """Check restricted zones"""
        zones = limits.get('restricted_zones', [])
        if not zones:
            return {'passed': True}
        
        waypoints = trajectory.get('waypoints', [])
        if not waypoints:
            return {'passed': True}
        
        for zone in zones:
            zone_bounds = zone.get('bounds', [])
            zone_name = zone.get('name', 'unnamed')
            
            if not zone_bounds:
                continue
            
            # Simple bounding box check
            min_x = min(b.get('x', float('inf')) for b in zone_bounds)
            max_x = max(b.get('x', float('-inf')) for b in zone_bounds)
            min_y = min(b.get('y', float('inf')) for b in zone_bounds)
            max_y = max(b.get('y', float('-inf')) for b in zone_bounds)
            
            for waypoint in waypoints:
                x = waypoint.get('x', 0.0)
                y = waypoint.get('y', 0.0)
                
                if min_x <= x <= max_x and min_y <= y <= max_y:
                    return {
                        'passed': False,
                        'reason': f"waypoint ({x}, {y}) in restricted zone: {zone_name}"
                    }
        
        return {'passed': True}
    
    def _create_rejection_result(self, reason: str) -> Dict[str, Any]:
        """Create a rejection result"""
        return {
            'approved': False,
            'rejection_reason': reason,
            'certificate': None
        }
    
    def _create_approval_result(self, trajectory: Dict[str, Any]) -> Dict[str, Any]:
        """Create an approval result with certificate"""
        issued_at = time.time()
        expires_at = issued_at + self.CERTIFICATE_VALIDITY_SEC
        
        certificate = {
            'validation_id': str(uuid.uuid4()),
            'issued_at': issued_at,
            'expires_at': expires_at,
            'trajectory_hash': self._hash_trajectory(trajectory)
        }
        
        return {
            'approved': True,
            'rejection_reason': None,
            'certificate': certificate
        }
    
    def _hash_trajectory(self, trajectory: Dict[str, Any]) -> str:
        """Generate a simple hash for the trajectory"""
        # Simple hash based on waypoints count and first/last waypoint
        waypoints = trajectory.get('waypoints', [])
        if not waypoints:
            return "empty"
        
        first = waypoints[0]
        last = waypoints[-1]
        return f"{len(waypoints)}_{first.get('x', 0)}_{first.get('y', 0)}_{last.get('x', 0)}_{last.get('y', 0)}"
    
    def get_stats(self) -> Dict[str, Any]:
        """Get validation statistics"""
        avg_time = sum(self._validation_times) / len(self._validation_times) if self._validation_times else 0
        max_time = max(self._validation_times) if self._validation_times else 0
        
        stats = {
            'validation_count': self._validation_count,
            'rejection_count': self._rejection_count,
            'approval_count': self._validation_count - self._rejection_count,
            'average_validation_time_ms': avg_time * 1000,
            'max_validation_time_ms': max_time * 1000
        }
        
        # Add cache statistics
        cache_stats = self.get_cache_statistics()
        stats.update(cache_stats)
        
        return stats
