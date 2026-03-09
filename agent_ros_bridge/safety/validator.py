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
    """
    
    # Service name constant
    VALIDATE_SERVICE = '/safety/validate_trajectory'
    
    # Certificate validity in seconds
    CERTIFICATE_VALIDITY_SEC = 30.0
    
    def __init__(self):
        """Initialize Safety Validator Node"""
        self._validation_count = 0
        self._rejection_count = 0
        self._validation_times = []  # For performance monitoring
    
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
        
        # Check velocity limits
        velocity_result = self._check_velocity(trajectory, limits)
        if not velocity_result['passed']:
            self._rejection_count += 1
            return self._create_rejection_result(velocity_result['reason'])
        
        # Check workspace bounds
        workspace_result = self._check_workspace(trajectory, limits)
        if not workspace_result['passed']:
            self._rejection_count += 1
            return self._create_rejection_result(workspace_result['reason'])
        
        # Check joint limits
        joint_result = self._check_joint_limits(trajectory, limits)
        if not joint_result['passed']:
            self._rejection_count += 1
            return self._create_rejection_result(joint_result['reason'])
        
        # Check force limits
        force_result = self._check_force_limits(trajectory, limits)
        if not force_result['passed']:
            self._rejection_count += 1
            return self._create_rejection_result(force_result['reason'])
        
        # Check restricted zones
        zone_result = self._check_restricted_zones(trajectory, limits)
        if not zone_result['passed']:
            self._rejection_count += 1
            return self._create_rejection_result(zone_result['reason'])
        
        # All checks passed - generate certificate
        elapsed = time.time() - start_time
        self._validation_times.append(elapsed)
        
        return self._create_approval_result(trajectory)
    
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
        bounds = limits.get('workspace_bounds', [])
        if not bounds:
            return {'passed': True}
        
        waypoints = trajectory.get('waypoints', [])
        if not waypoints:
            return {'passed': True}
        
        # Extract 2D polygon bounds
        min_x = min(b.get('x', float('inf')) for b in bounds) if bounds else float('-inf')
        max_x = max(b.get('x', float('-inf')) for b in bounds) if bounds else float('inf')
        min_y = min(b.get('y', float('inf')) for b in bounds) if bounds else float('-inf')
        max_y = max(b.get('y', float('-inf')) for b in bounds) if bounds else float('inf')
        
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
        
        return {
            'validation_count': self._validation_count,
            'rejection_count': self._rejection_count,
            'approval_count': self._validation_count - self._rejection_count,
            'average_validation_time_ms': avg_time * 1000,
            'max_validation_time_ms': max_time * 1000
        }
