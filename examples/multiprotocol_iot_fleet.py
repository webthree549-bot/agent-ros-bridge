"""
Example 2: Multi-Protocol IoT Fleet Management

This example demonstrates coordinating heterogeneous robots across
multiple communication protocols (WebSocket, gRPC, MQTT, TCP).

Key Features:
- Mixed protocols for different use cases
- Protocol-agnostic command dispatch
- Real-time performance optimization
- IoT integration at scale

Use Case: Smart factory with drones, AGVs, arms, and sensors
"""

from agent_ros_bridge import RobotAgent
from agent_ros_bridge.gateway_v2 import Bridge
from agent_ros_bridge.gateway_v2.config import BridgeConfig, TransportConfig
from dataclasses import dataclass
from typing import Dict, List, Any
import asyncio


@dataclass
class ProtocolConfig:
    """Configuration for a specific protocol."""
    name: str
    port: int
    use_case: str
    latency_target_ms: int


class MultiprotocolGateway:
    """
    Gateway supporting 4 protocols simultaneously.
    
    Protocols:
    - WebSocket (port 8765): Real-time, bidirectional
    - gRPC (port 50051): High-performance RPC
    - MQTT (port 1883): IoT messaging
    - TCP (port 9999): Raw socket streaming
    """
    
    PROTOCOLS = {
        'websocket': ProtocolConfig('WebSocket', 8765, 'Real-time video/control', 50),
        'grpc': ProtocolConfig('gRPC', 50051, 'High-performance RPC', 10),
        'mqtt': ProtocolConfig('MQTT', 1883, 'IoT pub/sub', 100),
        'tcp': ProtocolConfig('TCP', 9999, 'Raw data streaming', 5),
    }
    
    def __init__(self):
        self.config = BridgeConfig()
        self.gateway = Bridge(self.config)
        self.robots: Dict[str, dict] = {}  # robot_id -> {agent, protocol}
        
    def register_robot(self, robot_id: str, device_type: str, protocol: str):
        """
        Register robot with optimal protocol for its use case.
        
        Args:
            robot_id: Unique robot identifier
            device_type: Type of robot (drone, agv, arm, sensor)
            protocol: Communication protocol (websocket, grpc, mqtt, tcp)
        """
        if protocol not in self.PROTOCOLS:
            raise ValueError(f"Unknown protocol: {protocol}")
        
        # Create robot agent
        agent = RobotAgent(
            device_id=robot_id,
            device_type=device_type,
        )
        
        # Store with protocol info
        self.robots[robot_id] = {
            'agent': agent,
            'protocol': protocol,
            'config': self.PROTOCOLS[protocol],
        }
        
        print(f"✅ Registered {robot_id} ({device_type}) via {protocol}")
        return agent
    
    def dispatch_command(self, robot_id: str, command: str, **kwargs) -> dict:
        """
        Dispatch command to robot regardless of underlying protocol.
        
        The gateway automatically routes the command over the correct
        protocol based on robot registration.
        """
        if robot_id not in self.robots:
            raise ValueError(f"Robot {robot_id} not registered")
        
        robot_info = self.robots[robot_id]
        agent = robot_info['agent']
        protocol = robot_info['protocol']
        
        print(f"\n📡 Dispatching to {robot_id} via {protocol}")
        print(f"   Command: {command}")
        
        # Execute via appropriate protocol
        # (In production, this would route through the actual transport)
        result = agent.execute(command)
        
        return {
            'robot_id': robot_id,
            'protocol': protocol,
            'command': command,
            'success': result.success,
            'latency_ms': self._estimate_latency(protocol),
        }
    
    def _estimate_latency(self, protocol: str) -> int:
        """Estimate latency for protocol (simulated)."""
        latencies = {
            'websocket': 50,
            'grpc': 10,
            'mqtt': 100,
            'tcp': 5,
        }
        return latencies.get(protocol, 50)
    
    def get_protocol_stats(self) -> dict:
        """Get usage statistics per protocol."""
        stats = {p: {'count': 0, 'robots': []} for p in self.PROTOCOLS}
        
        for robot_id, info in self.robots.items():
            protocol = info['protocol']
            stats[protocol]['count'] += 1
            stats[protocol]['robots'].append(robot_id)
        
        return stats


class IoTFleetCoordinator:
    """
    Coordinates mixed IoT fleet with different protocols.
    
    Example Mission:
        1. Drone surveys area (WebSocket for video)
        2. AGV transports goods (MQTT for lightweight messaging)
        3. Arm packs items (gRPC for precise control)
        4. Sensors monitor environment (TCP for data streaming)
    """
    
    def __init__(self, gateway: MultiprotocolGateway):
        self.gateway = gateway
        
    def setup_mixed_fleet(self):
        """Setup fleet with different protocols."""
        print("\n🤖 Setting up mixed-protocol IoT fleet...")
        
        # Drone: WebSocket for real-time video + control
        self.gateway.register_robot(
            'surveillance_drone_01',
            'drone',
            'websocket',
        )
        
        # AGV: MQTT for IoT messaging
        self.gateway.register_robot(
            'delivery_agv_01',
            'mobile_robot',
            'mqtt',
        )
        
        # Robot Arm: gRPC for high-performance RPC
        self.gateway.register_robot(
            'packing_arm_01',
            'manipulator',
            'grpc',
        )
        
        # Sensors: TCP for raw data streaming
        self.gateway.register_robot(
            'env_sensors_01',
            'sensor_array',
            'tcp',
        )
        
        print("✅ Fleet setup complete!")
    
    def execute_coordinated_mission(self, mission_id: str):
        """
        Execute multi-step mission across different protocols.
        
        Each step uses the optimal protocol for that robot type.
        """
        print(f"\n🚀 Executing Mission: {mission_id}")
        print("=" * 60)
        
        mission_steps = [
            {
                'step': 1,
                'robot': 'surveillance_drone_01',
                'protocol': 'websocket',
                'task': 'Survey area A3 for obstacles',
            },
            {
                'step': 2,
                'robot': 'delivery_agv_01',
                'protocol': 'mqtt',
                'task': 'Transport package to area A3',
            },
            {
                'step': 3,
                'robot': 'packing_arm_01',
                'protocol': 'grpc',
                'task': 'Load package onto AGV',
            },
            {
                'step': 4,
                'robot': 'env_sensors_01',
                'protocol': 'tcp',
                'task': 'Monitor temperature during transport',
            },
        ]
        
        results = []
        for step in mission_steps:
            print(f"\n📋 Step {step['step']}: {step['task']}")
            print(f"   Robot: {step['robot']} ({step['protocol']})")
            
            result = self.gateway.dispatch_command(
                step['robot'],
                step['task'],
            )
            results.append(result)
            
            print(f"   ✅ Success: {result['success']}")
            print(f"   ⏱️  Latency: {result['latency_ms']}ms")
        
        return results
    
    def get_mission_report(self, results: List[dict]) -> dict:
        """Generate mission completion report."""
        total_steps = len(results)
        successful_steps = sum(1 for r in results if r['success'])
        avg_latency = sum(r['latency_ms'] for r in results) / total_steps
        
        protocols_used = {r['protocol'] for r in results}
        
        return {
            'total_steps': total_steps,
            'successful_steps': successful_steps,
            'success_rate': successful_steps / total_steps,
            'average_latency_ms': avg_latency,
            'protocols_used': list(protocols_used),
            'status': 'COMPLETE' if successful_steps == total_steps else 'PARTIAL',
        }


def main():
    """Demonstrate multi-protocol IoT fleet."""
    print("=" * 60)
    print("🌐 Multi-Protocol IoT Fleet Example")
    print("=" * 60)
    
    print("\n📡 Supported Protocols:")
    for name, config in MultiprotocolGateway.PROTOCOLS.items():
        print(f"   • {name.upper()}: {config.use_case} (target: {config.latency_target_ms}ms)")
    
    # Create gateway
    gateway = MultiprotocolGateway()
    
    # Setup fleet
    coordinator = IoTFleetCoordinator(gateway)
    coordinator.setup_mixed_fleet()
    
    # Show protocol distribution
    print("\n📊 Protocol Distribution:")
    stats = gateway.get_protocol_stats()
    for protocol, data in stats.items():
        if data['count'] > 0:
            print(f"   {protocol.upper()}: {data['count']} robots")
            for robot in data['robots']:
                print(f"      - {robot}")
    
    # Execute mission
    results = coordinator.execute_coordinated_mission('SURVEY_AND_TRANSPORT_001')
    
    # Generate report
    print("\n" + "=" * 60)
    print("📈 Mission Report")
    print("=" * 60)
    report = coordinator.get_mission_report(results)
    for key, value in report.items():
        print(f"   {key}: {value}")
    
    print("\n✅ Multi-protocol fleet example complete!")
    print("   Demonstrated WebSocket, gRPC, MQTT, and TCP coordination.")


if __name__ == '__main__':
    main()
