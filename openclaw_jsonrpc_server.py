#!/usr/bin/env python3
"""
JSON-RPC 2.0 Server for OpenClaw-ROS Bridge
Upgrades the existing TCP server to proper JSON-RPC protocol
"""
import socket
import json
import threading
import time
import uuid
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass, asdict
from enum import Enum

class ErrorCode(Enum):
    """JSON-RPC 2.0 error codes"""
    PARSE_ERROR = -32700
    INVALID_REQUEST = -32600
    METHOD_NOT_FOUND = -32601
    INVALID_PARAMS = -32602
    INTERNAL_ERROR = -32603
    SERVER_ERROR = -32000
    ROBOT_NOT_FOUND = -32001
    COMMAND_FAILED = -32002
    TIMEOUT = -32003

@dataclass
class JSONRPCRequest:
    """JSON-RPC 2.0 Request"""
    jsonrpc: str
    method: str
    id: Optional[str] = None
    params: Optional[Dict[str, Any]] = None
    
    @classmethod
    def from_dict(cls, data: dict) -> 'JSONRPCRequest':
        return cls(
            jsonrpc=data.get('jsonrpc', '2.0'),
            method=data.get('method', ''),
            id=data.get('id'),
            params=data.get('params', {})
        )

@dataclass
class JSONRPCResponse:
    """JSON-RPC 2.0 Response"""
    jsonrpc: str = "2.0"
    id: Optional[str] = None
    result: Optional[Dict[str, Any]] = None
    error: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> dict:
        result = {"jsonrpc": self.jsonrpc, "id": self.id}
        if self.error:
            result["error"] = self.error
        else:
            result["result"] = self.result
        return result

@dataclass
class RobotStatus:
    """Robot status information"""
    robot_id: str
    state: str
    connected: bool
    battery: float
    current_task: Optional[str] = None
    pose: Optional[Dict[str, float]] = None
    timestamp: float = 0.0
    
    def __post_init__(self):
        if self.timestamp == 0.0:
            self.timestamp = time.time()

class OpenClawJSONRPCServer:
    """
    JSON-RPC 2.0 compliant server for OpenClaw-ROS Bridge.
    
    Upgrades the existing TCP server with proper protocol support,
    batch requests, notifications, and error handling.
    """
    
    def __init__(self, host: str = "0.0.0.0", port: int = 9999):
        self.host = host
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # Method registry
        self.methods: Dict[str, Callable] = {
            # System methods
            "system.ping": self._ping,
            "system.status": self._system_status,
            "system.version": self._version,
            
            # Fleet methods
            "fleet.status": self._fleet_status,
            "fleet.deploy": self._fleet_deploy,
            "fleet.stop": self._fleet_stop,
            "fleet.list_robots": self._list_robots,
            
            # Robot control methods
            "robot.command": self._robot_command,
            "robot.query": self._robot_query,
            "robot.status": self._robot_status,
            
            # Sensor methods
            "sensor.read": self._sensor_read,
            "sensor.calibrate": self._sensor_calibrate,
            
            # Actuator methods
            "actuator.write": self._actuator_write,
            
            # Mission methods
            "mission.create": self._mission_create,
            "mission.start": self._mission_start,
            "mission.status": self._mission_status,
            "mission.cancel": self._mission_cancel,
        }
        
        # Simulated robot fleet (replace with actual ROS integration)
        self.robots: Dict[str, RobotStatus] = {
            "greenhouse": RobotStatus(
                robot_id="greenhouse",
                state="idle",
                connected=True,
                battery=85.0,
                pose=None
            )
        }
        self.missions: Dict[str, Dict] = {}
        
    def start(self) -> bool:
        """Start JSON-RPC server"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(5)
            self.running = True
            
            self.thread = threading.Thread(target=self._accept_connections, daemon=True)
            self.thread.start()
            
            print(f"[JSON-RPC] Server started on {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[JSON-RPC] Failed to start: {e}")
            return False
    
    def stop(self) -> None:
        """Stop server"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print("[JSON-RPC] Server stopped")
    
    def _accept_connections(self) -> None:
        """Accept incoming connections"""
        while self.running:
            try:
                self.client_socket, addr = self.server_socket.accept()
                print(f"[JSON-RPC] Client connected from {addr}")
                self._handle_client()
            except Exception as e:
                if self.running:
                    print(f"[JSON-RPC] Connection error: {e}")
    
    def _handle_client(self) -> None:
        """Handle client communication"""
        buffer = ""
        while self.running and self.client_socket:
            try:
                data = self.client_socket.recv(8192).decode('utf-8')
                if not data:
                    break
                
                buffer += data
                
                # Process complete messages (newline delimited)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self._process_message(line.strip())
                        
            except Exception as e:
                print(f"[JSON-RPC] Client error: {e}")
                break
        
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
            print("[JSON-RPC] Client disconnected")
    
    def _process_message(self, message: str) -> None:
        """Process JSON-RPC message"""
        try:
            # Parse JSON
            data = json.loads(message)
            
            # Handle batch requests (array)
            if isinstance(data, list):
                responses = []
                for item in data:
                    response = self._handle_single_request(item)
                    if response:  # Only add if not notification
                        responses.append(response)
                if responses:
                    self._send_response(responses)
            else:
                # Single request
                response = self._handle_single_request(data)
                if response:  # Only send if not notification
                    self._send_response(response)
                    
        except json.JSONDecodeError as e:
            self._send_error(None, ErrorCode.PARSE_ERROR, f"Parse error: {e}")
        except Exception as e:
            self._send_error(None, ErrorCode.INTERNAL_ERROR, f"Internal error: {e}")
    
    def _handle_single_request(self, data: dict) -> Optional[dict]:
        """Handle single JSON-RPC request"""
        try:
            # Validate request
            if data.get('jsonrpc') != '2.0':
                return self._create_error_response(
                    data.get('id'), 
                    ErrorCode.INVALID_REQUEST, 
                    "Invalid JSON-RPC version"
                )
            
            request = JSONRPCRequest.from_dict(data)
            
            # Check if notification (no id)
            is_notification = request.id is None
            
            # Validate method
            if not request.method:
                if not is_notification:
                    return self._create_error_response(
                        request.id,
                        ErrorCode.INVALID_REQUEST,
                        "Method not specified"
                    )
                return None
            
            # Execute method
            if request.method in self.methods:
                try:
                    result = self.methods[request.method](request.params or {})
                    if not is_notification:
                        return JSONRPCResponse(
                            id=request.id,
                            result=result
                        ).to_dict()
                except Exception as e:
                    if not is_notification:
                        return self._create_error_response(
                            request.id,
                            ErrorCode.INTERNAL_ERROR,
                            str(e)
                        )
            else:
                if not is_notification:
                    return self._create_error_response(
                        request.id,
                        ErrorCode.METHOD_NOT_FOUND,
                        f"Method '{request.method}' not found"
                    )
            
            return None  # Notification - no response
            
        except Exception as e:
            return self._create_error_response(
                data.get('id'),
                ErrorCode.INTERNAL_ERROR,
                str(e)
            )
    
    def _send_response(self, response: dict or list) -> None:
        """Send JSON-RPC response"""
        if self.client_socket:
            try:
                response_json = json.dumps(response) + '\n'
                self.client_socket.sendall(response_json.encode('utf-8'))
            except Exception as e:
                print(f"[JSON-RPC] Send error: {e}")
    
    def _send_error(self, id: Optional[str], code: ErrorCode, message: str) -> None:
        """Send error response"""
        response = self._create_error_response(id, code, message)
        self._send_response(response)
    
    def _create_error_response(self, id: Optional[str], code: ErrorCode, message: str) -> dict:
        """Create error response"""
        return JSONRPCResponse(
            id=id,
            error={
                "code": code.value,
                "message": message,
                "data": {"timestamp": time.time()}
            }
        ).to_dict()
    
    # ============== Method Implementations ==============
    
    def _ping(self, params: dict) -> dict:
        """System ping"""
        return {"pong": True, "timestamp": time.time()}
    
    def _system_status(self, params: dict) -> dict:
        """Get system status"""
        return {
            "status": "running",
            "robots_connected": len([r for r in self.robots.values() if r.connected]),
            "total_robots": len(self.robots),
            "timestamp": time.time()
        }
    
    def _version(self, params: dict) -> dict:
        """Get version info"""
        return {
            "jsonrpc": "2.0",
            "bridge_version": "2.0.0",
            "ros_version": "jazzy",
            "api_version": "1.0"
        }
    
    def _fleet_status(self, params: dict) -> dict:
        """Get fleet status"""
        return {
            "robots": [
                {
                    "id": r.robot_id,
                    "state": r.state,
                    "connected": r.connected,
                    "battery": r.battery,
                    "current_task": r.current_task
                }
                for r in self.robots.values()
            ]
        }
    
    def _fleet_deploy(self, params: dict) -> dict:
        """Deploy fleet"""
        # In real implementation, start ROS launch files
        return {"status": "deployed", "timestamp": time.time()}
    
    def _fleet_stop(self, params: dict) -> dict:
        """Stop fleet"""
        return {"status": "stopped", "timestamp": time.time()}
    
    def _list_robots(self, params: dict) -> dict:
        """List available robots"""
        return {"robots": list(self.robots.keys())}
    
    def _robot_command(self, params: dict) -> dict:
        """Send command to robot"""
        robot_id = params.get('robot_id', 'greenhouse')
        action = params.get('action')
        args = params.get('args', {})
        
        if robot_id not in self.robots:
            raise Exception(f"Robot '{robot_id}' not found")
        
        # Execute command (integrate with actual ROS here)
        return {
            "robot_id": robot_id,
            "action": action,
            "status": "executed",
            "result": self._execute_ros_command(robot_id, action, args),
            "timestamp": time.time()
        }
    
    def _robot_query(self, params: dict) -> dict:
        """Query robot state"""
        robot_id = params.get('robot_id', 'greenhouse')
        query = params.get('query')
        
        if robot_id not in self.robots:
            raise Exception(f"Robot '{robot_id}' not found")
        
        robot = self.robots[robot_id]
        return {
            "robot_id": robot_id,
            "state": robot.state,
            "battery": robot.battery,
            "connected": robot.connected,
            "query_result": self._execute_ros_query(robot_id, query)
        }
    
    def _robot_status(self, params: dict) -> dict:
        """Get robot status"""
        robot_id = params.get('robot_id', 'greenhouse')
        
        if robot_id not in self.robots:
            raise Exception(f"Robot '{robot_id}' not found")
        
        robot = self.robots[robot_id]
        return {
            "robot_id": robot.robot_id,
            "state": robot.state,
            "battery": robot.battery,
            "connected": robot.connected,
            "current_task": robot.current_task,
            "pose": robot.pose,
            "timestamp": robot.timestamp
        }
    
    def _sensor_read(self, params: dict) -> dict:
        """Read sensor data"""
        robot_id = params.get('robot_id', 'greenhouse')
        sensor = params.get('sensor', 'env')
        
        # Integrate with actual ROS sensor reading
        return {
            "robot_id": robot_id,
            "sensor": sensor,
            "data": {
                "temperature": 25.0,
                "humidity": 50.0,
                "timestamp": time.time()
            }
        }
    
    def _sensor_calibrate(self, params: dict) -> dict:
        """Calibrate sensor"""
        return {"status": "calibrated", "sensor": params.get('sensor')}
    
    def _actuator_write(self, params: dict) -> dict:
        """Write to actuator"""
        robot_id = params.get('robot_id', 'greenhouse')
        actuator = params.get('actuator')
        value = params.get('value')
        
        # Integrate with actual ROS actuator control
        return {
            "robot_id": robot_id,
            "actuator": actuator,
            "value": value,
            "status": "written",
            "timestamp": time.time()
        }
    
    def _mission_create(self, params: dict) -> dict:
        """Create mission"""
        mission_id = str(uuid.uuid4())[:8]
        mission = {
            "id": mission_id,
            "name": params.get('name', 'unnamed'),
            "steps": params.get('steps', []),
            "status": "created",
            "created_at": time.time()
        }
        self.missions[mission_id] = mission
        return {"mission_id": mission_id, "status": "created"}
    
    def _mission_start(self, params: dict) -> dict:
        """Start mission"""
        mission_id = params.get('mission_id')
        if mission_id not in self.missions:
            raise Exception(f"Mission '{mission_id}' not found")
        
        self.missions[mission_id]['status'] = 'running'
        return {"mission_id": mission_id, "status": "started"}
    
    def _mission_status(self, params: dict) -> dict:
        """Get mission status"""
        mission_id = params.get('mission_id')
        if mission_id not in self.missions:
            raise Exception(f"Mission '{mission_id}' not found")
        
        return self.missions[mission_id]
    
    def _mission_cancel(self, params: dict) -> dict:
        """Cancel mission"""
        mission_id = params.get('mission_id')
        if mission_id in self.missions:
            self.missions[mission_id]['status'] = 'cancelled'
            return {"mission_id": mission_id, "status": "cancelled"}
        raise Exception(f"Mission '{mission_id}' not found")
    
    # ============== ROS Integration Placeholders ==============
    
    def _execute_ros_command(self, robot_id: str, action: str, args: dict) -> dict:
        """Execute command via ROS (placeholder)"""
        # TODO: Integrate with actual ROS topics/services
        return {"executed": True, "action": action, "args": args}
    
    def _execute_ros_query(self, robot_id: str, query: str) -> dict:
        """Execute query via ROS (placeholder)"""
        # TODO: Integrate with actual ROS topics
        return {"query": query, "result": "placeholder"}

def main():
    """Run JSON-RPC server"""
    import signal
    
    server = OpenClawJSONRPCServer()
    
    def signal_handler(sig, frame):
        print("\n[JSON-RPC] Shutting down...")
        server.stop()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if server.start():
        print("[JSON-RPC] Press Ctrl+C to stop")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    else:
        print("[JSON-RPC] Failed to start server")

if __name__ == "__main__":
    main()
