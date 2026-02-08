#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenClaw TCP Server - Allows OpenClaw on macOS to connect to ROS Bridge in Docker"""
import socket
import json
import threading
import time
from typing import Dict, Any, Optional, Callable
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager
from openclaw_ros_bridge.hal import sensor_hal, actuator_hal

logger = get_logger(__name__)

class OpenClawTCPServer:
    """TCP Server that accepts connections from OpenClaw AI Agent"""
    
    def __init__(self, host: str = "0.0.0.0", port: int = 9999):
        self.host = host
        self.port = port
        self.server_socket: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.command_handler: Optional[Callable[[Dict[str, Any]], Dict[str, Any]]] = None
    
    def start(self) -> bool:
        """Start TCP server to accept OpenClaw connections"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.running = True
            
            # Start listener thread
            self.thread = threading.Thread(target=self._accept_connections, daemon=True)
            self.thread.start()
            
            logger.info(f"OpenClaw TCP Server started on {self.host}:{self.port}")
            return True
        except socket.error as e:
            logger.error(f"Failed to start TCP server: {e}")
            return False
    
    def stop(self) -> None:
        """Stop TCP server"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        logger.info("OpenClaw TCP Server stopped")
    
    def _accept_connections(self) -> None:
        """Accept incoming connections"""
        while self.running:
            try:
                logger.info("Waiting for OpenClaw connection...")
                self.client_socket, addr = self.server_socket.accept()
                logger.info(f"OpenClaw connected from {addr}")
                self._handle_client()
            except socket.error as e:
                if self.running:
                    logger.error(f"Connection error: {e}")
                break
    
    def _handle_client(self) -> None:
        """Handle client communication"""
        buffer = ""
        while self.running and self.client_socket:
            try:
                data = self.client_socket.recv(4096).decode('utf-8')
                if not data:
                    logger.info("OpenClaw disconnected")
                    break
                
                buffer += data
                
                # Process complete messages (newline delimited)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line:
                        self._process_message(line)
                        
            except socket.error as e:
                logger.error(f"Client error: {e}")
                break
        
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
    
    def _process_message(self, message: str) -> None:
        """Process incoming message from OpenClaw"""
        try:
            cmd = json.loads(message)
            logger.debug(f"Received command: {cmd}")
            
            # Execute command
            response = self._execute_command(cmd)
            
            # Send response
            if self.client_socket:
                response_json = json.dumps(response) + '\n'
                self.client_socket.sendall(response_json.encode('utf-8'))
                
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON: {e}")
            self._send_error("Invalid JSON")
        except Exception as e:
            logger.error(f"Command error: {e}")
            self._send_error(str(e))
    
    def _execute_command(self, cmd: Dict[str, Any]) -> Dict[str, Any]:
        """Execute command from OpenClaw"""
        action = cmd.get('action')
        
        if action == 'read_sensor':
            sensor_type = cmd.get('sensor', 'env')
            data = sensor_hal.read(sensor_type)
            return {'status': 'ok', 'data': data}
        
        elif action == 'write_actuator':
            actuator = cmd.get('actuator')
            value = cmd.get('value')
            if actuator:
                actuator_hal.write({actuator: value})
                return {'status': 'ok', 'message': f'{actuator} set to {value}'}
            return {'status': 'error', 'message': 'Missing actuator'}
        
        elif action == 'get_status':
            return {
                'status': 'ok',
                'ros': version_manager.ROS_DISTRO,
                'mock': version_manager.MOCK_MODE,
                'openclaw_version': version_manager.OC_VER
            }
        
        elif action == 'ping':
            return {'status': 'ok', 'pong': True}
        
        else:
            return {'status': 'error', 'message': f'Unknown action: {action}'}
    
    def _send_error(self, message: str) -> None:
        """Send error response"""
        if self.client_socket:
            error_json = json.dumps({'status': 'error', 'message': message}) + '\n'
            try:
                self.client_socket.sendall(error_json.encode('utf-8'))
            except socket.error:
                pass

# Global server instance
openclaw_server = OpenClawTCPServer()

def main():
    """Main entry point for running TCP server standalone"""
    import signal
    import sys
    
    def signal_handler(sig, frame):
        logger.info("Shutting down...")
        openclaw_server.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Initialize HAL
    sensor_hal.init_hardware()
    actuator_hal.init_hardware()
    
    # Start server
    if openclaw_server.start():
        logger.info("Press Ctrl+C to stop")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
    else:
        logger.error("Failed to start server")
        sys.exit(1)

if __name__ == "__main__":
    main()
