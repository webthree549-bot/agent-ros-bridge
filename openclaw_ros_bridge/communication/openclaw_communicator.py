#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""OpenClaw Communicator - TCP/IP wrapper for OpenClaw v1.x/v2.x"""
import socket
import json
import time
from typing import Callable, Dict, Any, Optional
from threading import Thread
from openclaw_ros_bridge.base.logger import get_logger
from openclaw_ros_bridge.version.version_manager import version_manager

logger = get_logger(__name__)

class OpenClawCommunicator:
    """OpenClaw TCP Communicator"""
    def __init__(self):
        self.tcp_config = version_manager.get_oc_tcp_config()
        self.mock_mode = version_manager.MOCK_MODE
        self._socket: Optional[socket.socket] = None
        self._connected = False
        self._recv_callback: Optional[Callable[[str], None]] = None
        self._heartbeat_thread: Optional[Thread] = None

    @property
    def is_connected(self) -> bool:
        return self.mock_mode or self._connected

    def connect(self) -> bool:
        if self.mock_mode:
            self._connected = True
            return True
        try:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._socket.connect((self.tcp_config["host"], self.tcp_config["port"]))
            self._connected = True
            logger.info(f"Connected to OpenClaw: {self.tcp_config['host']}:{self.tcp_config['port']}")
            return True
        except socket.error as e:
            logger.error(f"OpenClaw connection failed: {e}")
            return False

    def disconnect(self) -> None:
        self._connected = False
        if self._socket:
            self._socket.close()
            self._socket = None
        logger.info("Disconnected from OpenClaw")

    def send(self, data: Dict[str, Any]) -> bool:
        if not self.is_connected:
            return False
        if self.mock_mode:
            return True
        try:
            send_data = json.dumps(data) + "\n"
            self._socket.sendall(send_data.encode("utf-8"))
            return True
        except socket.error as e:
            logger.error(f"OpenClaw send failed: {e}")
            self._connected = False
            return False

    def set_recv_callback(self, callback: Callable[[str], None]) -> None:
        self._recv_callback = callback

openclaw_comm = OpenClawCommunicator()