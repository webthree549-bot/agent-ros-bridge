#!/usr/bin/env python3
"""HTTP Transport for OpenClaw Gateway with Advanced Visualization.

Features:
- Foxglove-like 3D visualization
- WebRTC video streaming
- Live plot/graph widgets
- Robot camera feed display
- Map/navigation view
- Real-time telemetry

Uses only Python standard library - no external dependencies.
"""

import asyncio
import json
import logging
import sys
from datetime import UTC, datetime
from typing import Any

# Add scripts to path for auth and rate limiting
sys.path.insert(0, '/workspace/scripts')

from agent_ros_bridge.gateway_v2.core import (
    Command,
    Event,
    Header,
    Identity,
    Message,
    Telemetry,
    Transport,
)

# Try to import optional features
try:
    from auth_manager import get_auth_manager
    AUTH_AVAILABLE = True
except ImportError:
    AUTH_AVAILABLE = False

try:
    from rate_limiter import get_rate_limiter
    RATE_LIMIT_AVAILABLE = True
except ImportError:
    RATE_LIMIT_AVAILABLE = False

logger = logging.getLogger("transport.http")


# Advanced dashboard with all features
DASHBOARD_HTML = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Agent ROS Bridge - Advanced Dashboard</title>
    
    <!-- Three.js for 3D -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
    
    <!-- Chart.js for plots -->
    <script src="https://cdn.jsdelivr.net/npm/chart.js@3.9.1/dist/chart.min.js"></script>
    
    <!-- Leaflet for maps -->
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
    <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
    
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            background: #0a0a0a;
            color: #e0e0e0;
            line-height: 1.6;
            overflow: hidden;
        }
        
        /* Grid Layout */
        .app-container {
            display: grid;
            grid-template-columns: 260px 1fr 320px;
            grid-template-rows: 50px 1fr 200px;
            height: 100vh;
            gap: 1px;
            background: #1a1a1a;
        }
        
        /* Header */
        .header {
            grid-column: 1 / -1;
            background: #141414;
            border-bottom: 2px solid #00ff88;
            display: flex;
            align-items: center;
            padding: 0 20px;
            justify-content: space-between;
        }
        .header h1 {
            color: #00ff88;
            font-size: 1.2em;
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .header-status {
            display: flex;
            gap: 15px;
            align-items: center;
        }
        .status-badge {
            padding: 4px 12px;
            border-radius: 12px;
            font-size: 0.75em;
            font-weight: 600;
        }
        .status-connected { background: #00ff88; color: #000; }
        .status-disconnected { background: #ff4444; color: #fff; }
        .btn {
            padding: 8px 16px;
            background: #00ff88;
            color: #000;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-weight: 600;
            font-size: 0.85em;
            transition: opacity 0.2s;
        }
        .btn:hover { opacity: 0.85; }
        .btn-secondary {
            background: #333;
            color: #00ff88;
            border: 1px solid #00ff88;
        }
        .btn:disabled { opacity: 0.5; cursor: not-allowed; }
        
        /* Sidebar - Topics & Controls */
        .sidebar-left {
            background: #141414;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }
        .panel-header {
            padding: 12px 15px;
            background: #1a1a1a;
            border-bottom: 1px solid #333;
            font-weight: 600;
            color: #00ff88;
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 0.9em;
        }
        .topic-list {
            flex: 1;
            overflow-y: auto;
            padding: 10px;
        }
        .topic-item {
            padding: 8px 10px;
            margin: 4px 0;
            background: #1a1a1a;
            border-radius: 6px;
            cursor: pointer;
            font-size: 0.8em;
            transition: all 0.2s;
            border: 1px solid transparent;
        }
        .topic-item:hover {
            background: #252525;
            border-color: #00ff88;
        }
        .topic-item.active {
            background: #00ff8822;
            border-color: #00ff88;
        }
        .topic-name { color: #fff; font-weight: 500; }
        .topic-type { color: #888; font-size: 0.75em; }
        
        /* Main Content Area with Tabs */
        .main-content {
            display: flex;
            flex-direction: column;
            background: #0a0a0a;
            overflow: hidden;
        }
        .tabs {
            display: flex;
            background: #141414;
            border-bottom: 1px solid #333;
        }
        .tab {
            padding: 10px 20px;
            cursor: pointer;
            border-bottom: 2px solid transparent;
            transition: all 0.2s;
            font-size: 0.85em;
        }
        .tab:hover { background: #1a1a1a; }
        .tab.active {
            border-bottom-color: #00ff88;
            color: #00ff88;
        }
        .tab-content {
            flex: 1;
            overflow: hidden;
            position: relative;
        }
        .tab-pane {
            display: none;
            width: 100%;
            height: 100%;
        }
        .tab-pane.active { display: block; }
        
        /* 3D View */
        #canvas-container {
            width: 100%;
            height: 100%;
            position: relative;
        }
        #render-canvas { width: 100%; height: 100%; }
        
        /* Camera Feed */
        .camera-container {
            width: 100%;
            height: 100%;
            display: flex;
            flex-direction: column;
            background: #000;
        }
        .camera-main {
            flex: 1;
            position: relative;
            background: #0a0a0a;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        #camera-video {
            max-width: 100%;
            max-height: 100%;
            border: 1px solid #333;
        }
        .camera-placeholder {
            text-align: center;
            color: #666;
        }
        .camera-thumbnails {
            display: flex;
            gap: 10px;
            padding: 10px;
            background: #141414;
            overflow-x: auto;
        }
        .camera-thumb {
            width: 120px;
            height: 80px;
            background: #1a1a1a;
            border: 2px solid #333;
            border-radius: 6px;
            cursor: pointer;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 0.75em;
            color: #888;
        }
        .camera-thumb.active {
            border-color: #00ff88;
        }
        
        /* Map View */
        #map-container {
            width: 100%;
            height: 100%;
            background: #1a1a1a;
        }
        #map {
            width: 100%;
            height: 100%;
        }
        
        /* Plots View */
        .plots-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: 1fr 1fr;
            gap: 10px;
            padding: 10px;
            height: 100%;
            overflow-y: auto;
        }
        .plot-card {
            background: #141414;
            border: 1px solid #333;
            border-radius: 8px;
            padding: 10px;
            display: flex;
            flex-direction: column;
        }
        .plot-title {
            color: #00ff88;
            font-size: 0.85em;
            margin-bottom: 8px;
            display: flex;
            justify-content: space-between;
        }
        .plot-canvas {
            flex: 1;
            position: relative;
        }
        
        /* Right Sidebar */
        .sidebar-right {
            background: #141414;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }
        .properties-panel {
            flex: 1;
            overflow-y: auto;
            padding: 12px;
        }
        .property-group {
            margin-bottom: 15px;
            background: #1a1a1a;
            border-radius: 8px;
            padding: 12px;
        }
        .property-group h3 {
            color: #00ff88;
            font-size: 0.8em;
            margin-bottom: 10px;
            padding-bottom: 5px;
            border-bottom: 1px solid #333;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }
        .property-row {
            display: flex;
            justify-content: space-between;
            padding: 5px 0;
            font-size: 0.8em;
            border-bottom: 1px solid #252525;
        }
        .property-row:last-child { border-bottom: none; }
        .property-label { color: #888; }
        .property-value { 
            color: #fff; 
            font-family: 'SF Mono', Monaco, monospace;
            font-weight: 500;
        }
        
        /* Bottom Panel - Console */
        .bottom-panel {
            grid-column: 1 / -1;
            background: #0a0a0a;
            border-top: 1px solid #333;
            display: flex;
            flex-direction: column;
        }
        .console-tabs {
            display: flex;
            background: #141414;
            border-bottom: 1px solid #333;
        }
        .console-tab {
            padding: 8px 16px;
            cursor: pointer;
            font-size: 0.8em;
            border-bottom: 2px solid transparent;
        }
        .console-tab.active {
            border-bottom-color: #00ff88;
            color: #00ff88;
        }
        .console-content {
            flex: 1;
            overflow: hidden;
            display: flex;
        }
        .console-pane {
            display: none;
            flex: 1;
            overflow-y: auto;
            padding: 10px;
            font-family: 'SF Mono', Monaco, monospace;
            font-size: 0.8em;
        }
        .console-pane.active { display: block; }
        .console-entry {
            padding: 3px 0;
            border-bottom: 1px solid #1a1a1a;
        }
        .console-time { color: #666; margin-right: 8px; }
        .console-info { color: #00ff88; }
        .console-warn { color: #ffaa00; }
        .console-error { color: #ff4444; }
        
        /* Stats Overlay */
        .stats-overlay {
            position: absolute;
            top: 10px;
            left: 10px;
            background: rgba(0, 0, 0, 0.85);
            padding: 12px 16px;
            border-radius: 8px;
            font-family: monospace;
            font-size: 0.75em;
            pointer-events: none;
            border: 1px solid #333;
        }
        .stat-row { display: flex; gap: 20px; margin: 3px 0; }
        .stat-label { color: #888; }
        .stat-value { color: #00ff88; font-weight: bold; }
        
        /* WebRTC Controls */
        .webrtc-controls {
            padding: 10px;
            background: #1a1a1a;
            display: flex;
            gap: 10px;
            align-items: center;
        }
        .stream-status {
            display: inline-flex;
            align-items: center;
            gap: 6px;
            padding: 4px 10px;
            border-radius: 12px;
            font-size: 0.75em;
            font-weight: 600;
        }
        .stream-active { background: #00ff8822; color: #00ff88; border: 1px solid #00ff88; }
        .stream-inactive { background: #ff444422; color: #ff4444; border: 1px solid #ff4444; }
    </style>
</head>
<body>
    <div class="app-container">
        <!-- Header -->
        <header class="header">
            <h1>🤖 Agent ROS Bridge</h1>
            <div class="header-status">
                <span id="ws-status" class="status-badge status-disconnected">● Disconnected</span>
                <span id="webrtc-status" class="stream-status stream-inactive">📹 Video: Off</span>
                <button class="btn" onclick="connectWebSocket()">Connect</button>
                <button class="btn btn-secondary" onclick="disconnectWebSocket()">Disconnect</button>
            </div>
        </header>
        
        <!-- Left Sidebar - Topics -->
        <aside class="sidebar-left">
            <div class="panel-header">
                <span>📡 Topics</span>
            </div>
            <div class="topic-list" id="topic-list">
                <div class="topic-item active" onclick="selectTopic('/tf')">
                    <div class="topic-name">/tf</div>
                    <div class="topic-type">TF transforms</div>
                </div>
                <div class="topic-item" onclick="selectTopic('/odom')">
                    <div class="topic-name">/odom</div>
                    <div class="topic-type">Odometry</div>
                </div>
                <div class="topic-item" onclick="selectTopic('/scan')">
                    <div class="topic-name">/scan</div>
                    <div class="topic-type">Laser scan</div>
                </div>
                <div class="topic-item" onclick="selectTopic('/camera/image_raw')">
                    <div class="topic-name">/camera/image_raw</div>
                    <div class="topic-type">Camera feed</div>
                </div>
                <div class="topic-item" onclick="selectTopic('/cmd_vel')">
                    <div class="topic-name">/cmd_vel</div>
                    <div class="topic-type">Velocity command</div>
                </div>
                <div class="topic-item" onclick="selectTopic('/map')">
                    <div class="topic-name">/map</div>
                    <div class="topic-type">Occupancy grid</div>
                </div>
            </div>
        </aside>
        
        <!-- Main Content - Tabbed Interface -->
        <main class="main-content">
            <div class="tabs">
                <div class="tab active" onclick="switchTab('3d')" id="tab-3d">🎮 3D View</div>
                <div class="tab" onclick="switchTab('camera')" id="tab-camera">📷 Camera</div>
                <div class="tab" onclick="switchTab('map')" id="tab-map">🗺️ Map</div>
                <div class="tab" onclick="switchTab('plots')" id="tab-plots">📊 Plots</div>
            </div>
            
            <div class="tab-content">
                <!-- 3D View Tab -->
                <div class="tab-pane active" id="pane-3d">
                    <div id="canvas-container">
                        <canvas id="render-canvas"></canvas>
                        <div class="stats-overlay">
                            <div class="stat-row">
                                <span class="stat-label">FPS:</span>
                                <span class="stat-value" id="fps-counter">60</span>
                            </div>
                            <div class="stat-row">
                                <span class="stat-label">Robots:</span>
                                <span class="stat-value" id="robot-count">1</span>
                            </div>
                            <div class="stat-row">
                                <span class="stat-label">Topics:</span>
                                <span class="stat-value" id="topic-count">0</span>
                            </div>
                        </div>
                    </div>
                </div>
                
                <!-- Camera Tab -->
                <div class="tab-pane" id="pane-camera">
                    <div class="camera-container">
                        <div class="webrtc-controls">
                            <button class="btn" onclick="startVideoStream()">▶️ Start Stream</button>
                            <button class="btn btn-secondary" onclick="stopVideoStream()">⏹️ Stop</button>
                            <select id="camera-select" class="btn btn-secondary">
                                <option value="front">Front Camera</option>
                                <option value="rear">Rear Camera</option>
                                <option value="depth">Depth Camera</option>
                            </select>
                        </div>
                        <div class="camera-main">
                            <video id="camera-video" autoplay playsinline muted style="display: none;"></video>
                            <canvas id="camera-canvas" style="max-width: 100%; max-height: 100%;"></canvas>
                            <div class="camera-placeholder" id="camera-placeholder">
                                <div>📷 Camera Feed</div>
                                <div style="font-size: 0.85em; margin-top: 8px;">Click "Start Stream" to begin</div>
                            </div>
                        </div>
                        <div class="camera-thumbnails">
                            <div class="camera-thumb active" onclick="switchCamera('front')">Front</div>
                            <div class="camera-thumb" onclick="switchCamera('rear')">Rear</div>
                            <div class="camera-thumb" onclick="switchCamera('depth')">Depth</div>
                        </div>
                    </div>
                </div>
                
                <!-- Map Tab -->
                <div class="tab-pane" id="pane-map">
                    <div id="map-container">
                        <div id="map"></div>
                    </div>
                </div>
                
                <!-- Plots Tab -->
                <div class="tab-pane" id="pane-plots">
                    <div class="plots-grid">
                        <div class="plot-card">
                            <div class="plot-title">
                                <span>Linear Velocity</span>
                                <span style="color: #00ff88;">● Live</span>
                            </div>
                            <div class="plot-canvas">
                                <canvas id="plot-velocity"></canvas>
                            </div>
                        </div>
                        <div class="plot-card">
                            <div class="plot-title">
                                <span>Angular Velocity</span>
                                <span style="color: #00ff88;">● Live</span>
                            </div>
                            <div class="plot-canvas">
                                <canvas id="plot-angular"></canvas>
                            </div>
                        </div>
                        <div class="plot-card">
                            <div class="plot-title">
                                <span>Position (X, Y)</span>
                                <span style="color: #00aa88;">● 2D Trajectory</span>
                            </div>
                            <div class="plot-canvas">
                                <canvas id="plot-position"></canvas>
                            </div>
                        </div>
                        <div class="plot-card">
                            <div class="plot-title">
                                <span>Laser Scan</span>
                                <span style="color: #ff8800;">● /scan</span>
                            </div>
                            <div class="plot-canvas">
                                <canvas id="plot-laser"></canvas>
                            </div>
                        </div>
                    </div>
                </div>
            </div>
        </main>
        
        <!-- Right Sidebar - Properties -->
        <aside class="sidebar-right">
            <div class="panel-header">
                <span>⚙️ Properties</span>
            </div>
            <div class="properties-panel">
                <div class="property-group">
                    <h3>📍 Position</h3>
                    <div class="property-row">
                        <span class="property-label">X</span>
                        <span class="property-value" id="pos-x">0.00 m</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">Y</span>
                        <span class="property-value" id="pos-y">0.00 m</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">Z</span>
                        <span class="property-value" id="pos-z">0.00 m</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">Yaw</span>
                        <span class="property-value" id="pos-yaw">0.00°</span>
                    </div>
                </div>
                
                <div class="property-group">
                    <h3>🚀 Velocity</h3>
                    <div class="property-row">
                        <span class="property-label">Linear X</span>
                        <span class="property-value" id="vel-linear">0.00 m/s</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">Angular Z</span>
                        <span class="property-value" id="vel-angular">0.00 rad/s</span>
                    </div>
                </div>
                
                <div class="property-group">
                    <h3>🛡️ Safety</h3>
                    <div class="property-row">
                        <span class="property-label">Mode</span>
                        <span class="property-value" style="color: #00ff88;">Safe</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">Shadow Hours</span>
                        <span class="property-value" id="shadow-hours">0.0</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">Agreement</span>
                        <span class="property-value" id="agreement">0%</span>
                    </div>
                </div>
                
                <div class="property-group">
                    <h3>📊 System</h3>
                    <div class="property-row">
                        <span class="property-label">Battery</span>
                        <span class="property-value" id="battery">100%</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">WiFi Signal</span>
                        <span class="property-value" id="wifi">-45 dBm</span>
                    </div>
                    <div class="property-row">
                        <span class="property-label">CPU Usage</span>
                        <span class="property-value" id="cpu">12%</span>
                    </div>
                </div>
            </div>
        </aside>
        
        <!-- Bottom Panel - Console -->
        <div class="bottom-panel">
            <div class="console-tabs">
                <div class="console-tab active" onclick="switchConsoleTab('logs')" id="console-tab-logs">📝 Logs</div>
                <div class="console-tab" onclick="switchConsoleTab('topics')" id="console-tab-topics">📡 Topic Data</div>
                <div class="console-tab" onclick="switchConsoleTab('webrtc')" id="console-tab-webrtc">📹 WebRTC</div>
            </div>
            <div class="console-content">
                <div class="console-pane active" id="console-pane-logs">
                    <div id="console-output"></div>
                </div>
                <div class="console-pane" id="console-pane-topics">
                    <div id="topic-output">Select a topic to view data...</div>
                </div>
                <div class="console-pane" id="console-pane-webrtc">
                    <div id="webrtc-output">WebRTC signaling channel ready...</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        // Global state
        let ws = null;
        let pc = null; // WebRTC peer connection
        let currentTab = '3d';
        let currentCamera = 'front';
        let isStreaming = false;
        
        // Three.js globals
        let scene, camera, renderer, controls;
        let robot;
        let animationId;
        let lastTime = performance.now();
        let frameCount = 0;
        
        // Chart.js globals
        let charts = {};
        
        // Map globals
        let map = null;
        let robotMarker = null;
        let pathLine = null;
        let pathCoords = [];
        
        // Data history for plots
        const maxDataPoints = 50;
        const plotData = {
            velocity: { labels: [], data: [] },
            angular: { labels: [], data: [] },
            positionX: { labels: [], data: [] },
            positionY: { labels: [], data: [] },
            laser: { labels: [], data: [] }
        };
        
        // ==================== Initialization ====================
        
        window.onload = function() {
            init3D();
            initCharts();
            initMap();
            log('Dashboard initialized');
            log('Click Connect to join gateway');
        };
        
        // ==================== 3D Visualization ====================
        
        function init3D() {
            const container = document.getElementById('canvas-container');
            const canvas = document.getElementById('render-canvas');
            
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x0a0a0a);
            
            camera = new THREE.PerspectiveCamera(75, container.clientWidth / container.clientHeight, 0.1, 1000);
            camera.position.set(5, 5, 5);
            camera.lookAt(0, 0, 0);
            
            renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
            renderer.setSize(container.clientWidth, container.clientHeight);
            renderer.setPixelRatio(window.devicePixelRatio);
            
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.05;
            
            // Grid and axes
            const gridHelper = new THREE.GridHelper(20, 20, 0x00ff88, 0x333333);
            scene.add(gridHelper);
            
            const axesHelper = new THREE.AxesHelper(2);
            scene.add(axesHelper);
            
            // Lighting
            const ambientLight = new THREE.AmbientLight(0x404040);
            scene.add(ambientLight);
            const directionalLight = new THREE.DirectionalLight(0xffffff, 1);
            directionalLight.position.set(5, 10, 7);
            scene.add(directionalLight);
            
            // Robot
            createRobot();
            
            window.addEventListener('resize', onWindowResize);
            animate3D();
        }
        
        function createRobot() {
            robot = new THREE.Group();
            
            // Base
            const baseGeo = new THREE.CylinderGeometry(0.15, 0.15, 0.05, 32);
            const baseMat = new THREE.MeshPhongMaterial({ color: 0x00ff88 });
            const base = new THREE.Mesh(baseGeo, baseMat);
            robot.add(base);
            
            // Wheels
            const wheelGeo = new THREE.CylinderGeometry(0.05, 0.05, 0.02, 16);
            const wheelMat = new THREE.MeshPhongMaterial({ color: 0x333333 });
            
            const leftWheel = new THREE.Mesh(wheelGeo, wheelMat);
            leftWheel.rotation.z = Math.PI / 2;
            leftWheel.position.set(0, 0, 0.12);
            robot.add(leftWheel);
            
            const rightWheel = new THREE.Mesh(wheelGeo, wheelMat);
            rightWheel.rotation.z = Math.PI / 2;
            rightWheel.position.set(0, 0, -0.12);
            robot.add(rightWheel);
            
            // Lidar
            const lidarGeo = new THREE.CylinderGeometry(0.03, 0.03, 0.08, 16);
            const lidarMat = new THREE.MeshPhongMaterial({ color: 0xff6600 });
            const lidar = new THREE.Mesh(lidarGeo, lidarMat);
            lidar.position.y = 0.06;
            robot.add(lidar);
            
            // Direction arrow
            const arrowGeo = new THREE.ConeGeometry(0.05, 0.1, 16);
            const arrowMat = new THREE.MeshPhongMaterial({ color: 0x00ff88 });
            const arrow = new THREE.Mesh(arrowGeo, arrowMat);
            arrow.rotation.x = -Math.PI / 2;
            arrow.position.x = 0.15;
            robot.add(arrow);
            
            scene.add(robot);
        }
        
        function onWindowResize() {
            const container = document.getElementById('canvas-container');
            if (container) {
                camera.aspect = container.clientWidth / container.clientHeight;
                camera.updateProjectionMatrix();
                renderer.setSize(container.clientWidth, container.clientHeight);
            }
        }
        
        function animate3D() {
            animationId = requestAnimationFrame(animate3D);
            
            const currentTime = performance.now();
            frameCount++;
            if (currentTime - lastTime >= 1000) {
                document.getElementById('fps-counter').textContent = frameCount;
                frameCount = 0;
                lastTime = currentTime;
            }
            
            controls.update();
            renderer.render(scene, camera);
        }
        
        // ==================== Charts ====================
        
        function initCharts() {
            const commonOptions = {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                plugins: {
                    legend: { display: false }
                },
                scales: {
                    x: { display: false },
                    y: {
                        grid: { color: '#333' },
                        ticks: { color: '#888', font: { size: 10 } }
                    }
                },
                elements: {
                    point: { radius: 0 },
                    line: { tension: 0.4, borderWidth: 2 }
                }
            };
            
            // Velocity chart
            charts.velocity = new Chart(document.getElementById('plot-velocity'), {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'Linear Velocity',
                        data: [],
                        borderColor: '#00ff88',
                        backgroundColor: 'rgba(0, 255, 136, 0.1)',
                        fill: true
                    }]
                },
                options: commonOptions
            });
            
            // Angular velocity chart
            charts.angular = new Chart(document.getElementById('plot-angular'), {
                type: 'line',
                data: {
                    labels: [],
                    datasets: [{
                        label: 'Angular Velocity',
                        data: [],
                        borderColor: '#ff8800',
                        backgroundColor: 'rgba(255, 136, 0, 0.1)',
                        fill: true
                    }]
                },
                options: commonOptions
            });
            
            // Position chart (2D trajectory)
            charts.position = new Chart(document.getElementById('plot-position'), {
                type: 'scatter',
                data: {
                    datasets: [{
                        label: 'Robot Position',
                        data: [],
                        borderColor: '#0088ff',
                        backgroundColor: '#0088ff',
                        pointRadius: 3
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    animation: false,
                    plugins: { legend: { display: false } },
                    scales: {
                        x: {
                            grid: { color: '#333' },
                            ticks: { color: '#888', font: { size: 10 } },
                            title: { display: true, text: 'X (m)', color: '#888' }
                        },
                        y: {
                            grid: { color: '#333' },
                            ticks: { color: '#888', font: { size: 10 } },
                            title: { display: true, text: 'Y (m)', color: '#888' }
                        }
                    }
                }
            });
            
            // Laser scan chart (polar-like line chart)
            charts.laser = new Chart(document.getElementById('plot-laser'), {
                type: 'line',
                data: {
                    labels: Array(360).fill(''),
                    datasets: [{
                        label: 'Distance',
                        data: Array(360).fill(0),
                        borderColor: '#ff4444',
                        backgroundColor: 'rgba(255, 68, 68, 0.1)',
                        fill: true,
                        pointRadius: 0
                    }]
                },
                options: commonOptions
            });
        }
        
        function updatePlots(data) {
            const now = new Date().toLocaleTimeString();
            
            // Update velocity plot
            if (data.linear !== undefined) {
                plotData.velocity.labels.push(now);
                plotData.velocity.data.push(data.linear.x || 0);
                if (plotData.velocity.labels.length > maxDataPoints) {
                    plotData.velocity.labels.shift();
                    plotData.velocity.data.shift();
                }
                charts.velocity.data.labels = plotData.velocity.labels;
                charts.velocity.data.datasets[0].data = plotData.velocity.data;
                charts.velocity.update('none');
            }
            
            // Update angular velocity plot
            if (data.angular !== undefined) {
                plotData.angular.labels.push(now);
                plotData.angular.data.push(data.angular.z || 0);
                if (plotData.angular.labels.length > maxDataPoints) {
                    plotData.angular.labels.shift();
                    plotData.angular.data.shift();
                }
                charts.angular.data.labels = plotData.angular.labels;
                charts.angular.data.datasets[0].data = plotData.angular.data;
                charts.angular.update('none');
            }
            
            // Update position plot (trajectory)
            if (data.position !== undefined) {
                charts.position.data.datasets[0].data.push({
                    x: data.position.x,
                    y: data.position.y
                });
                if (charts.position.data.datasets[0].data.length > maxDataPoints) {
                    charts.position.data.datasets[0].data.shift();
                }
                charts.position.update('none');
                
                // Update map trajectory
                if (map && robotMarker) {
                    const lat = 37.7749 + data.position.y * 0.0001;
                    const lng = -122.4194 + data.position.x * 0.0001;
                    robotMarker.setLatLng([lat, lng]);
                    pathCoords.push([lat, lng]);
                    if (pathLine) {
                        pathLine.setLatLngs(pathCoords);
                    }
                }
            }
            
            // Update laser scan
            if (data.laser !== undefined) {
                charts.laser.data.datasets[0].data = data.laser;
                charts.laser.update('none');
            }
        }
        
        // ==================== Map ====================
        
        function initMap() {
            // Initialize when map tab is first opened
        }
        
        function initMapWhenNeeded() {
            if (map) return;
            
            map = L.map('map').setView([37.7749, -122.4194], 18);
            
            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                attribution: '© OpenStreetMap contributors',
                maxZoom: 22
            }).addTo(map);
            
            // Robot marker
            const robotIcon = L.divIcon({
                className: 'robot-marker',
                html: '<div style="background: #00ff88; width: 16px; height: 16px; border-radius: 50%; border: 3px solid #fff; box-shadow: 0 0 10px rgba(0,255,136,0.5);"></div>',
                iconSize: [16, 16],
                iconAnchor: [8, 8]
            });
            
            robotMarker = L.marker([37.7749, -122.4194], { icon: robotIcon }).addTo(map);
            
            // Path line
            pathLine = L.polyline([], {
                color: '#00ff88',
                weight: 3,
                opacity: 0.8
            }).addTo(map);
        }
        
        // ==================== WebRTC Video ====================
        
        async function startVideoStream() {
            if (isStreaming) return;
            
            try {
                log('Starting WebRTC video stream...');
                
                // Create peer connection
                pc = new RTCPeerConnection({
                    iceServers: [{ urls: 'stun:stun.l.google.com:19302' }]
                });
                
                // Handle incoming stream
                pc.ontrack = (event) => {
                    const video = document.getElementById('camera-video');
                    video.srcObject = event.streams[0];
                    video.style.display = 'block';
                    document.getElementById('camera-placeholder').style.display = 'none';
                    document.getElementById('webrtc-status').textContent = '📹 Video: On';
                    document.getElementById('webrtc-status').className = 'stream-status stream-active';
                    log('Video stream received');
                };
                
                // Send offer to server via WebSocket
                const offer = await pc.createOffer();
                await pc.setLocalDescription(offer);
                
                if (ws && ws.readyState === WebSocket.OPEN) {
                    ws.send(JSON.stringify({
                        type: 'webrtc-offer',
                        sdp: offer.sdp,
                        camera: currentCamera
                    }));
                }
                
                isStreaming = true;
                
            } catch (err) {
                log('Failed to start video: ' + err.message, 'error');
            }
        }
        
        function stopVideoStream() {
            if (!isStreaming) return;
            
            if (pc) {
                pc.close();
                pc = null;
            }
            
            const video = document.getElementById('camera-video');
            video.srcObject = null;
            video.style.display = 'none';
            document.getElementById('camera-placeholder').style.display = 'block';
            document.getElementById('webrtc-status').textContent = '📹 Video: Off';
            document.getElementById('webrtc-status').className = 'stream-status stream-inactive';
            
            isStreaming = false;
            log('Video stream stopped');
        }
        
        function switchCamera(camera) {
            currentCamera = camera;
            document.querySelectorAll('.camera-thumb').forEach(thumb => {
                thumb.classList.remove('active');
            });
            event.target.classList.add('active');
            log('Switched to ' + camera + ' camera');
            
            if (isStreaming) {
                stopVideoStream();
                setTimeout(startVideoStream, 500);
            }
        }
        
        // ==================== WebSocket ====================
        
        function connectWebSocket() {
            if (ws && ws.readyState === WebSocket.OPEN) {
                log('Already connected');
                return;
            }
            
            // Try ROS2 bridge first (port 8766), fallback to gateway (port 8765)
            const bridgeUrl = 'ws://localhost:8766/';
            const gatewayUrl = 'ws://localhost:8765/';
            
            // Try bridge connection first for real-time ROS data
            tryConnectWebSocket(bridgeUrl, 'ROS Bridge');
        }
        
        function tryConnectWebSocket(url, name) {
            log('Connecting to ' + name + ' at ' + url + '...');
            log('Connecting to ' + url + '...');
            
            try {
                ws = new WebSocket(url);
                
                ws.onopen = () => {
                    updateStatus(true);
                    log('WebSocket connected');
                    subscribeTopic('/tf');
                    subscribeTopic('/odom');
                    document.getElementById('topic-count').textContent = '2';
                };
                
                ws.onmessage = (event) => {
                    const data = JSON.parse(event.data);
                    handleMessage(data);
                };
                
                ws.onerror = (err) => {
                    log('WebSocket error', 'error');
                };
                
                ws.onclose = () => {
                    updateStatus(false);
                    log('WebSocket disconnected', 'warn');
                    ws = null;
                    stopVideoStream();
                };
                
            } catch (e) {
                log('Connection failed: ' + e.message, 'error');
            }
        }
        
        function disconnectWebSocket() {
            if (ws) {
                ws.close();
            }
            stopVideoStream();
        }
        
        function subscribeTopic(topic) {
            if (!ws || ws.readyState !== WebSocket.OPEN) return;
            ws.send(JSON.stringify({ type: 'subscribe', topic: topic }));
            log('Subscribed to ' + topic);
        }
        
        function handleMessage(data) {
            // Handle ROS2 bridge telemetry format
            if (data.type === 'telemetry' && data.data) {
                const rosData = data.data;
                
                // Process TF data
                if (rosData.tf && rosData.tf.translation) {
                    const pos = rosData.tf.translation;
                    document.getElementById('pos-x').textContent = pos.x.toFixed(2) + ' m';
                    document.getElementById('pos-y').textContent = pos.y.toFixed(2) + ' m';
                    document.getElementById('pos-z').textContent = pos.z.toFixed(2) + ' m';
                    
                    if (robot) {
                        robot.position.x = pos.x;
                        robot.position.z = pos.y;
                        robot.position.y = pos.z;
                    }
                    
                    updatePlots({ position: pos });
                }
                
                // Process Odometry data
                if (rosData.odom && rosData.odom.twist) {
                    const twist = rosData.odom.twist;
                    document.getElementById('vel-linear').textContent = (twist.linear?.x || 0).toFixed(2) + ' m/s';
                    document.getElementById('vel-angular').textContent = (twist.angular?.z || 0).toFixed(2) + ' rad/s';
                    updatePlots({ linear: twist.linear, angular: twist.angular });
                }
                
                return;
            }
            
            // Handle legacy format (direct pose/twist)
            if (data.pose && data.pose.position) {
                document.getElementById('pos-x').textContent = data.pose.position.x.toFixed(2) + ' m';
                document.getElementById('pos-y').textContent = data.pose.position.y.toFixed(2) + ' m';
                document.getElementById('pos-z').textContent = data.pose.position.z.toFixed(2) + ' m';
                
                if (robot) {
                    robot.position.x = data.pose.position.x;
                    robot.position.z = data.pose.position.y;
                    robot.position.y = data.pose.position.z;
                }
                
                updatePlots({ position: data.pose.position });
            }
            
            if (data.twist) {
                document.getElementById('vel-linear').textContent = (data.twist.linear?.x || 0).toFixed(2) + ' m/s';
                document.getElementById('vel-angular').textContent = (data.twist.angular?.z || 0).toFixed(2) + ' rad/s';
                updatePlots({ linear: data.twist.linear, angular: data.twist.angular });
            }
            
            // Handle WebRTC signaling
            if (data.type === 'webrtc-answer' && pc) {
                pc.setRemoteDescription(new RTCSessionDescription({
                    type: 'answer',
                    sdp: data.sdp
                }));
            }
            
            if (data.type === 'webrtc-ice' && pc) {
                pc.addIceCandidate(new RTCIceCandidate(data.candidate));
            }
        }
        
        // ==================== UI Functions ====================
        
        function switchTab(tab) {
            currentTab = tab;
            
            document.querySelectorAll('.tab').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.tab-pane').forEach(p => p.classList.remove('active'));
            
            document.getElementById('tab-' + tab).classList.add('active');
            document.getElementById('pane-' + tab).classList.add('active');
            
            if (tab === 'map') {
                initMapWhenNeeded();
                setTimeout(() => map.invalidateSize(), 100);
            }
        }
        
        function switchConsoleTab(tab) {
            document.querySelectorAll('.console-tab').forEach(t => t.classList.remove('active'));
            document.querySelectorAll('.console-pane').forEach(p => p.classList.remove('active'));
            
            document.getElementById('console-tab-' + tab).classList.add('active');
            document.getElementById('console-pane-' + tab).classList.add('active');
        }
        
        function selectTopic(topic) {
            document.querySelectorAll('.topic-item').forEach(item => {
                item.classList.remove('active');
            });
            event.currentTarget.classList.add('active');
            
            document.getElementById('topic-output').textContent = 'Selected: ' + topic;
            subscribeTopic(topic);
        }
        
        function updateStatus(connected) {
            const badge = document.getElementById('ws-status');
            if (connected) {
                badge.textContent = '● Connected';
                badge.className = 'status-badge status-connected';
            } else {
                badge.textContent = '● Disconnected';
                badge.className = 'status-badge status-disconnected';
            }
        }
        
        function log(message, type = 'info') {
            const consoleEl = document.getElementById('console-output');
            const entry = document.createElement('div');
            entry.className = 'console-entry';
            const time = new Date().toLocaleTimeString();
            entry.innerHTML = `<span class="console-time">${time}</span><span class="console-${type}">${message}</span>`;
            consoleEl.appendChild(entry);
            consoleEl.scrollTop = consoleEl.scrollHeight;
        }
        
        window.onbeforeunload = () => {
            if (ws) ws.close();
            if (pc) pc.close();
            cancelAnimationFrame(animationId);
        };
    </script>
</body>
</html>'''


class HTTPTransport(Transport):
    """HTTP transport with advanced visualization dashboard.
    
    Features:
    - Foxglove-like 3D visualization
    - WebRTC video streaming support
    - Live plot/graph widgets
    - Robot camera feed display
    - Map/navigation view
    - Real-time telemetry
    
    Uses only Python standard library.
    """

    def __init__(self, config: dict[str, Any], name: str = "http"):
        """Initialize HTTP transport."""
        super().__init__(name, config)
        self.host = config.get("host", "0.0.0.0")
        self.port = config.get("port", 8080)
        self.dashboard_enabled = config.get("dashboard_enabled", True)
        self.server = None
        self._clients: dict[str, Any] = {}
        self._client_counter = 0

    async def start(self) -> bool:
        """Start HTTP server."""
        try:
            self.server = await asyncio.start_server(
                self._handle_request, self.host, self.port
            )
            self.running = True
            logger.info(f"HTTP transport started on http://{self.host}:{self.port}/")
            return True
        except Exception as e:
            logger.error(f"Failed to start HTTP transport: {e}")
            return False

    async def stop(self) -> None:
        """Stop HTTP server."""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
        self.running = False
        self._clients.clear()
        logger.info("HTTP transport stopped")

    async def send(self, message: Message, recipient: str) -> bool:
        """Send message to HTTP client."""
        if recipient in self._clients:
            if "pending" not in self._clients[recipient]:
                self._clients[recipient]["pending"] = []
            self._clients[recipient]["pending"].append(message)
            return True
        return False

    async def broadcast(self, message: Message) -> list[str]:
        """Broadcast to all HTTP clients."""
        recipients = []
        for client_id in self._clients:
            if await self.send(message, client_id):
                recipients.append(client_id)
        return recipients

    async def _handle_request(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        """Handle HTTP request."""
        try:
            request_line = await reader.readline()
            if not request_line:
                return
            
            method, path, _ = request_line.decode().strip().split(" ", 2)
            
            # Read headers
            headers = {}
            while True:
                line = await reader.readline()
                if line == b"\r\n":
                    break
                if b":" in line:
                    key, value = line.decode().strip().split(":", 1)
                    headers[key.strip().lower()] = value.strip()
            
            # Handle routes
            if path == "/" or path == "/index.html":
                await self._send_dashboard(writer)
            elif path == "/api/status":
                await self._send_json(writer, {
                    "status": "running",
                    "transport": "http",
                    "features": ["3d", "camera", "map", "plots", "webrtc"],
                    "clients": len(self._clients),
                    "timestamp": datetime.now(UTC).isoformat()
                })
            elif path == "/api/health":
                await self._send_json(writer, {"status": "healthy"})
            elif path == "/api/topics":
                await self._send_json(writer, {
                    "topics": [
                        {"name": "/tf", "type": "tf2_msgs/TFMessage"},
                        {"name": "/odom", "type": "nav_msgs/Odometry"},
                        {"name": "/scan", "type": "sensor_msgs/LaserScan"},
                        {"name": "/camera/image_raw", "type": "sensor_msgs/Image"},
                        {"name": "/cmd_vel", "type": "geometry_msgs/Twist"},
                        {"name": "/map", "type": "nav_msgs/OccupancyGrid"},
                    ]
                })
            elif path == "/api/cameras":
                await self._send_json(writer, {
                    "cameras": [
                        {"id": "front", "name": "Front Camera", "resolution": "640x480"},
                        {"id": "rear", "name": "Rear Camera", "resolution": "640x480"},
                        {"id": "depth", "name": "Depth Camera", "resolution": "640x480"},
                    ]
                })
            else:
                await self._send_error(writer, 404, "Not Found")
                
        except Exception as e:
            logger.error(f"HTTP request error: {e}")
            await self._send_error(writer, 500, "Internal Server Error")
        finally:
            writer.close()
            await writer.wait_closed()

    async def _send_dashboard(self, writer: asyncio.StreamWriter):
        """Send HTML dashboard."""
        body = DASHBOARD_HTML.encode("utf-8")
        headers = (
            f"HTTP/1.1 200 OK\r\n"
            f"Content-Type: text/html\r\n"
            f"Content-Length: {len(body)}\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        ).encode()
        writer.write(headers + body)
        await writer.drain()

    async def _send_json(self, writer: asyncio.StreamWriter, data: dict):
        """Send JSON response."""
        body = json.dumps(data).encode("utf-8")
        headers = (
            f"HTTP/1.1 200 OK\r\n"
            f"Content-Type: application/json\r\n"
            f"Content-Length: {len(body)}\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        ).encode()
        writer.write(headers + body)
        await writer.drain()

    async def _send_error(self, writer: asyncio.StreamWriter, code: int, message: str):
        """Send error response."""
        body = f"Error {code}: {message}".encode()
        headers = (
            f"HTTP/1.1 {code} {message}\r\n"
            f"Content-Type: text/plain\r\n"
            f"Content-Length: {len(body)}\r\n"
            f"Connection: close\r\n"
            f"\r\n"
        ).encode()
        writer.write(headers + body)
        await writer.drain()
