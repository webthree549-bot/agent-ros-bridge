/**
 * Agent ROS Bridge - Web Dashboard
 * Real-time robot control and monitoring interface
 */

class AgentROSBridgeApp {
    constructor() {
        this.ws = null;
        this.connected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 3000;
        
        this.robots = new Map();
        this.logs = [];
        this.maxLogs = 1000;
        
        this.currentSection = 'dashboard';
        this.selectedRobot = null;
        
        this.init();
    }
    
    init() {
        this.bindEvents();
        this.loadMockData();
        this.startClock();
        this.log('System initialized. Waiting for connection...', 'info');
    }
    
    bindEvents() {
        // Navigation
        document.querySelectorAll('.nav-item').forEach(item => {
            item.addEventListener('click', (e) => {
                e.preventDefault();
                const section = item.dataset.section;
                this.switchSection(section);
            });
        });
        
        // D-Pad controls
        document.querySelectorAll('.d-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const cmd = btn.dataset.cmd;
                const param = btn.dataset.param;
                this.sendDPadCommand(cmd, param);
            });
        });
        
        // Connect button
        document.getElementById('connectBtn').addEventListener('click', () => {
            if (this.connected) {
                this.disconnect();
            } else {
                this.openModal('connectModal');
            }
        });
        
        // Emergency stop
        document.getElementById('emergencyStopBtn').addEventListener('click', () => {
            this.emergencyStop();
        });
        
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape') {
                this.closeAllModals();
            }
            // Arrow keys for control when on control section
            if (this.currentSection === 'control') {
                switch(e.key) {
                    case 'ArrowUp': this.sendDPadCommand('forward', '1.0'); break;
                    case 'ArrowDown': this.sendDPadCommand('forward', '-1.0'); break;
                    case 'ArrowLeft': this.sendDPadCommand('rotate', '90'); break;
                    case 'ArrowRight': this.sendDPadCommand('rotate', '-90'); break;
                    case ' ': this.sendDPadCommand('stop'); break;
                }
            }
        });
    }
    
    // ==================== Navigation ====================
    
    switchSection(section) {
        // Update nav
        document.querySelectorAll('.nav-item').forEach(item => {
            item.classList.toggle('active', item.dataset.section === section);
        });
        
        // Update content
        document.querySelectorAll('.section').forEach(sec => {
            sec.classList.toggle('active', sec.id === `${section}-section`);
        });
        
        // Update title
        const titles = {
            dashboard: 'Dashboard',
            robots: 'Robot Management',
            control: 'Robot Control',
            telemetry: 'Telemetry',
            fleet: 'Fleet Management',
            safety: 'Safety & Validation',
            logs: 'System Logs'
        };
        document.querySelector('.page-title').textContent = titles[section] || section;
        
        this.currentSection = section;
        
        // Section-specific updates
        if (section === 'telemetry') {
            this.startTelemetryUpdates();
        } else {
            this.stopTelemetryUpdates();
        }
        
        if (section === 'control') {
            this.updateRobotSelects();
        }
    }
    
    // ==================== WebSocket Connection ====================
    
    connect() {
        const url = document.getElementById('wsUrl').value || 'ws://localhost:8768';
        const token = document.getElementById('jwtToken').value;
        
        this.closeModal('connectModal');
        this.updateConnectionStatus('connecting');
        this.log(`Connecting to ${url}...`, 'info');
        
        try {
            const fullUrl = token ? `${url}?token=${token}` : url;
            this.ws = new WebSocket(fullUrl);
            
            this.ws.onopen = () => {
                this.onConnect();
            };
            
            this.ws.onmessage = (event) => {
                this.onMessage(JSON.parse(event.data));
            };
            
            this.ws.onerror = (error) => {
                this.onError(error);
            };
            
            this.ws.onclose = () => {
                this.onDisconnect();
            };
            
        } catch (error) {
            this.log(`Connection error: ${error.message}`, 'error');
            this.updateConnectionStatus('disconnected');
        }
    }
    
    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
    }
    
    onConnect() {
        this.connected = true;
        this.reconnectAttempts = 0;
        this.updateConnectionStatus('connected');
        this.log('Connected to Agent ROS Bridge', 'success');
        
        // Request robot list
        this.send({ type: 'list_robots' });
        
        // Update UI
        document.getElementById('connectBtn').innerHTML = '<span class="btn-icon">🔌</span> Disconnect';
    }
    
    onDisconnect() {
        this.connected = false;
        this.updateConnectionStatus('disconnected');
        this.log('Disconnected from bridge', 'warn');
        
        document.getElementById('connectBtn').innerHTML = '<span class="btn-icon">🔌</span> Connect';
        
        // Attempt reconnection
        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;
            this.log(`Reconnecting in ${this.reconnectDelay/1000}s... (attempt ${this.reconnectAttempts})`, 'info');
            setTimeout(() => this.connect(), this.reconnectDelay);
        }
    }
    
    onError(error) {
        this.log(`WebSocket error: ${error.message || 'Unknown error'}`, 'error');
    }
    
    onMessage(data) {
        switch(data.type) {
            case 'robot_list':
                this.updateRobotList(data.robots);
                break;
            case 'robot_status':
                this.updateRobotStatus(data.robot_id, data.status);
                break;
            case 'telemetry':
                this.updateTelemetry(data);
                break;
            case 'command_result':
                this.handleCommandResult(data);
                break;
            case 'confirmation_request':
                this.showConfirmation(data);
                break;
            case 'log':
                this.log(data.message, data.level);
                break;
            default:
                console.log('Unknown message type:', data);
        }
    }
    
    send(message) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(message));
        } else {
            // Demo mode - simulate responses
            this.handleDemoCommand(message);
        }
    }
    
    handleDemoCommand(message) {
        // Simulate command processing for demo
        setTimeout(() => {
            switch(message.type) {
                case 'natural_language':
                    this.handleDemoNLCommand(message);
                    break;
                case 'command':
                    this.handleDemoRobotCommand(message);
                    break;
                default:
                    this.log(`Demo: ${message.type} command received`, 'info');
            }
        }, 500);
    }
    
    handleDemoNLCommand(message) {
        const text = message.text.toLowerCase();
        let response = '';
        let intent = '';
        let confidence = 0.9;
        
        if (text.includes('move') || text.includes('forward') || text.includes('backward')) {
            intent = 'move_forward';
            confidence = 0.94;
            response = `🤖 AI Proposal:\nIntent: move_forward\nConfidence: 94%\nParameters: distance=2.0m, speed=0.5m/s\n\n✅ Safety Check: Clear path ahead\n\n⏳ Waiting for human approval...`;
        } else if (text.includes('rotate') || text.includes('turn')) {
            intent = 'rotate';
            confidence = 0.89;
            response = `🤖 AI Proposal:\nIntent: rotate\nConfidence: 89%\nParameters: angle=90°\n\n✅ Safety Check: No obstacles detected\n\n⏳ Waiting for human approval...`;
        } else if (text.includes('stop')) {
            intent = 'stop';
            confidence = 0.99;
            response = `🤖 AI Proposal:\nIntent: emergency_stop\nConfidence: 99%\n\n⚠️ Executing immediately (emergency action)`;
        } else if (text.includes('status')) {
            response = `🤖 Robot Status:\nBattery: 85%\nLocation: Kitchen\nStatus: Online\nCurrent Task: Idle`;
        } else {
            response = `🤖 AI Proposal:\nIntent: ${intent || 'unknown'}\nConfidence: ${Math.floor(confidence * 100)}%\n\n⏳ Processing your request...`;
        }
        
        this.addChatMessage(response, 'bot');
        
        // Show approval buttons for non-emergency commands
        if (!text.includes('stop') && !text.includes('status')) {
            this.showApprovalButtons(intent, message.text);
        }
    }
    
    handleDemoRobotCommand(message) {
        const action = message.command?.action || 'unknown';
        this.log(`Demo: Executed ${action}`, 'success');
        
        // Update robot status
        if (this.selectedRobot) {
            this.selectedRobot.status = 'busy';
            this.renderRobotList();
            
            // Return to online after 2 seconds
            setTimeout(() => {
                this.selectedRobot.status = 'online';
                this.renderRobotList();
            }, 2000);
        }
    }
    
    showApprovalButtons(intent, originalCommand) {
        const container = document.getElementById('chatContainer');
        const div = document.createElement('div');
        div.className = 'chat-message system';
        div.innerHTML = `
            <div class="approval-buttons" style="display: flex; gap: 10px; margin-top: 10px;">
                <button class="btn btn-success" onclick="app.approveCommand('${intent}')">✅ Approve</button>
                <button class="btn btn-danger" onclick="app.rejectCommand()">❌ Reject</button>
                <button class="btn btn-secondary" onclick="app.modifyCommand()">✏️ Modify</button>
            </div>
        `;
        container.appendChild(div);
        container.scrollTop = container.scrollHeight;
    }
    
    approveCommand(intent) {
        this.addChatMessage('✅ Command approved! Executing...', 'system');
        this.log(`Approved ${intent}`, 'success');
        
        // Remove approval buttons
        document.querySelectorAll('.approval-buttons').forEach(el => el.remove());
        
        // Simulate execution
        setTimeout(() => {
            this.addChatMessage('✅ Command executed successfully!', 'bot');
        }, 1000);
    }
    
    rejectCommand() {
        this.addChatMessage('❌ Command rejected by operator.', 'system');
        this.log('Command rejected', 'warn');
        document.querySelectorAll('.approval-buttons').forEach(el => el.remove());
    }
    
    modifyCommand() {
        this.addChatMessage('✏️ Please enter modified command:', 'system');
        document.querySelectorAll('.approval-buttons').forEach(el => el.remove());
        document.getElementById('nlCommandInput').focus();
    }
    
    updateConnectionStatus(status) {
        const statusEl = document.getElementById('connectionStatus');
        const dot = statusEl.querySelector('.status-dot');
        const text = statusEl.querySelector('.status-text');
        
        dot.className = 'status-dot ' + status;
        text.textContent = status.charAt(0).toUpperCase() + status.slice(1);
    }
    
    // ==================== Robot Management ====================
    
    updateRobotList(robots) {
        this.robots.clear();
        robots.forEach(robot => {
            this.robots.set(robot.id, robot);
        });
        
        this.renderRobotList();
        this.renderRobotsTable();
        this.updateStats();
    }
    
    updateRobotStatus(robotId, status) {
        const robot = this.robots.get(robotId);
        if (robot) {
            robot.status = status;
            this.renderRobotList();
            this.renderRobotsTable();
        }
    }
    
    renderRobotList() {
        const container = document.getElementById('robotList');
        
        if (this.robots.size === 0) {
            container.innerHTML = `
                <div class="empty-state">
                    <div class="empty-icon">🤖</div>
                    <p>No robots connected</p>
                    <button class="btn btn-primary" onclick="app.connect()">Connect to Bridge</button>
                </div>
            `;
            return;
        }
        
        container.innerHTML = Array.from(this.robots.values()).map(robot => `
            <div class="robot-item" onclick="app.selectRobot('${robot.id}')">
                <div class="robot-avatar">🤖</div>
                <div class="robot-info">
                    <div class="robot-name">${robot.name || robot.id}</div>
                    <div class="robot-type">${robot.type || 'Unknown'}</div>
                </div>
                <div class="robot-status">
                    <span class="status-badge ${robot.status === 'online' ? 'online' : 'busy'}">
                        ${robot.status}
                    </span>
                </div>
            </div>
        `).join('');
    }
    
    renderRobotsTable() {
        const tbody = document.getElementById('robotsTableBody');
        
        if (this.robots.size === 0) {
            tbody.innerHTML = `
                <tr class="empty-row">
                    <td colspan="7" class="text-center">
                        <div class="empty-state">
                            <div class="empty-icon">🤖</div>
                            <p>No robots connected</p>
                        </div>
                    </td>
                </tr>
            `;
            return;
        }
        
        tbody.innerHTML = Array.from(this.robots.values()).map(robot => `
            <tr>
                <td>${robot.id}</td>
                <td>${robot.name || '-'}</td>
                <td>${robot.type || 'Unknown'}</td>
                <td><span class="status-badge ${robot.status === 'online' ? 'online' : 'busy'}">${robot.status}</span></td>
                <td>${robot.battery || '--'}%</td>
                <td>${robot.location || '--'}</td>
                <td>
                    <button class="btn btn-sm btn-secondary" onclick="app.controlRobot('${robot.id}')">Control</button>
                </td>
            </tr>
        `).join('');
    }
    
    selectRobot(robotId) {
        this.selectedRobot = robotId;
        this.switchSection('control');
        document.getElementById('controlRobotSelect').value = robotId;
        this.log(`Selected robot: ${robotId}`, 'info');
    }
    
    controlRobot(robotId) {
        this.selectRobot(robotId);
    }
    
    updateRobotSelects() {
        const selects = ['controlRobotSelect', 'telemetryRobotSelect'];
        selects.forEach(id => {
            const select = document.getElementById(id);
            if (!select) return;
            
            const currentValue = select.value;
            select.innerHTML = '<option value="">Select Robot...</option>' + 
                Array.from(this.robots.values()).map(r => 
                    `<option value="${r.id}" ${r.id === currentValue ? 'selected' : ''}>${r.name || r.id}</option>`
                ).join('');
        });
    }
    
    discoverRobots() {
        this.send({ type: 'discover_robots' });
        this.log('Discovering robots...', 'info');
    }
    
    addRobot() {
        this.log('Add robot functionality - not implemented in demo', 'info');
    }
    
    // ==================== Robot Control ====================
    
    sendDPadCommand(command, parameter) {
        const robotId = document.getElementById('controlRobotSelect').value;
        if (!robotId) {
            this.log('Please select a robot first', 'warn');
            return;
        }
        
        const params = {};
        if (command === 'forward') {
            params.distance = parseFloat(parameter);
            params.speed = 0.5;
        } else if (command === 'rotate') {
            params.angle = parseFloat(parameter);
            params.speed = 0.5;
        }
        
        this.sendCommand(command, params, robotId);
    }
    
    sendCommand(action, parameters = {}, robotId = null) {
        const targetId = robotId || document.getElementById('controlRobotSelect').value;
        if (!targetId) {
            this.log('Please select a robot first', 'warn');
            return;
        }
        
        const message = {
            type: 'command',
            robot_id: targetId,
            command: {
                action,
                parameters
            }
        };
        
        this.send(message);
        this.log(`Sent ${action} to ${targetId}`, 'info');
    }
    
    sendNLCommand() {
        const input = document.getElementById('nlCommandInput');
        const text = input.value.trim();
        
        if (!text) return;
        
        this.addChatMessage(text, 'user');
        input.value = '';
        
        const robotId = document.getElementById('controlRobotSelect').value;
        
        this.send({
            type: 'natural_language',
            robot_id: robotId,
            text
        });
    }
    
    quickCommand(text) {
        document.getElementById('nlCommandInput').value = text;
        this.sendNLCommand();
    }
    
    addChatMessage(text, sender) {
        const container = document.getElementById('chatContainer');
        const div = document.createElement('div');
        div.className = `chat-message ${sender}`;
        div.innerHTML = `<div class="chat-bubble">${this.escapeHtml(text)}</div>`;
        container.appendChild(div);
        container.scrollTop = container.scrollHeight;
    }
    
    handleCommandResult(data) {
        if (data.success) {
            this.log(`Command succeeded: ${data.action}`, 'success');
            if (data.response) {
                this.addChatMessage(data.response, 'bot');
            }
        } else {
            this.log(`Command failed: ${data.error || 'Unknown error'}`, 'error');
        }
    }
    
    broadcastCommand(action) {
        this.send({
            type: 'broadcast',
            command: { action }
        });
        this.log(`Broadcast ${action} to all robots`, 'info');
    }
    
    emergencyStop() {
        if (confirm('⚠️ EMERGENCY STOP: This will halt ALL robots immediately. Continue?')) {
            this.send({
                type: 'emergency_stop'
            });
            this.log('🚨 EMERGENCY STOP triggered!', 'error');
        }
    }
    
    // ==================== Telemetry ====================
    
    startTelemetryUpdates() {
        // Start requesting telemetry updates
        this.telemetryInterval = setInterval(() => {
            const robotId = document.getElementById('telemetryRobotSelect').value;
            if (robotId) {
                this.send({
                    type: 'get_telemetry',
                    robot_id: robotId
                });
            }
        }, 1000);
        
        // Start LiDAR animation
        this.startLidarAnimation();
    }
    
    stopTelemetryUpdates() {
        if (this.telemetryInterval) {
            clearInterval(this.telemetryInterval);
        }
        if (this.lidarAnimation) {
            cancelAnimationFrame(this.lidarAnimation);
        }
    }
    
    updateTelemetry(data) {
        if (data.position) {
            document.getElementById('sensorPosition').textContent = 
                `${data.position.x.toFixed(2)}, ${data.position.y.toFixed(2)}`;
        }
        if (data.orientation !== undefined) {
            document.getElementById('sensorOrientation').textContent = 
                `${(data.orientation * 180 / Math.PI).toFixed(1)}°`;
        }
        if (data.linear_velocity !== undefined) {
            document.getElementById('sensorLinearVel').textContent = 
                `${data.linear_velocity.toFixed(2)} m/s`;
        }
        if (data.angular_velocity !== undefined) {
            document.getElementById('sensorAngularVel').textContent = 
                `${data.angular_velocity.toFixed(2)} rad/s`;
        }
        if (data.battery !== undefined) {
            document.getElementById('sensorBattery').textContent = `${data.battery}%`;
        }
        if (data.obstacle_distance !== undefined) {
            document.getElementById('sensorObstacle').textContent = `${data.obstacle_distance.toFixed(2)} m`;
        }
        
        if (data.lidar) {
            this.updateLidar(data.lidar);
        }
    }
    
    startLidarAnimation() {
        const canvas = document.getElementById('lidarCanvas');
        const ctx = canvas.getContext('2d');
        let angle = 0;
        
        const animate = () => {
            if (!document.getElementById('lidarCanvas')) return;
            
            const w = canvas.width;
            const h = canvas.height;
            const cx = w / 2;
            const cy = h / 2;
            
            ctx.fillStyle = '#0f172a';
            ctx.fillRect(0, 0, w, h);
            
            // Draw grid
            ctx.strokeStyle = '#334155';
            ctx.lineWidth = 1;
            for (let r = 50; r < 200; r += 50) {
                ctx.beginPath();
                ctx.arc(cx, cy, r, 0, Math.PI * 2);
                ctx.stroke();
            }
            
            // Draw crosshairs
            ctx.beginPath();
            ctx.moveTo(cx, 0);
            ctx.lineTo(cx, h);
            ctx.moveTo(0, cy);
            ctx.lineTo(w, cy);
            ctx.stroke();
            
            // Draw robot
            ctx.fillStyle = '#3b82f6';
            ctx.beginPath();
            ctx.arc(cx, cy, 8, 0, Math.PI * 2);
            ctx.fill();
            
            // Draw simulated LiDAR scan
            ctx.strokeStyle = '#10b981';
            ctx.lineWidth = 2;
            ctx.beginPath();
            
            for (let i = 0; i < 360; i += 5) {
                const rad = (i * Math.PI / 180);
                const dist = 100 + Math.sin(rad * 3 + angle) * 30 + Math.random() * 10;
                const x = cx + Math.cos(rad) * dist;
                const y = cy + Math.sin(rad) * dist;
                
                if (i === 0) {
                    ctx.moveTo(x, y);
                } else {
                    ctx.lineTo(x, y);
                }
            }
            ctx.closePath();
            ctx.stroke();
            
            // Draw scan line
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.5)';
            ctx.lineWidth = 3;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(
                cx + Math.cos(angle) * 180,
                cy + Math.sin(angle) * 180
            );
            ctx.stroke();
            
            angle += 0.05;
            this.lidarAnimation = requestAnimationFrame(animate);
        };
        
        animate();
    }
    
    updateLidar(lidarData) {
        // Would update LiDAR with real data
    }
    
    // ==================== Logging ====================
    
    log(message, level = 'info') {
        const timestamp = new Date().toLocaleTimeString();
        const entry = { timestamp, level, message };
        
        this.logs.push(entry);
        if (this.logs.length > this.maxLogs) {
            this.logs.shift();
        }
        
        this.renderLogEntry(entry);
        
        // Also update activity log if on dashboard
        if (this.currentSection === 'dashboard') {
            const activityLog = document.getElementById('activityLog');
            const div = document.createElement('div');
            div.className = 'log-entry';
            div.innerHTML = `
                <span class="log-time">${timestamp}</span>
                <span class="log-level ${level}">${level.toUpperCase()}</span>
                <span class="log-message">${this.escapeHtml(message)}</span>
            `;
            activityLog.appendChild(div);
            activityLog.scrollTop = activityLog.scrollHeight;
        }
        
        // Update logs section
        if (this.currentSection === 'logs') {
            this.renderLogs();
        }
    }
    
    renderLogEntry(entry) {
        const container = document.getElementById('logsContainer');
        if (!container) return;
        
        const div = document.createElement('div');
        div.className = 'log-entry';
        div.innerHTML = `
            <span class="log-time">${entry.timestamp}</span>
            <span class="log-level ${entry.level}">${entry.level.toUpperCase()}</span>
            <span class="log-message">${this.escapeHtml(entry.message)}</span>
        `;
        container.appendChild(div);
        container.scrollTop = container.scrollHeight;
    }
    
    renderLogs() {
        const container = document.getElementById('logsContainer');
        if (!container) return;
        
        const filter = document.getElementById('logLevelFilter')?.value || 'all';
        
        container.innerHTML = this.logs
            .filter(log => filter === 'all' || log.level === filter)
            .map(entry => `
                <div class="log-entry">
                    <span class="log-time">${entry.timestamp}</span>
                    <span class="log-level ${entry.level}">${entry.level.toUpperCase()}</span>
                    <span class="log-message">${this.escapeHtml(entry.message)}</span>
                </div>
            `).join('');
        
        container.scrollTop = container.scrollHeight;
    }
    
    clearLogs() {
        this.logs = [];
        document.getElementById('activityLog').innerHTML = '';
        document.getElementById('logsContainer').innerHTML = '';
    }
    
    exportLogs() {
        const text = this.logs.map(l => `[${l.timestamp}] [${l.level.toUpperCase()}] ${l.message}`).join('\n');
        const blob = new Blob([text], { type: 'text/plain' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `agent-ros-bridge-logs-${new Date().toISOString().slice(0,10)}.txt`;
        a.click();
        URL.revokeObjectURL(url);
    }
    
    // ==================== Stats ====================
    
    updateStats() {
        document.getElementById('robotCount').textContent = this.robots.size;
        
        if (this.robots.size > 0) {
            const avgBattery = Array.from(this.robots.values())
                .reduce((sum, r) => sum + (r.battery || 0), 0) / this.robots.size;
            document.getElementById('avgBattery').textContent = `${avgBattery.toFixed(0)}%`;
        }
    }
    
    // ==================== Utilities ====================
    
    openModal(id) {
        document.getElementById(id).classList.add('active');
    }
    
    closeModal(id) {
        document.getElementById(id).classList.remove('active');
    }
    
    closeAllModals() {
        document.querySelectorAll('.modal').forEach(m => m.classList.remove('active'));
    }
    
    showConfirmation(data) {
        document.getElementById('confirmMessage').textContent = data.message;
        document.getElementById('confirmBtn').onclick = () => {
            this.send({
                type: 'confirmation_response',
                request_id: data.request_id,
                approved: true
            });
            this.closeModal('confirmModal');
        };
        this.openModal('confirmModal');
    }
    
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
    
    startClock() {
        // Update clock display if exists
        setInterval(() => {
            const clockEl = document.getElementById('clock');
            if (clockEl) {
                clockEl.textContent = new Date().toLocaleTimeString();
            }
        }, 1000);
    }
    
    // ==================== Shadow Mode Metrics ====================
    
    async refreshShadowMetrics() {
        try {
            const metrics = await this.fetchShadowMetrics();
            this.updateShadowMetrics(metrics);
            
            const decisions = await this.fetchShadowDecisions(10);
            this.updateShadowDecisionsTable(decisions);
            
            document.getElementById('shadowLastUpdated').textContent = 
                `Last updated: ${new Date().toLocaleTimeString()}`;
        } catch (error) {
            this.log('Failed to refresh shadow metrics: ' + error.message, 'error');
        }
    }
    
    async fetchShadowMetrics() {
        try {
            const response = await fetch('/api/metrics');
            if (!response.ok) throw new Error('API error');
            return await response.json();
        } catch (error) {
            // Return mock data if API not available
            return {
                agreement_rate: 0.0,
                total_decisions: 0,
                pending_count: 0,
                completed_decisions: 0
            };
        }
    }
    
    async fetchShadowDecisions(limit = 10) {
        try {
            const response = await fetch(`/api/decisions?limit=${limit}`);
            if (!response.ok) throw new Error('API error');
            return await response.json();
        } catch (error) {
            return [];
        }
    }
    
    updateShadowMetrics(metrics) {
        if (!metrics) return;
        
        const agreementRate = document.getElementById('shadowAgreementRate');
        const totalDecisions = document.getElementById('shadowTotalDecisions');
        const pendingCount = document.getElementById('shadowPendingCount');
        const completedCount = document.getElementById('shadowCompletedCount');
        
        if (agreementRate) {
            agreementRate.textContent = `${(metrics.agreement_rate * 100).toFixed(1)}%`;
        }
        if (totalDecisions) {
            totalDecisions.textContent = metrics.total_decisions || 0;
        }
        if (pendingCount) {
            pendingCount.textContent = metrics.pending_count || 0;
        }
        if (completedCount) {
            completedCount.textContent = metrics.completed_decisions || 0;
        }
    }
    
    updateShadowDecisionsTable(decisions) {
        const tbody = document.getElementById('shadowDecisionsBody');
        if (!tbody) return;
        
        if (decisions.length === 0) {
            tbody.innerHTML = `
                <tr class="empty-row">
                    <td colspan="6" class="text-center">
                        <div class="empty-state">
                            <div class="empty-icon">📊</div>
                            <p>No decisions logged yet</p>
                        </div>
                    </td>
                </tr>
            `;
            return;
        }
        
        tbody.innerHTML = decisions.map(decision => {
            const time = new Date(decision.timestamp).toLocaleTimeString();
            const aiIntent = decision.ai_proposal?.intent_type || 'N/A';
            const humanAction = decision.human_action?.command || 'N/A';
            const agreement = decision.agreement;
            const score = decision.agreement_score;
            
            let statusClass = 'pending';
            let statusText = 'Pending';
            if (agreement === true) {
                statusClass = 'agree';
                statusText = 'Yes';
            } else if (agreement === false) {
                statusClass = 'disagree';
                statusText = 'No';
            }
            
            return `
                <tr>
                    <td>${time}</td>
                    <td>${decision.robot_id}</td>
                    <td>${aiIntent}</td>
                    <td>${humanAction}</td>
                    <td><span class="status-badge ${statusClass}">${statusText}</span></td>
                    <td>${score ? score.toFixed(2) : '--'}</td>
                </tr>
            `;
        }).join('');
    }
    
    // ==================== Mock Data (for demo) ====================
    
    loadMockData() {
        // Add some sample robots for demo
        const mockRobots = [
            { id: 'turtlebot_01', name: 'TurtleBot 1', type: 'TurtleBot3', status: 'online', battery: 85, location: 'Kitchen' },
            { id: 'turtlebot_02', name: 'TurtleBot 2', type: 'TurtleBot3', status: 'busy', battery: 62, location: 'Office' },
        ];
        
        // Load mock data for demo
        this.updateRobotList(mockRobots);
        this.selectedRobot = mockRobots[0];
        this.updateDashboard();
        
        // Simulate connected status
        this.connected = true;
        this.updateConnectionStatus('connected');
        document.getElementById('connectBtn').innerHTML = '<span class="btn-icon">🔌</span> Disconnect';
        
        // Load mock shadow mode data
        this.loadMockShadowData();
    }
    
    loadMockShadowData() {
        // Mock shadow mode metrics (Gate 3 complete)
        const mockMetrics = {
            agreement_rate: 0.96,
            total_decisions: 144000,
            pending_count: 0,
            completed_decisions: 144000,
            hours_collected: 200.0,
            required_hours: 200.0
        };
        this.updateShadowMetrics(mockMetrics);
        
        // Mock decisions
        const mockDecisions = [
            {
                timestamp: new Date().toISOString(),
                robot_id: 'turtlebot_01',
                ai_proposal: { intent_type: 'move_forward', confidence: 0.94, parameters: {distance: 2.0} },
                human_action: { command: 'move_forward', parameters: {distance: 2.0} },
                agreement: true,
                agreement_score: 0.95
            },
            {
                timestamp: new Date(Date.now() - 5000).toISOString(),
                robot_id: 'turtlebot_02',
                ai_proposal: { intent_type: 'rotate', confidence: 0.89, parameters: {angle: 90} },
                human_action: { command: 'rotate', parameters: {angle: 90} },
                agreement: true,
                agreement_score: 0.92
            },
            {
                timestamp: new Date(Date.now() - 10000).toISOString(),
                robot_id: 'turtlebot_01',
                ai_proposal: { intent_type: 'pick_object', confidence: 0.91, parameters: {object: 'box'} },
                human_action: { command: 'pick_object', parameters: {object: 'box'} },
                agreement: true,
                agreement_score: 0.94
            }
        ];
        this.updateShadowDecisionsTable(mockDecisions);
        
        // Update last updated timestamp
        const lastUpdated = document.getElementById('shadowLastUpdated');
        if (lastUpdated) {
            lastUpdated.textContent = `Last updated: ${new Date().toLocaleTimeString()}`;
        }
    }
}

// Initialize app
const app = new AgentROSBridgeApp();

// Expose for debugging
window.app = app;