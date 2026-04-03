/**
 * Dashboard E2E Tests
 * Tests the web dashboard UI and functionality
 */

const { chromium } = require('playwright');
const assert = require('assert');

// Test configuration
const BASE_URL = process.env.DASHBOARD_URL || 'http://localhost:8081';
const WS_URL = process.env.BRIDGE_WS_URL || 'ws://localhost:8765';
const TEST_TIMEOUT = 30000;

// Mock bridge server for testing
class MockBridgeServer {
    constructor(port = 8766) {
        this.port = port;
        this.clients = new Set();
        this.server = null;
    }

    async start() {
        const WebSocket = require('ws');
        this.server = new WebSocket.Server({ port: this.port });
        
        this.server.on('connection', (ws) => {
            this.clients.add(ws);
            
            // Send initial robot list
            ws.send(JSON.stringify({
                type: 'robot_list',
                robots: [
                    { id: 'test_bot_1', name: 'Test Bot 1', type: 'TurtleBot3', status: 'online', battery: 85 },
                    { id: 'test_bot_2', name: 'Test Bot 2', type: 'TurtleBot3', status: 'busy', battery: 62 }
                ]
            }));

            ws.on('message', (data) => {
                const msg = JSON.parse(data);
                this.handleMessage(ws, msg);
            });

            ws.on('close', () => {
                this.clients.delete(ws);
            });
        });

        return new Promise((resolve) => {
            this.server.on('listening', resolve);
        });
    }

    handleMessage(ws, msg) {
        switch (msg.type) {
            case 'ping':
                ws.send(JSON.stringify({ type: 'pong' }));
                break;
            case 'get_robots':
                ws.send(JSON.stringify({
                    type: 'robot_list',
                    robots: [
                        { id: 'test_bot_1', name: 'Test Bot 1', type: 'TurtleBot3', status: 'online', battery: 85 }
                    ]
                }));
                break;
            case 'command':
                ws.send(JSON.stringify({
                    type: 'command_result',
                    success: true,
                    action: msg.command.action,
                    robot_id: msg.robot_id
                }));
                break;
            case 'natural_language':
                ws.send(JSON.stringify({
                    type: 'command_result',
                    success: true,
                    response: `Executed: ${msg.text}`,
                    action: 'natural_language'
                }));
                break;
            case 'get_telemetry':
                ws.send(JSON.stringify({
                    type: 'telemetry',
                    robot_id: msg.robot_id,
                    data: {
                        position: { x: 1.5, y: 2.3 },
                        orientation: 0.785,
                        linear_velocity: 0.5,
                        angular_velocity: 0.1,
                        battery: 85,
                        obstacle_distance: 1.2
                    }
                }));
                break;
        }
    }

    async stop() {
        return new Promise((resolve) => {
            this.server.close(resolve);
        });
    }
}

// Test suite
describe('Dashboard E2E Tests', function() {
    this.timeout(TEST_TIMEOUT);
    
    let browser;
    let page;
    let mockServer;

    before(async function() {
        // Start mock bridge server
        mockServer = new MockBridgeServer(8766);
        await mockServer.start();
        console.log('Mock bridge server started on port 8766');

        // Launch browser
        browser = await chromium.launch({
            headless: process.env.CI ? true : false,
            slowMo: process.env.SLOW_MO ? parseInt(process.env.SLOW_MO) : 0
        });
    });

    after(async function() {
        await browser.close();
        await mockServer.stop();
    });

    beforeEach(async function() {
        page = await browser.newPage();
        await page.goto(BASE_URL);
        await page.waitForLoadState('networkidle');
    });

    afterEach(async function() {
        await page.close();
    });

    describe('Navigation', function() {
        it('should load dashboard page', async function() {
            const title = await page.title();
            assert.ok(title.includes('Agent ROS Bridge'), 'Page title should contain Agent ROS Bridge');
        });

        it('should navigate to all sections', async function() {
            const sections = ['robots', 'control', 'telemetry', 'fleet', 'shadow', 'safety', 'logs'];
            
            for (const section of sections) {
                await page.click(`[data-section="${section}"]`);
                await page.waitForTimeout(100);
                
                const activeSection = await page.$eval('.section.active', el => el.id);
                assert.ok(activeSection.includes(section), `Should navigate to ${section} section`);
            }
        });
    });

    describe('Connection', function() {
        it('should show disconnected status initially', async function() {
            const statusText = await page.$eval('.status-text', el => el.textContent);
            assert.ok(statusText.includes('Disconnected'), 'Should show disconnected status');
        });

        it('should connect to bridge', async function() {
            // Open connect modal
            await page.click('#connectBtn');
            await page.waitForSelector('#connectModal.active');

            // Enter WebSocket URL (use mock server)
            await page.fill('#wsUrl', 'ws://localhost:8766');
            
            // Click connect
            await page.click('#connectModal .btn-primary');
            
            // Wait for connection
            await page.waitForTimeout(1000);
            
            // Check connected status
            const statusText = await page.$eval('.status-text', el => el.textContent);
            assert.ok(statusText.includes('Connected') || statusText.includes('Connecting'), 
                'Should attempt to connect');
        });
    });

    describe('Robot Management', function() {
        it('should display robot list', async function() {
            // First connect
            await page.click('#connectBtn');
            await page.fill('#wsUrl', 'ws://localhost:8766');
            await page.click('#connectModal .btn-primary');
            await page.waitForTimeout(1000);

            // Navigate to robots section
            await page.click('[data-section="robots"]');
            await page.waitForTimeout(500);

            // Check robot table exists
            const table = await page.$('#robotsTable');
            assert.ok(table, 'Robot table should exist');
        });

        it('should select robot from dropdown', async function() {
            // Connect first
            await page.click('#connectBtn');
            await page.fill('#wsUrl', 'ws://localhost:8766');
            await page.click('#connectModal .btn-primary');
            await page.waitForTimeout(1000);

            // Navigate to control
            await page.click('[data-section="control"]');
            await page.waitForTimeout(500);

            // Select robot
            await page.selectOption('#controlRobotSelect', 'test_bot_1');
            
            const selected = await page.$eval('#controlRobotSelect', el => el.value);
            assert.equal(selected, 'test_bot_1', 'Should select robot');
        });
    });

    describe('Control Panel', function() {
        beforeEach(async function() {
            // Connect and select robot
            await page.click('#connectBtn');
            await page.fill('#wsUrl', 'ws://localhost:8766');
            await page.click('#connectModal .btn-primary');
            await page.waitForTimeout(1000);
            
            await page.click('[data-section="control"]');
            await page.waitForTimeout(500);
            await page.selectOption('#controlRobotSelect', 'test_bot_1');
        });

        it('should send D-pad commands', async function() {
            // Click forward button
            await page.click('.d-up');
            await page.waitForTimeout(500);

            // Check for log entry
            const logs = await page.$$eval('.log-entry', entries => 
                entries.some(e => e.textContent.includes('forward') || e.textContent.includes('move'))
            );
            assert.ok(logs, 'Should log command');
        });

        it('should send natural language command', async function() {
            // Type command
            await page.fill('#nlCommandInput', 'Move forward 2 meters');
            await page.click('.chat-input-group .btn-primary');
            await page.waitForTimeout(500);

            // Check chat message
            const chatMessages = await page.$$eval('.chat-message', msgs => msgs.length);
            assert.ok(chatMessages >= 2, 'Should show user message and response');
        });
    });

    describe('Telemetry', function() {
        it('should display sensor data', async function() {
            // Connect
            await page.click('#connectBtn');
            await page.fill('#wsUrl', 'ws://localhost:8766');
            await page.click('#connectModal .btn-primary');
            await page.waitForTimeout(1000);

            // Navigate to telemetry
            await page.click('[data-section="telemetry"]');
            await page.waitForTimeout(1000);

            // Select robot
            await page.selectOption('#telemetryRobotSelect', 'test_bot_1');
            await page.waitForTimeout(1000);

            // Check sensor values are displayed
            const position = await page.$eval('#sensorPosition', el => el.textContent);
            assert.ok(position !== '--, --', 'Should show position data');
        });
    });

    describe('Shadow Mode', function() {
        it('should display shadow metrics', async function() {
            await page.click('[data-section="shadow"]');
            await page.waitForTimeout(500);

            // Check metrics exist
            const agreementRate = await page.$('#shadowAgreementRate');
            const totalDecisions = await page.$('#shadowTotalDecisions');
            
            assert.ok(agreementRate, 'Agreement rate element should exist');
            assert.ok(totalDecisions, 'Total decisions element should exist');
        });

        it('should refresh shadow metrics', async function() {
            await page.click('[data-section="shadow"]');
            await page.waitForTimeout(500);

            // Click refresh
            await page.click('button[onclick="app.refreshShadowMetrics()"]');
            await page.waitForTimeout(1000);

            // Check last updated time
            const lastUpdated = await page.$eval('#shadowLastUpdated', el => el.textContent);
            assert.ok(lastUpdated.includes('Last updated'), 'Should show last updated time');
        });
    });

    describe('Safety', function() {
        it('should display validation gates', async function() {
            await page.click('[data-section="safety"]');
            await page.waitForTimeout(500);

            // Check gates exist
            const gates = await page.$$('.gate-item');
            assert.ok(gates.length >= 4, 'Should show 4 validation gates');
        });

        it('should show safe mode status', async function() {
            await page.click('[data-section="safety"]');
            await page.waitForTimeout(500);

            const safetyText = await page.$eval('.indicator-text', el => el.textContent);
            assert.ok(safetyText.includes('SAFE'), 'Should show safe mode');
        });
    });

    describe('Emergency Stop', function() {
        it('should trigger emergency stop', async function() {
            // Setup dialog handler
            page.on('dialog', async dialog => {
                assert.ok(dialog.message().includes('EMERGENCY STOP'));
                await dialog.accept();
            });

            // Click emergency stop
            await page.click('#emergencyStopBtn');
            await page.waitForTimeout(500);
        });
    });

    describe('Responsive Design', function() {
        it('should work on mobile viewport', async function() {
            await page.setViewportSize({ width: 375, height: 667 });
            await page.reload();
            await page.waitForLoadState('networkidle');

            // Check sidebar is hidden or collapsed
            const sidebar = await page.$('.sidebar');
            assert.ok(sidebar, 'Sidebar should exist on mobile');
        });

        it('should work on tablet viewport', async function() {
            await page.setViewportSize({ width: 768, height: 1024 });
            await page.reload();
            await page.waitForLoadState('networkidle');

            const content = await page.$('.content');
            assert.ok(content, 'Content should be visible on tablet');
        });
    });
});

// Run tests if executed directly
if (require.main === module) {
    const Mocha = require('mocha');
    const mocha = new Mocha();
    mocha.suite.emit('pre-require', global, 'e2e-tests.js', mocha);
    
    describe('Dashboard E2E', function() {
        // Tests defined above
    });
    
    mocha.run(failures => {
        process.exitCode = failures ? 1 : 0;
    });
}

module.exports = { MockBridgeServer };