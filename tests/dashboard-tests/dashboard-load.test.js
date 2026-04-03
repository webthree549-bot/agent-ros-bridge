/**
 * Dashboard Load Testing
 * Tests WebSocket performance under load
 */

const WebSocket = require('ws');
const { performance } = require('perf_hooks');

// Configuration
const WS_URL = process.env.BRIDGE_WS_URL || 'ws://localhost:8765';
const CONCURRENT_USERS = parseInt(process.env.CONCURRENT_USERS) || 100;
const DURATION_MS = parseInt(process.env.DURATION_MS) || 60000; // 1 minute
const RAMP_UP_MS = parseInt(process.env.RAMP_UP_MS) || 10000; // 10 seconds

// Metrics
const metrics = {
    connections: {
        attempted: 0,
        successful: 0,
        failed: 0,
        disconnected: 0
    },
    messages: {
        sent: 0,
        received: 0,
        errors: 0
    },
    latency: [],
    errors: []
};

class LoadTestClient {
    constructor(id) {
        this.id = id;
        this.ws = null;
        this.connected = false;
        this.messageCount = 0;
        this.startTime = null;
    }

    async connect() {
        metrics.connections.attempted++;
        this.startTime = performance.now();

        return new Promise((resolve, reject) => {
            this.ws = new WebSocket(WS_URL);

            const timeout = setTimeout(() => {
                reject(new Error('Connection timeout'));
            }, 5000);

            this.ws.on('open', () => {
                clearTimeout(timeout);
                this.connected = true;
                metrics.connections.successful++;
                
                const latency = performance.now() - this.startTime;
                metrics.latency.push(latency);
                
                console.log(`[Client ${this.id}] Connected (${latency.toFixed(2)}ms)`);
                resolve();
            });

            this.ws.on('message', (data) => {
                metrics.messages.received++;
                this.messageCount++;
                
                try {
                    const msg = JSON.parse(data);
                    this.handleMessage(msg);
                } catch (e) {
                    // Binary or non-JSON message
                }
            });

            this.ws.on('error', (error) => {
                clearTimeout(timeout);
                metrics.connections.failed++;
                metrics.errors.push(`Client ${this.id}: ${error.message}`);
                reject(error);
            });

            this.ws.on('close', () => {
                this.connected = false;
                metrics.connections.disconnected++;
                console.log(`[Client ${this.id}] Disconnected`);
            });
        });
    }

    handleMessage(msg) {
        // Simulate realistic dashboard usage
        switch (msg.type) {
            case 'robot_list':
                // Request telemetry for first robot
                if (msg.robots && msg.robots.length > 0) {
                    this.send({
                        type: 'get_telemetry',
                        robot_id: msg.robots[0].id
                    });
                }
                break;
            case 'telemetry':
                // Normal telemetry update
                break;
        }
    }

    send(data) {
        if (this.connected && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(data));
            metrics.messages.sent++;
        }
    }

    simulateDashboardActivity() {
        if (!this.connected) return;

        const actions = [
            () => this.send({ type: 'get_robots' }),
            () => this.send({ type: 'ping' }),
            () => this.send({
                type: 'command',
                robot_id: 'test_bot',
                command: { action: 'status' }
            })
        ];

        // Random action every 2-5 seconds
        const interval = 2000 + Math.random() * 3000;
        setTimeout(() => {
            if (this.connected) {
                const action = actions[Math.floor(Math.random() * actions.length)];
                action();
                this.simulateDashboardActivity();
            }
        }, interval);
    }

    disconnect() {
        if (this.ws) {
            this.ws.close();
        }
    }
}

async function runLoadTest() {
    console.log('╔════════════════════════════════════════════════════════╗');
    console.log('║     Dashboard Load Test                                ║');
    console.log('╚════════════════════════════════════════════════════════╝');
    console.log(`\nConfiguration:`);
    console.log(`  WebSocket URL: ${WS_URL}`);
    console.log(`  Concurrent Users: ${CONCURRENT_USERS}`);
    console.log(`  Duration: ${DURATION_MS / 1000}s`);
    console.log(`  Ramp-up: ${RAMP_UP_MS / 1000}s\n`);

    const clients = [];
    const startTime = performance.now();

    // Ramp up connections
    console.log('🚀 Ramping up connections...');
    const rampUpInterval = RAMP_UP_MS / CONCURRENT_USERS;

    for (let i = 0; i < CONCURRENT_USERS; i++) {
        const client = new LoadTestClient(i + 1);
        clients.push(client);

        try {
            await client.connect();
            client.simulateDashboardActivity();
        } catch (error) {
            console.error(`[Client ${i + 1}] Connection failed:`, error.message);
        }

        // Stagger connections
        await new Promise(resolve => setTimeout(resolve, rampUpInterval));
    }

    console.log(`\n✅ Ramp-up complete: ${metrics.connections.successful}/${CONCURRENT_USERS} connected`);
    console.log(`⏱️  Running test for ${DURATION_MS / 1000}s...\n`);

    // Run for specified duration
    await new Promise(resolve => setTimeout(resolve, DURATION_MS));

    // Disconnect all clients
    console.log('\n🔌 Disconnecting clients...');
    clients.forEach(client => client.disconnect());

    // Wait for disconnections
    await new Promise(resolve => setTimeout(resolve, 1000));

    // Generate report
    const duration = performance.now() - startTime;
    generateReport(duration);
}

function generateReport(duration) {
    console.log('\n╔════════════════════════════════════════════════════════╗');
    console.log('║     Load Test Results                                  ║');
    console.log('╚════════════════════════════════════════════════════════╝\n');

    // Connection stats
    console.log('📊 Connection Statistics:');
    console.log(`  Attempted: ${metrics.connections.attempted}`);
    console.log(`  Successful: ${metrics.connections.successful} (${((metrics.connections.successful / metrics.connections.attempted) * 100).toFixed(1)}%)`);
    console.log(`  Failed: ${metrics.connections.failed}`);
    console.log(`  Disconnected: ${metrics.connections.disconnected}`);

    // Message stats
    console.log('\n📨 Message Statistics:');
    console.log(`  Sent: ${metrics.messages.sent}`);
    console.log(`  Received: ${metrics.messages.received}`);
    console.log(`  Errors: ${metrics.messages.errors}`);
    console.log(`  Msg/sec: ${((metrics.messages.sent + metrics.messages.received) / (duration / 1000)).toFixed(1)}`);

    // Latency stats
    if (metrics.latency.length > 0) {
        const sorted = metrics.latency.sort((a, b) => a - b);
        const avg = sorted.reduce((a, b) => a + b, 0) / sorted.length;
        const p50 = sorted[Math.floor(sorted.length * 0.5)];
        const p95 = sorted[Math.floor(sorted.length * 0.95)];
        const p99 = sorted[Math.floor(sorted.length * 0.99)];
        const min = sorted[0];
        const max = sorted[sorted.length - 1];

        console.log('\n⏱️  Connection Latency (ms):');
        console.log(`  Min: ${min.toFixed(2)}`);
        console.log(`  Avg: ${avg.toFixed(2)}`);
        console.log(`  P50: ${p50.toFixed(2)}`);
        console.log(`  P95: ${p95.toFixed(2)}`);
        console.log(`  P99: ${p99.toFixed(2)}`);
        console.log(`  Max: ${max.toFixed(2)}`);
    }

    // Duration
    console.log(`\n⏱️  Test Duration: ${(duration / 1000).toFixed(2)}s`);

    // Errors
    if (metrics.errors.length > 0) {
        console.log(`\n⚠️  Errors (${metrics.errors.length}):`);
        metrics.errors.slice(0, 10).forEach(err => console.log(`  - ${err}`));
        if (metrics.errors.length > 10) {
            console.log(`  ... and ${metrics.errors.length - 10} more`);
        }
    }

    // Success criteria
    const successRate = (metrics.connections.successful / metrics.connections.attempted) * 100;
    const avgLatency = metrics.latency.reduce((a, b) => a + b, 0) / metrics.latency.length;

    console.log('\n╔════════════════════════════════════════════════════════╗');
    if (successRate >= 95 && avgLatency < 1000) {
        console.log('║  ✅ TEST PASSED                                        ║');
    } else {
        console.log('║  ❌ TEST FAILED                                        ║');
    }
    console.log('╚════════════════════════════════════════════════════════╝');

    // Exit code
    process.exit(successRate >= 95 && avgLatency < 1000 ? 0 : 1);
}

// Run if executed directly
if (require.main === module) {
    runLoadTest().catch(error => {
        console.error('Load test failed:', error);
        process.exit(1);
    });
}

module.exports = { runLoadTest, LoadTestClient };