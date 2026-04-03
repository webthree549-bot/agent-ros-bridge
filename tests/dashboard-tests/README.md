# Dashboard Testing Suite

Comprehensive testing for the Agent ROS Bridge Web Dashboard including E2E, load, and security tests.

## Test Types

### 1. E2E Tests (`dashboard-e2e.test.js`)
End-to-end browser automation tests using Playwright.

**Coverage:**
- ✅ Page navigation (all 8 sections)
- ✅ WebSocket connection
- ✅ Robot management
- ✅ Control panel (D-pad, natural language)
- ✅ Telemetry display
- ✅ Shadow mode metrics
- ✅ Safety status
- ✅ Emergency stop
- ✅ Responsive design (mobile/tablet)

**Run:**
```bash
cd tests/dashboard-tests
npm install
npm run test:e2e
```

**With visible browser:**
```bash
npm run test:e2e:headed
```

### 2. Load Tests (`dashboard-load.test.js`)
WebSocket performance and concurrency testing.

**Metrics:**
- Connection success rate
- Message throughput (msg/sec)
- Connection latency (min/avg/p95/p99/max)
- Concurrent user capacity

**Run:**
```bash
# Default: 100 users, 60 seconds
cd tests/dashboard-tests
npm run test:load

# 100 concurrent users
npm run test:load:100

# 500 concurrent users, 2 minutes
npm run test:load:500

# Custom configuration
CONCURRENT_USERS=200 DURATION_MS=90000 RAMP_UP_MS=15000 npm run test:load
```

**Environment Variables:**
- `CONCURRENT_USERS`: Number of simulated clients (default: 100)
- `DURATION_MS`: Test duration in milliseconds (default: 60000)
- `RAMP_UP_MS`: Connection ramp-up time (default: 10000)
- `BRIDGE_WS_URL`: WebSocket URL (default: ws://localhost:8765)

### 3. Security Audit (`dashboard-security-audit.js`)
Static analysis for security vulnerabilities.

**Checks:**
- ✅ Hardcoded secrets detection
- ✅ XSS prevention (innerHTML sanitization)
- ✅ eval() and dangerous function usage
- ✅ LocalStorage sensitive data
- ✅ HTTPS/WSS enforcement
- ✅ CORS configuration
- ✅ Content Security Policy
- ✅ WebSocket authentication

**Run:**
```bash
cd tests/dashboard-tests
npm run test:security
```

**Or directly:**
```bash
node tests/security/dashboard-security-audit.js
```

## Quick Start

### Prerequisites

```bash
# Install Node.js 16+
node --version

# Install dependencies
cd tests/dashboard-tests
npm install

# Install Playwright browsers
npx playwright install chromium
```

### Run All Tests

```bash
npm run test:all
```

### CI/CD Integration

```bash
# Run tests suitable for CI (no headed browser)
npm run test:ci
```

## Test Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DASHBOARD_URL` | http://localhost:8081 | Dashboard URL for E2E |
| `BRIDGE_WS_URL` | ws://localhost:8765 | WebSocket URL |
| `CONCURRENT_USERS` | 100 | Load test users |
| `DURATION_MS` | 60000 | Load test duration |
| `RAMP_UP_MS` | 10000 | Load test ramp-up |
| `HEADLESS` | true | Run browser headless |
| `SLOW_MO` | 0 | Slow motion delay (ms) |

### Mock Bridge Server

The E2E tests include a mock bridge server that simulates:
- Robot list responses
- Command acknowledgments
- Telemetry data
- Natural language processing

No real bridge or robots needed for testing.

## Test Results

### E2E Test Output
```
  Dashboard E2E Tests
    Navigation
      ✓ should load dashboard page
      ✓ should navigate to all sections
    Connection
      ✓ should show disconnected status initially
      ✓ should connect to bridge
    ...

  15 passing (12s)
```

### Load Test Output
```
╔════════════════════════════════════════════════════════╗
║     Load Test Results                                  ║
╚════════════════════════════════════════════════════════╝

📊 Connection Statistics:
  Attempted: 100
  Successful: 100 (100.0%)
  Failed: 0

📨 Message Statistics:
  Sent: 5234
  Received: 5234
  Msg/sec: 174.5

⏱️  Connection Latency (ms):
  Min: 12.34
  Avg: 45.67
  P95: 89.12
  P99: 123.45
  Max: 156.78

╔════════════════════════════════════════════════════════╗
║  ✅ TEST PASSED                                        ║
╚════════════════════════════════════════════════════════╝
```

### Security Audit Output
```
╔════════════════════════════════════════════════════════╗
║     Security Audit Report                              ║
╚════════════════════════════════════════════════════════╝

✅ PASSED (8):
  ✓ app.js: Input validation/sanitization detected
  ✓ app.js: JWT token authentication detected
  ...

⚠️  WARNINGS (2):
  ⚠ index.html: Using ws:// instead of wss://
  ⚠ app.js: innerHTML assignment - verify sanitization

╔════════════════════════════════════════════════════════╗
║  ⚠️  2 WARNINGS - Review recommended                   ║
╚════════════════════════════════════════════════════════╝
```

## Interpreting Results

### E2E Tests
- **All passing**: Dashboard functions correctly
- **Failures**: Check specific test for UI/regression issues

### Load Tests
- **Success rate >= 95%**: Good
- **Avg latency < 1000ms**: Acceptable
- **P99 latency < 2000ms**: Good user experience

### Security Audit
- **All passed**: No critical issues
- **Warnings**: Review and fix before production
- **Failed**: Must fix before deployment

## Continuous Integration

### GitHub Actions Example

```yaml
name: Dashboard Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: '18'
      
      - name: Install dependencies
        working-directory: tests/dashboard-tests
        run: npm install
      
      - name: Install Playwright
        run: npx playwright install chromium
      
      - name: Run security audit
        working-directory: tests/dashboard-tests
        run: npm run test:security
      
      - name: Run E2E tests
        working-directory: tests/dashboard-tests
        run: npm run test:e2e
      
      - name: Run load tests
        working-directory: tests/dashboard-tests
        run: npm run test:load:100
```

## Troubleshooting

### E2E Tests Timeout
```bash
# Increase timeout
npm run test:e2e -- --timeout 60000
```

### Load Test Connection Refused
```bash
# Ensure bridge is running
docker-compose up -d bridge

# Or use mock server
MOCK_BRIDGE=true npm run test:load
```

### Security Audit False Positives
The security audit uses pattern matching and may flag safe code. Review each warning manually.

## Contributing

1. Add new tests to appropriate file
2. Follow existing test patterns
3. Update this README
4. Run full test suite before submitting PR

## License

MIT - Same as Agent ROS Bridge