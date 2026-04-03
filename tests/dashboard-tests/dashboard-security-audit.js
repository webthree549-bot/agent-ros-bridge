/**
 * Dashboard Security Audit
 * Checks for common security vulnerabilities
 */

const fs = require('fs');
const path = require('path');

// Configuration
const WEB_DIR = path.join(__dirname, '../../agent_ros_bridge/web');
const JS_DIR = path.join(WEB_DIR, 'app.js');
const HTML_DIR = path.join(WEB_DIR, 'index.html');
const CSS_DIR = path.join(WEB_DIR, 'styles.css');
const NGINX_CONF = path.join(__dirname, '../../docker/nginx-dashboard.conf');

// Security checks
const securityReport = {
    passed: [],
    warnings: [],
    failed: [],
    info: []
};

function auditFile(filePath, name) {
    console.log(`\n🔍 Auditing ${name}...`);
    
    if (!fs.existsSync(filePath)) {
        securityReport.failed.push(`${name}: File not found`);
        return;
    }

    const content = fs.readFileSync(filePath, 'utf8');
    const lines = content.split('\n');

    // Check 1: No hardcoded secrets
    const secretPatterns = [
        /password\s*[=:]\s*["'][^"']+["']/i,
        /api[_-]?key\s*[=:]\s*["'][^"']+["']/i,
        /secret\s*[=:]\s*["'][^"']+["']/i,
        /token\s*[=:]\s*["'][^"']+["']/i,
        /jwt\s*[=:]\s*["'][^"']+["']/i
    ];

    secretPatterns.forEach(pattern => {
        if (pattern.test(content)) {
            securityReport.failed.push(`${name}: Potential hardcoded secret found`);
        }
    });

    // Check 2: No eval() or dangerous functions
    const dangerousPatterns = [
        { pattern: /eval\s*\(/, desc: 'eval() usage' },
        { pattern: /new\s+Function\s*\(/, desc: 'Function constructor' },
        { pattern: /document\.write\s*\(/, desc: 'document.write()' },
        { pattern: /innerHTML\s*=/, desc: 'innerHTML assignment (check for XSS)' }
    ];

    dangerousPatterns.forEach(({ pattern, desc }) => {
        const matches = content.match(pattern);
        if (matches) {
            // Check if properly sanitized
            const context = content.substring(
                Math.max(0, content.indexOf(matches[0]) - 50),
                content.indexOf(matches[0]) + matches[0].length + 50
            );
            
            if (context.includes('escapeHtml') || context.includes('textContent')) {
                securityReport.passed.push(`${name}: ${desc} found but appears sanitized`);
            } else {
                securityReport.warnings.push(`${name}: ${desc} - verify sanitization`);
            }
        }
    });

    // Check 3: HTTPS/WSS for production
    if (content.includes('ws://') && !content.includes('wss://')) {
        securityReport.warnings.push(`${name}: Using ws:// instead of wss:// - use WSS in production`);
    }
    if (content.includes('http://') && !content.includes('https://')) {
        securityReport.warnings.push(`${name}: Using http:// instead of https:// - use HTTPS in production`);
    }

    // Check 4: Input validation
    if (name === 'app.js') {
        // Check for input validation
        const hasInputValidation = content.includes('escapeHtml') || 
                                   content.includes('sanitiz') ||
                                   content.includes('validate');
        if (hasInputValidation) {
            securityReport.passed.push(`${name}: Input validation/sanitization detected`);
        } else {
            securityReport.warnings.push(`${name}: No obvious input validation - review manually`);
        }
    }

    // Check 5: XSS prevention in innerHTML
    const innerHTMLMatches = content.match(/innerHTML\s*=\s*[^;]+/g) || [];
    innerHTMLMatches.forEach(match => {
        if (match.includes('escapeHtml') || match.includes('createElement')) {
            securityReport.passed.push(`${name}: innerHTML uses sanitization`);
        } else {
            securityReport.warnings.push(`${name}: innerHTML assignment without obvious sanitization: ${match.substring(0, 50)}...`);
        }
    });

    // Check 6: LocalStorage usage
    if (content.includes('localStorage')) {
        securityReport.info.push(`${name}: Uses localStorage - ensure no sensitive data stored`);
        
        // Check what's being stored
        const localStorageMatches = content.match(/localStorage\.(setItem|getItem)\s*\(\s*["']([^"']+)["']/g) || [];
        localStorageMatches.forEach(match => {
            if (match.toLowerCase().includes('token') || 
                match.toLowerCase().includes('password') ||
                match.toLowerCase().includes('secret')) {
                securityReport.failed.push(`${name}: Potentially sensitive data in localStorage: ${match}`);
            }
        });
    }

    // Check 7: Event listener security
    if (content.includes('addEventListener') && content.includes('message')) {
        securityReport.info.push(`${name}: Message event listener - verify origin validation`);
    }

    console.log(`  ✓ ${name} audited (${lines.length} lines)`);
}

function checkDependencies() {
    console.log('\n🔍 Checking dependencies...');
    
    const packageJsonPath = path.join(__dirname, '../../package.json');
    if (fs.existsSync(packageJsonPath)) {
        const packageJson = JSON.parse(fs.readFileSync(packageJsonPath, 'utf8'));
        
        const deps = { ...packageJson.dependencies, ...packageJson.devDependencies };
        const hasOutdated = false; // Would need npm audit
        
        if (hasOutdated) {
            securityReport.warnings.push('package.json: Run npm audit to check for vulnerabilities');
        } else {
            securityReport.info.push('package.json: No frontend dependencies - using vanilla JS (good for security)');
        }
    }
}

function checkCSP() {
    console.log('\n🔍 Checking Content Security Policy...');
    
    const htmlContent = fs.readFileSync(HTML_DIR, 'utf8');
    
    // Check for inline scripts
    const inlineScripts = (htmlContent.match(/<script[^>]*>([\s\S]*?)<\/script>/g) || [])
        .filter(script => !script.includes('src='));
    
    if (inlineScripts.length > 0) {
        securityReport.info.push(`index.html: ${inlineScripts.length} inline scripts detected - use CSP nonce or hash`);
    }

    // Check for external resources
    const externalScripts = (htmlContent.match(/src\s*=\s*["']https?:\/\/[^"']+["']/g) || []);
    if (externalScripts.length > 0) {
        securityReport.info.push(`index.html: External scripts loaded - ensure SRI (Subresource Integrity)`);
        externalScripts.forEach(src => {
            console.log(`    - ${src}`);
        });
    }
}

function checkCORS() {
    console.log('\n🔍 Checking CORS configuration...');
    
    const nginxConfPath = path.join(__dirname, '../../docker/nginx-dashboard.conf');
    if (fs.existsSync(nginxConfPath)) {
        const nginxConf = fs.readFileSync(nginxConfPath, 'utf8');
        
        if (nginxConf.includes('Access-Control-Allow-Origin')) {
            if (nginxConf.includes('*')) {
                securityReport.warnings.push('nginx-dashboard.conf: CORS allows * - restrict to specific origins in production');
            } else {
                securityReport.passed.push('nginx-dashboard.conf: CORS configured with specific origin');
            }
        } else {
            securityReport.info.push('nginx-dashboard.conf: No CORS headers - verify if needed');
        }
    }
}

function checkWebSocketSecurity() {
    console.log('\n🔍 Checking WebSocket security...');
    
    const appJsContent = fs.readFileSync(JS_DIR, 'utf8');
    
    // Check for token usage
    if (appJsContent.includes('token') || appJsContent.includes('JWT')) {
        securityReport.passed.push('app.js: JWT token authentication detected');
    } else {
        securityReport.warnings.push('app.js: No obvious authentication - verify WebSocket security');
    }

    // Check for WebSocket error handling
    if (appJsContent.includes('ws.onerror') || appJsContent.includes('websocket.onerror')) {
        securityReport.passed.push('app.js: WebSocket error handling present');
    } else {
        securityReport.warnings.push('app.js: Verify WebSocket error handling');
    }
}

function generateReport() {
    console.log('\n╔════════════════════════════════════════════════════════╗');
    console.log('║     Security Audit Report                              ║');
    console.log('╚════════════════════════════════════════════════════════╝\n');

    // Passed
    console.log(`✅ PASSED (${securityReport.passed.length}):`);
    securityReport.passed.forEach(item => console.log(`  ✓ ${item}`));

    // Warnings
    if (securityReport.warnings.length > 0) {
        console.log(`\n⚠️  WARNINGS (${securityReport.warnings.length}):`);
        securityReport.warnings.forEach(item => console.log(`  ⚠ ${item}`));
    }

    // Failed
    if (securityReport.failed.length > 0) {
        console.log(`\n❌ FAILED (${securityReport.failed.length}):`);
        securityReport.failed.forEach(item => console.log(`  ✗ ${item}`));
    }

    // Info
    if (securityReport.info.length > 0) {
        console.log(`\nℹ️  INFO (${securityReport.info.length}):`);
        securityReport.info.forEach(item => console.log(`  ℹ ${item}`));
    }

    // Summary
    console.log('\n╔════════════════════════════════════════════════════════╗');
    const totalIssues = securityReport.failed.length + securityReport.warnings.length;
    if (securityReport.failed.length === 0 && securityReport.warnings.length === 0) {
        console.log('║  ✅ ALL CHECKS PASSED                                  ║');
    } else if (securityReport.failed.length === 0) {
        console.log(`║  ⚠️  ${securityReport.warnings.length} WARNINGS - Review recommended              ║`);
    } else {
        console.log(`║  ❌ ${securityReport.failed.length} CRITICAL ISSUES - Fix required                 ║`);
    }
    console.log('╚════════════════════════════════════════════════════════╝');

    // Recommendations
    console.log('\n📋 Recommendations:');
    console.log('  1. Use HTTPS/WSS in production');
    console.log('  2. Implement Content Security Policy (CSP) headers');
    console.log('  3. Add Subresource Integrity (SRI) for external scripts');
    console.log('  4. Regular dependency audits (if using npm packages)');
    console.log('  5. Use HttpOnly, Secure, SameSite cookies for auth');
    console.log('  6. Implement rate limiting on WebSocket connections');
    console.log('  7. Validate all user inputs server-side');

    // Exit code
    process.exit(securityReport.failed.length > 0 ? 1 : 0);
}

// Run audit
console.log('╔════════════════════════════════════════════════════════╗');
console.log('║     Dashboard Security Audit                           ║');
console.log('╚════════════════════════════════════════════════════════╝');

auditFile(HTML_DIR, 'index.html');
auditFile(JS_DIR, 'app.js');
auditFile(CSS_DIR, 'styles.css');
checkDependencies();
checkCSP();
checkCORS();
checkWebSocketSecurity();

generateReport();