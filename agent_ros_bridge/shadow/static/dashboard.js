/**
 * Shadow Mode Dashboard JavaScript
 */

// API base URL
const API_BASE = '/api';

// Fetch metrics from API
async function fetchMetrics() {
    try {
        const response = await fetch(`${API_BASE}/metrics`);
        const data = await response.json();
        return data;
    } catch (error) {
        console.error('Failed to fetch metrics:', error);
        return null;
    }
}

// Fetch recent decisions
async function fetchDecisions(limit = 10) {
    try {
        const response = await fetch(`${API_BASE}/decisions?limit=${limit}`);
        const data = await response.json();
        return data;
    } catch (error) {
        console.error('Failed to fetch decisions:', error);
        return [];
    }
}

// Update metrics display
function updateMetrics(metrics) {
    if (!metrics) return;

    const agreementRate = document.getElementById('agreement-rate');
    const totalDecisions = document.getElementById('total-decisions');
    const pendingCount = document.getElementById('pending-count');
    const completedCount = document.getElementById('completed-count');

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

// Update decisions table
function updateDecisionsTable(decisions) {
    const tbody = document.getElementById('decisions-table');
    if (!tbody) return;

    if (decisions.length === 0) {
        tbody.innerHTML = `
            <tr>
                <td colspan="6" style="text-align: center; color: #999;">No decisions logged yet</td>
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
                <td><span class="status ${statusClass}">${statusText}</span></td>
                <td>${score ? score.toFixed(2) : '--'}</td>
            </tr>
        `;
    }).join('');
}

// Refresh all metrics
async function refreshMetrics() {
    const metrics = await fetchMetrics();
    updateMetrics(metrics);

    const decisions = await fetchDecisions(10);
    updateDecisionsTable(decisions);

    // Update last updated time
    const lastUpdated = document.getElementById('last-updated');
    if (lastUpdated) {
        lastUpdated.textContent = `Last updated: ${new Date().toLocaleTimeString()}`;
    }
}

// Auto-refresh every 5 seconds
setInterval(refreshMetrics, 5000);

// Initial load
document.addEventListener('DOMContentLoaded', refreshMetrics);
