"""
Human Confirmation UI

Web interface for operators to approve/reject/modify AI suggestions.
Integrates with ShadowModeHooks for real-time decision tracking.

Features:
- Display AI proposals with confidence, reasoning, entities
- Approve/Reject/Modify actions
- REST API for frontend
- WebSocket for real-time updates
- Auto-approve high confidence proposals
- Safety warnings for low confidence
"""

import asyncio
import time
from dataclasses import dataclass, field
from typing import Any

# Standard library imports complete

# Optional FastAPI imports
try:
    from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
    from fastapi.middleware.cors import CORSMiddleware
    from fastapi.responses import HTMLResponse

    FASTAPI_AVAILABLE = True
except ImportError:
    FASTAPI_AVAILABLE = False
    FastAPI = None  # type: ignore
    WebSocket = None  # type: ignore
    WebSocketDisconnect = None  # type: ignore


@dataclass
class Proposal:
    """Represents an AI proposal awaiting confirmation"""

    proposal_id: str
    robot_id: str
    intent_type: str
    confidence: float
    entities: list[dict[str, Any]] = field(default_factory=list)
    reasoning: str = ""
    timestamp: float = field(default_factory=time.time)
    status: str = "pending"  # pending, approved, rejected, modified


@dataclass
class UIConfig:
    """Configuration for Confirmation UI"""

    port: int = 8080
    auto_approve_threshold: float = 1.0  # 1.0 = never auto-approve
    require_confirmation: bool = True
    timeout_seconds: float = 60.0
    enable_websocket: bool = True
    show_reasoning: bool = True
    show_confidence: bool = True


class ConfirmationUI:
    """
    Human confirmation interface for AI proposals.

    Provides:
    - Web interface for operators
    - REST API for integration
    - WebSocket for real-time updates
    - Integration with ShadowModeHooks
    """

    def __init__(
        self,
        port: int = 8080,
        shadow_hooks=None,
        auto_approve_threshold: float = 1.0,
        require_confirmation: bool = True,
        timeout_seconds: float = 60.0,
    ):
        """
        Initialize confirmation UI.

        Args:
            port: Web server port
            shadow_hooks: ShadowModeHooks instance
            auto_approve_threshold: Confidence threshold for auto-approval
            require_confirmation: Whether to require explicit confirmation
            timeout_seconds: Proposal timeout
        """
        self.port = port
        self.config = UIConfig(
            port=port,
            auto_approve_threshold=auto_approve_threshold,
            require_confirmation=require_confirmation,
            timeout_seconds=timeout_seconds,
        )

        self._shadow_hooks = shadow_hooks
        self._proposals: dict[str, Proposal] = {}
        self._server = None
        self._websocket_clients: set = set()

    @property
    def timeout_seconds(self) -> float:
        """Get timeout configuration"""
        return self.config.timeout_seconds

    def _get_shadow_hooks(self):
        """Lazy init shadow hooks"""
        if self._shadow_hooks is None:
            from ..shadow.hooks import ShadowModeHooks

            self._shadow_hooks = ShadowModeHooks()
        return self._shadow_hooks

    def receive_proposal(self, proposal_data: dict[str, Any]) -> str:
        """
        Receive a new AI proposal.

        Args:
            proposal_data: Proposal from AI system

        Returns:
            proposal_id: ID of the created proposal
        """
        proposal_id = proposal_data.get("proposal_id", f"prop_{int(time.time()*1000)}")

        proposal = Proposal(
            proposal_id=proposal_id,
            robot_id=proposal_data.get("robot_id", "unknown"),
            intent_type=proposal_data.get("intent_type", "UNKNOWN"),
            confidence=proposal_data.get("confidence", 0.0),
            entities=proposal_data.get("entities", []),
            reasoning=proposal_data.get("reasoning", ""),
        )

        self._proposals[proposal_id] = proposal

        # Check auto-approve
        if (
            proposal.confidence >= self.config.auto_approve_threshold
            and not self.config.require_confirmation
        ):
            self.approve_proposal(proposal_id)
        else:
            # Broadcast to connected clients
            self._broadcast_update(
                {
                    "type": "new_proposal",
                    "proposal": self._proposal_to_dict(proposal),
                }
            )

        return proposal_id

    def display_proposal(self, proposal: dict[str, Any]) -> str:
        """
        Generate HTML for displaying a proposal.

        Args:
            proposal: Proposal data

        Returns:
            HTML string
        """
        intent_type = proposal.get("intent_type", "UNKNOWN")
        confidence = proposal.get("confidence", 0.0)
        robot_id = proposal.get("robot_id", "unknown")
        entities = proposal.get("entities", [])
        reasoning = proposal.get("reasoning", "")

        # Determine risk level
        risk_level = self._calculate_risk_level(confidence, intent_type)
        risk_class = f"risk-{risk_level.lower()}"

        # Build entities HTML
        entities_html = ""
        for entity in entities:
            entities_html += (
                f"<span class='entity'>{entity.get('type')}: {entity.get('value')}</span>"
            )

        # Warning for low confidence
        warning_html = ""
        if confidence < 0.7:
            warning_html = "<div class='warning'>⚠️ Low Confidence</div>"

        html = f"""
        <div class="proposal {risk_class}" id="proposal-{proposal.get('proposal_id')}">
            <div class="proposal-header">
                <span class="intent-type">{intent_type}</span>
                <span class="robot-id">Robot: {robot_id}</span>
                <span class="confidence">{confidence*100:.1f}%</span>
            </div>
            {warning_html}
            <div class="proposal-body">
                <div class="entities">
                    <strong>Entities:</strong> {entities_html or 'None'}
                </div>
                <div class="reasoning">
                    <strong>Reasoning:</strong> {reasoning or 'N/A'}
                </div>
                <div class="risk-level">
                    <strong>Risk:</strong> {risk_level}
                </div>
            </div>
            <div class="proposal-actions">
                <button class="btn-approve" onclick="approve('{proposal.get('proposal_id')}')">
                    ✓ Approve
                </button>
                <button class="btn-reject" onclick="showRejectForm('{proposal.get('proposal_id')}')">
                    ✗ Reject
                </button>
                <button class="btn-modify" onclick="showModifyForm('{proposal.get('proposal_id')}')">
                    ✎ Modify
                </button>
            </div>
        </div>
        """

        return html

    def _calculate_risk_level(self, confidence: float, intent_type: str) -> str:
        """Calculate risk level for display"""
        if intent_type in ["EMERGENCY_STOP", "SAFETY"]:
            return "HIGH"
        if confidence < 0.5:
            return "HIGH"
        if confidence < 0.7:
            return "MEDIUM"
        return "LOW"

    def _proposal_to_dict(self, proposal: Proposal) -> dict[str, Any]:
        """Convert proposal to dictionary"""
        return {
            "proposal_id": proposal.proposal_id,
            "robot_id": proposal.robot_id,
            "intent_type": proposal.intent_type,
            "confidence": proposal.confidence,
            "entities": proposal.entities,
            "reasoning": proposal.reasoning,
            "timestamp": proposal.timestamp,
            "status": proposal.status,
        }

    def approve_proposal(self, proposal_id: str) -> dict[str, Any]:
        """
        Approve a proposal and execute the command.

        Args:
            proposal_id: ID of proposal to approve

        Returns:
            Result dict
        """
        if proposal_id not in self._proposals:
            return {"success": False, "error": "Proposal not found"}

        proposal = self._proposals[proposal_id]
        proposal.status = "approved"

        # Execute command
        result = self._execute_command(proposal)

        # Log to shadow hooks
        if self._shadow_hooks:
            self._shadow_hooks.on_human_command(
                {
                    "robot_id": proposal.robot_id,
                    "command": proposal.intent_type.lower(),
                    "parameters": {"entities": proposal.entities},
                }
            )

        # Broadcast update
        self._broadcast_update(
            {
                "type": "proposal_approved",
                "proposal_id": proposal_id,
            }
        )

        self._broadcast_metrics()

        return {"success": True, "result": result}

    def reject_proposal(self, proposal_id: str, reason: str = "") -> dict[str, Any]:
        """
        Reject a proposal.

        Args:
            proposal_id: ID of proposal to reject
            reason: Rejection reason

        Returns:
            Result dict
        """
        if not reason:
            raise ValueError("Rejection reason is required")

        if proposal_id not in self._proposals:
            return {"success": False, "error": "Proposal not found"}

        proposal = self._proposals[proposal_id]
        proposal.status = "rejected"

        # Log to shadow hooks
        if self._shadow_hooks:
            self._shadow_hooks.on_human_rejected(
                robot_id=proposal.robot_id,
                ai_proposal_id=proposal_id,
                rejection_reason=reason,
            )

        # Broadcast update
        self._broadcast_update(
            {
                "type": "proposal_rejected",
                "proposal_id": proposal_id,
                "reason": reason,
            }
        )

        self._broadcast_metrics()

        return {"success": True}

    def modify_proposal(
        self,
        proposal_id: str,
        original: dict[str, Any],
        modified: dict[str, Any],
    ) -> dict[str, Any]:
        """
        Modify and execute a proposal.

        Args:
            proposal_id: ID of proposal
            original: Original parameters
            modified: Modified parameters

        Returns:
            Result dict
        """
        if proposal_id not in self._proposals:
            return {"success": False, "error": "Proposal not found"}

        proposal = self._proposals[proposal_id]
        proposal.status = "modified"

        # Execute modified command
        result = self._execute_command(proposal, modified_params=modified)

        # Log to shadow hooks
        if self._shadow_hooks:
            self._shadow_hooks.on_human_modified(
                robot_id=proposal.robot_id,
                ai_proposal_id=proposal_id,
                original=original,
                modified=modified,
            )

        # Broadcast update
        self._broadcast_update(
            {
                "type": "proposal_modified",
                "proposal_id": proposal_id,
            }
        )

        self._broadcast_metrics()

        return {"success": True, "result": result}

    def _execute_command(
        self,
        proposal: Proposal,
        modified_params: dict | None = None,
    ) -> dict[str, Any]:
        """Execute the approved/modified command"""
        # Execute via shadow hooks if available
        if self._shadow_hooks and hasattr(self._shadow_hooks, "_bridge"):
            bridge = self._shadow_hooks._bridge
            if bridge:
                try:
                    # Execute the command on the robot
                    command = {
                        "action": proposal.intent_type,
                        "robot_id": proposal.robot_id,
                        "parameters": modified_params or {},
                    }
                    result = bridge.execute(command)
                    return {
                        "executed": True,
                        "command": proposal.intent_type,
                        "robot_id": proposal.robot_id,
                        "result": result,
                        "timestamp": time.time(),
                    }
                except Exception as e:
                    return {
                        "executed": False,
                        "error": str(e),
                        "timestamp": time.time(),
                    }

        # Fallback: return mock result
        return {
            "executed": True,
            "command": proposal.intent_type,
            "robot_id": proposal.robot_id,
            "timestamp": time.time(),
        }

    def get_pending_proposals(self) -> list[dict[str, Any]]:
        """Get all pending proposals"""
        return [
            self._proposal_to_dict(p) for p in self._proposals.values() if p.status == "pending"
        ]

    def get_metrics(self) -> dict[str, Any]:
        """Get UI metrics"""
        if self._shadow_hooks:
            return self._shadow_hooks.get_stats()

        # Fallback to local metrics
        total = len(self._proposals)
        approved = sum(1 for p in self._proposals.values() if p.status == "approved")
        rejected = sum(1 for p in self._proposals.values() if p.status == "rejected")

        return {
            "total_decisions": total,
            "approved": approved,
            "rejected": rejected,
            "pending": total - approved - rejected,
        }

    # API Endpoints (for web server integration)

    def api_get_pending(self) -> list[dict[str, Any]]:
        """API: Get pending proposals"""
        return self.get_pending_proposals()

    def api_approve(self, proposal_id: str) -> dict[str, Any]:
        """API: Approve proposal"""
        return self.approve_proposal(proposal_id)

    def api_reject(self, proposal_id: str, reason: str) -> dict[str, Any]:
        """API: Reject proposal"""
        return self.reject_proposal(proposal_id, reason)

    def api_get_metrics(self) -> dict[str, Any]:
        """API: Get metrics"""
        return self.get_metrics()

    # Web Interface

    def render_interface(self) -> str:
        """Render the full HTML interface"""
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <title>AI Proposal Confirmation</title>
            <style>
                body { font-family: Arial, sans-serif; margin: 20px; }
                .proposal { border: 1px solid #ccc; padding: 15px; margin: 10px 0; border-radius: 5px; }
                .proposal-header { display: flex; justify-content: space-between; font-weight: bold; }
                .intent-type { color: #2196F3; }
                .confidence { color: #4CAF50; }
                .warning { color: #FF9800; background: #FFF3E0; padding: 5px; margin: 5px 0; }
                .risk-high { border-left: 4px solid #f44336; }
                .risk-medium { border-left: 4px solid #FF9800; }
                .risk-low { border-left: 4px solid #4CAF50; }
                .proposal-actions { margin-top: 10px; }
                button { padding: 8px 16px; margin-right: 10px; cursor: pointer; }
                .btn-approve { background: #4CAF50; color: white; border: none; }
                .btn-reject { background: #f44336; color: white; border: none; }
                .btn-modify { background: #2196F3; color: white; border: none; }
                .entity { background: #e3f2fd; padding: 2px 6px; margin: 2px; border-radius: 3px; display: inline-block; }
                #metrics { position: fixed; top: 20px; right: 20px; background: #f5f5f5; padding: 15px; border-radius: 5px; }
            </style>
        </head>
        <body>
            <h1>🤖 AI Proposal Confirmation</h1>
            <div id="metrics">
                <h3>Metrics</h3>
                <div id="metrics-content">Loading...</div>
            </div>
            <div id="proposals">
                <p>Waiting for proposals...</p>
            </div>
            <script>
                function approve(proposalId) {
                    fetch(`/api/approve/${proposalId}`, {method: 'POST'})
                        .then(r => r.json())
                        .then(data => { if(data.success) location.reload(); });
                }
                function showRejectForm(proposalId) {
                    const reason = prompt('Reason for rejection:');
                    if(reason) {
                        fetch(`/api/reject/${proposalId}`, {
                            method: 'POST',
                            headers: {'Content-Type': 'application/json'},
                            body: JSON.stringify({reason})
                        }).then(r => r.json())
                          .then(data => { if(data.success) location.reload(); });
                    }
                }
                function showModifyForm(proposalId) {
                    alert('Modify form would open here');
                }
                // Poll for updates
                setInterval(() => {
                    fetch('/api/metrics').then(r => r.json()).then(m => {
                        document.getElementById('metrics-content').innerHTML =
                            `Total: ${m.total_decisions || 0}<br>Agreement: ${((m.agreement_rate || 0)*100).toFixed(1)}%`;
                    });
                }, 2000);
            </script>
        </body>
        </html>
        """
        return html

    def start_server(self) -> None:
        """Start the FastAPI web server with WebSocket support"""
        if not FASTAPI_AVAILABLE:
            raise ImportError("FastAPI is required. Install with: pip install fastapi uvicorn")

        self._app = self._create_fastapi_app()
        self._server_task = asyncio.create_task(self._run_server())

    def _create_fastapi_app(self) -> FastAPI:
        """Create FastAPI application with routes"""
        app = FastAPI(
            title="Agent ROS Bridge - Confirmation UI",
            description="Human confirmation interface for AI proposals",
            version="0.6.5",
        )

        # CORS middleware
        app.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        @app.get("/", response_class=HTMLResponse)
        async def get_dashboard():
            """Serve the main dashboard"""
            return self._generate_dashboard_html()

        @app.get("/api/proposals")
        async def get_proposals():
            """Get all pending proposals"""
            return {"proposals": self.get_pending_proposals()}

        @app.get("/api/proposals/{proposal_id}")
        async def get_proposal(proposal_id: str):
            """Get specific proposal"""
            if proposal_id not in self._proposals:
                raise HTTPException(status_code=404, detail="Proposal not found")
            return self._proposal_to_dict(self._proposals[proposal_id])

        @app.post("/api/proposals/{proposal_id}/approve")
        async def approve_proposal(proposal_id: str):
            """Approve a proposal"""
            result = self.approve_proposal(proposal_id)
            return result

        @app.post("/api/proposals/{proposal_id}/reject")
        async def reject_proposal(proposal_id: str):
            """Reject a proposal"""
            result = self.reject_proposal(proposal_id)
            return result

        @app.post("/api/proposals/{proposal_id}/modify")
        async def modify_proposal(proposal_id: str, request: dict):
            """Modify and approve a proposal"""
            modified_params = request.get("modified_params", {})
            result = self.modify_proposal(proposal_id, modified_params)
            return result

        @app.get("/api/metrics")
        async def get_metrics():
            """Get current metrics"""
            return self.get_metrics()

        @app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket endpoint for real-time updates"""
            await websocket.accept()
            self._websocket_clients.add(websocket)
            try:
                while True:
                    # Keep connection alive and handle client messages
                    data = await websocket.receive_text()
                    # Echo back or handle commands
                    await websocket.send_json({"type": "ack", "data": data})
            except WebSocketDisconnect:
                self._websocket_clients.discard(websocket)
            except Exception:
                self._websocket_clients.discard(websocket)

        return app

    async def _run_server(self) -> None:
        """Run the uvicorn server"""
        import uvicorn

        config = uvicorn.Config(
            self._app,
            host="0.0.0.0",  # nosec B104 - UI server requires external operator access
            port=self.port,
            log_level="info",
        )
        server = uvicorn.Server(config)
        await server.serve()

    def _broadcast_update(self, message: dict[str, Any]) -> None:
        """Broadcast update to WebSocket clients"""
        if not self._websocket_clients:
            return

        # Create async task to broadcast
        asyncio.create_task(self._async_broadcast(message))

    async def _async_broadcast(self, message: dict[str, Any]) -> None:
        """Async broadcast to all connected WebSocket clients"""
        disconnected = set()

        for websocket in self._websocket_clients:
            try:
                await websocket.send_json(message)
            except Exception:
                disconnected.add(websocket)

        # Remove disconnected clients
        self._websocket_clients -= disconnected

    def _broadcast_metrics(self) -> None:
        """Broadcast metrics update"""
        metrics = self.get_metrics()
        self._broadcast_update(
            {
                "type": "metrics_update",
                "metrics": metrics,
            }
        )


# Convenience function
def create_confirmation_ui(
    port: int = 8080,
    shadow_hooks=None,
    auto_approve_threshold: float = 1.0,
) -> ConfirmationUI:
    """
    Create and configure confirmation UI.

    Args:
        port: Web server port
        shadow_hooks: ShadowModeHooks instance
        auto_approve_threshold: Auto-approve confidence threshold

    Returns:
        Configured ConfirmationUI
    """
    return ConfirmationUI(
        port=port,
        shadow_hooks=shadow_hooks,
        auto_approve_threshold=auto_approve_threshold,
    )
