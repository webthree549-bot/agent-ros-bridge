"""Tests for MCP transport."""

import asyncio
import json
from unittest.mock import AsyncMock, MagicMock, Mock, call, patch

import pytest

from agent_ros_bridge.integrations.mcp_transport import MCPServerTransport


class TestMCPServerTransportInitialization:
    """Test MCP transport initialization."""

    def test_init_default(self):
        """Test default initialization."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge)
        assert mcp.bridge is mock_bridge
        assert mcp.mode == "stdio"
        assert mcp.running is False
        assert mcp.tools == []

    def test_init_sse_mode(self):
        """Test SSE mode initialization."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge, mode="sse")
        assert mcp.mode == "sse"


class TestMCPServerTransportStart:
    """Test starting MCP server."""

    @pytest.mark.asyncio
    async def test_start_stdio_mode(self):
        """Test starting in stdio mode."""
        mock_bridge = Mock()
        mock_bridge.get_actions = Mock(return_value=["navigate", "move"])
        mcp = MCPServerTransport(mock_bridge, mode="stdio")

        # Mock _run_stdio to avoid blocking
        mcp._run_stdio = AsyncMock()

        await mcp.start()

        assert mcp.running is True
        assert len(mcp.tools) == 2

    @pytest.mark.asyncio
    async def test_start_sse_mode(self):
        """Test starting in SSE mode."""
        mock_bridge = Mock()
        mock_bridge.get_actions = Mock(return_value=[])
        mcp = MCPServerTransport(mock_bridge, mode="sse")
        mcp._run_sse = AsyncMock()

        await mcp.start()

        assert mcp.running is True
        mcp._run_sse.assert_called_once()

    @pytest.mark.asyncio
    async def test_start_unknown_mode(self):
        """Test starting with unknown mode."""
        mock_bridge = Mock()
        mock_bridge.get_actions = Mock(return_value=[])
        mcp = MCPServerTransport(mock_bridge, mode="unknown")

        with pytest.raises(ValueError, match="Unknown mode"):
            await mcp.start()


class TestMCPServerTransportDiscoverTools:
    """Test tool discovery."""

    @pytest.mark.asyncio
    async def test_discover_tools(self):
        """Test discovering tools from bridge."""
        mock_bridge = Mock()
        mock_bridge.get_actions = Mock(return_value=["navigate", "move", "status"])
        mcp = MCPServerTransport(mock_bridge)

        await mcp._discover_tools()

        assert len(mcp.tools) == 3
        assert mcp.tools[0]["name"] == "navigate"
        assert "description" in mcp.tools[0]

    @pytest.mark.asyncio
    async def test_discover_tools_no_bridge(self):
        """Test discovering tools without bridge."""
        mcp = MCPServerTransport(None)

        await mcp._discover_tools()

        assert mcp.tools == []


class TestMCPServerTransportHandleMessage:
    """Test message handling."""

    @pytest.mark.asyncio
    async def test_handle_tools_list(self):
        """Test handling tools/list request."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge)
        mcp.tools = [{"name": "navigate", "description": "Navigate"}]

        message = {"jsonrpc": "2.0", "id": 1, "method": "tools/list"}
        response = await mcp._handle_message(message)

        assert response["jsonrpc"] == "2.0"
        assert response["id"] == 1
        assert "tools" in response["result"]

    @pytest.mark.asyncio
    async def test_handle_initialize(self):
        """Test handling initialize request."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge)

        message = {"jsonrpc": "2.0", "id": 1, "method": "initialize"}
        response = await mcp._handle_message(message)

        assert response["jsonrpc"] == "2.0"
        assert response["id"] == 1
        assert response["result"]["protocolVersion"] == "2024-11-05"
        assert "serverInfo" in response["result"]

    @pytest.mark.asyncio
    async def test_handle_tools_call(self):
        """Test handling tools/call request."""
        mock_bridge = Mock()
        mock_bridge.execute_action = AsyncMock(return_value={"success": True})
        mcp = MCPServerTransport(mock_bridge)

        message = {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "tools/call",
            "params": {"name": "navigate", "arguments": {"x": 5}},
        }
        response = await mcp._handle_message(message)

        assert response["jsonrpc"] == "2.0"
        assert response["id"] == 1
        assert "content" in response["result"]

    @pytest.mark.asyncio
    async def test_handle_unknown_method(self):
        """Test handling unknown method."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge)

        message = {"jsonrpc": "2.0", "id": 1, "method": "unknown"}
        response = await mcp._handle_message(message)

        assert response is None


class TestMCPServerTransportExecuteTool:
    """Test tool execution."""

    @pytest.mark.asyncio
    async def test_execute_tool_success(self):
        """Test successful tool execution."""
        mock_bridge = Mock()
        mock_bridge.execute_action = AsyncMock(return_value={"success": True})
        mcp = MCPServerTransport(mock_bridge)

        result = await mcp._execute_tool("navigate", {"x": 5})

        assert result["isError"] is False
        assert "content" in result

    @pytest.mark.asyncio
    async def test_execute_tool_no_bridge(self):
        """Test tool execution without bridge."""
        mcp = MCPServerTransport(None)

        result = await mcp._execute_tool("navigate", {"x": 5})

        assert result["isError"] is True
        assert "Bridge not available" in result["content"][0]["text"]

    @pytest.mark.asyncio
    async def test_execute_tool_no_execute_action(self):
        """Test tool execution without execute_action."""
        mock_bridge = Mock()
        # No execute_action attribute
        del mock_bridge.execute_action
        mcp = MCPServerTransport(mock_bridge)

        result = await mcp._execute_tool("navigate", {"x": 5})

        assert result["isError"] is True
        assert "doesn't support" in result["content"][0]["text"]

    @pytest.mark.asyncio
    async def test_execute_tool_exception(self):
        """Test tool execution with exception."""
        mock_bridge = Mock()
        mock_bridge.execute_action = AsyncMock(side_effect=Exception("Failed"))
        mcp = MCPServerTransport(mock_bridge)

        result = await mcp._execute_tool("navigate", {"x": 5})

        assert result["isError"] is True
        assert "Failed" in result["content"][0]["text"]


class TestMCPServerTransportSendMessage:
    """Test sending messages."""

    @pytest.mark.asyncio
    async def test_send_message(self):
        """Test sending message."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge)

        with patch("builtins.print") as mock_print:
            await mcp._send_message({"test": "message"})
            mock_print.assert_called_once()
            # Check JSON is printed
            call_args = mock_print.call_args[0][0]
            assert json.loads(call_args) == {"test": "message"}


class TestMCPServerTransportStop:
    """Test stopping server."""

    @pytest.mark.asyncio
    async def test_stop(self):
        """Test stopping server."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge)
        mcp.running = True

        await mcp.stop()

        assert mcp.running is False


class TestMCPServerTransportRunSSE:
    """Test SSE mode."""

    @pytest.mark.asyncio
    async def test_run_sse(self):
        """Test SSE mode execution."""
        mock_bridge = Mock()
        mcp = MCPServerTransport(mock_bridge, mode="sse")

        # Just verify it doesn't crash
        await mcp._run_sse()
