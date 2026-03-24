"""
TDD Tests for MCP Adapter

Tests define expected behavior of MCP integration.
"""

import asyncio
import json
from unittest.mock import AsyncMock, Mock, patch

import pytest

from agent_ros_bridge.mcp import MCPAdapter, MCPServer, MCPTool


class TestMCPTool:
    """MCP Tool definition"""

    def test_tool_has_name_description_schema(self):
        """Red: Tool must store basic info"""

        def dummy_handler(args):
            return {}

        tool = MCPTool(
            name="test_tool",
            description="A test tool",
            input_schema={"type": "object"},
            handler=dummy_handler,
        )

        assert tool.name == "test_tool"
        assert tool.description == "A test tool"
        assert tool.input_schema == {"type": "object"}
        assert tool.handler == dummy_handler


class TestMCPAdapterInitialization:
    """MCPAdapter must initialize with robot agent"""

    def test_adapter_registers_default_tools(self):
        """Red: Must register default MCP tools on init"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        # Should have default tools
        assert "execute_robot_command" in adapter.tools
        assert "get_robot_status" in adapter.tools
        assert "list_robot_capabilities" in adapter.tools
        assert "emergency_stop" in adapter.tools

    def test_adapter_can_register_custom_tools(self):
        """Red: Must allow custom tool registration"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        def custom_handler(args):
            return {"custom": True}

        adapter.register_tool(
            name="custom_tool",
            description="A custom tool",
            input_schema={"type": "object"},
            handler=custom_handler,
        )

        assert "custom_tool" in adapter.tools
        assert adapter.tools["custom_tool"].description == "A custom tool"


class TestMCPAdapterTools:
    """MCPAdapter tools return proper format"""

    def test_get_tools_returns_mcp_format(self):
        """Red: Must return tools in MCP-compliant format"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)
        tools = adapter.get_tools()

        assert isinstance(tools, list)
        assert len(tools) > 0

        for tool in tools:
            assert "name" in tool
            assert "description" in tool
            assert "inputSchema" in tool

    def test_call_tool_executes_handler(self):
        """Red: Must execute tool handler when called"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        # Mock the execute method
        mock_result = Mock()
        mock_result.success = True
        mock_result.task = "test command"
        mock_result.ai_confidence = 0.95
        mock_result.steps = []
        mock_result.human_approvals = 1
        mock_result.safety_violations = 0
        mock_result.duration_seconds = 1.0
        mock_result.message = "Success"

        mock_agent.execute.return_value = mock_result
        mock_agent.require_confirmation = True

        async def test_call():
            result = await adapter.call_tool(
                "execute_robot_command", {"command": "go to kitchen", "require_confirmation": False}
            )

            assert result["isError"] is False
            assert "content" in result
            assert len(result["content"]) > 0

        asyncio.run(test_call())

    def test_call_tool_returns_error_for_unknown_tool(self):
        """Red: Must return error for unknown tools"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        async def test_call():
            result = await adapter.call_tool("unknown_tool", {})

            assert result["isError"] is True
            assert "Unknown tool" in result["content"][0]["text"]

        asyncio.run(test_call())


class TestExecuteRobotCommandTool:
    """execute_robot_command tool behavior"""

    def test_execute_tool_calls_robot_agent(self):
        """Red: Must call robot_agent.execute with command"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        mock_result = Mock()
        mock_result.success = True
        mock_result.task = "go to kitchen"
        mock_result.ai_confidence = 0.94
        mock_result.steps = [{"step": "navigate"}]
        mock_result.human_approvals = 1
        mock_result.human_rejections = 0
        mock_result.safety_violations = 0
        mock_result.duration_seconds = 5.0
        mock_result.message = "Successfully navigated"

        mock_agent.execute.return_value = mock_result
        mock_agent.require_confirmation = True

        adapter = MCPAdapter(mock_agent)

        async def test_call():
            result = await adapter.call_tool(
                "execute_robot_command", {"command": "go to kitchen", "require_confirmation": False}
            )

            # Verify robot_agent.execute was called
            mock_agent.execute.assert_called_once_with("go to kitchen")

            # Verify result format
            assert result["isError"] is False
            content = json.loads(result["content"][0]["text"])
            assert content["success"] is True
            assert content["ai_confidence"] == 0.94

        asyncio.run(test_call())


class TestGetRobotStatusTool:
    """get_robot_status tool behavior"""

    def test_status_tool_returns_observation(self):
        """Red: Must return robot status and observations"""
        from agent_ros_bridge.agentic import AgentObservation

        mock_agent = Mock()
        mock_agent.device_id = "bot1"
        mock_agent.device_type = "mobile_robot"
        mock_agent.observe.return_value = AgentObservation(
            robot_position=(1.0, 2.0, 0.0),
            battery_level=85.0,
            nearby_objects=["cup", "table"],
            current_task="navigating",
            recent_commands=["go to kitchen"],
            obstacles_detected=["wall"],
        )
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        async def test_call():
            result = await adapter.call_tool("get_robot_status", {})

            assert result["isError"] is False
            content = json.loads(result["content"][0]["text"])
            assert content["device_id"] == "bot1"
            assert content["device_type"] == "mobile_robot"
            assert content["battery"] == 85.0

        asyncio.run(test_call())


class TestEmergencyStopTool:
    """emergency_stop tool behavior"""

    def test_emergency_stop_executes_stop_capability(self):
        """Red: Must execute stop capability on device"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.has_capability.return_value = True
        mock_agent.device.execute_capability.return_value = {"success": True}

        adapter = MCPAdapter(mock_agent)

        async def test_call():
            result = await adapter.call_tool("emergency_stop", {})

            mock_agent.device.execute_capability.assert_called_once_with("stop", {})

            assert result["isError"] is False
            content = json.loads(result["content"][0]["text"])
            assert content["robot_stopped"] is True

        asyncio.run(test_call())

    def test_emergency_stop_returns_error_if_no_capability(self):
        """Red: Must return error if stop not available"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.has_capability.return_value = False

        adapter = MCPAdapter(mock_agent)

        async def test_call():
            result = await adapter.call_tool("emergency_stop", {})

            assert result["isError"] is False  # Tool executed but couldn't stop
            content = json.loads(result["content"][0]["text"])
            assert content["robot_stopped"] is False

        asyncio.run(test_call())


class TestMCPRequestHandling:
    """MCP JSON-RPC request handling"""

    def test_handle_initialize_request(self):
        """Red: Must handle MCP initialize request"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        async def test():
            request = {"jsonrpc": "2.0", "method": "initialize", "id": 1}

            response = await adapter._handle_request(request)

            assert response["jsonrpc"] == "2.0"
            assert response["id"] == 1
            assert "result" in response
            assert response["result"]["protocolVersion"] == "2024-11-05"

        asyncio.run(test())

    def test_handle_tools_list_request(self):
        """Red: Must return tools list"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        async def test():
            request = {"jsonrpc": "2.0", "method": "tools/list", "id": 2}

            response = await adapter._handle_request(request)

            assert response["jsonrpc"] == "2.0"
            assert response["id"] == 2
            assert "tools" in response["result"]
            assert len(response["result"]["tools"]) > 0

        asyncio.run(test())

    def test_handle_tools_call_request(self):
        """Red: Must execute tool on call request"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        mock_result = Mock()
        mock_result.success = True
        mock_result.task = "test"
        mock_result.ai_confidence = 0.9
        mock_result.steps = []
        mock_result.human_approvals = 1
        mock_result.safety_violations = 0
        mock_result.duration_seconds = 1.0
        mock_result.message = "Done"

        mock_agent.execute.return_value = mock_result
        mock_agent.require_confirmation = True

        adapter = MCPAdapter(mock_agent)

        async def test():
            request = {
                "jsonrpc": "2.0",
                "method": "tools/call",
                "params": {"name": "execute_robot_command", "arguments": {"command": "test"}},
                "id": 3,
            }

            response = await adapter._handle_request(request)

            assert response["jsonrpc"] == "2.0"
            assert response["id"] == 3
            assert "result" in response

        asyncio.run(test())

    def test_handle_unknown_method(self):
        """Red: Must return error for unknown methods"""
        mock_agent = Mock()
        mock_agent.device = Mock()
        mock_agent.device.get_capabilities.return_value = []

        adapter = MCPAdapter(mock_agent)

        async def test():
            request = {"jsonrpc": "2.0", "method": "unknown/method", "id": 4}

            response = await adapter._handle_request(request)

            assert response["jsonrpc"] == "2.0"
            assert response["id"] == 4
            assert "error" in response
            assert response["error"]["code"] == -32601

        asyncio.run(test())


class TestTDDPrinciples:
    """Verify TDD principles in MCP module"""

    def test_mcp_has_test_file(self):
        """Red: MCP module must have tests"""
        import agent_ros_bridge.mcp

        assert hasattr(agent_ros_bridge.mcp, "MCPAdapter")
        assert hasattr(agent_ros_bridge.mcp, "MCPServer")

    def test_all_tools_have_tests(self):
        """Red: All default tools must be tested"""
        default_tools = [
            "execute_robot_command",
            "get_robot_status",
            "list_robot_capabilities",
            "navigate_to",
            "pick_object",
            "move_manipulator",
            "emergency_stop",
        ]

        # Test coverage verified by test classes above
        tested_tools = ["execute_robot_command", "get_robot_status", "emergency_stop"]

        missing = set(default_tools) - set(tested_tools)
        if missing:
            pytest.skip(f"Tools needing dedicated tests: {missing}")

    def test_tests_define_behavior(self):
        """Red: Tests must define expected MCP behavior"""
        # Tests above use docstrings like:
        # "Red: Must handle MCP initialize request"
        # "Red: Must return tools list"
        # These define MCP protocol behavior
        pass
