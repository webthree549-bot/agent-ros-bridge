"""Unit tests for metrics collection."""

import pytest

from agent_ros_bridge.metrics import MetricConfig, HealthChecker


@pytest.mark.unit
class TestMetricConfig:
    """Test MetricConfig dataclass."""
    
    def test_default_config(self):
        """Test default configuration."""
        config = MetricConfig()
        assert config.enabled is True
        assert config.port == 9090
        assert config.host == "0.0.0.0"
        assert config.namespace == "agent_ros_bridge"
    
    def test_custom_config(self):
        """Test custom configuration."""
        config = MetricConfig(port=8080, host="localhost")
        assert config.port == 8080
        assert config.host == "localhost"


@pytest.mark.unit
class TestHealthChecker:
    """Test HealthChecker functionality."""
    
    @pytest.fixture
    def health(self, bridge):
        """Create health checker."""
        return HealthChecker(bridge)
    
    @pytest.mark.asyncio
    async def test_register_check(self, health):
        """Test registering a health check."""
        async def check():
            return True, "OK"
        
        health.register_check("test", check)
        assert "test" in health._checks
    
    @pytest.mark.asyncio
    async def test_check_health(self, health):
        """Test running health checks."""
        async def good_check():
            return True, "All good"
        
        async def bad_check():
            return False, "Something wrong"
        
        health.register_check("good", good_check)
        health.register_check("bad", bad_check)
        
        result = await health.check_health()
        
        assert result["status"] == "unhealthy"
        assert result["checks"]["good"]["status"] == "healthy"
        assert result["checks"]["bad"]["status"] == "unhealthy"
    
    @pytest.mark.asyncio
    async def test_check_health_error(self, health):
        """Test health check that raises exception."""
        async def error_check():
            raise RuntimeError("Boom")
        
        health.register_check("error", error_check)
        
        result = await health.check_health()
        
        assert result["status"] == "unhealthy"
        assert result["checks"]["error"]["status"] == "error"
    
    @pytest.mark.asyncio
    async def test_check_ready(self, health, bridge):
        """Test ready check."""
        # No transports, so should be ready
        assert await health.check_ready() is True
