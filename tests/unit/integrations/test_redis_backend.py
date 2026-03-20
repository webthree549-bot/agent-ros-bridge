"""Tests for Redis backend module."""

from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from agent_ros_bridge.integrations.redis_backend import RedisContextBackend


class TestRedisContextBackendInitialization:
    """Test Redis backend initialization."""

    def test_init_default_url(self):
        """Test default Redis URL."""
        backend = RedisContextBackend()
        assert backend.redis_url == "redis://localhost:6379/0"
        assert backend.client is None
        assert backend._connected is False

    def test_init_custom_url(self):
        """Test custom Redis URL."""
        backend = RedisContextBackend("redis://custom:6379/1")
        assert backend.redis_url == "redis://custom:6379/1"


class TestRedisConnect:
    """Test Redis connection."""

    @pytest.mark.asyncio
    async def test_connect_success(self):
        """Test successful connection."""
        with patch("agent_ros_bridge.integrations.redis_backend.redis") as mock_redis:
            mock_client = AsyncMock()
            mock_client.ping = AsyncMock()
            mock_redis.from_url = AsyncMock(return_value=mock_client)

            backend = RedisContextBackend()
            await backend.connect()

            assert backend._connected is True
            assert backend.client is not None

    @pytest.mark.asyncio
    async def test_connect_failure(self):
        """Test connection failure."""
        with patch("agent_ros_bridge.integrations.redis_backend.redis") as mock_redis:
            mock_redis.from_url = AsyncMock(side_effect=Exception("Connection refused"))

            backend = RedisContextBackend()
            with pytest.raises(Exception, match="Connection refused"):
                await backend.connect()

            assert backend._connected is False


class TestRedisDisconnect:
    """Test Redis disconnection."""

    @pytest.mark.asyncio
    async def test_disconnect(self):
        """Test disconnection."""
        backend = RedisContextBackend()
        backend.client = AsyncMock()
        backend._connected = True

        await backend.disconnect()

        assert backend._connected is False
        backend.client.close.assert_called_once()


class TestGetContext:
    """Test getting context."""

    @pytest.mark.asyncio
    async def test_get_context_not_connected(self):
        """Test get context when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.get_context("session1")

    @pytest.mark.asyncio
    async def test_get_context_found(self):
        """Test getting existing context."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.get = AsyncMock(return_value='{"key": "value"}')

        result = await backend.get_context("session1")

        assert result == {"key": "value"}
        backend.client.get.assert_called_once_with("context:session1")

    @pytest.mark.asyncio
    async def test_get_context_not_found(self):
        """Test getting non-existent context."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.get = AsyncMock(return_value=None)

        result = await backend.get_context("session1")

        assert result is None


class TestSaveContext:
    """Test saving context."""

    @pytest.mark.asyncio
    async def test_save_context_not_connected(self):
        """Test save context when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.save_context("session1", {"key": "value"})

    @pytest.mark.asyncio
    async def test_save_context(self):
        """Test saving context."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.setex = AsyncMock()

        await backend.save_context("session1", {"key": "value"}, ttl=3600)

        backend.client.setex.assert_called_once()
        call_args = backend.client.setex.call_args
        # setex(key, ttl, value) - positional args
        assert call_args[0][0] == "context:session1"
        assert call_args[0][1] == 3600


class TestDeleteContext:
    """Test deleting context."""

    @pytest.mark.asyncio
    async def test_delete_context_not_connected(self):
        """Test delete context when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.delete_context("session1")

    @pytest.mark.asyncio
    async def test_delete_context(self):
        """Test deleting context."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.delete = AsyncMock()

        await backend.delete_context("session1")

        backend.client.delete.assert_called_once_with("context:session1")


class TestLogInteraction:
    """Test logging interactions."""

    @pytest.mark.asyncio
    async def test_log_interaction_not_connected(self):
        """Test log interaction when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.log_interaction("session1", "cmd", {}, {})

    @pytest.mark.asyncio
    async def test_log_interaction(self):
        """Test logging interaction."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.lpush = AsyncMock()
        backend.client.ltrim = AsyncMock()
        backend.client.expire = AsyncMock()

        await backend.log_interaction("session1", "go forward", {"tool": "move"}, {"success": True})

        backend.client.lpush.assert_called_once()
        backend.client.ltrim.assert_called_once_with("history:session1", 0, 99)
        backend.client.expire.assert_called_once()


class TestGetHistory:
    """Test getting history."""

    @pytest.mark.asyncio
    async def test_get_history_not_connected(self):
        """Test get history when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.get_history("session1")

    @pytest.mark.asyncio
    async def test_get_history(self):
        """Test getting history."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.lrange = AsyncMock(return_value=['{"command": "go"}'])

        result = await backend.get_history("session1", n=5)

        assert len(result) == 1
        assert result[0]["command"] == "go"
        backend.client.lrange.assert_called_once_with("history:session1", 0, 4)


class TestLearnLocation:
    """Test learning locations."""

    @pytest.mark.asyncio
    async def test_learn_location_not_connected(self):
        """Test learn location when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.learn_location("session1", "kitchen", {"x": 10, "y": 5})

    @pytest.mark.asyncio
    async def test_learn_location(self):
        """Test learning location."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.hset = AsyncMock()

        await backend.learn_location("session1", "kitchen", {"x": 10, "y": 5})

        backend.client.hset.assert_called_once()
        call_args = backend.client.hset.call_args
        assert call_args[0][0] == "locations:session1"
        assert call_args[0][1] == "kitchen"


class TestGetLocation:
    """Test getting locations."""

    @pytest.mark.asyncio
    async def test_get_location_not_connected(self):
        """Test get location when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.get_location("session1", "kitchen")

    @pytest.mark.asyncio
    async def test_get_location_found(self):
        """Test getting existing location."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.hget = AsyncMock(return_value='{"x": 10, "y": 5}')

        result = await backend.get_location("session1", "kitchen")

        assert result == {"x": 10, "y": 5}

    @pytest.mark.asyncio
    async def test_get_location_not_found(self):
        """Test getting non-existent location."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.hget = AsyncMock(return_value=None)

        result = await backend.get_location("session1", "kitchen")

        assert result is None


class TestGetAllLocations:
    """Test getting all locations."""

    @pytest.mark.asyncio
    async def test_get_all_locations_not_connected(self):
        """Test get all locations when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.get_all_locations("session1")

    @pytest.mark.asyncio
    async def test_get_all_locations(self):
        """Test getting all locations."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.hgetall = AsyncMock(
            return_value={
                "kitchen": '{"x": 10, "y": 5}',
                "office": '{"x": 20, "y": 10}',
            }
        )

        result = await backend.get_all_locations("session1")

        assert len(result) == 2
        assert result["kitchen"] == {"x": 10, "y": 5}
        assert result["office"] == {"x": 20, "y": 10}


class TestPublishUpdate:
    """Test publishing updates."""

    @pytest.mark.asyncio
    async def test_publish_not_connected(self):
        """Test publish when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.publish_update("channel1", {"msg": "hello"})

    @pytest.mark.asyncio
    async def test_publish_update(self):
        """Test publishing update."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.publish = AsyncMock()

        await backend.publish_update("channel1", {"msg": "hello"})

        backend.client.publish.assert_called_once()


class TestHealthCheck:
    """Test health check."""

    @pytest.mark.asyncio
    async def test_health_check_not_connected(self):
        """Test health check when not connected."""
        backend = RedisContextBackend()
        result = await backend.health_check()
        assert result is False

    @pytest.mark.asyncio
    async def test_health_check_success(self):
        """Test successful health check."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.ping = AsyncMock()

        result = await backend.health_check()

        assert result is True

    @pytest.mark.asyncio
    async def test_health_check_failure(self):
        """Test failed health check."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.ping = AsyncMock(side_effect=Exception("Connection lost"))

        result = await backend.health_check()

        assert result is False


class TestGetStats:
    """Test getting stats."""

    @pytest.mark.asyncio
    async def test_get_stats_not_connected(self):
        """Test get stats when not connected."""
        backend = RedisContextBackend()
        with pytest.raises(RuntimeError, match="Not connected"):
            await backend.get_stats()

    @pytest.mark.asyncio
    async def test_get_stats(self):
        """Test getting stats."""
        backend = RedisContextBackend()
        backend._connected = True
        backend.client = AsyncMock()
        backend.client.info = AsyncMock(
            return_value={
                "connected_clients": 5,
                "used_memory_human": "1.5M",
                "total_commands_processed": 1000,
                "keyspace_hits": 800,
                "keyspace_misses": 200,
            }
        )

        result = await backend.get_stats()

        assert result["connected_clients"] == 5
        assert result["used_memory_human"] == "1.5M"
