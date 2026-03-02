#!/usr/bin/env python3
"""gRPC Transport Example.

This example demonstrates:
1. Creating a bridge with gRPC transport
2. Starting the gRPC server
3. Client connection and command sending
4. Telemetry streaming
5. Health checking

Usage:
    # Terminal 1: Start the server
    $ python examples/grpc_example.py server

    # Terminal 2: Run client commands
    $ python examples/grpc_example.py client

    # Or run both in test mode
    $ python examples/grpc_example.py test
"""

import asyncio
import logging
import sys

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s")
logger = logging.getLogger("grpc_example")


async def run_server():
    """Run gRPC server example."""
    from agent_ros_bridge.gateway_v2.core import Bridge
    from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport

    logger.info("=" * 60)
    logger.info("Starting gRPC Server Example")
    logger.info("=" * 60)

    # Create bridge
    bridge = Bridge()
    logger.info("✓ Bridge created")

    # Create gRPC transport
    grpc_transport = GRPCTransport({
        "host": "0.0.0.0",
        "port": 50051,
        "reflection": True,
    })

    bridge.transport_manager.register(grpc_transport)
    logger.info("✓ gRPC transport registered on port 50051")

    # Start bridge
    await bridge.start()
    logger.info("✓ Bridge started")
    logger.info("")
    logger.info("Server ready! Try these from another terminal:")
    logger.info("  python examples/grpc_example.py client")
    logger.info("")
    logger.info("Press Ctrl+C to stop")
    logger.info("=" * 60)

    try:
        # Print stats periodically
        while True:
            await asyncio.sleep(10)
            stats = grpc_transport.get_stats()
            logger.info(f"[Stats] Clients: {stats['connected_clients']}")
    except asyncio.CancelledError:
        pass
    finally:
        await bridge.stop()
        logger.info("Server stopped")


async def run_client():
    """Run gRPC client example."""
    from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCClient

    logger.info("=" * 60)
    logger.info("Starting gRPC Client Example")
    logger.info("=" * 60)

    # Create client
    client = GRPCClient(target="localhost:50051")

    try:
        # Connect
        await client.connect()
        logger.info("✓ Connected to server")

        # Health check
        logger.info("\n1. Health Check:")
        health = await client.health_check()
        logger.info(f"   Server health: {health}")

        # Send commands
        logger.info("\n2. Send Commands:")

        # Discover command
        result = await client.send_command("discover", {})
        logger.info(f"   discover: success={result['success']}")

        # Fleet list command
        result = await client.send_command("fleet.list", {})
        logger.info(f"   fleet.list: {result}")

        # Subscribe to telemetry
        logger.info("\n3. Subscribe to Telemetry (5 seconds):")
        try:
            msg_count = 0
            async for telemetry in client.subscribe_telemetry(["status", "heartbeat"]):
                logger.info(f"   Telemetry: {telemetry.topic}")
                msg_count += 1
                if msg_count >= 5:
                    break
            logger.info(f"   Received {msg_count} telemetry messages")
        except Exception as e:
            logger.warning(f"   Subscription ended: {e}")

        logger.info("\n✓ Client operations completed")

    except Exception as e:
        logger.error(f"Client error: {e}")
    finally:
        await client.close()
        logger.info("✓ Client disconnected")


async def run_test():
    """Run server and client in same process for testing."""
    from agent_ros_bridge.gateway_v2.core import Bridge
    from agent_ros_bridge.gateway_v2.transports.grpc_transport import GRPCTransport, GRPCClient

    logger.info("=" * 60)
    logger.info("gRPC Integration Test")
    logger.info("=" * 60)

    # Create bridge with gRPC transport
    bridge = Bridge()
    grpc_transport = GRPCTransport({
        "host": "127.0.0.1",
        "port": 50052,  # Use different port for testing
        "reflection": False,
    })
    bridge.transport_manager.register(grpc_transport)

    # Start server
    await bridge.start()
    logger.info("✓ Server started on port 50052")

    # Give server time to start
    await asyncio.sleep(0.5)

    try:
        # Create client and connect
        client = GRPCClient(target="127.0.0.1:50052")
        await client.connect()
        logger.info("✓ Client connected")

        # Test health check
        health = await client.health_check()
        assert health["success"], "Health check failed"
        logger.info(f"✓ Health check: {health['result']}")

        # Test commands
        result = await client.send_command("discover", {})
        assert result["success"], f"Discover failed: {result}"
        logger.info("✓ Discover command works")

        result = await client.send_command("fleet.list", {})
        assert result["success"], f"Fleet list failed: {result}"
        logger.info("✓ Fleet list command works")

        # Test telemetry subscription (brief)
        msg_count = 0
        async for telemetry in client.subscribe_telemetry(["_heartbeat"]):
            msg_count += 1
            if msg_count >= 2:
                break
        logger.info(f"✓ Telemetry subscription works ({msg_count} messages)")

        # Get server stats
        stats = grpc_transport.get_stats()
        assert stats["connected_clients"] >= 1, "Expected at least 1 client"
        logger.info(f"✓ Server stats: {stats['connected_clients']} clients connected")

        await client.close()
        logger.info("✓ Client closed")

        logger.info("\n" + "=" * 60)
        logger.info("All tests passed!")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"Test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        await bridge.stop()
        logger.info("Server stopped")

    return 0


def main():
    """Main entry point."""
    if len(sys.argv) < 2:
        print("Usage:")
        print(f"  {sys.argv[0]} server  # Run gRPC server")
        print(f"  {sys.argv[0]} client  # Run gRPC client")
        print(f"  {sys.argv[0]} test    # Run integration test")
        sys.exit(1)

    mode = sys.argv[1]

    if mode == "server":
        asyncio.run(run_server())
    elif mode == "client":
        asyncio.run(run_client())
    elif mode == "test":
        exit_code = asyncio.run(run_test())
        sys.exit(exit_code)
    else:
        print(f"Unknown mode: {mode}")
        sys.exit(1)


if __name__ == "__main__":
    main()
