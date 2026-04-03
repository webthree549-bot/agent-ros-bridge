#!/usr/bin/env python3
"""ROS2 to WebSocket bridge for real-time dashboard data."""

import asyncio
import json
import logging
import sys

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ros2_websocket_bridge")

sys.path.insert(0, '/opt/ros/jazzy/lib/python3.12/site-packages')
sys.path.insert(0, '/opt/ros/jazzy/local/lib/python3.12/dist-packages')

try:
    import rclpy
    from rclpy.node import Node
    from tf2_msgs.msg import TFMessage
    from nav_msgs.msg import Odometry
    ROS2_AVAILABLE = True
except ImportError as e:
    logger.warning(f"ROS2 not available: {e}")
    ROS2_AVAILABLE = False

try:
    import websockets
    WEBSOCKET_AVAILABLE = True
except ImportError:
    logger.error("websockets not installed")
    WEBSOCKET_AVAILABLE = False


class BridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_ws_bridge')
        self.ws_clients = set()
        self.data = {'tf': None, 'odom': None}
        
        if ROS2_AVAILABLE:
            self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_timer(0.1, self.on_timer)
            logger.info("Bridge initialized")
    
    def on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id == 'base_link':
                self.data['tf'] = {'x': t.transform.translation.x, 'y': t.transform.translation.y}
    
    def on_odom(self, msg):
        self.data['odom'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'wz': msg.twist.twist.angular.z
        }
    
    def on_timer(self):
        asyncio.create_task(self.broadcast())
    
    async def broadcast(self):
        if not self.ws_clients:
            return
        msg = json.dumps({'type': 'telemetry', 'data': self.data})
        bad = set()
        for c in self.ws_clients:
            try:
                await c.send(msg)
            except:
                bad.add(c)
        self.ws_clients -= bad


async def handler(ws, path, node):
    node.ws_clients.add(ws)
    logger.info(f"Client: {len(node.ws_clients)}")
    try:
        async for _ in ws:
            pass
    except:
        pass
    finally:
        node.ws_clients.discard(ws)
        logger.info(f"Client left: {len(node.ws_clients)}")


async def ros2_spin(node):
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.01)
        await asyncio.sleep(0.01)


async def main():
    if not ROS2_AVAILABLE or not WEBSOCKET_AVAILABLE:
        logger.error("Missing deps")
        return
    
    rclpy.init()
    node = BridgeNode()
    
    srv = websockets.serve(lambda ws, path: handler(ws, path, node), '0.0.0.0', 8766)
    logger.info("Bridge on ws://0.0.0.0:8766")
    
    try:
        await asyncio.gather(srv, ros2_spin(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
